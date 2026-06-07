#include "dronecan_node.h"
#include "app_descriptor.h"
#include <string.h>

/* RTC_BOOT_CANBL / RTC_BOOT_FWOK are defined in dronecan_node.h,
 * values must match AP_HAL_ChibiOS/hwdef/common/watchdog.h. */

/* Enable RTC peripheral clock and write backup register 0. */
static inline void rtc_write_bkp0(uint32_t value)
{
    RCC->APB1ENR  |= RCC_APB1ENR_PWREN;   /* enable PWR clock */
    PWR->CR       |= PWR_CR_DBP;           /* disable backup domain write protection */
    RCC->BDCR     |= RCC_BDCR_RTCEN;      /* enable RTC */
    RTC->BKP0R     = value;
}

/* -------------------------------------------------------------------------
 * Canard instance – single global, 4 KB memory pool
 * -------------------------------------------------------------------------*/
static CanardInstance g_canard;
static uint8_t        g_canard_pool[4096];

static uint8_t g_node_status_tid = 0;  /* transfer ID counter for NodeStatus */

/* Forward reference to CAN1 handle created by CubeIDE in main.c */
extern CAN_HandleTypeDef hcan1;

/* -------------------------------------------------------------------------
 * Forward declarations (all private)
 * -------------------------------------------------------------------------*/
static bool     should_accept(const CanardInstance *ins,
                               uint64_t *out_sig,
                               uint16_t dtid,
                               CanardTransferType tt,
                               uint8_t src);
static void     on_reception(CanardInstance *ins, CanardRxTransfer *t);
static void     handle_get_node_info(CanardInstance *ins, CanardRxTransfer *t);
static void     handle_begin_fw_update(CanardInstance *ins, CanardRxTransfer *t);
static void     send_node_status(void);
static void     pump_tx(void);
static void     request_fw_update(uint8_t server_node_id);

/* =========================================================================
 * Public API
 * =========================================================================*/

void dronecan_init(void)
{
    /* --- CAN acceptance filter: pass ALL frames (extended) to FIFO0 --- */
    CAN_FilterTypeDef f = {0};
    f.FilterActivation     = ENABLE;
    f.FilterBank           = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterIdHigh         = 0x0000;
    f.FilterIdLow          = 0x0000;
    f.FilterMaskIdHigh     = 0x0000;
    f.FilterMaskIdLow      = 0x0000;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &f);

    /* Enable FIFO0 pending interrupt (NVIC already armed by MX_CAN1_Init) */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* Start CAN peripheral */
    HAL_CAN_Start(&hcan1);

    /* --- Initialise libcanard --- */
    canardInit(&g_canard,
               g_canard_pool, sizeof(g_canard_pool),
               on_reception,
               should_accept,
               NULL);
    canardSetLocalNodeID(&g_canard, DRONECAN_NODE_ID);
}

void dronecan_process(void)
{
    static uint32_t last_status_ms = 0;
    uint32_t now = HAL_GetTick();

    /* NodeStatus broadcast every 1 000 ms */
    if ((now - last_status_ms) >= 1000U)
    {
        last_status_ms = now;
        send_node_status();
    }

    /* Drain TX queue each cycle */
    pump_tx();

    /* Remove stale incomplete multi-frame receptions (~1 s interval) */
    canardCleanupStaleTransfers(&g_canard, (uint64_t)now * 1000ULL);
}

/* =========================================================================
 * HAL CAN callback – overrides the __weak default in stm32f4xx_hal_can.c.
 * Called from CAN1_RX0_IRQHandler → HAL_CAN_IRQHandler.
 * =========================================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t raw[8];

    /* Drain all available frames in FIFO0 */
    while (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, raw) == HAL_OK)
    {
        CanardCANFrame frame;
        if (hdr.IDE == CAN_ID_EXT)
        {
            frame.id = (hdr.ExtId & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
        }
        else
        {
            frame.id = (hdr.StdId & CANARD_CAN_STD_ID_MASK);
        }
        frame.data_len = (uint8_t)hdr.DLC;
        frame.iface_id = 0;
        memcpy(frame.data, raw, hdr.DLC);

        canardHandleRxFrame(&g_canard,
                            &frame,
                            (uint64_t)HAL_GetTick() * 1000ULL);
    }
}

/* =========================================================================
 * Private helpers
 * =========================================================================*/

static void pump_tx(void)
{
    const CanardCANFrame *txf;
    while ((txf = canardPeekTxQueue(&g_canard)) != NULL)
    {
        CAN_TxHeaderTypeDef h;
        uint32_t mbox;
        h.ExtId              = txf->id & CANARD_CAN_EXT_ID_MASK;
        h.IDE                = CAN_ID_EXT;
        h.RTR                = CAN_RTR_DATA;
        h.DLC                = txf->data_len;
        h.TransmitGlobalTime = DISABLE;

        if (HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)txf->data, &mbox) == HAL_OK)
        {
            canardPopTxQueue(&g_canard);
        }
        else
        {
            break;  /* All three mailboxes busy – retry next call */
        }
    }
}

/* ----- NodeStatus broadcast (DTID 341) --------------------------------- */
static void send_node_status(void)
{
    uint32_t uptime = HAL_GetTick() / 1000U;
    uint8_t buf[7];
    buf[0] = (uint8_t)(uptime >>  0);
    buf[1] = (uint8_t)(uptime >>  8);
    buf[2] = (uint8_t)(uptime >> 16);
    buf[3] = (uint8_t)(uptime >> 24);
    /* byte4: health[1:0]=0(OK), mode[4:2]=0(OPERATIONAL), sub_mode[7:5]=0 */
    buf[4] = 0x00U;
    buf[5] = 0x00U;  /* vendor_specific_status_code low  */
    buf[6] = 0x00U;  /* vendor_specific_status_code high */

    canardBroadcast(&g_canard,
                    UAVCAN_NODE_STATUS_SIGNATURE,
                    UAVCAN_NODE_STATUS_DTID,
                    &g_node_status_tid,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buf, sizeof(buf));
}

/* ----- GetNodeInfo response (service DTID 1) --------------------------- */
static void handle_get_node_info(CanardInstance *ins, CanardRxTransfer *t)
{
    uint8_t buf[64];
    uint8_t *p = buf;

    /* ---- NodeStatus (7 bytes) ---------------------------------------- */
    uint32_t uptime = HAL_GetTick() / 1000U;
    *p++ = (uint8_t)(uptime >>  0);
    *p++ = (uint8_t)(uptime >>  8);
    *p++ = (uint8_t)(uptime >> 16);
    *p++ = (uint8_t)(uptime >> 24);
    *p++ = 0x00U;   /* health=OK(0), mode=OPERATIONAL(0), sub_mode=0 */
    *p++ = 0x00U;   /* vendor_specific low */
    *p++ = 0x00U;   /* vendor_specific high */

    /* ---- SoftwareVersion (3 bytes, no optional fields) --------------- */
    *p++ = APP_VERSION_MAJOR;   /* major */
    *p++ = APP_VERSION_MINOR;   /* minor */
    *p++ = 0x00U;               /* optional_field_flags = 0 */

    /* ---- HardwareVersion (19 bytes) ---------------------------------- */
    /* board_id 6000 = 0x1770: major = high byte, minor = low byte */
    *p++ = 0x17U;               /* hw_major */
    *p++ = 0x70U;               /* hw_minor */
    /* STM32F407 96-bit UID at 0x1FFF7A10 (12 bytes) + 4 zero padding */
    const uint8_t *uid = (const uint8_t *)0x1FFF7A10U;
    memcpy(p, uid, 12);  p += 12;
    memset(p, 0, 4);     p +=  4;
    *p++ = 0x00U;               /* certificate_of_authenticity length = 0 */

    /* ---- Node name (tail array, no length prefix) -------------------- */
    const char *name = DRONECAN_NODE_NAME;
    uint8_t nlen = (uint8_t)strlen(name);
    memcpy(p, name, nlen);
    p += nlen;

    uint8_t tid = t->transfer_id;
    canardRequestOrRespond(ins,
                           t->source_node_id,
                           UAVCAN_GET_NODE_INFO_SIGNATURE,
                           (uint8_t)UAVCAN_GET_NODE_INFO_DTID,
                           &tid,
                           CANARD_TRANSFER_PRIORITY_LOW,
                           CanardResponse,
                           buf,
                           (uint16_t)(p - buf));
}

/* ----- BeginFirmwareUpdate handler (service DTID 40) ------------------- */
static void handle_begin_fw_update(CanardInstance *ins, CanardRxTransfer *t)
{
    /* Extract source_node_id from request payload byte 0 (may be 0 → use sender) */
    uint8_t srv_nid = (t->payload_len >= 1) ? t->payload_head[0] : 0;
    if (srv_nid == 0) srv_nid = t->source_node_id;

    /* Respond OK before resetting */
    uint8_t resp[1] = {0};   /* error_code = ERROR_OK */
    uint8_t tid = t->transfer_id;
    canardRequestOrRespond(ins,
                           t->source_node_id,
                           UAVCAN_BEGIN_FW_UPDATE_SIGNATURE,
                           (uint8_t)UAVCAN_BEGIN_FW_UPDATE_DTID,
                           &tid,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardResponse,
                           resp, sizeof(resp));

    pump_tx();  /* Flush response before reset */

    request_fw_update(srv_nid);  /* This calls NVIC_SystemReset() */
}

/* ----- Reset into bootloader for CAN firmware update ------------------ */
static void request_fw_update(uint8_t server_node_id)
{
    (void)server_node_id;
    /* Write RTC_BOOT_CANBL magic – bootloader reads BKP0R on reset and stays
     * in bootloader mode to accept a firmware upload over DroneCAN. */
    rtc_write_bkp0(RTC_BOOT_CANBL | (uint32_t)DRONECAN_NODE_ID);
    NVIC_SystemReset();
}

/* ----- canard callbacks ------------------------------------------------ */
static bool should_accept(const CanardInstance *ins,
                           uint64_t *out_sig,
                           uint16_t dtid,
                           CanardTransferType tt,
                           uint8_t src)
{
    (void)ins; (void)src;

    if (tt == CanardTransferTypeRequest)
    {
        if (dtid == UAVCAN_GET_NODE_INFO_DTID) {
            *out_sig = UAVCAN_GET_NODE_INFO_SIGNATURE;
            return true;
        }
        if (dtid == UAVCAN_BEGIN_FW_UPDATE_DTID) {
            *out_sig = UAVCAN_BEGIN_FW_UPDATE_SIGNATURE;
            return true;
        }
    }
    return false;
}

static void on_reception(CanardInstance *ins, CanardRxTransfer *t)
{
    if (t->transfer_type == CanardTransferTypeRequest)
    {
        if (t->data_type_id == UAVCAN_GET_NODE_INFO_DTID)
        {
            handle_get_node_info(ins, t);
        }
        else if (t->data_type_id == UAVCAN_BEGIN_FW_UPDATE_DTID)
        {
            handle_begin_fw_update(ins, t);
        }
    }
}
