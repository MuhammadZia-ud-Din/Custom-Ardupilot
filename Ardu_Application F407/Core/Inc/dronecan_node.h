#ifndef DRONECAN_NODE_H
#define DRONECAN_NODE_H

#include "main.h"
#include "canard.h"

/* Node identity – must match hwdef.dat values */
#define DRONECAN_NODE_ID    14U
#define DRONECAN_NODE_NAME  "internal-app.puf"

/* RTC backup-register magic values (must match AP_HAL_ChibiOS watchdog.h) */
#define RTC_BOOT_CANBL  0xb0080000UL   /* app → bootloader: stay for CAN update  */
#define RTC_BOOT_FWOK   0x890b4f89UL   /* app → bootloader: firmware is healthy  */

/* Data type IDs (UAVCAN v0 / DroneCAN) */
#define UAVCAN_NODE_STATUS_DTID            341U
#define UAVCAN_GET_NODE_INFO_DTID          1U
#define UAVCAN_BEGIN_FW_UPDATE_DTID        40U

/* Data type signatures – must be exact or canard drops the frame */
#define UAVCAN_NODE_STATUS_SIGNATURE       0x0F0868D0C1A7C6F1ULL
#define UAVCAN_GET_NODE_INFO_SIGNATURE     0xEE468A8121C46A9EULL
#define UAVCAN_BEGIN_FW_UPDATE_SIGNATURE   0xB7D725DF72724126ULL

/* Initialise canard, configure CAN filter, start CAN peripheral.
 * Call once after MX_CAN1_Init() and MX_TIM6_Init(). */
void dronecan_init(void);

/* Non-blocking periodic work: broadcast NodeStatus every 1 s, drain TX queue.
 * Call unconditionally in the main while(1) loop. */
void dronecan_process(void);

#endif /* DRONECAN_NODE_H */
