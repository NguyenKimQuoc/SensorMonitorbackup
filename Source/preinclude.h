#define SECURE 1
#define TC_LINKKEY_JOIN
#define NV_INIT
#define NV_RESTORE

#define TP2_LEGACY_ZC

#define xZTOOL_P1
#define MT_TASK
#define MT_APP_FUNC
#define MT_SYS_FUNC
#define MT_ZDO_FUNC
#define MT_ZDO_MGMT
#define MT_APP_CNF_FUNC
//#define LEGACY_LCD_DEBUG
//#define LCD_SUPPORTED DEBUG
#define MULTICAST_ENABLED FALSE
#define ZCL_READ
#define ZCL_BASIC
#define ZCL_IDENTIFY
#define ZCL_REPORTING_DEVICE

#define ZSTACK_DEVICE_BUILD (DEVICE_BUILD_ENDDEVICE)

#define BDB_FINDING_BINDING_CAPABILITY_ENABLED 1
#define BDB_REPORTING TRUE

#define POWER_SAVING
#define ISR_KEYINTERRUPT
#define NWK_AUTO_POLL

#define BDB_MAX_CLUSTERENDPOINTS_REPORTING 10
//#define ZG_BUILD_ENDDEVICE_TYPE TRUE
#define DISABLE_GREENPOWER_BASIC_PROXY
//#define DEFAULT_CHANLIST 0x07FFF800  // ????? ??? ?????? ?? ???? ???????

#define DO_DEBUG_UART

// i2c
#define OCM_CLK_PORT  0
#define OCM_DATA_PORT   0
#define OCM_CLK_PIN   6
#define OCM_DATA_PIN    5


#ifdef DO_DEBUG_UART
#define HAL_UART TRUE
#define HAL_UART_DMA 1
#define INT_HEAP_LEN (2685 - 0x4B - 0xBB)
#endif


#define HAL_KEY_P0_INPUT_PINS (BV(7)|BV(1))

#define HAL_KEY TRUE
#include "hal_board_cfg_SensorMonitor.h"