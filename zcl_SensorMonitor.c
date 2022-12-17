#include <stdio.h>
#include <stdlib.h>

#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "MT_SYS.h"
#include "OSAL_PwrMgr.h"

#include "nwk_util.h"

#include "zcl_ms.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_diagnostic.h"
#include "zcl_SensorMonitor.h"

#include "bdb.h"
#include "bdb_interface.h"
#include "gp_interface.h"



#if defined ( INTER_PAN )
#if defined ( BDB_TL_INITIATOR )
#include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR
#if defined ( BDB_TL_TARGET )
#include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET
#endif // INTER_PAN

#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
#include "bdb_touchlink.h"
#endif

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

// my library
#include "uart.h"
#include "bitmasks.h"
#include "delay.h"
#include "hal_i2c.h"

#include "sht20.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "BH1750.h"
/*********************************************************************
* MACROS
*/


/*********************************************************************
* CONSTANTS
*/


/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

//extern bool requestNewTrustCenterLinkKey;
byte zclSensorMonitor_TaskID;
//int16 zclSensorMonitor_MeasuredValue;
afAddrType_t zclSensorMonitor_DstAddr;


/*********************************************************************
* GLOBAL FUNCTIONS
*/
void zclSensorMonitor_ReportTemp( void );
/*********************************************************************
* LOCAL VARIABLES
*/
uint8 SeqNum = 0;

uint8 giGenAppScreenMode = GENERIC_MAINMODE;   // display the main screen mode first

uint8 gPermitDuration = 0;    // permit joining default to disabled

devStates_t zclSensorMonitor_NwkState = DEV_INIT;

uint8 error = 0; //variable for error code. For codes see system.h
uint8 userRegister; //variable for user register
bool endOfBattery; //variable for end of battery
nt16 sRH; //variable for raw humidity ticks
float humidityRH; //variable for relative humidity[%RH] as float
char humitityOutStr[21]; //output string for humidity value
nt16 sT; //variable for raw temperature ticks
float temperatureC; //variable for temperature[°C] as float
char temperatureOutStr[21]; //output string for temperature value
uint8 SerialNumber_SHT2x[8]; //64bit serial number
double BH1750_lux;
char BH1750Str[30];
/*********************************************************************
* LOCAL FUNCTIONS
*/
static void zclSensorMonitor_HandleKeys( byte shift, byte keys );
static void zclSensorMonitor_BasicResetCB( void );
static void zclSensorMonitor_ProcessIdentifyTimeChange( uint8 endpoint );
static void zclSensorMonitor_BindNotification( bdbBindNotificationData_t *data );
#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
static void zclSensorMonitor_ProcessTouchlinkTargetEnable( uint8 enable );
#endif

static void zclSensorMonitor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);

// app display functions
//static void zclSensorMonitor_LcdDisplayUpdate( void );
#ifdef LCD_SUPPORTED
static void zclSensorMonitor_LcdDisplayMainMode( void );
static void zclSensorMonitor_LcdDisplayHelpMode( void );
#endif

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSensorMonitor_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSensorMonitor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSensorMonitor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSensorMonitor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSensorMonitor_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSensorMonitor_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSensorMonitor_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif

static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);

/*********************************************************************
* STATUS STRINGS
*/
#ifdef LCD_SUPPORTED
const char sDeviceName[]   = "  Generic App";
const char sClearLine[]    = " ";
const char sSwSensorMonitor[]      = "SW1:GENAPP_TODO";  // SENSORMONITOR_TODO
const char sSwBDBMode[]     = "SW2: Start BDB";
char sSwHelp[]             = "SW4: Help       ";  // last character is * if NWK open
#endif

/*********************************************************************
* ZCL General Profile Callback table
*/
static zclGeneral_AppCallbacks_t zclSensorMonitor_CmdCallbacks =
{
  zclSensorMonitor_BasicResetCB,             // Basic Cluster Reset command
  NULL,                                   // Identify Trigger Effect command
  NULL,//zclSensorMonitor_OnOffCB,                                   // On/Off cluster commands
  NULL,                                   // On/Off cluster enhanced command Off with Effect
  NULL,                                   // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                   // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                   // Level Control Move to Level command
  NULL,                                   // Level Control Move command
  NULL,                                   // Level Control Step command
  NULL,                                   // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                   // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                  // Scene Store Request command
  NULL,                                  // Scene Recall Request command
  NULL,                                  // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                  // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                  // Get Event Log command
  NULL,                                  // Publish Event Log command
#endif
  NULL,                                  // RSSI Location command
  NULL                                   // RSSI Location Response command
};

/*********************************************************************
* SENSORMONITOR_TODO: Add other callback structures for any additional application specific 
*       Clusters being used, see available callback structures below.
*
*       bdbTL_AppCallbacks_t 
*       zclApplianceControl_AppCallbacks_t 
*       zclApplianceEventsAlerts_AppCallbacks_t 
*       zclApplianceStatistics_AppCallbacks_t 
*       zclElectricalMeasurement_AppCallbacks_t 
*       zclGeneral_AppCallbacks_t 
*       zclGp_AppCallbacks_t 
*       zclHVAC_AppCallbacks_t 
*       zclLighting_AppCallbacks_t 
*       zclMS_AppCallbacks_t 
*       zclPollControl_AppCallbacks_t 
*       zclPowerProfile_AppCallbacks_t 
*       zclSS_AppCallbacks_t  
*
*/

/*********************************************************************
* @fn          zclSensorMonitor_Init
*
* @brief       Initialization function for the zclGeneral layer.
*
* @param       none
*
* @return      none
*/
void zclSensorMonitor_Init( byte task_id )
{
//  requestNewTrustCenterLinkKey = FALSE;
  
  zclSensorMonitor_TaskID = task_id;
  
  // This app is part of the Home Automation Profile
  bdb_RegisterSimpleDescriptor( &zclSensorMonitor_SimpleDesc );
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SENSORMONITOR_ENDPOINT, &zclSensorMonitor_CmdCallbacks );
  
  // SENSORMONITOR_TODO: Register other cluster command callbacks here
  
  // Register the application's attribute list
  zcl_registerAttrList( SENSORMONITOR_ENDPOINT, zclSensorMonitor_NumAttributes, zclSensorMonitor_Attrs );
  
  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSensorMonitor_TaskID );
  
#ifdef ZCL_DISCOVER
  // Register the application's command list
  zcl_registerCmdList( SENSORMONITOR_ENDPOINT, zclCmdsArraySize, zclSensorMonitor_Cmds );
#endif
  
  // Register low voltage NV memory protection application callback
  RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSensorMonitor_TaskID );
  
  bdb_RegisterCommissioningStatusCB( zclSensorMonitor_ProcessCommissioningStatus );
  bdb_RegisterIdentifyTimeChangeCB( zclSensorMonitor_ProcessIdentifyTimeChange );
  bdb_RegisterBindNotificationCB( zclSensorMonitor_BindNotification );
  
#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
  bdb_RegisterTouchlinkTargetEnableCB( zclSensorMonitor_ProcessTouchlinkTargetEnable );
#endif
  
#ifdef ZCL_DIAGNOSTIC
  // Register the application's callback function to read/write attribute data.
  // This is only required when the attribute data format is unknown to ZCL.
  zcl_registerReadWriteCB( SENSORMONITOR_ENDPOINT, zclDiagnostic_ReadWriteAttrCB, NULL );
  
  if ( zclDiagnostic_InitStats() == ZSuccess )
  {
    // Here the user could start the timer to save Diagnostics to NV
  }
#endif
  
  
#ifdef LCD_SUPPORTED
  HalLcdWriteString ( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif  // LCD_SUPPORTED
  
//  NLME_SetPollRate(0);
//  NLME_SetQueuedPollRate(0);
//  NLME_SetResponseRate(0);
//   
//  P0SEL &= ~(0xFF);
//  P1SEL &= ~(0xFF);
//  P2SEL &= ~(b00000111);
//  P0INP |= 0xFF;//BV(3)|BV(2);
//  P1INP |= 0xFF;//BV(7)|BV(6)|BV(5)|BV(4)|BV(3)|BV(2)|BV(1);
//  P2INP |= BV(4)|BV(3)|BV(2)|BV(1)|BV(0);
  
//  P0SEL |= b00001100;                    // 0=GPIO 1=Peripheral (ADC, UART)

//  P1DIR |= BV(0)|BV(1);
//  P1 |= BV(1);
  
//  P0SEL &= ~(BV(1));
//  P0DIR &= ~(BV(1));
  
//  UART_Init();

//  HalI2CInit();
////  UART_String("start");
//  BH1750_Init(ONE_TIME_HIGH_RES_MODE);
//  ssd1306_Init();
//  ssd1306_DisplaySensorFrame();
//  osal_start_timerEx( zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT, 10 );
  bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING |
                         BDB_COMMISSIONING_MODE_FINDING_BINDING);
}

/*********************************************************************
* @fn          zclSample_event_loop
*
* @brief       Event Loop Processor for zclGeneral.
*
* @param       none
*
* @return      none
*/
uint16 zclSensorMonitor_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSensorMonitor_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
      case ZCL_INCOMING_MSG:
        // Incoming ZCL Foundation command/response messages
        zclSensorMonitor_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
//        osal_mem_free(((zclIncomingMsg_t *)MSGpkt)->attrCmd);
        break;
        
      case KEY_CHANGE:
        zclSensorMonitor_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
        
      case ZDO_STATE_CHANGE:
        zclSensorMonitor_NwkState = (devStates_t)(MSGpkt->hdr.status);
        
        // now on the network
        if ( (zclSensorMonitor_NwkState == DEV_ZB_COORD) ||
            (zclSensorMonitor_NwkState == DEV_ROUTER)   ||
              (zclSensorMonitor_NwkState == DEV_END_DEVICE) )
        {
          giGenAppScreenMode = GENERIC_MAINMODE;
          //            zclSensorMonitor_LcdDisplayUpdate();
        }
        break;
        
      default:
        break;
      }
      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SENSORMONITOR_MAIN_SCREEN_EVT )
  {
    giGenAppScreenMode = GENERIC_MAINMODE;
    
    
    return ( events ^ SENSORMONITOR_MAIN_SCREEN_EVT );
  }
  
#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SENSORMONITOR_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SENSORMONITOR_END_DEVICE_REJOIN_EVT );
  }
#endif
  
  /* SENSORMONITOR_TODO: handle app events here */
  
  //  if ( events & SENSORMONITOR_EVT_LONG )
  //  {
  //    if ( bdbAttributes.bdbNodeIsOnANetwork )
  //    {
  //      zclSensorMonitor_LeaveNetwork();
  //    }
  //    else 
  //    {
  //      bdb_StartCommissioning(
  //        BDB_COMMISSIONING_MODE_NWK_FORMATION | 
  //        BDB_COMMISSIONING_MODE_NWK_STEERING | 
  //        BDB_COMMISSIONING_MODE_FINDING_BINDING | 
  //        BDB_COMMISSIONING_MODE_INITIATOR_TL
  //      );
  //    }
  //    
  //    return ( events ^ SENSORMONITOR_EVT_LONG );
  //  }
  
  if ( events & SENSORMONITOR_REPORTING_EVT )
  {
    // toggle LED 2 state, start another timer for 500ms
    //HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );
    //osal_start_timerEx( zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT, 3000 );
//    P0_6 = ~P0_6;
    //    DHT22_Measure();
    //    zclSensorMonitor_ReportTemp();
    P1DIR |= BV(0);
    P1 |= BV(0);
//    ssd1306_SetDisplayOn(1);
    zclSensorMonitor_ReportTemp();
    
//    BH1750_SetMode(ONE_TIME_HIGH_RES_MODE);
//    waitMeasurementReady(true);
//    BH1750_ReadLight(&BH1750_lux);
//    sprintf(BH1750Str, "%d lux", (uint16)(BH1750_lux));
//    UART_String(BH1750Str);  
//    ssd1306_SetDisplayOn(0);
    P1 &= ~(BV(0));
    P1INP |= 0xFF;
//    osal_pwrmgr_device( PWRMGR_BATTERY );
//    OSAL_SET_CPU_INTO_SLEEP( 999999999 );
    osal_start_timerEx( zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT, 10000 );
//    SystemResetSoft();              
    return ( events ^ SENSORMONITOR_REPORTING_EVT );
  }
  
/*  
  if ( events & SENSORMONITOR_EVT_3 )
  {
  
  return ( events ^ SENSORMONITOR_EVT_3 );
}
  */
  
  // Discard unknown events
  return 0;
}


/*********************************************************************
* @fn      zclSensorMonitor_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_5
*                 HAL_KEY_SW_4
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
static void zclSensorMonitor_HandleKeys( byte shift, byte keys )
{
//  if ( keys & HAL_KEY_SW_1 )
//  {
//    static bool LED_OnOff = FALSE;
//    
//    giGenAppScreenMode = GENERIC_MAINMODE;
//    
//    /* SENSORMONITOR_TODO: add app functionality to hardware keys here */
//    
//    // for example, start/stop LED 2 toggling with 500ms period
//    if (LED_OnOff)
//    { 
//      // if the LED is blinking, stop the osal timer and turn the LED off
//      osal_stop_timerEx(zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT);
//      HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
//      LED_OnOff = FALSE;
//    }
//    else
//    {
//      // turn on LED 2 and start an osal timer to toggle it after 500ms, search
//      // for SENSORMONITOR_REPORTING_EVT to see event handling after expired timer
//      osal_start_timerEx( zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT, 500 );
//      HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
//      LED_OnOff = TRUE;
//    }
//  }
//  // Start the BDB commissioning method
//  if ( keys & HAL_KEY_SW_2 )
//  {
//    giGenAppScreenMode = GENERIC_MAINMODE;
//
//    bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING | BDB_COMMISSIONING_MODE_INITIATOR_TL);
//  }
//  if ( keys & HAL_KEY_SW_3 )
//  {
//    giGenAppScreenMode = GENERIC_MAINMODE;
//  
//    // touchlink target commissioning, if enabled  
//#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
//    bdb_StartCommissioning(BDB_COMMISSIONING_MODE_FINDING_BINDING);
//    touchLinkTarget_EnableCommissioning( 30000 );
//#endif
//    
//  }
//  if ( keys & HAL_KEY_SW_4 )
//  {
//    
//   giGenAppScreenMode = giGenAppScreenMode ? GENERIC_MAINMODE : GENERIC_HELPMODE;
//#ifdef LCD_SUPPORTED
//    HalLcdWriteString( (char *)sClearLine, HAL_LCD_LINE_2 );
//#endif
//    
//  }
//  if ( keys & HAL_KEY_SW_5 )
//  {
//    bdb_resetLocalAction();
//  }

//  zclSensorMonitor_LcdDisplayUpdate();
}


/*********************************************************************
* @fn      zclSensorMonitor_ProcessCommissioningStatus
*
* @brief   Callback in which the status of the commissioning process are reported
*
* @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
*
* @return  none
*/
static void zclSensorMonitor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
  case BDB_COMMISSIONING_FORMATION:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
      bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
    }
    else
    {
      //Want to try other channels?
      //try with bdb_setChannelAttribute
    }
    break;
  case BDB_COMMISSIONING_NWK_STEERING:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //YOUR JOB:
      //We are on the nwk, what now?
      osal_start_timerEx( zclSensorMonitor_TaskID, SENSORMONITOR_REPORTING_EVT, 10 );
    }
    else
    {
      //See the possible errors for nwk steering procedure
      //No suitable networks found
      //Want to try other channels?
      //try with bdb_setChannelAttribute
    }
    break;
  case BDB_COMMISSIONING_FINDING_BINDING:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
    {
      //YOUR JOB:
    }
    else
    {
      //YOUR JOB:
      //retry?, wait for user interaction?
    }
    break;
  case BDB_COMMISSIONING_INITIALIZATION:
    //Initialization notification can only be successful. Failure on initialization
    //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
    
    //YOUR JOB:
    //We are on a network, what now?
    
    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
  case BDB_COMMISSIONING_PARENT_LOST:
    if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
    {
      //We did recover from losing parent
    }
    else
    {
      //Parent not found, attempt to rejoin again after a fixed delay
      osal_start_timerEx(zclSensorMonitor_TaskID, SENSORMONITOR_END_DEVICE_REJOIN_EVT, SENSORMONITOR_END_DEVICE_REJOIN_DELAY);
    }
    break;
#endif 
  }
}

/*********************************************************************
* @fn      zclSensorMonitor_ProcessIdentifyTimeChange
*
* @brief   Called to process any change to the IdentifyTime attribute.
*
* @param   endpoint - in which the identify has change
*
* @return  none
*/
static void zclSensorMonitor_ProcessIdentifyTimeChange( uint8 endpoint )
{
  (void) endpoint;
  
  if ( zclSensorMonitor_IdentifyTime > 0 )
  {
    HalLedBlink ( HAL_LED_2, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_OFF );
  }
}

/*********************************************************************
* @fn      zclSensorMonitor_BindNotification
*
* @brief   Called when a new bind is added.
*
* @param   data - pointer to new bind data
*
* @return  none
*/
static void zclSensorMonitor_BindNotification( bdbBindNotificationData_t *data )
{
  // SENSORMONITOR_TODO: process the new bind information
}


/*********************************************************************
* @fn      zclSensorMonitor_ProcessTouchlinkTargetEnable
*
* @brief   Called to process when the touchlink target functionality
*          is enabled or disabled
*
* @param   none
*
* @return  none
*/
#if ( defined ( BDB_TL_TARGET ) && (BDB_TOUCHLINK_CAPABILITY_ENABLED == TRUE) )
static void zclSensorMonitor_ProcessTouchlinkTargetEnable( uint8 enable )
{
  if ( enable )
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
  }
  else
  {
    HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
  }
}
#endif

/*********************************************************************
* @fn      zclSensorMonitor_BasicResetCB
*
* @brief   Callback from the ZCL General Cluster Library
*          to set all the Basic Cluster attributes to default values.
*
* @param   none
*
* @return  none
*/
static void zclSensorMonitor_BasicResetCB( void )
{
  
  /* SENSORMONITOR_TODO: remember to update this function with any
  application-specific cluster attribute variables */
  
  zclSensorMonitor_ResetAttributesToDefaultValues();
  
}
/*********************************************************************
* @fn      zclSampleApp_BatteryWarningCB
*
* @brief   Called to handle battery-low situation.
*
* @param   voltLevel - level of severity
*
* @return  none
*/
void zclSampleApp_BatteryWarningCB( uint8 voltLevel )
{
  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
  {
    // Send warning message to the gateway and blink LED
  }
  else if ( voltLevel == VOLT_LEVEL_BAD )
  {
    // Shut down the system
  }
}

/******************************************************************************
*
*  Functions for processing ZCL Foundation incoming Command/Response messages
*
*****************************************************************************/

/*********************************************************************
* @fn      zclSensorMonitor_ProcessIncomingMsg
*
* @brief   Process ZCL Foundation incoming message
*
* @param   pInMsg - pointer to the received message
*
* @return  none
*/
static void zclSensorMonitor_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg )
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
  case ZCL_CMD_READ_RSP:
    zclSensorMonitor_ProcessInReadRspCmd( pInMsg );
    break;
#endif
#ifdef ZCL_WRITE
  case ZCL_CMD_WRITE_RSP:
    zclSensorMonitor_ProcessInWriteRspCmd( pInMsg );
    break;
#endif
  case ZCL_CMD_CONFIG_REPORT:
  case ZCL_CMD_CONFIG_REPORT_RSP:
  case ZCL_CMD_READ_REPORT_CFG:
  case ZCL_CMD_READ_REPORT_CFG_RSP:
  case ZCL_CMD_REPORT:
    //bdb_ProcessIncomingReportingMsg( pInMsg );
    break;
    
  case ZCL_CMD_DEFAULT_RSP:
    zclSensorMonitor_ProcessInDefaultRspCmd( pInMsg );
    break;
#ifdef ZCL_DISCOVER
  case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
    zclSensorMonitor_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
    zclSensorMonitor_ProcessInDiscCmdsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_RSP:
    zclSensorMonitor_ProcessInDiscAttrsRspCmd( pInMsg );
    break;
    
  case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
    zclSensorMonitor_ProcessInDiscAttrsExtRspCmd( pInMsg );
    break;
#endif
  default:
    break;
  }
  
  if ( pInMsg->attrCmd )
    osal_mem_free( pInMsg->attrCmd );
}

#ifdef ZCL_READ
/*********************************************************************
* @fn      zclSensorMonitor_ProcessInReadRspCmd
*
* @brief   Process the "Profile" Read Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;
  
  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for (i = 0; i < readRspCmd->numAttr; i++)
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }
  
  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
* @fn      zclSensorMonitor_ProcessInWriteRspCmd
*
* @brief   Process the "Profile" Write Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;
  
  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }
  
  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
* @fn      zclSensorMonitor_ProcessInDefaultRspCmd
*
* @brief   Process the "Profile" Default Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;
  
  // Device is notified of the Default Response command.
  (void)pInMsg;
  
  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
* @fn      zclSensorMonitor_ProcessInDiscCmdsRspCmd
*
* @brief   Process the Discover Commands Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return ( TRUE );
}

/*********************************************************************
* @fn      zclSensorMonitor_ProcessInDiscAttrsRspCmd
*
* @brief   Process the "Profile" Discover Attributes Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return ( TRUE );
}

/*********************************************************************
* @fn      zclSensorMonitor_ProcessInDiscAttrsExtRspCmd
*
* @brief   Process the "Profile" Discover Attributes Extended Response Command
*
* @param   pInMsg - incoming message to process
*
* @return  none
*/
static uint8 zclSensorMonitor_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;
  
  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }
  
  return ( TRUE );
}
#endif // ZCL_DISCOVER

//static void zclSensorMonitor_OnOffCB(uint8 cmd)
//{
//  // ???????? ?????, ?????? ?????? ???????
//  // ????? ????????? ??????? ?????
//  afIncomingMSGPacket_t *pPtr = zcl_getRawAFMsg();
//  zclSensorMonitor_DstAddr.addr.shortAddr = pPtr->srcAddr.addr.shortAddr;
//  //  HalLedSet ( HAL_LED_2, HAL_LED_MODE_TOGGLE );
//  // ????????
//  if (cmd == COMMAND_ON) {
//    P0_6 = 1;
//  }
//  // ?????????
//  else if (cmd == COMMAND_OFF) {
//    P0_6 = 0;
//  }
//  // ???????????
//  //  else if (cmd == COMMAND_TOGGLE) {
//  //    updateRelay(RELAY_STATE == 0);
//  //  }
//}

void zclSensorMonitor_ReportTemp( void )
{
  error = 0;
  error |= SHT2x_SoftReset();
  //error |= SHT2x_GetSerialNumber(SerialNumber_SHT2x);
  error |= SHT2x_ReadUserRegister(&userRegister);
  userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_12_14BIT;
  error |= SHT2x_WriteUserRegister(&userRegister);
  error |= SHT2x_MeasurePoll(HUMIDITY, &sRH);
  error |= SHT2x_MeasurePoll(TEMP, &sT);
  temperatureC = SHT2x_CalcTemperatureC(sT.u16);
  humidityRH = SHT2x_CalcRH(sRH.u16);
  error |= SHT2x_ReadUserRegister(&userRegister);
//  sprintf(humitityOutStr, "%d %%",(int)(humidityRH*100.0));
//  UART_String(humitityOutStr);
//  sprintf(temperatureOutStr, "%d C",(int)(temperatureC*100.0));
//  UART_String(temperatureOutStr);
  ssd1306_DisplaySensor((int)(temperatureC*100.0), (int)(humidityRH*100.0));
  if(error != 0)
  { 
//    UART_String("Error occurred");
//    UART_String("Humidity RH: --.-- %%");
//    UART_String("Temperature: --.--€C");
    TemperatureValue = 0;
    HumidityValue = 0;
  }else{
    TemperatureValue = (int)(temperatureC*100.0);
    HumidityValue = (int)(humidityRH*100.0);
  }
    
  const uint8 NUM_ATTRIBUTES = 1;
  
  zclReportCmd_t *pReportCmd;
  
  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) +
                              (NUM_ATTRIBUTES * sizeof(zclReport_t)));
  if (pReportCmd != NULL) {
    pReportCmd->numAttr = NUM_ATTRIBUTES;
    
    pReportCmd->attrList[0].attrID = ATTRID_MS_TEMPERATURE_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
    pReportCmd->attrList[0].attrData = (void *)(&TemperatureValue);
     
    zclSensorMonitor_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSensorMonitor_DstAddr.addr.shortAddr = 0;
    zclSensorMonitor_DstAddr.endPoint = 1;
    
    zcl_SendReportCmd(SENSORMONITOR_ENDPOINT, &zclSensorMonitor_DstAddr,
                      ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, pReportCmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, false, SeqNum++);
  }
  
  osal_mem_free(pReportCmd);
  
  pReportCmd = osal_mem_alloc(sizeof(zclReportCmd_t) +
                              (NUM_ATTRIBUTES * sizeof(zclReport_t)));
  if (pReportCmd != NULL) {
    pReportCmd->numAttr = NUM_ATTRIBUTES;
    
    pReportCmd->attrList[0].attrID = ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE;
    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
    pReportCmd->attrList[0].attrData = (void *)(&HumidityValue);
     
    zclSensorMonitor_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    zclSensorMonitor_DstAddr.addr.shortAddr = 0;
    zclSensorMonitor_DstAddr.endPoint = 1;
    
    zcl_SendReportCmd(SENSORMONITOR_ENDPOINT, &zclSensorMonitor_DstAddr,
                      ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY, pReportCmd,
                      ZCL_FRAME_CLIENT_SERVER_DIR, false, SeqNum++);
  }
  osal_mem_free(pReportCmd);
}
/****************************************************************************
****************************************************************************/


