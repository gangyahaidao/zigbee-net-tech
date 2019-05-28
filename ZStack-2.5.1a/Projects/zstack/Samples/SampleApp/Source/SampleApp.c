/**************************************************************************************************
Filename:       SampleApp.c
Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
Revision:       $Revision: 19453 $

Description:    Sample Application (no Profile).


Copyright 2007 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
This application isn't intended to do anything useful, it is
intended to be a simple example of an application's structure.

This application sends it's messages either as broadcast or
broadcast filtered group messages.  The other (more normal)
message addressing is unicast.  Most of the other sample
applications are written to support the unicast message model.

Key control:
SW1:  Sends a flash command to all devices in Group 1.
SW2:  Adds/Removes (toggles) this device in and out
of Group 1.  This will enable and disable the
reception of the flash command.
*********************************************************************/

/*********************************************************************
* INCLUDES
*/
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

/*********************************************************************
* MACROS
*/
#define DATA_PIN P0_5            //定义P0.5口为继电器的控制端

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SAMPLEAPP_APP_THRESH )
#define SAMPLE_APP_THRESH  64
#endif

#if !defined( SAMPLEAPP_APP_RX_SZ )
#define SAMPLE_APP_RX_SZ  128
#endif

#if !defined( SAMPLEAPP_APP_TX_SZ )
#define SAMPLE_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SAMPLE_APP_IDLE  6
#endif

// This is the max byte count per OTA message.
#if !defined( SAMPLE_APP_TX_MAX )
#define SAMPLE_APP_TX_MAX  10
#endif


#define UART0        0x00

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/
uint8 AppTitle[] = "ALD relay&LED"; //应用程序名称

uint8 LedState = 0x30;

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;
afAddrType_t GenericApp_DstAddr;
afAddrType_t EndPoint_DstAddr;

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
// This variable will be received when
// SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; // 用于协调器广播消息
afAddrType_t SampleApp_TxAddr; // 用于终端设备向协调器报告短地址信息

aps_Group_t SampleApp_Group;
uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
/*********************************************************************
* LOCAL FUNCTIONS
*/
void SampleApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void AfSendAddrInfo(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendHeartBeatMessage(void);
/*********************************************************************
* NETWORK LAYER CALLBACKS
*/

/*********************************************************************
* PUBLIC FUNCTIONS
*/

/*********************************************************************
* @fn      SampleApp_Init
*
* @brief   Initialization function for the Generic App Task.
*          This is called during initialization and should contain
*          any application specific initialization (ie. hardware
*          initialization/setup, table initialization, power up
*          notificaiton ... ).
*
* @param   task_id - the ID assigned by OSAL.  This ID should be
*                    used to send messages and set timers.
*
* @return  none
*/
void SampleApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  P0SEL &= ~0x20;               //设置P0.5口为普通IO
  P0DIR |= 0x20;                //设置P0.5口为输出
  DATA_PIN = 0;                 //继电器默认断开 继电器默认是高电平触发可自行切换
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
#if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES
  
#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif
  
  GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  GenericApp_DstAddr.endPoint = 0;
  GenericApp_DstAddr.addr.shortAddr = 0;
  
  // Setup for the periodic message's destination address
  // Broadcast to everyone
  // 协调器广播发送数据到终端
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF; // 广播地址
  
  // 终端单播发送数据到协调器
  SampleApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_TxAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_TxAddr.addr.shortAddr = 0x00; // 单播协调器地址
  
  // 协调器单播发送到指定地址的终端上  
  EndPoint_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  EndPoint_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  EndPoint_DstAddr.addr.shortAddr = 0x00; // 在发送的时候修改终端地址
    
  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT; // 端点号
  SampleApp_epDesc.task_id = &SampleApp_TaskID; // 端点对应的任务
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc; // 简单描述符
  SampleApp_epDesc.latencyReq = noLatencyReqs;  
  afRegister( &SampleApp_epDesc ); // 注册一个应用的端点描述符，注册一个新的端点到任务，这样当有新的消息到来时直接发送到指定的任务中
  
  // 绑定按键事件到任务ID
  RegisterForKeys( SampleApp_TaskID );
  // 将底层的一些事件消息引入到应用层的注册方法，自定义区解析一些消息
  ZDO_RegisterForZDOMsg( SampleApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( SampleApp_TaskID, Match_Desc_rsp );
  
  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
  // 初始化串口
  MT_UartInit();                    //串口初始化
  MT_UartRegisterTaskID(task_id);   //注册串口任务
  HalUARTWrite(0,"UartInit OK\n", sizeof("UartInit OK\n"));
}

/*********************************************************************
* @fn      SampleApp_ProcessEvent
*
* @brief   Generic Application Task event processor.  This function
*          is called to process all events for the task.  Events
*          include timers, messages and any other user defined events.
*          系统各种事件的处理函数
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events - events to process.  This is a bit map and can
*                   contain more than one event.
*
* @return  none
*/
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID ); // 接收一个用户自定义任务的消息
    while ( MSGpkt )
    {      
      switch ( MSGpkt->hdr.event )
      {
        // 系统任务事件里面的ZDO消息
        case ZDO_CB_MSG:
          myprintf("ZDO_CB_MSG\n");
          SampleApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );   
          break;
          
        // 系统任务事件里面的接收数据确认消息
        case AF_DATA_CONFIRM_CMD:
          myprintf("AF_DATA_CONFIRM_CMD\n");
          
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            myprintf("AF_DATA_CONFIRM_CMD failed\n");
          }
          break;
        
          // 接收到一个按键事件
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
          
          // 无线网络收到数据
        case AF_INCOMING_MSG_CMD:
          myprintf("AF_INCOMING_MSG_CMD\n");
          SampleApp_MessageMSGCB( MSGpkt );
          break;
          
          // 设备组网或者入网成功，可以发送一个消息，并开启一个心跳事件的定时器
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status); // 另外MSGpkt->srcAddr.addr.shortAddr;可用于获取终端节点的网络短地址
          myprintf("SampleApp_NwkState = %d\n", SampleApp_NwkState);
          
          if(SampleApp_NwkState == DEV_ZB_COORD) {
            myprintf("COORD device started\n");
          } else if(SampleApp_NwkState == DEV_ROUTER) {
            myprintf("ROUTER device started\n");
          } else if(SampleApp_NwkState == DEV_END_DEVICE) {
            myprintf("END device started\n");
          }
          if ((SampleApp_NwkState == DEV_ZB_COORD) || (SampleApp_NwkState == DEV_ROUTER) || (SampleApp_NwkState == DEV_END_DEVICE)) { // 不同的终端都开启定时任务，协调器通过串口上传心跳信息，终端通过AF无线发送包含当前节点短地址的心跳信息
            // osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          break;
          
        default:
          myprintf("unknown pkg-event = 0x%x\n", MSGpkt->hdr.event);
          break;
      }      
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );      
      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // 接收到一个定时器事件
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    SampleApp_SendHeartBeatMessage(); // 心跳功能函数
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*3); // 3000ms再次开启定时器
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT); // 清除此已经处理事件标志位
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
* Event Generation Functions
*/
/*********************************************************************
* @fn      SampleApp_HandleKeys
*
* @brief   按键事件处理函数
*
* @param   shift - true if in shift/alt.
* @param   keys - bit field for key events. Valid entries:
*                 HAL_KEY_SW_2
*                 HAL_KEY_SW_1
*
* @return  none
*/
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
#if defined(ZDO_COORDINATOR)       //按协调器S1才发数据  
  if ( keys & HAL_KEY_SW_6 ){ // 按键S1,以广播方式发数据    
    myprintf("S1\n");
    SampleApp_SendPeriodicMessage();
  } else if(keys & HAL_KEY_SW_1) { // 按键S2
    myprintf("S2\n");
  }
#endif

#ifndef ZDO_COORDINATOR
  if ( keys & HAL_KEY_SW_6 ){
    myprintf("S1\n");
  }else if ( keys & HAL_KEY_SW_1 ){ // 按键S2
    myprintf("S2\n");
    AfSendAddrInfo(); // 只用在路由和终端设备上，向协调器上传自身地址
  }
#endif
}

/*********************************************************************
* LOCAL FUNCTIONS
*/

/*********************************************************************
* @fn      SampleApp_MessageMSGCB
*
* @brief   Data message processor callback.  This function processes
*          any incoming data - probably from other devices.  So, based
*          on cluster ID, perform the intended action.
*
* @param   none
*
* @return  none
*/
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8 data;
  
  switch ( pkt->clusterId )
  {    
  case SAMPLEAPP_PERIODIC_CLUSTERID: // 协调器在发送数据的时候指定的此消息对应的命令字，即表示哪种类型的控制命令
#ifdef ZDO_COORDINATOR //协调器接收到终端的无线数据 
    // 使用串口打印终端上传的地址
    data = (uint8)pkt->cmd.Data[0];
    if(data == 0x38) { // // 接收到终端上传的短地址和IEEE地址
      int i = 0;
      myprintf("RX net addr = ");
      myprintf("0x%x", pkt->cmd.Data[1]);
      myprintf("%x", pkt->cmd.Data[2]);
      myprintf("; IEEE addr = ");
      for(i = 3; i < pkt->cmd.DataLength; i++) {
        myprintf("0x%x,", pkt->cmd.Data[i]);
      }
      myprintf("\n");
    }
#else //路由器和终端接收数据处理 
    data = (uint8)pkt->cmd.Data[0];
    //继电器默认高电平触发 1吸合  0断开
    if(data == 0x30) 
    {
      DATA_PIN = 0;        //继电器断开 继电器默认是高电平触发可自行切换
      HalLedSet(HAL_LED_1, LED_ON);  
    } else if(data == 0x31) 
    {
      DATA_PIN = 1;        //继电器吸合
      HalLedSet(HAL_LED_1, LED_OFF); 
    } else if(data == 'K') {
      HalLedBlink(HAL_LED_2, 0, 50, 500);
    }
#endif    
    break;
    
  default:
    break;
  }
}

/**
* 协调器向终端发送广播类型消息
*/
void SampleApp_SendPeriodicMessage( void )
{
  if(LedState == 0x30) {
    LedState = 0x31;
  } else {
    LedState = 0x30;
  }  
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                      SAMPLEAPP_PERIODIC_CLUSTERID,
                      1,
                      &LedState,
                      &SampleApp_TransID,
                      AF_DISCV_ROUTE,
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    if(LedState == 0x31) {
      HalLedSet(HAL_LED_1, LED_ON);  
    } else {
      HalLedSet(HAL_LED_1, LED_OFF); 
    }
  }
  else
  {
    // Error occurred in request to send.
  }
}

/**
  心跳功能函数
*/
void SampleApp_SendHeartBeatMessage(void) {
#ifdef ZDO_COORDINATOR // 如果是协调器，使用串口向树莓派发送心跳信息
  myprintf("#HB@\n");
  HalLedBlink(HAL_LED_1, 1, 50, 500);
#endif    
#ifndef ZDO_COORDINATOR // 如果是终端，使用AF向协调器发送包含短地址的心跳消息
  AfSendAddrInfo();
  HalLedBlink(HAL_LED_1, 1, 50, 500);
#endif
}

/**
  发送终端地址信息到协调器
*/
void AfSendAddrInfo(void) {
  uint16 shortAddr;
  uint8 strBuf[11] = {0};  
  
  shortAddr = NLME_GetShortAddr();
  strBuf[0] = 0x38;  // 发送数据的标识，便于协调器解析
  strBuf[1] = HI_UINT16(shortAddr); // 存放地址的高8位
  strBuf[2] = LO_UINT16(shortAddr); // 存放低8位
  osal_memcpy(&strBuf[3], NLME_GetExtAddr(), 8); // 获取IEEE地址    
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // 发送的目的地址+端点地址+传送模式
                      (endPointDesc_t *)&SampleApp_epDesc, // 源终端的描述
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // 被profile指定的有效的集群号
                      11,                                  // 发送数据长度
                      strBuf,                              // 发送数据缓冲区
                      &SampleApp_TransID,                  // 消息发送ID
                      AF_DISCV_ROUTE,                      // 有效位掩码的发送选项
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // 传送跳数，通常设置为AF_DEFAULT_RADIUS
  {
    myprintf("EP AfSendAddrInfo() failed\n");
  }
}

/*********************************************************************
 * @fn      SampleApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  myprintf("inMsg->clusterID = 0x%x\n", inMsg->clusterID);
  ZDO_ActiveEndpointRsp_t *pRsp = NULL;
  
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        myprintf("End_Device_Bind_rsp ok\n");
      } else {
        myprintf("End_Device_Bind_rsp failed\n");
      }
      break;
    case Match_Desc_rsp:
      myprintf("Match_Desc_rsp\n");
      pRsp = ZDO_ParseEPListRsp( inMsg );
      if ( pRsp )
      {
        if ( pRsp->status == ZSuccess && pRsp->cnt )
        {
          GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
          GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
          // Take the first endpoint, Can be changed to search through endpoints
          GenericApp_DstAddr.endPoint = pRsp->epList[0];
          
        }
        osal_mem_free( pRsp );
      }
      break;
    default:
      break;
  }
}