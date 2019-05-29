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
#include "SinglyLinkedList.h"

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
#define COIN_PIN P0_2            //定义P0.2口为中断方式接收投币器信号引脚
#define SIMULATE_PIN P0_5        //定义P0.5口为替换投币器模拟信号发生引脚
#define UART0        0x00

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

LinkList LL; // 定义线性链表存储终端地址以及心跳信息

/*********************************************************************
* LOCAL FUNCTIONS
*/
void AfSendAddrInfo(uint8 cmd);
void AfReplyGetCoinCmd(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( int8 whichKey );
void SampleApp_SendHeartBeatMessageCoor(void);
void SampleApp_SendHeartBeatMessageEnd(void);

/**
  P1端口中断处理函数
*/
HAL_ISR_FUNCTION( coinPort1Isr, P1INT_VECTOR ) // P1_2配置为投币器电平变化中断引脚，在HAL/Common/hal_drivers.c中进行的引脚初始化
{
  // ???????????????????????????????终端操作
  myprintf("P0 interrupt\n");
  P1IFG = 0;       //清中断标志
  P1IF = 0;        //清中断标志
}

/**
* 自定义任务初始化函数
*/
void SampleApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  P0SEL &= ~0x20;         //设置P0.5口为普通IO
  P0DIR |= 0x20;          //设置P0.5口为输出
  SIMULATE_PIN = 0;       // 投币器信号模拟输出引脚
  
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
  
  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
  // 初始化串口
  MT_UartInit();                    //串口初始化
  MT_UartRegisterTaskID(task_id);   //注册串口任务
  myprintf("UartInit OK\n");
  
  int ret = InitList_L(&LL); // 单链表初始化
  if(ret == OVERFLOW) {
    myprintf("InitList_L OVERFLOW\n");
  }
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
    uint8 coodStarted[3] = {'#', 0x45, '@'};
    while ( MSGpkt )
    {      
      switch ( MSGpkt->hdr.event )
      {
        // 系统任务事件里面的接收数据确认消息
        case AF_DATA_CONFIRM_CMD:
          // myprintf("AF_DATA_CONFIRM_CMD\n");
          
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
            // 向上位机发送已经启动消息
            mySendByteBuf(coodStarted, 3);
          } else if(SampleApp_NwkState == DEV_ROUTER) {
            myprintf("ROUTER device started\n");
          } else if(SampleApp_NwkState == DEV_END_DEVICE) {
            myprintf("END device started\n");            
            AfSendAddrInfo(0x71); // 向协调器发送入网消息
            HalLedBlink(HAL_LED_1, 1, 50, 500);
          }
          osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // 开启定时器
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
#ifdef ZDO_COORDINATOR
    SampleApp_SendHeartBeatMessageCoor(); // 协调器心跳功能函数
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT); // 1000ms再次开启定时器
#else
    SampleApp_SendHeartBeatMessageEnd(); // 终端心跳功能函数
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*2); // 2000ms再次开启定时器
#endif
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
#ifdef ZDO_COORDINATOR       //按协调器S1才发数据  
  if ( keys & HAL_KEY_SW_6 ){ // 按键S1,以广播方式发数据    
    SampleApp_SendPeriodicMessage(1);
  } else if(keys & HAL_KEY_SW_1) { // 按键S2
    SampleApp_SendPeriodicMessage(2);
  }
#endif

#ifndef ZDO_COORDINATOR
  if ( keys & HAL_KEY_SW_6 ){
    AfSendAddrInfo(0x51); // 只用在路由和终端设备上，向协调器上传自身地址，协调器收到此消息后打印出地址
  }else if ( keys & HAL_KEY_SW_1 ){ // 按键S2
    myprintf("S2\n");    
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
#define INIT_LEFT_SEC 5
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8 data;
  uint8 cmd;
  int ret = 0;
  
  switch ( pkt->clusterId )
  {    
  case SAMPLEAPP_PERIODIC_CLUSTERID: // 协调器在发送数据的时候指定的此消息对应的命令字，即表示哪种类型的控制命令
//协调器接收到终端的无线数据 
#ifdef ZDO_COORDINATOR
    data = pkt->cmd.Data[0];
    cmd = pkt->cmd.Data[1];
    if(data == '#') {
      if(cmd == 0x71 && pkt->cmd.Data[12] == '@') { // 接收到终端上传的短地址和IEEE地址
        ret = checkEPInList(LL, pkt->cmd.Data + 4); // 使用IEEE地址检查终端是否已经在链表中
        if(ret == -1) { // 如果当前终端没有在列表中        
          ListInsert_L(LL, ListLength_L(LL)+1, pkt->cmd.Data[2], pkt->cmd.Data[3], &(pkt->cmd.Data[4]), INIT_LEFT_SEC); // 添加一个线性列表元素到链表尾部 
          myprintf("Insert new EP in list\n");
        } else { // 更新终端所在节点的在线时长，标记为在线状态
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);
          // myprintf("Already In List, update()\n");
        }
        // myprintf("length = %d\n", ListLength_L(LL));
      } else if(cmd == 0x72 && pkt->cmd.Data[10] == '@') { // 收到终端回复的确认收到投币命令
        mySendByteBuf(pkt->cmd.Data, 11); // 转发回复消息到上位机
      } else if(cmd == 0x51 && pkt->cmd.Data[12] == '@') { // 终端按键S1上传的长短地址信息
        mySendByteBuf(pkt->cmd.Data, 13);
        myprintf("\nRECV EP Key S1 addr info\n");
      }
    }
//路由器和终端接收数据处理     
#else
    data = pkt->cmd.Data[0];
    cmd = pkt->cmd.Data[1];
    if(data == '#')
    {
      if(cmd == 0x81 && pkt->cmd.Data[3] == '@') { // 收到投递指定币数的消息        
        myprintf("begin add coin = %d\n", pkt->cmd.Data[2]); // 进行投币动作
        AfReplyGetCoinCmd(); // 回复已经收到投币命令0x72
      } else if(cmd == 0x31 && pkt->cmd.Data[3] == '@') { // LED2闪灯测试命令
        HalLedBlink(HAL_LED_2, 2, 25, 500);
      }
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
void SampleApp_SendPeriodicMessage( int8 key )
{
  if(key == 1) { // 按键S1
    uint8 sendKeyS1Buf[4] = {'#', 0x31, 0, '@'};
    
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                        SAMPLEAPP_PERIODIC_CLUSTERID,
                        4,
                        sendKeyS1Buf,
                        &SampleApp_TransID,
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      HalLedBlink(HAL_LED_2, 2, 50, 500);
    }
  } else if(key == 2) { // 按键S2
    myprintf("press key S2\n");
  }
}

/**
  协调器心跳功能函数，使用串口向树莓派发送心跳信息
*/
void SampleApp_SendHeartBeatMessageCoor(void) {
  // 遍历终端列表，将所有节点剩余时间-1，如果小于0，则置0
  uint8 heartBeatBuf[3] = {'#', 0x46, '@'};
  
  decAllListEPLeftSec(LL); // 将终端列表中元素在线时长-1，如果检测到超时则将IEEE发送到上位机
  mySendByteBuf(heartBeatBuf, 3);
  HalLedBlink(HAL_LED_1, 1, 50, 500);
}

/**
  终端心跳函数，用于定时向协调器发送本机地址信息
*/
void SampleApp_SendHeartBeatMessageEnd(void) { 
  AfSendAddrInfo(0x71);
  HalLedBlink(HAL_LED_1, 1, 50, 500);
}

/**
  发送终端地址信息到协调器
*/
void AfSendAddrInfo(uint8 cmd) {
  uint16 shortAddr;
  uint8 strBuf[13] = {0};  
  
  shortAddr = NLME_GetShortAddr();
  strBuf[0] = '#';  // 发送数据的标识，便于协调器解析
  strBuf[1] = cmd; // 消息类型标识
  strBuf[2] = HI_UINT16(shortAddr); // 存放地址的高8位
  strBuf[3] = LO_UINT16(shortAddr); // 存放低8位
  osal_memcpy(&strBuf[4], NLME_GetExtAddr(), 8); // 复制IEEE地址
  strBuf[12] = '@'; // 尾部字节
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // 发送的目的地址+端点地址+传送模式
                      (endPointDesc_t *)&SampleApp_epDesc, // 源终端的描述
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // 被profile指定的有效的集群号
                      13,                                  // 发送数据长度
                      strBuf,                              // 发送数据缓冲区
                      &SampleApp_TransID,                  // 消息发送ID
                      AF_DISCV_ROUTE,                      // 有效位掩码的发送选项
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // 传送跳数，通常设置为AF_DEFAULT_RADIUS
  {
    myprintf("EP AfSendAddrInfo() failed\n");
  }
}

/**
  终端发送已经收到投币开始游戏命令
*/
void AfReplyGetCoinCmd(void) {
  uint8 strBuf[11] = {0};  
  
  strBuf[0] = '#';  // 发送数据的标识，便于协调器解析
  strBuf[1] = 0x72; // 消息类型标识
  osal_memcpy(&strBuf[2], NLME_GetExtAddr(), 8); // 复制IEEE地址
  strBuf[10] = '@'; // 尾部字节
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // 发送的目的地址+端点地址+传送模式
                      (endPointDesc_t *)&SampleApp_epDesc, // 源终端的描述
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // 被profile指定的有效的集群号
                      11,                                  // 发送数据长度
                      strBuf,                              // 发送数据缓冲区
                      &SampleApp_TransID,                  // 消息发送ID
                      AF_DISCV_ROUTE,                      // 有效位掩码的发送选项
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // 传送跳数，通常设置为AF_DEFAULT_RADIUS
  {
    myprintf("EP AfReplyGetCoinCmd() failed\n");
  }
}
