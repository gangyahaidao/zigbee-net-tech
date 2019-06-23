/*********************************************************************
需要修改的文件包含功能如下：
ZMain/ZMain.c是主函数
HAL/hal_drivers.c进行外部中断的初始化
MT/MT_UART.c中进行串口消息的接收
SampleApp.c中定义中断响应函数，以及事件消息处理函数等
SingleLinkedList.c中定义了单链表的实现
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

#include "OSAL_Clock.h"

#include "common.h"
/*******
  一些测试用的宏开关
*/
// #define NEED_TEST_DIStANCE

/*********************************************************************
* MACROS
*/
#define COIN_RECV_PIN      P0_4      //中断输入下降沿触发，定义P0.7口为中断方式接收投币器信号引脚40ms
#define REWARD_OUTPUT_PIN  P0_6      //中断输入下降沿触发，彩票奖励输出个数信号引脚，输出一个彩票奖励则产生一个40ms的低电平

#define COIN_SIMULATE_PIN  P1_2      //输出，定义P1.2口为替换投币器模拟信号发生引脚

#define UART0        0x00
#define COIN_SIGNAL_HALF_TIME 50 // Ms
#define REWARD_SIGNAL_HALF_TIME 50

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
void EndDeviceSendAfMessageByCmd(uint8 cmd);
void EndDeviceSendHeartBeat(void);
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( int8 whichKey );
void SampleApp_SendHeartBeatMessageCoor(void);
void printAddrInfoHex(uint8* buf, uint8 dataLen);
void convertHexToStrLCD(uint16 addr, uint8 lcdLineNum);

int8 NeedCoinDelivered = 0; // 线上开启游戏需要处理的投币个数
int8 NeedCoinTransport = 0; // 需要转发的投币器信号数量
uint8 rewardCount = 0;  // 一次游戏出彩票个数统计
uint8 startRewardInterrupt = 0;
UTCTime lastRecvRewardSecTime = 0;//=osal_getClock();

/**
  终端设备P1端口中断处理函数，用来接收投币器的信号
*/
#ifdef END_DEVICE
uint8 pre_coin_pin_value = 1;
uint8 coin_pin_value = 1;
uint8 pre_reward_pin_value = 1;
uint8 reward_pin_value = 1;
HAL_ISR_FUNCTION( port0Isr, P0INT_VECTOR ) // P0_5配置为投币器电平变化中断引脚，在HAL/Common/hal_drivers.c中进行的引脚初始化
{ 
  IEN1 &= ~(0x1 << 5); // 关闭端口0中断
  if((P0IFG >> 4) & 0x1 == 1) { // 中断p0_4引脚产生中断，投币信号    
    coin_pin_value = COIN_RECV_PIN;
    if(coin_pin_value == 0) {
      if(coin_pin_value != pre_coin_pin_value) {
        pre_coin_pin_value = coin_pin_value;
        NeedCoinTransport = 1;
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TRANSPORT_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // 产生一个定时器事件    
      }
    } else {
      pre_coin_pin_value = 1;
    }
    P0IFG &= ~(0x1 << 4);       //端口0中断状态标志    
    //myprintf("get p0_4 coin input interrupt pin = %d\n", COIN_RECV_PIN);
  } 
  if((P0IFG >> 6) & 0x1 == 1) { // P0_6引脚，彩票机出一个彩票中断信号，转发到游戏机并上传服务器
    startRewardInterrupt = 1;
    lastRecvRewardSecTime = osal_getClock(); // 设置收到信号当前的秒数，在定时器中进行检测
    reward_pin_value = REWARD_OUTPUT_PIN;
    if(reward_pin_value == 0) {
      if(reward_pin_value != pre_reward_pin_value) {
        pre_reward_pin_value = reward_pin_value;
        rewardCount += 1;
      }      
    } else {
      pre_reward_pin_value = 1;
    }
    P0IFG &= ~(0x1 << 6);
    myprintf("\nP0_6=%d", REWARD_OUTPUT_PIN); // 此条串口打印语句不能去掉，否则计数会产生不准确结果
  }
  P0IF = 0;        //端口0中断标志，0表示无中断未决，1表示有中断未决，需要每次触发之后置0，不然会一直触发中断 
  IEN1 |= (0x1 << 5); // 端口0中断使能
}
#endif

/**
* 自定义任务初始化函数
*/
void SampleApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
#ifdef END_DEVICE
  //P1_2输出，投币器模拟信号产生引脚
  P1SEL &= ~(0x01 << 2);         //设置P1.2口为普通IO
  P1DIR |= (0x01 << 2);          //设置P1.2口为输出
  COIN_SIMULATE_PIN = 1;       // 投币器信号模拟输出引脚  
#endif
  
#if defined ( BUILD_ALL_DEVICES )
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES
  
#if defined ( HOLD_AUTO_START )
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
  EndPoint_DstAddr.addr.shortAddr = 0x00; // 在发送的时候动态修改终端地址
  
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

#ifdef ZDO_COORDINATOR   
  int ret = InitList_L(&LL); // 单链表初始化
  if(ret == OVERFLOW) {
    myprintf("InitList_L OVERFLOW\n");
  }
#endif  
}

/*********************************************************************
*          系统各种事件的处理函数
*/
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;
  ZStatus_t sentStatus;
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    uint8 coodStarted[3] = {'#', 0x45, '@'};
    while ( MSGpkt )
    {      
      switch ( MSGpkt->hdr.event )
      {
        // 系统任务事件里面的接收数据确认消息
        case AF_DATA_CONFIRM_CMD:          
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            myprintf("Got ERROR:AF_DATA_CONFIRM_CMD failed\n");
          }
          break;
        
          // 接收到一个按键事件
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
          
          // 无线网络收到数据
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
          
          // 设备组网或者入网成功，可以发送一个消息，并开启一个心跳事件的定时器
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status); // 另外MSGpkt->srcAddr.addr.shortAddr;可用于获取终端节点的网络短地址                    
          // 判断不同的设备类型
          if(SampleApp_NwkState == DEV_ZB_COORD) { // 协调器组建好网络            
            mySendByteBuf(coodStarted, 3); // 向上位机发送已经启动消息            
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // 开启定时器
            HalLedBlink(HAL_LED_2, 3, 50, 250);
          } else if(SampleApp_NwkState == DEV_ROUTER) { // 路由器加入网络
            HalLedBlink(HAL_LED_2, 3, 50, 250);
          } else if(SampleApp_NwkState == DEV_END_DEVICE) { // 终端加入网络
            EndDeviceSendAfMessageByCmd(ENDDEVICE_NETWORK_READY); // 向协调器发送入网消息
            HalLedBlink(HAL_LED_2, 3, 50, 250);
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // 开启定时器
            IEN1 |= (0x1 << 5); // 开启端口0中断
          }
          break;
          
        default:        
          myprintf("unknown pkg-event = 0x%x\n", MSGpkt->hdr.event);      
          break;
      }      
      
      osal_msg_deallocate( (uint8 *)MSGpkt );// Release the memory
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );// Next - if one is available
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  //myprintf("events = 0x%x\n", events);
  // 接收到一个定时器事件
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
#ifdef ZDO_COORDINATOR // 协调器
    SampleApp_SendHeartBeatMessageCoor(); // 协调器心跳功能函数，同时检测掉线的终端，将掉线信息发送到服务器
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT); // 1000ms再次开启定时器
#endif
    
#ifdef END_DEVICE // 终端设备
    EndDeviceSendHeartBeat(); // 终端心跳功能函数，心跳函数中还检查彩票引脚输出信号，一定时间内没有彩票出来则上传出彩票结果
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*3/2); // 1500ms再次开启定时器
#endif
    
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT); // 清除此已经处理事件标志位
  }  

#ifdef END_DEVICE  // 接收到线上金币的开启信号，模拟电平变化的定时器事件
  else if(events & SAMPLEAPP_SIMULATE_COIN_MSG_EVT) {
    myprintf("Get COIN_MSG_EVT, NeedCoinDelivered = %d, events = 0x%x\n", NeedCoinDelivered, events);    
    if(COIN_SIMULATE_PIN == 1) {
      COIN_SIMULATE_PIN = 0; // 产生一个下降沿
    } else if(COIN_SIMULATE_PIN == 0) {
      COIN_SIMULATE_PIN = 1; // 完整产生了一个脉冲
      NeedCoinDelivered--;
    }
    if(NeedCoinDelivered > 0) { // 还需要开启定时器
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // 低电平和高电平至少保持50ms
    } 
    
    return (events ^ SAMPLEAPP_SIMULATE_COIN_MSG_EVT);
  } 
  else if(events & SAMPLEAPP_TRANSPORT_COIN_MSG_EVT) { // 需要转发投币器的投币信号
    //myprintf("Get transport coin EVT, NeedCoinTransport = %d\n", NeedCoinTransport);
    
    if(COIN_SIMULATE_PIN == 1) {
      COIN_SIMULATE_PIN = 0; // 产生一个下降沿
    } else if(COIN_SIMULATE_PIN == 0) {
      COIN_SIMULATE_PIN = 1; // 完整产生了一个脉冲
      NeedCoinTransport--;
    }
    if(NeedCoinTransport > 0) { // 还需要开启定时器
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TRANSPORT_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // 低电平和高电平至少保持50ms
    }
    return (events ^ SAMPLEAPP_TRANSPORT_COIN_MSG_EVT);
  }
#endif  

  return 0;
}

/*********************************************************************
* LOCAL FUNCTIONS
*/
#define INIT_LEFT_SEC 5
#ifdef NEED_TEST_DIStANCE
uint16 recvTestMsgCount = 0;
#endif
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8* buffer = NULL;
  uint8 recvLen = 0;
  uint8 outputBuffer[SERIAL_BUFFER_SIZE] = {0};
  uint8 outputLen = 0;
  bool check = false;
  uint8 cmd = 0x0;
  
  // 协调器在发送数据的时候指定的此消息对应的命令字，即表示哪种类型的控制命令
  if(pkt->clusterId == SAMPLEAPP_PERIODIC_CLUSTERID) {
    //pkt->cmd.Data数据数组; pkt->cmd.DataLength 数据长度
    buffer = pkt->cmd.Data;
    recvLen = pkt->cmd.DataLength;
    re_replace_data(buffer, recvLen, outputBuffer, &outputLen); // 还原被替换的特殊数据
    check = check_xor(outputBuffer, outputLen); // 数据校验
    cmd = outputBuffer[1];  
    
//协调器接收到终端的无线数据 
#ifdef ZDO_COORDINATOR
    if(check) {      
      int8 ret = 0;
      if(cmd == ENDDEVICE_NETWORK_READY) { // 终端入网
        uint8 machineId = outputBuffer[4];
        ret = checkEPIsInListByMachineId(LL, machineId);
        if(ret != -1) { // 已经在链表中，更新倒计时
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);       
        } else if(ret == -1) { // 没有在链表中
          ListInsert_L(LL, ListLength_L(LL)+1, outputBuffer[5], outputBuffer[6], &outputBuffer[7], INIT_LEFT_SEC); // 添加一个线性列表元素到链表尾部 
          myprintf("Insert new EP in list\n");
          printAddrInfoHex(outputBuffer+5, 10); // 打印中断的长短地址信息         
        }
        // myprintf("length = %d\n", ListLength_L(LL));      
      } else if(cmd == ENDDEVICE_HEARTBEAT) { // 终端心跳
        uint8 machineId = outputBuffer[4];
        ret = checkEPIsInListByMachineId(LL, machineId);
        if(ret != -1) { // 已经在链表中，更新倒计时
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);
        }
      } else if(cmd == ENDDEVICE_REPLY_RECVED_COIN || cmd == ENDDEVICE_REPORT_GAME_REWARD) { // 将数据转发到树莓派，回复收到投币命令、上报游戏奖励彩票个数
        mySendByteBuf(outputBuffer, outputLen);        
      }          
    } else {
      myprintf("coord recv end_device message check failed\n");
    }
#endif    
    
#ifdef END_DEVICE //终端接收数据处理
    if(check) {
      if(cmd == TO_PAY_COIN) { // 服务器发送的模拟投币动作
        NeedCoinDelivered = outputBuffer[4];
        COIN_SIMULATE_PIN = 1; // 初始高电平
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // 产生定时器事件                
        
        EndDeviceSendAfMessageByCmd(ENDDEVICE_REPLY_RECVED_COIN); //回复已经收到投币命令
        myprintf("NeedCoinDelivered = %d\n", NeedCoinDelivered);        
      } else if(cmd == TEST_DISTANCE_FLASH_LED) {
        HalLedBlink(HAL_LED_1, 1, 50, 500);
#ifdef NEED_TEST_DIStANCE
        recvTestMsgCount++;
        HalLcdWriteString("Recv:", HAL_LCD_LINE_5);
        convertHexToStrLCD(recvTestMsgCount, HAL_LCD_LINE_6);
#endif 
      }    
    } else {
      myprintf("end device check af data failed\n");
    }     
#endif    

    
  }
}

/**
  协调器心跳功能函数
*/
#ifdef NEED_TEST_DIStANCE
  uint16 totalSendTestCount = 0;
  uint16 sendTestCount = 0;
#endif  
void SampleApp_SendHeartBeatMessageCoor(void) {  
  // 将终端列表中元素在线时长减1，如果检测到超时则将IEEE发送到上位机
  decAllListEPLeftSec(LL); 
  HalLedBlink(HAL_LED_1, 1, 50, 500);
#ifdef NEED_TEST_DIStANCE  
  // 发送稳定性测试数据到终端设备
  totalSendTestCount++;
  if(totalSendTestCount % 3 == 0) {
    SampleApp_SendPeriodicMessage(1);
    myprintf("sendTestCount = %d\n", totalSendTestCount); // 串口打印发送的次数
  }
#endif  
}

/**
  终端心跳函数，用于定时向协调器发送心跳，包含本终端机器编号
*/
void EndDeviceSendHeartBeat(void) {
  EndDeviceSendAfMessageByCmd(ENDDEVICE_HEARTBEAT);
  
  if(startRewardInterrupt == 1 && (osal_getClock() - lastRecvRewardSecTime) >= 2) { // 如果触发了奖励中断，且奖励结束距离当前时间大于指定时间则认为是一次奖励结束
    if(rewardCount == 0) {
      startRewardInterrupt = 0;
      return;
    }    
    EndDeviceSendAfMessageByCmd(ENDDEVICE_REPORT_GAME_REWARD); // 发送奖励彩票个数到协调器
    myprintf("Upload reward result count, rewardCount = %d\n", rewardCount);    
    rewardCount = 0; // 置0
    startRewardInterrupt = 0;    
  }
}

/**
  终端用于发送各种信息到协调器
*/
void EndDeviceSendAfMessageByCmd(uint8 cmd) {
  uint8 strBuf[SEND_BUF_SIZE] = {0};
  uint8 strBufLen = 0;
  uint8 outputBuf[SEND_BUF_SIZE] = {0};
  uint8 outputBufLen = 0;
  
  if(cmd == ENDDEVICE_NETWORK_READY) {
    uint16 shortAddr = NLME_GetShortAddr(); // 获取自身网络设备地址，获取父设备网络地址：uint16 NLME_GetCoordShortAddr( void );  
    strBuf[0] = MACHINE_ID; // 此终端编号
    strBuf[1] = HI_UINT16(shortAddr); // 存放地址的高8位
    strBuf[2] = LO_UINT16(shortAddr); // 存放低8位
    osal_memcpy(&strBuf[3], NLME_GetExtAddr(), 8); // 复制IEEE地址  
    strBufLen = 11;
  } else if(cmd == ENDDEVICE_HEARTBEAT) {
    strBuf[0] = MACHINE_ID; // 此终端编号
    strBufLen = 1;
  } else if(cmd == ENDDEVICE_REPLY_RECVED_COIN) { // 终端设备回复已经收到金币
    strBuf[0] = MACHINE_ID; // 此终端编号
    strBufLen = 1;
  } else if(cmd == ENDDEVICE_REPORT_GAME_REWARD) { // 终端上传奖励彩票个数
    strBuf[0] = MACHINE_ID; // 此终端编号
    strBuf[1] = rewardCount;
    strBufLen = 2;
  }

  // 进行数据协议封装
  encodeData(cmd, strBuf, strBufLen, outputBuf, &outputBufLen);
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // 发送的目的地址+端点地址+传送模式
                      (endPointDesc_t *)&SampleApp_epDesc, // 源终端的描述
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // 被profile指定的有效的集群号
                      outputBufLen,                                  // 发送数据长度
                      outputBuf,                              // 发送数据缓冲区
                      &SampleApp_TransID,                  // 消息发送ID
                      AF_DISCV_ROUTE,                      // 有效位掩码的发送选项
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // 传送跳数，通常设置为AF_DEFAULT_RADIUS
  {
    myprintf("send cmd = 0x%x failed\n", cmd); 
  }
}

/**
  协调器打印终端上传的长短地址
*/
void printAddrInfoHex(uint8* buf, uint8 dataLen) {
  int i = 0;
  
  myprintf("\n");
  for(i = 0; i < dataLen; i++) {
    if(i == 2) {
      myprintf("; IEEE = ");
    }
    myprintf("0x%x,", buf[i]);
  }
  myprintf("\n");
}

/**
  将十六进制数转变成字符
*/
void convertHexToStrLCD(uint16 addr, uint8 lcdLineNum) {
  uint8 i;
  uint8 *xad;
  uint8 lcd_buf[2*2+1];

  // Display the extended address.
  xad = (uint8*)&addr + 2 - 1;

  for (i = 0; i < 2*2; xad--)
  {
    uint8 ch;
    ch = (*xad >> 4) & 0x0F;
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7'); // +7可以转变成大写字符ABCDEF
    ch = *xad & 0x0F;
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
  }
  lcd_buf[2*2] = '\0';
  HalLcdWriteString( (char*)lcd_buf, lcdLineNum);
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
      HalLedBlink(HAL_LED_1, 1, 50, 250);
    } else {
      myprintf("SampleApp_SendPeriodicMessage() failed\n");
    }
  }
}


/*********************************************************************
* Event Generation Functions
* @brief   按键事件处理函数
*/
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
#ifdef ZDO_COORDINATOR       //按协调器S1才发数据  
  if ( keys & HAL_KEY_SW_6 ){ // 按键S1,以广播方式发数据    
    SampleApp_SendPeriodicMessage(1);
  } else if(keys & HAL_KEY_SW_1) { // 按键S2
    //SampleApp_SendPeriodicMessage(2);
  }
#endif
}