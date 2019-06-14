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
PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/*********************************************************************
��Ҫ�޸ĵ��ļ������������£�
ZMain/ZMain.c��������
HAL/hal_drivers.c�����ⲿ�жϵĳ�ʼ��
MT/MT_UART.c�н��д�����Ϣ�Ľ���
SampleApp.c�ж����ж���Ӧ�������Լ��¼���Ϣ��������
SingleLinkedList.c�ж����˵������ʵ��
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

/*******
  һЩ�����õĺ꿪��
*/
// #define NEED_TEST_DIStANCE

/*********************************************************************
* MACROS
*/
#define COIN_PIN P0_5            //����P0.5��Ϊ�жϷ�ʽ����Ͷ�����ź�����
#define SIMULATE_PIN P0_7        //����P0.7��Ϊ�滻Ͷ����ģ���źŷ�������
#define UART0        0x00
#define COIN_SIGNAL_HALF_TIME 40

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

afAddrType_t SampleApp_Periodic_DstAddr; // ����Э�����㲥��Ϣ
afAddrType_t SampleApp_TxAddr; // �����ն��豸��Э��������̵�ַ��Ϣ

aps_Group_t SampleApp_Group;
uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

LinkList LL; // ������������洢�ն˵�ַ�Լ�������Ϣ

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
void printAddrInfoHex(uint8* buf, uint8 dataLen);
void convertHexToStr(uint16 addr, uint8 lcdLineNum);

#ifdef END_DEVICE
int8 NeedCoinDelivered = 0; // �����ն���Ҫ�����Ͷ�Ҹ���
int8 NeedCoinTransport = 0; // ��Ҫת����Ͷ�����ź�����
#endif

/**
  �ն��豸P1�˿��жϴ���������������Ͷ�������ź�
*/
#ifdef END_DEVICE
HAL_ISR_FUNCTION( coinPort0Isr, P0INT_VECTOR ) // P0_5����ΪͶ������ƽ�仯�ж����ţ���HAL/Common/hal_drivers.c�н��е����ų�ʼ��
{  
  P0IFG &= ~(0x1 << 5);       //�˿�0�ж�״̬��־
  P0IF = 0;        //�˿�0�жϱ�־��0��ʾ���ж�δ����1��ʾ���ж�δ������Ҫÿ�δ���֮����0����Ȼ��һֱ�����ж�
  
  SIMULATE_PIN = 1; // ��ʼ�ߵ�ƽ 
  NeedCoinTransport = 1;
  osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TRANSPORT_COIN_MSG_EVT, 50); // ����һ����ʱ���¼�
  HalLedBlink(HAL_LED_2, 5, 50, 250);
}
#endif

/**
* �Զ��������ʼ������
*/
void SampleApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  P0SEL &= ~(0x01 << 7);         //����P0.7��Ϊ��ͨIO
  P0DIR |= (0x01 << 7);          //����P0.7��Ϊ���
  SIMULATE_PIN = 1;       // Ͷ�����ź�ģ���������
  
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
  // Э�����㲥�������ݵ��ն�
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF; // �㲥��ַ
  
  // �ն˵����������ݵ�Э����
  SampleApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_TxAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_TxAddr.addr.shortAddr = 0x00; // ����Э������ַ
  
  // Э�����������͵�ָ����ַ���ն���  
  EndPoint_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  EndPoint_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  EndPoint_DstAddr.addr.shortAddr = 0x00; // �ڷ��͵�ʱ��̬�޸��ն˵�ַ
  
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT; // �˵��
  SampleApp_epDesc.task_id = &SampleApp_TaskID; // �˵��Ӧ������
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc; // ��������
  SampleApp_epDesc.latencyReq = noLatencyReqs;  
  afRegister( &SampleApp_epDesc ); // ע��һ��Ӧ�õĶ˵���������ע��һ���µĶ˵㵽�������������µ���Ϣ����ʱֱ�ӷ��͵�ָ����������
  
  // �󶨰����¼�������ID
  RegisterForKeys( SampleApp_TaskID );
  
  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
  // ��ʼ������
  MT_UartInit();                    //���ڳ�ʼ��
  MT_UartRegisterTaskID(task_id);   //ע�ᴮ������
  myprintf("UartInit OK\n");

#ifdef ZDO_COORDINATOR   
  int ret = InitList_L(&LL); // �������ʼ��
  if(ret == OVERFLOW) {
    myprintf("InitList_L OVERFLOW\n");
  }
#endif  
}

/*********************************************************************
* @fn      SampleApp_ProcessEvent
*
* @brief   Generic Application Task event processor.  This function
*          is called to process all events for the task.  Events
*          include timers, messages and any other user defined events.
*          ϵͳ�����¼��Ĵ�����
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
        // ϵͳ�����¼�����Ľ�������ȷ����Ϣ
        case AF_DATA_CONFIRM_CMD:          
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            myprintf("ERROR:AF_DATA_CONFIRM_CMD failed\n");
          }
          break;
        
          // ���յ�һ�������¼�
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
          
          // ���������յ�����
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
          
          // �豸�������������ɹ������Է���һ����Ϣ��������һ�������¼��Ķ�ʱ��
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status); // ����MSGpkt->srcAddr.addr.shortAddr;�����ڻ�ȡ�ն˽ڵ������̵�ַ          
          
          // �жϲ�ͬ���豸����
          if(SampleApp_NwkState == DEV_ZB_COORD) { // Э�����齨������            
            mySendByteBuf(coodStarted, 3); // ����λ�������Ѿ�������Ϣ            
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // ������ʱ��
            HalLedBlink(HAL_LED_2, 3, 50, 250);
          } else if(SampleApp_NwkState == DEV_ROUTER) { // ·������������
            HalLedBlink(HAL_LED_2, 3, 50, 250);
          } else if(SampleApp_NwkState == DEV_END_DEVICE) { // �ն˼�������
            AfSendAddrInfo(0x71); // ��Э��������������Ϣ
            HalLedBlink(HAL_LED_2, 3, 50, 250);
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // ������ʱ��
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
  
  // ���յ�һ����ʱ���¼�
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
#ifdef ZDO_COORDINATOR // Э����
    SampleApp_SendHeartBeatMessageCoor(); // Э�����������ܺ�����ͬʱ�����ߵ��նˣ���������Ϣ���͵�������
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT); // 1000ms�ٴο�����ʱ��
#endif
    
#ifdef END_DEVICE // �ն��豸
    SampleApp_SendHeartBeatMessageEnd(); // �ն��������ܺ���
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*2); // 2000ms�ٴο�����ʱ��
#endif
    
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT); // ������Ѿ������¼���־λ
  }  
  
#ifdef END_DEVICE  // ���յ����Ͻ�ҵĿ����źţ�ģ���ƽ�仯�Ķ�ʱ���¼�
  else if(events & SAMPLEAPP_SIMULATE_COIN_MSG_EVT) {
    myprintf("Get COIN_MSG_EVT, NeedCoinDelivered = %d\n", NeedCoinDelivered);
    if(SIMULATE_PIN == 1) {
      SIMULATE_PIN = 0; // ����һ���½���
    } else if(SIMULATE_PIN == 0) {
      SIMULATE_PIN = 1; // ����������һ������
      NeedCoinDelivered--;
    }
    if(NeedCoinDelivered > 0) { // ����Ҫ������ʱ��
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // �͵�ƽ�͸ߵ�ƽ���ٱ���50ms
    }    
    return (events ^ SAMPLEAPP_SIMULATE_COIN_MSG_EVT);
  } else if(events & SAMPLEAPP_TRANSPORT_COIN_MSG_EVT) { // ��Ҫת��Ͷ������Ͷ���ź�
    myprintf("Get transport coin EVT");
    
    if(SIMULATE_PIN == 1) {
      SIMULATE_PIN = 0; // ����һ���½���
    } else if(SIMULATE_PIN == 0) {
      SIMULATE_PIN = 1; // ����������һ������
      NeedCoinTransport--;
    }
    if(NeedCoinTransport > 0) { // ����Ҫ������ʱ��
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TRANSPORT_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // �͵�ƽ�͸ߵ�ƽ���ٱ���50ms
    }
    return (events ^ SAMPLEAPP_TRANSPORT_COIN_MSG_EVT);
  }
#endif  
  
  return 0;
}

/*********************************************************************
* Event Generation Functions
*/
/*********************************************************************
* @fn      SampleApp_HandleKeys
*
* @brief   �����¼�������
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
#ifdef ZDO_COORDINATOR       //��Э����S1�ŷ�����  
  if ( keys & HAL_KEY_SW_6 ){ // ����S1,�Թ㲥��ʽ������    
    SampleApp_SendPeriodicMessage(1);
  } else if(keys & HAL_KEY_SW_1) { // ����S2
    SampleApp_SendPeriodicMessage(2);
  }
#endif

#ifdef END_DEVICE
  if ( keys & HAL_KEY_SW_6 ){
    AfSendAddrInfo(0x51); // ֻ����·�ɺ��ն��豸�ϣ���Э�����ϴ������ַ��Э�����յ�����Ϣ���ӡ����ַ
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
#ifdef NEED_TEST_DIStANCE
uint16 recvTestMsgCount = 0;
#endif
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
#ifndef ROUTER_EB  
  uint8 data;
  uint8 cmd;
#endif  
#ifdef ZDO_COORDINATOR  
  int8 ret = 0;
#endif  
  
  switch ( pkt->clusterId )
  {    
  case SAMPLEAPP_PERIODIC_CLUSTERID: // Э�����ڷ������ݵ�ʱ��ָ���Ĵ���Ϣ��Ӧ�������֣�����ʾ�������͵Ŀ�������
//Э�������յ��ն˵��������� 
#ifdef ZDO_COORDINATOR
    data = pkt->cmd.Data[0];
    cmd = pkt->cmd.Data[1];
    if(data == '#') {
      if(cmd == 0x71 && pkt->cmd.Data[12] == '@') { // ���յ��ն��ϴ��Ķ̵�ַ��IEEE��ַ
        ret = checkEPInList(LL, pkt->cmd.Data + 4); // ʹ��IEEE��ַ����ն��Ƿ��Ѿ���������
        if(ret == -1) { // �����ǰ�ն�û�����б���        
          ListInsert_L(LL, ListLength_L(LL)+1, pkt->cmd.Data[2], pkt->cmd.Data[3], &(pkt->cmd.Data[4]), INIT_LEFT_SEC); // ���һ�������б�Ԫ�ص�����β�� 
          myprintf("Insert new EP in list\n");
        } else { // �����ն����ڽڵ������ʱ�������Ϊ����״̬
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);
          // myprintf("Already In List, update()\n");
        }
        // myprintf("length = %d\n", ListLength_L(LL));
      } else if(cmd == 0x72 && pkt->cmd.Data[10] == '@') { // �յ��ն˻ظ���ȷ���յ�Ͷ������
        mySendByteBuf(pkt->cmd.Data, 11); // ת���ظ���Ϣ����λ��        
      } else if(cmd == 0x51 && pkt->cmd.Data[12] == '@') { // �ն˰���S1�ϴ��ĳ��̵�ַ��Ϣ
        printAddrInfoHex(pkt->cmd.Data+2, 10);
        mySendByteBuf(pkt->cmd.Data, 13);        
      }
    }
#endif    
#ifdef END_DEVICE //�ն˽������ݴ���
    data = pkt->cmd.Data[0];
    cmd = pkt->cmd.Data[1];
    if(data == '#')
    {
      if(cmd == 0x81 && pkt->cmd.Data[3] == '@') { // �յ�Ͷ��ָ����������Ϣ
        NeedCoinDelivered = pkt->cmd.Data[2]; // ��ʼģ��Ͷ���ź�        
        SIMULATE_PIN = 1; // ��ʼ�ߵ�ƽ
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // ������ʱ���¼�                
        AfReplyGetCoinCmd();  // �ظ��Ѿ��յ�Ͷ������0x72
      } else if(cmd == 0x31 && pkt->cmd.Data[3] == '@') { // LED2���Ʋ�������
        HalLedBlink(HAL_LED_1, 1, 50, 500);
#ifdef NEED_TEST_DIStANCE
        recvTestMsgCount++;
        HalLcdWriteString("Recv:", HAL_LCD_LINE_5);
        convertHexToStr(recvTestMsgCount, HAL_LCD_LINE_6);
#endif        
      }
    }        
#endif    
    break;
    
  default:
    break;
  }
}

/**
* Э�������ն˷��͹㲥������Ϣ
*/
void SampleApp_SendPeriodicMessage( int8 key )
{
  if(key == 1) { // ����S1
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
    }
  } else if(key == 2) { // ����S2
    // myprintf("press key S2\n");
  }
}

/**
  Э�����������ܺ�����ʹ�ô�������ݮ�ɷ���������Ϣ
*/
#ifdef NEED_TEST_DIStANCE
  uint16 totalSendTestCount = 0;
  uint16 sendTestCount = 0;
#endif  
void SampleApp_SendHeartBeatMessageCoor(void) {
  // �����ն��б������нڵ�ʣ��ʱ��-1�����С��0������0
  uint8 heartBeatBuf[3] = {'#', 0x46, '@'};
  mySendByteBuf(heartBeatBuf, 3);
  
  // ���ն��б���Ԫ������ʱ��-1�������⵽��ʱ��IEEE���͵���λ��
  decAllListEPLeftSec(LL); 
  HalLedBlink(HAL_LED_2, 1, 50, 500);
#ifdef NEED_TEST_DIStANCE  
  // �����ȶ��Բ������ݵ��ն��豸
  totalSendTestCount++;
  if(totalSendTestCount % 3 == 0) {
    SampleApp_SendPeriodicMessage(1);
    myprintf("sendTestCount = %d\n", totalSendTestCount); // ���ڴ�ӡ���͵Ĵ���
  }
#endif  
}

/**
  �ն��������������ڶ�ʱ��Э�������ͱ�����ַ��Ϣ
*/
void SampleApp_SendHeartBeatMessageEnd(void) { 
  AfSendAddrInfo(0x71);  
}

/**
  �����ն˵�ַ��Ϣ��Э����
*/
void AfSendAddrInfo(uint8 cmd) {
  uint16 shortAddr;
  uint8 strBuf[13] = {0};  
  
  shortAddr = NLME_GetShortAddr(); // ��ȡ���������豸��ַ����ȡ���豸�����ַ��uint16 NLME_GetCoordShortAddr( void );
  strBuf[0] = '#';  // �������ݵı�ʶ������Э��������
  strBuf[1] = cmd; // ��Ϣ���ͱ�ʶ
  strBuf[2] = HI_UINT16(shortAddr); // ��ŵ�ַ�ĸ�8λ
  strBuf[3] = LO_UINT16(shortAddr); // ��ŵ�8λ
  osal_memcpy(&strBuf[4], NLME_GetExtAddr(), 8); // ����IEEE��ַ
  strBuf[12] = '@'; // β���ֽ�
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // ���͵�Ŀ�ĵ�ַ+�˵��ַ+����ģʽ
                      (endPointDesc_t *)&SampleApp_epDesc, // Դ�ն˵�����
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // ��profileָ������Ч�ļ�Ⱥ��
                      13,                                  // �������ݳ���
                      strBuf,                              // �������ݻ�����
                      &SampleApp_TransID,                  // ��Ϣ����ID
                      AF_DISCV_ROUTE,                      // ��Чλ����ķ���ѡ��
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // ����������ͨ������ΪAF_DEFAULT_RADIUS
  {
    HalLedBlink(HAL_LED_1, 2, 50, 250); // ����ʧ�ܿ�����˸����
  } else {
    HalLedBlink(HAL_LED_2, 1, 50, 500); // ���ͳɹ�������˸һ��
  }
}

/**
  �ն˷����Ѿ��յ�Ͷ�ҿ�ʼ��Ϸ����
*/
void AfReplyGetCoinCmd(void) {
  uint8 strBuf[11] = {0};  
  
  strBuf[0] = '#';  // �������ݵı�ʶ������Э��������
  strBuf[1] = 0x72; // ��Ϣ���ͱ�ʶ
  osal_memcpy(&strBuf[2], NLME_GetExtAddr(), 8); // ����IEEE��ַ
  strBuf[10] = '@'; // β���ֽ�
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // ���͵�Ŀ�ĵ�ַ+�˵��ַ+����ģʽ
                      (endPointDesc_t *)&SampleApp_epDesc, // Դ�ն˵�����
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // ��profileָ������Ч�ļ�Ⱥ��
                      11,                                  // �������ݳ���
                      strBuf,                              // �������ݻ�����
                      &SampleApp_TransID,                  // ��Ϣ����ID
                      AF_DISCV_ROUTE,                      // ��Чλ����ķ���ѡ��
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // ����������ͨ������ΪAF_DEFAULT_RADIUS
  {
    HalLedBlink(HAL_LED_1, 4, 50, 250); // ����ʧ�ܿ�����˸����
  } else {
    HalLedBlink(HAL_LED_1, 2, 50, 500); // ���ͳɹ�������˸2��
  }
}

/**
  Э������ӡ�ն��ϴ��ĳ��̵�ַ
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
  ��ʮ��������ת����ַ�
*/
void convertHexToStr(uint16 addr, uint8 lcdLineNum) {
  uint8 i;
  uint8 *xad;
  uint8 lcd_buf[2*2+1];

  // Display the extended address.
  xad = (uint8*)&addr + 2 - 1;

  for (i = 0; i < 2*2; xad--)
  {
    uint8 ch;
    ch = (*xad >> 4) & 0x0F;
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7'); // +7����ת��ɴ�д�ַ�ABCDEF
    ch = *xad & 0x0F;
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
  }
  lcd_buf[2*2] = '\0';
  HalLcdWriteString( (char*)lcd_buf, lcdLineNum);
}