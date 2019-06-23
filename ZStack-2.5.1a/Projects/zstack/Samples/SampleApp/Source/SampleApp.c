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

#include "OSAL_Clock.h"

#include "common.h"
/*******
  һЩ�����õĺ꿪��
*/
// #define NEED_TEST_DIStANCE

/*********************************************************************
* MACROS
*/
#define COIN_RECV_PIN      P0_4      //�ж������½��ش���������P0.7��Ϊ�жϷ�ʽ����Ͷ�����ź�����40ms
#define REWARD_OUTPUT_PIN  P0_6      //�ж������½��ش�������Ʊ������������ź����ţ����һ����Ʊ���������һ��40ms�ĵ͵�ƽ

#define COIN_SIMULATE_PIN  P1_2      //���������P1.2��Ϊ�滻Ͷ����ģ���źŷ�������

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

afAddrType_t SampleApp_Periodic_DstAddr; // ����Э�����㲥��Ϣ
afAddrType_t SampleApp_TxAddr; // �����ն��豸��Э��������̵�ַ��Ϣ

aps_Group_t SampleApp_Group;
uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

LinkList LL; // ������������洢�ն˵�ַ�Լ�������Ϣ

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

int8 NeedCoinDelivered = 0; // ���Ͽ�����Ϸ��Ҫ�����Ͷ�Ҹ���
int8 NeedCoinTransport = 0; // ��Ҫת����Ͷ�����ź�����
uint8 rewardCount = 0;  // һ����Ϸ����Ʊ����ͳ��
uint8 startRewardInterrupt = 0;
UTCTime lastRecvRewardSecTime = 0;//=osal_getClock();

/**
  �ն��豸P1�˿��жϴ���������������Ͷ�������ź�
*/
#ifdef END_DEVICE
uint8 pre_coin_pin_value = 1;
uint8 coin_pin_value = 1;
uint8 pre_reward_pin_value = 1;
uint8 reward_pin_value = 1;
HAL_ISR_FUNCTION( port0Isr, P0INT_VECTOR ) // P0_5����ΪͶ������ƽ�仯�ж����ţ���HAL/Common/hal_drivers.c�н��е����ų�ʼ��
{ 
  IEN1 &= ~(0x1 << 5); // �رն˿�0�ж�
  if((P0IFG >> 4) & 0x1 == 1) { // �ж�p0_4���Ų����жϣ�Ͷ���ź�    
    coin_pin_value = COIN_RECV_PIN;
    if(coin_pin_value == 0) {
      if(coin_pin_value != pre_coin_pin_value) {
        pre_coin_pin_value = coin_pin_value;
        NeedCoinTransport = 1;
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_TRANSPORT_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // ����һ����ʱ���¼�    
      }
    } else {
      pre_coin_pin_value = 1;
    }
    P0IFG &= ~(0x1 << 4);       //�˿�0�ж�״̬��־    
    //myprintf("get p0_4 coin input interrupt pin = %d\n", COIN_RECV_PIN);
  } 
  if((P0IFG >> 6) & 0x1 == 1) { // P0_6���ţ���Ʊ����һ����Ʊ�ж��źţ�ת������Ϸ�����ϴ�������
    startRewardInterrupt = 1;
    lastRecvRewardSecTime = osal_getClock(); // �����յ��źŵ�ǰ���������ڶ�ʱ���н��м��
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
    myprintf("\nP0_6=%d", REWARD_OUTPUT_PIN); // �������ڴ�ӡ��䲻��ȥ������������������׼ȷ���
  }
  P0IF = 0;        //�˿�0�жϱ�־��0��ʾ���ж�δ����1��ʾ���ж�δ������Ҫÿ�δ���֮����0����Ȼ��һֱ�����ж� 
  IEN1 |= (0x1 << 5); // �˿�0�ж�ʹ��
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
  
#ifdef END_DEVICE
  //P1_2�����Ͷ����ģ���źŲ�������
  P1SEL &= ~(0x01 << 2);         //����P1.2��Ϊ��ͨIO
  P1DIR |= (0x01 << 2);          //����P1.2��Ϊ���
  COIN_SIMULATE_PIN = 1;       // Ͷ�����ź�ģ���������  
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
*          ϵͳ�����¼��Ĵ�����
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
            myprintf("Got ERROR:AF_DATA_CONFIRM_CMD failed\n");
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
            EndDeviceSendAfMessageByCmd(ENDDEVICE_NETWORK_READY); // ��Э��������������Ϣ
            HalLedBlink(HAL_LED_2, 3, 50, 250);
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT ); // ������ʱ��
            IEN1 |= (0x1 << 5); // �����˿�0�ж�
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
  // ���յ�һ����ʱ���¼�
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
#ifdef ZDO_COORDINATOR // Э����
    SampleApp_SendHeartBeatMessageCoor(); // Э�����������ܺ�����ͬʱ�����ߵ��նˣ���������Ϣ���͵�������
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT); // 1000ms�ٴο�����ʱ��
#endif
    
#ifdef END_DEVICE // �ն��豸
    EndDeviceSendHeartBeat(); // �ն��������ܺ��������������л�����Ʊ��������źţ�һ��ʱ����û�в�Ʊ�������ϴ�����Ʊ���
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*3/2); // 1500ms�ٴο�����ʱ��
#endif
    
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT); // ������Ѿ������¼���־λ
  }  

#ifdef END_DEVICE  // ���յ����Ͻ�ҵĿ����źţ�ģ���ƽ�仯�Ķ�ʱ���¼�
  else if(events & SAMPLEAPP_SIMULATE_COIN_MSG_EVT) {
    myprintf("Get COIN_MSG_EVT, NeedCoinDelivered = %d, events = 0x%x\n", NeedCoinDelivered, events);    
    if(COIN_SIMULATE_PIN == 1) {
      COIN_SIMULATE_PIN = 0; // ����һ���½���
    } else if(COIN_SIMULATE_PIN == 0) {
      COIN_SIMULATE_PIN = 1; // ����������һ������
      NeedCoinDelivered--;
    }
    if(NeedCoinDelivered > 0) { // ����Ҫ������ʱ��
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // �͵�ƽ�͸ߵ�ƽ���ٱ���50ms
    } 
    
    return (events ^ SAMPLEAPP_SIMULATE_COIN_MSG_EVT);
  } 
  else if(events & SAMPLEAPP_TRANSPORT_COIN_MSG_EVT) { // ��Ҫת��Ͷ������Ͷ���ź�
    //myprintf("Get transport coin EVT, NeedCoinTransport = %d\n", NeedCoinTransport);
    
    if(COIN_SIMULATE_PIN == 1) {
      COIN_SIMULATE_PIN = 0; // ����һ���½���
    } else if(COIN_SIMULATE_PIN == 0) {
      COIN_SIMULATE_PIN = 1; // ����������һ������
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
  
  // Э�����ڷ������ݵ�ʱ��ָ���Ĵ���Ϣ��Ӧ�������֣�����ʾ�������͵Ŀ�������
  if(pkt->clusterId == SAMPLEAPP_PERIODIC_CLUSTERID) {
    //pkt->cmd.Data��������; pkt->cmd.DataLength ���ݳ���
    buffer = pkt->cmd.Data;
    recvLen = pkt->cmd.DataLength;
    re_replace_data(buffer, recvLen, outputBuffer, &outputLen); // ��ԭ���滻����������
    check = check_xor(outputBuffer, outputLen); // ����У��
    cmd = outputBuffer[1];  
    
//Э�������յ��ն˵��������� 
#ifdef ZDO_COORDINATOR
    if(check) {      
      int8 ret = 0;
      if(cmd == ENDDEVICE_NETWORK_READY) { // �ն�����
        uint8 machineId = outputBuffer[4];
        ret = checkEPIsInListByMachineId(LL, machineId);
        if(ret != -1) { // �Ѿ��������У����µ���ʱ
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);       
        } else if(ret == -1) { // û����������
          ListInsert_L(LL, ListLength_L(LL)+1, outputBuffer[5], outputBuffer[6], &outputBuffer[7], INIT_LEFT_SEC); // ���һ�������б�Ԫ�ص�����β�� 
          myprintf("Insert new EP in list\n");
          printAddrInfoHex(outputBuffer+5, 10); // ��ӡ�жϵĳ��̵�ַ��Ϣ         
        }
        // myprintf("length = %d\n", ListLength_L(LL));      
      } else if(cmd == ENDDEVICE_HEARTBEAT) { // �ն�����
        uint8 machineId = outputBuffer[4];
        ret = checkEPIsInListByMachineId(LL, machineId);
        if(ret != -1) { // �Ѿ��������У����µ���ʱ
          updateListEleStatus(LL, ret, INIT_LEFT_SEC);
        }
      } else if(cmd == ENDDEVICE_REPLY_RECVED_COIN || cmd == ENDDEVICE_REPORT_GAME_REWARD) { // ������ת������ݮ�ɣ��ظ��յ�Ͷ������ϱ���Ϸ������Ʊ����
        mySendByteBuf(outputBuffer, outputLen);        
      }          
    } else {
      myprintf("coord recv end_device message check failed\n");
    }
#endif    
    
#ifdef END_DEVICE //�ն˽������ݴ���
    if(check) {
      if(cmd == TO_PAY_COIN) { // ���������͵�ģ��Ͷ�Ҷ���
        NeedCoinDelivered = outputBuffer[4];
        COIN_SIMULATE_PIN = 1; // ��ʼ�ߵ�ƽ
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SIMULATE_COIN_MSG_EVT, COIN_SIGNAL_HALF_TIME); // ������ʱ���¼�                
        
        EndDeviceSendAfMessageByCmd(ENDDEVICE_REPLY_RECVED_COIN); //�ظ��Ѿ��յ�Ͷ������
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
  Э�����������ܺ���
*/
#ifdef NEED_TEST_DIStANCE
  uint16 totalSendTestCount = 0;
  uint16 sendTestCount = 0;
#endif  
void SampleApp_SendHeartBeatMessageCoor(void) {  
  // ���ն��б���Ԫ������ʱ����1�������⵽��ʱ��IEEE���͵���λ��
  decAllListEPLeftSec(LL); 
  HalLedBlink(HAL_LED_1, 1, 50, 500);
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
  �ն��������������ڶ�ʱ��Э���������������������ն˻������
*/
void EndDeviceSendHeartBeat(void) {
  EndDeviceSendAfMessageByCmd(ENDDEVICE_HEARTBEAT);
  
  if(startRewardInterrupt == 1 && (osal_getClock() - lastRecvRewardSecTime) >= 2) { // ��������˽����жϣ��ҽ����������뵱ǰʱ�����ָ��ʱ������Ϊ��һ�ν�������
    if(rewardCount == 0) {
      startRewardInterrupt = 0;
      return;
    }    
    EndDeviceSendAfMessageByCmd(ENDDEVICE_REPORT_GAME_REWARD); // ���ͽ�����Ʊ������Э����
    myprintf("Upload reward result count, rewardCount = %d\n", rewardCount);    
    rewardCount = 0; // ��0
    startRewardInterrupt = 0;    
  }
}

/**
  �ն����ڷ��͸�����Ϣ��Э����
*/
void EndDeviceSendAfMessageByCmd(uint8 cmd) {
  uint8 strBuf[SEND_BUF_SIZE] = {0};
  uint8 strBufLen = 0;
  uint8 outputBuf[SEND_BUF_SIZE] = {0};
  uint8 outputBufLen = 0;
  
  if(cmd == ENDDEVICE_NETWORK_READY) {
    uint16 shortAddr = NLME_GetShortAddr(); // ��ȡ���������豸��ַ����ȡ���豸�����ַ��uint16 NLME_GetCoordShortAddr( void );  
    strBuf[0] = MACHINE_ID; // ���ն˱��
    strBuf[1] = HI_UINT16(shortAddr); // ��ŵ�ַ�ĸ�8λ
    strBuf[2] = LO_UINT16(shortAddr); // ��ŵ�8λ
    osal_memcpy(&strBuf[3], NLME_GetExtAddr(), 8); // ����IEEE��ַ  
    strBufLen = 11;
  } else if(cmd == ENDDEVICE_HEARTBEAT) {
    strBuf[0] = MACHINE_ID; // ���ն˱��
    strBufLen = 1;
  } else if(cmd == ENDDEVICE_REPLY_RECVED_COIN) { // �ն��豸�ظ��Ѿ��յ����
    strBuf[0] = MACHINE_ID; // ���ն˱��
    strBufLen = 1;
  } else if(cmd == ENDDEVICE_REPORT_GAME_REWARD) { // �ն��ϴ�������Ʊ����
    strBuf[0] = MACHINE_ID; // ���ն˱��
    strBuf[1] = rewardCount;
    strBufLen = 2;
  }

  // ��������Э���װ
  encodeData(cmd, strBuf, strBufLen, outputBuf, &outputBufLen);
  if ( AF_DataRequest( &SampleApp_TxAddr,                  // ���͵�Ŀ�ĵ�ַ+�˵��ַ+����ģʽ
                      (endPointDesc_t *)&SampleApp_epDesc, // Դ�ն˵�����
                      SAMPLEAPP_PERIODIC_CLUSTERID,        // ��profileָ������Ч�ļ�Ⱥ��
                      outputBufLen,                                  // �������ݳ���
                      outputBuf,                              // �������ݻ�����
                      &SampleApp_TransID,                  // ��Ϣ����ID
                      AF_DISCV_ROUTE,                      // ��Чλ����ķ���ѡ��
                      AF_DEFAULT_RADIUS ) != afStatus_SUCCESS ) // ����������ͨ������ΪAF_DEFAULT_RADIUS
  {
    myprintf("send cmd = 0x%x failed\n", cmd); 
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
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7'); // +7����ת��ɴ�д�ַ�ABCDEF
    ch = *xad & 0x0F;
    lcd_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
  }
  lcd_buf[2*2] = '\0';
  HalLcdWriteString( (char*)lcd_buf, lcdLineNum);
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
    } else {
      myprintf("SampleApp_SendPeriodicMessage() failed\n");
    }
  }
}


/*********************************************************************
* Event Generation Functions
* @brief   �����¼�������
*/
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
#ifdef ZDO_COORDINATOR       //��Э����S1�ŷ�����  
  if ( keys & HAL_KEY_SW_6 ){ // ����S1,�Թ㲥��ʽ������    
    SampleApp_SendPeriodicMessage(1);
  } else if(keys & HAL_KEY_SW_1) { // ����S2
    //SampleApp_SendPeriodicMessage(2);
  }
#endif
}