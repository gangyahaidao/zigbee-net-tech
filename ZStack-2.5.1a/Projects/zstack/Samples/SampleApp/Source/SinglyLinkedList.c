/*******************************************
 *                                         *
 *                                         *
 * 文件名: SinglyLinkedList.c              *
 *                                         *
 *                                         *
 *******************************************/

#ifndef SINGLYLINKEDLIST_C
#define SINGLYLINKEDLIST_C

#include "SinglyLinkedList.h" 		
#include "OSAL.h"
#include "MT_UART.h"
#include "common.h"

/**
查找某个指定机器编号的设备是否在列表中
存在：0
不存在：-1
*/
int findEndDeviceShortAddrByMachineid(LinkList L, uint8 machineId, uint8* addr) {
  int i = 0;
  LinkList p = L->next;
  while(p) {
    i++;
    if(p->data.machineId == machineId) {
      addr[0] = p->data.shortAddrH;
      addr[0] = p->data.shortAddrL;
      return 0;
    }
    p = p->next;
  }
  
  return -1;
}

Status InitList_L(LinkList *L)                  // 初始化单链表L
{
	(*L) = (LinkList)osal_msg_allocate(sizeof(LNode)); // osal_msg_deallocate
	if(!(*L)) {
          return (OVERFLOW);
        }        
	(*L)->next = NULL;	

	return OK;
}

void DestroyList_L(LinkList *L)			//销毁所有结点 
{
	LinkList p = *L;	

	while(p)
	{
		p = (*L)->next;
		osal_msg_deallocate((uint8*)*L);
		(*L) = p;
	}
}

int ListLength_L(LinkList L)
{
	LinkList p;
	int i = 0;
	
	if(L)
	{
          i = 0;
          p = L->next;
          while(p)
          {
            i++;
            p = p->next;
          }		
	}
	
	return i;
}

/**
  检查某个IEEE地址终端是否在列表中，如果已经在列表中返回位置
*/
int checkEPIsInListByIEEE(LinkList L, uint8* ieeeAddrP) {
  int i = 0;
  LinkList p = L->next;
  while(p) {
    i++;
    if(osal_memcmp(p->data.IEEEArr, ieeeAddrP, 8) == TRUE) {
      return i;
    }
    p = p->next;
  }
  
  return -1;
}

/**
根据终端编号查找终端是否在列表中
*/
int checkEPIsInListByMachineId(LinkList L, uint8 machineId) {
  int i = 0;
  LinkList p = L->next;
  while(p) {
    i++;
    if(p->data.machineId == machineId) {
      return i;
    }
    p = p->next;
  }
  
  return -1;
}

/**
  更新指定位置元素的时间状态值
*/
void updateListEleStatus(LinkList L, int i, int initLeftSec) {
  int j = 1;
  LinkList p = L->next;
  
  while(p && j<i)  //p不为空且还未到达i处
  {
    j++;
    p = p->next;
  }
  p->data.leftSecCount = initLeftSec;
}

/**
  遍历终端列表，将所有节点剩余时间-1，如果小于0，则置0
*/
void  decAllListEPLeftSec(LinkList L) {
  uint8 strBuf[SEND_BUF_SIZE] = {0};
  uint8 strBufLen = 0;
  uint8 outputBuf[SEND_BUF_SIZE] = {0};
  uint8 outputBufLen = 0;
  
  LinkList p = L->next;
  int i = 0;
  while(p) {
    i++;
    p->data.leftSecCount -= 1;
    if(p->data.leftSecCount < 0) {
      p->data.leftSecCount = 0;
      
      strBuf[0] = p->data.machineId;  
      strBufLen = 0x1;
      encodeData(COORD_REPORT_OFFLINE_ENDDEVICE, strBuf, strBufLen, outputBuf, &outputBufLen);
      mySendByteBuf(outputBuf, outputBufLen); // 将终端掉线信息发送到树莓派
      myprintf("one end device offline, machineId = 0x%x\n", p->data.machineId);
    }
    // myprintf("i = %d, leftSecCount = %d\n", i, p->data.leftSecCount);
    p = p->next;
  }
}

/*════╗
║ 算法2.8║ 
╚════*/
Status GetElem_L(LinkList L, int i, LElemType_L *e)
{
	int j = 1;
	LinkList p = L->next;
	
	while(p && j<i)						//p不为空且还未到达i处
	{
		j++;
		p = p->next;
	}

	if(!p || j>i)						//第i个元素不存在 
		return ERROR;

	*e = p->data;

	return OK; 
}

/**
  向列表中插入一个终端上传的地址
  参数：
    插入的位置
    短地址高字节
    短地址低字节
    ieee地址8个字节
    心跳检测的次数，协调器检查每次减1，终端上传每次加1
*/
Status ListInsert_L(LinkList L, int i, uint8 addrH, uint8 addrL, uint8* ieeeAddrP, int8 delaySec)
{
	LinkList p, s;
	int j = 0;	
	p = L;
	
	while(p && j<i-1)					//寻找第i-1个结点 
	{
		p = p->next;
		++j;
	}	
	if(!p || j>i-1)
        {
		return ERROR;
        }

	s = (LinkList)osal_msg_allocate(sizeof(LNode));
	if(!s)
        {
		return (OVERFLOW);
        }
	s->data.shortAddrH = addrH;
        s->data.shortAddrL = addrL;
        osal_memcpy(s->data.IEEEArr, ieeeAddrP, 8);
        s->data.leftSecCount = delaySec;
        
	s->next = p->next;
	p->next = s;

	return OK;
}

/*═════╗
║ 算法2.10 ║ 
╚═════*/
Status ListDelete_L(LinkList L, int i)
{
	LinkList pre, p; 
	int j;

	pre = L;
	j = 1; 

	while(pre->next && j<i)			//寻找第i个结点，并令pre指向其前驱 
	{
		pre = pre->next;
		++j;
	}
	
	if(!pre->next || j>i)			//删除位置不合理
		return ERROR;

	p = pre->next;
	pre->next = p->next;
	osal_msg_deallocate((uint8 *)p);

	return OK; 
}


#endif 
