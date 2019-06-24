/*******************************************
 *                                         *
 *                                         *
 * �ļ���: SinglyLinkedList.c              *
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
����ĳ��ָ��������ŵ��豸�Ƿ����б���
���ڣ�0
�����ڣ�-1
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

Status InitList_L(LinkList *L)                  // ��ʼ��������L
{
	(*L) = (LinkList)osal_msg_allocate(sizeof(LNode)); // osal_msg_deallocate
	if(!(*L)) {
          return (OVERFLOW);
        }        
	(*L)->next = NULL;	

	return OK;
}

void DestroyList_L(LinkList *L)			//�������н�� 
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
  ���ĳ��IEEE��ַ�ն��Ƿ����б��У�����Ѿ����б��з���λ��
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
�����ն˱�Ų����ն��Ƿ����б���
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
  ����ָ��λ��Ԫ�ص�ʱ��״ֵ̬
*/
void updateListEleStatus(LinkList L, int i, int initLeftSec) {
  int j = 1;
  LinkList p = L->next;
  
  while(p && j<i)  //p��Ϊ���һ�δ����i��
  {
    j++;
    p = p->next;
  }
  p->data.leftSecCount = initLeftSec;
}

/**
  �����ն��б������нڵ�ʣ��ʱ��-1�����С��0������0
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
      mySendByteBuf(outputBuf, outputBufLen); // ���ն˵�����Ϣ���͵���ݮ��
      myprintf("one end device offline, machineId = 0x%x\n", p->data.machineId);
    }
    // myprintf("i = %d, leftSecCount = %d\n", i, p->data.leftSecCount);
    p = p->next;
  }
}

/*�T�T�T�T�[
�U �㷨2.8�U 
�^�T�T�T�T*/
Status GetElem_L(LinkList L, int i, LElemType_L *e)
{
	int j = 1;
	LinkList p = L->next;
	
	while(p && j<i)						//p��Ϊ���һ�δ����i��
	{
		j++;
		p = p->next;
	}

	if(!p || j>i)						//��i��Ԫ�ز����� 
		return ERROR;

	*e = p->data;

	return OK; 
}

/**
  ���б��в���һ���ն��ϴ��ĵ�ַ
  ������
    �����λ��
    �̵�ַ���ֽ�
    �̵�ַ���ֽ�
    ieee��ַ8���ֽ�
    �������Ĵ�����Э�������ÿ�μ�1���ն��ϴ�ÿ�μ�1
*/
Status ListInsert_L(LinkList L, int i, uint8 addrH, uint8 addrL, uint8* ieeeAddrP, int8 delaySec)
{
	LinkList p, s;
	int j = 0;	
	p = L;
	
	while(p && j<i-1)					//Ѱ�ҵ�i-1����� 
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

/*�T�T�T�T�T�[
�U �㷨2.10 �U 
�^�T�T�T�T�T*/
Status ListDelete_L(LinkList L, int i)
{
	LinkList pre, p; 
	int j;

	pre = L;
	j = 1; 

	while(pre->next && j<i)			//Ѱ�ҵ�i����㣬����preָ����ǰ�� 
	{
		pre = pre->next;
		++j;
	}
	
	if(!pre->next || j>i)			//ɾ��λ�ò�����
		return ERROR;

	p = pre->next;
	pre->next = p->next;
	osal_msg_deallocate((uint8 *)p);

	return OK; 
}


#endif 
