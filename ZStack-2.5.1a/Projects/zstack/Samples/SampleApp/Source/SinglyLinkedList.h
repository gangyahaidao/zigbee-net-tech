/*******************************************
 *                                         *
 *                                         *
 * �ļ���: SinglyLinkedList.h              *
 *                                         *
 * ��  ��: ��������ز����б�              *
 *                                         *
 *******************************************/

#ifndef SINGLYLINKEDLIST_H
#define SINGLYLINKEDLIST_H

#include "hal_types.h"
#include "Status.h"



typedef struct {
  uint8 machineId; // �ն��豸���
  uint8 shortAddrH;
  uint8 shortAddrL;
  uint8 IEEEArr[8]; // �ն˹̶���ַ��������Կ���ɾ������Ϣ
  int8  leftSecCount; // ʣ�������
} LElemType_L;

/* ������ṹ�� */
typedef struct LNode
{
  LElemType_L data;
  struct LNode* next;
}LNode;
typedef LNode* LinkList;		//ָ���������ָ�� 

/**
����ĳ��ָ��������ŵ��豸�Ƿ����б���
���ڣ�0
�����ڣ�-1
*/
int findEndDeviceShortAddrByMachineid(LinkList L, uint8 machineId, uint8* addr);

/**
  ���ĳ��IEEE��ַ�ն��Ƿ����б��У�����Ѿ����б��з���λ��
*/
int checkEPIsInListByIEEE(LinkList L, uint8* ieeeAddrP);
int checkEPIsInListByMachineId(LinkList L, uint8 machineId);

/**
  ����ָ��λ��Ԫ�ص�ʱ��״ֵ̬
*/
void updateListEleStatus(LinkList L, int i, int initLeftSec);

/**
  �����ն��б������нڵ�ʣ��ʱ��-1�����С��0������0
*/
void  decAllListEPLeftSec(LinkList L);

/* ��������ͷ��㣩�����б� */ 
Status InitList_L(LinkList *L);
/*����������������������
��(01)��ʼ��������L�� ��
����������������������*/

void DestroyList_L(LinkList *L);
/*����������������������������������������
��(03)���ٵ�����L����ͬͨ���һ�����١� ��
����������������������������������������*/

int ListLength_L(LinkList L);
/*��������������������������
��(05)���ص�����LԪ�ظ��� ��
��������������������������*/ 

Status GetElem_L(LinkList L, int i, LElemType_L *e);
/*����������������������������������������
��(06)�㷨2.8����e���յ�����L�е�i��Ԫ�ة�
����������������������������������������*/ 

Status ListInsert_L(LinkList L, int i, uint8 machineId, uint8 addrH, uint8 addrL, uint8* ieeeAddrP, int8 delaySec);
/*��������������������������������������������
��(10)�㷨2.9���ڵ�����L��i��λ��֮ǰ����e����
��������������������������������������������*/

Status ListDelete_L(LinkList L, int i);
/*������������������������������������������������������
��(11)�㷨2.10��ɾ��������L��i��λ�õ�ֵ������e���ա� ��
������������������������������������������������������*/ 

#endif
