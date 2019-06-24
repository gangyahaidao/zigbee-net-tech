/*******************************************
 *                                         *
 *                                         *
 * 文件名: SinglyLinkedList.h              *
 *                                         *
 * 内  容: 单链表相关操作列表              *
 *                                         *
 *******************************************/

#ifndef SINGLYLINKEDLIST_H
#define SINGLYLINKEDLIST_H

#include "hal_types.h"
#include "Status.h"



typedef struct {
  uint8 machineId; // 终端设备编号
  uint8 shortAddrH;
  uint8 shortAddrL;
  uint8 IEEEArr[8]; // 终端固定地址，后面可以考虑删除此信息
  int8  leftSecCount; // 剩余的秒数
} LElemType_L;

/* 单链表结构体 */
typedef struct LNode
{
  LElemType_L data;
  struct LNode* next;
}LNode;
typedef LNode* LinkList;		//指向单链表结点的指针 

/**
查找某个指定机器编号的设备是否在列表中
存在：0
不存在：-1
*/
int findEndDeviceShortAddrByMachineid(LinkList L, uint8 machineId, uint8* addr);

/**
  检查某个IEEE地址终端是否在列表中，如果已经在列表中返回位置
*/
int checkEPIsInListByIEEE(LinkList L, uint8* ieeeAddrP);
int checkEPIsInListByMachineId(LinkList L, uint8 machineId);

/**
  更新指定位置元素的时间状态值
*/
void updateListEleStatus(LinkList L, int i, int initLeftSec);

/**
  遍历终端列表，将所有节点剩余时间-1，如果小于0，则置0
*/
void  decAllListEPLeftSec(LinkList L);

/* 单链表（带头结点）函数列表 */ 
Status InitList_L(LinkList *L);
/*━━━━━━━━━━┓
┃(01)初始化单链表L。 ┃
┗━━━━━━━━━━*/

void DestroyList_L(LinkList *L);
/*━━━━━━━━━━━━━━━━━━━┓
┃(03)销毁单链表L，连同通结点一起销毁。 ┃
┗━━━━━━━━━━━━━━━━━━━*/

int ListLength_L(LinkList L);
/*━━━━━━━━━━━━┓
┃(05)返回单链表L元素个数 ┃
┗━━━━━━━━━━━━*/ 

Status GetElem_L(LinkList L, int i, LElemType_L *e);
/*━━━━━━━━━━━━━━━━━━━┓
┃(06)算法2.8：用e接收单链表L中第i个元素┃
┗━━━━━━━━━━━━━━━━━━━*/ 

Status ListInsert_L(LinkList L, int i, uint8 machineId, uint8 addrH, uint8 addrL, uint8* ieeeAddrP, int8 delaySec);
/*━━━━━━━━━━━━━━━━━━━━━┓
┃(10)算法2.9：在单链表L第i个位置之前插入e。┃
┗━━━━━━━━━━━━━━━━━━━━━*/

Status ListDelete_L(LinkList L, int i);
/*━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃(11)算法2.10：删除单链表L第i个位置的值，并用e接收。 ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━*/ 

#endif
