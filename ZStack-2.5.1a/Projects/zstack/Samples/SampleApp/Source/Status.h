#ifndef STATUS_H
#define STATUS_H

/* ״̬�� */
#define	TRUE		1			//�� 
#define	FALSE		0			//��
#define	true		1			//�� 
#define	false		0			//��
#define YES             1			//��
#define NO              0			//�� 
#define	OK		1			//ͨ��
#define	ERROR		0			//����
#define	INFEASIBLE	-1			//������

#ifndef _MATH_H_ 				//ϵͳ�����д�״̬�붨�壬Ҫ�����ͻ 
#define	OVERFLOW	-2			//��ջ����
#define UNDERFLOW	-3			//��ջ����
#endif 

#ifndef NULL
#define NULL ((void*)0)
#endif

/* ״̬��ʶ������ */
typedef int Status;

#endif
