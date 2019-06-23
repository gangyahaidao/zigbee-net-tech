#ifndef STATUS_H
#define STATUS_H

/* 状态码 */
#define	TRUE		1			//真 
#define	FALSE		0			//假
#define	true		1			//真 
#define	false		0			//假
#define YES             1			//是
#define NO              0			//否 
#define	OK		1			//通过
#define	ERROR		0			//错误
#define	INFEASIBLE	-1			//不可行

#ifndef _MATH_H_ 				//系统中已有此状态码定义，要避免冲突 
#define	OVERFLOW	-2			//堆栈上溢
#define UNDERFLOW	-3			//堆栈下溢
#endif 

#ifndef NULL
#define NULL ((void*)0)
#endif

/* 状态码识别类型 */
typedef int Status;

#endif
