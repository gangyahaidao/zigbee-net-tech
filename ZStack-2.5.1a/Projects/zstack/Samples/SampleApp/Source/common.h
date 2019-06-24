#ifndef COMMON_H
#define COMMON_H

#include "hal_types.h"
#include "OSAL.h"

#define SEND_BUF_SIZE 32
#define SERIAL_BUFFER_SIZE 32

#define MACHINE_ID 0x01 // 终端设备的机器编号

#define HEAD_BYTE 0x7E

#define TO_PAY_COIN 0x50 // 线上进行投币操作：设备编号 + 投币数量byte
#define TEST_DISTANCE_FLASH_LED 0x51 // 用于闪灯测试距离稳定性

#define ENDDEVICE_NETWORK_READY 0x80 // 终端入网命令:machineId + addrH + addrL + IEEE8个byte
#define ENDDEVICE_HEARTBEAT 0x81 // 终端心跳命令：machineId
#define ENDDEVICE_REPLY_RECVED_COIN 0x82 // 终端回复已经收到金币命令：machineId
#define ENDDEVICE_REPORT_GAME_REWARD 0x83 // 终端上传奖励彩票个数：machineId + 彩票个数byte

#define COORD_REPORT_OFFLINE_ENDDEVICE 0x90 // 协调器上传掉线终端信息：machineId

uint8* re_replace_data(uint8* buffer, uint8 length, uint8* destArr, uint8* destLengthPtr);
bool check_xor(uint8* destArr, uint8 length);

void encodeData(uint8 cmd, uint8* content, uint8 contentLen, uint8* outputBuf, uint8* outputLen);

#endif