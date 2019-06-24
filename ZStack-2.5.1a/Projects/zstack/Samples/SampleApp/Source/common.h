#ifndef COMMON_H
#define COMMON_H

#include "hal_types.h"
#include "OSAL.h"

#define SEND_BUF_SIZE 32
#define SERIAL_BUFFER_SIZE 32

#define MACHINE_ID 0x01 // �ն��豸�Ļ������

#define HEAD_BYTE 0x7E

#define TO_PAY_COIN 0x50 // ���Ͻ���Ͷ�Ҳ������豸��� + Ͷ������byte
#define TEST_DISTANCE_FLASH_LED 0x51 // �������Ʋ��Ծ����ȶ���

#define ENDDEVICE_NETWORK_READY 0x80 // �ն���������:machineId + addrH + addrL + IEEE8��byte
#define ENDDEVICE_HEARTBEAT 0x81 // �ն��������machineId
#define ENDDEVICE_REPLY_RECVED_COIN 0x82 // �ն˻ظ��Ѿ��յ�������machineId
#define ENDDEVICE_REPORT_GAME_REWARD 0x83 // �ն��ϴ�������Ʊ������machineId + ��Ʊ����byte

#define COORD_REPORT_OFFLINE_ENDDEVICE 0x90 // Э�����ϴ������ն���Ϣ��machineId

uint8* re_replace_data(uint8* buffer, uint8 length, uint8* destArr, uint8* destLengthPtr);
bool check_xor(uint8* destArr, uint8 length);

void encodeData(uint8 cmd, uint8* content, uint8 contentLen, uint8* outputBuf, uint8* outputLen);

#endif