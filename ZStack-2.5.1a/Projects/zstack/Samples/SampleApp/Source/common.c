#include "common.h"


/**
 * 解析服务器发送过来的数据，步骤一：数据替换
*/
uint8* re_replace_data(uint8* buffer, uint8 length, uint8* destArr, uint8* destLengthPtr){
    int i = 0;
    int j = 0;
    for(i = 0; i < length; i++) {
        if(buffer[i] == 0x7D && buffer[i+1] == 0x02) {
            destArr[j++] = 0x7E;
            i++;
        } else if(buffer[i] == 0x7D && buffer[i+1] == 0x01) {
            destArr[j++] = 0x7D;
            i++;
        } else {
            destArr[j++] = buffer[i];
        }
    }
    *destLengthPtr = j; // 替换之后的长度
    return destArr;
}
/**
 * 解析服务器发送过来的数据，步骤二：数据校验
*/
bool check_xor(uint8* destArr, uint8 length) {
    uint8 XOR = destArr[length-2];
    uint8 contentLen = destArr[3];

    if(contentLen > 0) {
        uint8 calculate = destArr[1];
        int i = 0;
        for(i = 2; i < length-2; i++) {
            calculate = calculate ^ destArr[i];
        }
        if(XOR != calculate) {
            return FALSE;
        }
    }
    return TRUE;
}

//**************************************************************************//
/**
 * 按照协议封装数据
 * Data Format: 0x7E + 1byteCmd + encType + 1byteLength + data[] + xor + 0x7E
*/
void encodeData(uint8 cmd, uint8* content, uint8 contentLen, uint8* outputBuf, uint8* outputLen) {
    uint8 sendBuf[SEND_BUF_SIZE] = {0};
    int index = 0;

    sendBuf[index++] = 0x7E;
    sendBuf[index++] = cmd;
    sendBuf[index++] = 0x0; // 加密方式0x0表示不加密
    sendBuf[index++] = contentLen; // 消息体长度
    if(contentLen > 0) { // 如果消息体中有内容才进行组装
        osal_memcpy(sendBuf+index, content, contentLen);
        index += contentLen;
    }    
    // calculate xor
    uint8 XOR = sendBuf[1];
    int i = 0;
    for(i = 2; i < index; i++) {
        XOR ^= sendBuf[i];
    }
    sendBuf[index++] = XOR;
    sendBuf[index++] = 0x7E;
    uint8 tmpOutLen = 0;
    outputBuf[tmpOutLen++] = 0x7E;
    for(i = 1; i < index-1; i++) {
        if(sendBuf[i] == 0x7E) {
            outputBuf[tmpOutLen++] = 0x7D;
            outputBuf[tmpOutLen++] = 0x02;
        } else if(sendBuf[i] == 0x7D) {
            outputBuf[tmpOutLen++] = 0x7D;
            outputBuf[tmpOutLen++] = 0x01;
        } else {
            outputBuf[tmpOutLen++] = sendBuf[i];
        }
    }
    outputBuf[tmpOutLen] = 0x7E;
    *outputLen = tmpOutLen;
}