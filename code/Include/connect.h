#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include <string.h>
#define datasize (uint8_t)9 // uint8_t  ��Ϊunsigned char ��λ
#define HMIUart huart2
typedef struct UartRecieve
{
    uint8_t Rxbuf;
    uint8_t Uart_Data[datasize];
    uint8_t Flag;    // 0xff��ʾ������ɣ�0x0e��ʾ�ȴ����գ�0xee��ʾ���ճ�������������������չ���ݴ�������
    uint8_t Pointer; // ����ָ��
} UartBuff;
#define BufLen 44 
//���������ݽṹ��
//��������ʹ��DMA�����ж�
typedef struct JY901_DMA{
	uint8_t DataBuf[BufLen];
	float yaw;
	float wz;
	float wz_last;
}JY901;

extern JY901 jy901_ins;
extern int state_val;
extern UartBuff UartBuff_Ins;;
extern int mission_select;
void UartDateHandler(UartBuff *UartBuff1);
void HMISends(char *buf1);
void HMISendb(uint8_t k);
void get_yaw_az(JY901 *j);
#endif
