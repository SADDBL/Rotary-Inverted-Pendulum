#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "main.h"
#include <string.h>
#define datasize (uint8_t)9 // uint8_t  即为unsigned char 八位
#define HMIUart huart2
typedef struct UartRecieve
{
    uint8_t Rxbuf;
    uint8_t Uart_Data[datasize];
    uint8_t Flag;    // 0xff表示接收完成，0x0e表示等待接收，0xee表示接收出错，定义其他可用于扩展数据处理类型
    uint8_t Pointer; // 数组指针
} UartBuff;
#define BufLen 44 
//陀螺仪数据结构体
//接收数据使用DMA空闲中断
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
