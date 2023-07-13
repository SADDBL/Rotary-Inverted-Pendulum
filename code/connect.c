#include "control.h"
#include "connect.h"
#include "usart.h"
int mission_select=10;
/**
  * @brief  �����ַ���
  * @param  *buf1 �ַ���
  * @retval None
  */
void HMISends(char *buf1){
	int len = strlen(buf1);
	HAL_UART_Transmit(&HMIUart,(uint8_t*)buf1,len,0xffff);
	HMISendb(0xff);
}

/**
  * @brief  ����3����ͬ�ֽ�
  * @param  k �ַ�
  * @retval None
  */
void HMISendb(uint8_t k){
	uint8_t str[3];
	str[0] = k;
	str[1] = k;
	str[2] = k;
	HAL_UART_Transmit(&HMIUart,str,3,0xffff);
}

UartBuff UartBuff_Ins;
void UartDateHandler(UartBuff *UartBuff1)
{
    if ((UartBuff1->Flag != 0x80) && (UartBuff1->Flag != 0xee))
    {
        if (UartBuff1->Flag == 0x40)
        {
            /***** ���������ݴ��� *****/
            if (UartBuff1->Pointer < 2)
            {
                UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
            }
            if (UartBuff1->Pointer == 2)
            {
                UartBuff1->Flag = 0xff;
                // ����������ݴ������������
								mission_select = UartBuff1->Uart_Data[1]-'0';
								state_val=0;
                // ������ֹ
                UartBuff1->Flag = 0x0e;
                UartBuff1->Pointer = 0;
                memset(UartBuff1->Uart_Data, 0, datasize);
            }
            /*****  *****/

        }
        // �ȴ���������ͷ
        else if (UartBuff1->Flag == 0x0e)
        {
            // ����ͷ�жϣ�ȷ��������������
            if (UartBuff1->Rxbuf == 'h')
            {
                UartBuff1->Flag = 0x40;
                UartBuff1->Uart_Data[UartBuff1->Pointer++] = UartBuff1->Rxbuf;
            }
            else
            {
                UartBuff1->Flag = 0xee;
            }
        }
    }
}

// ���ڻص�����ģ��
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        UartDateHandler(&UartBuff_Ins);
        // ���ϴ���
        if (UartBuff_Ins.Flag == 0xee)
        {
            UartBuff_Ins.Flag = 0;
            UartBuff_Ins.Pointer = 0;
        }
    }

    HAL_UART_Receive_IT(&huart2, &(UartBuff_Ins.Rxbuf), 1);
}

JY901 jy901_ins;
//DMA��������������
void USART3_IRQHandler(void)
{
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))//idle��־����λ
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);//�����־λ
		HAL_UART_DMAStop(&huart3); //  ֹͣDMA���䣬��ֹ
	}
	HAL_UART_Receive_DMA(&huart3,jy901_ins.DataBuf,BufLen);//���´�DMA����
  HAL_UART_IRQHandler(&huart3);
}

void get_yaw_az(JY901 *j){
	j->yaw = ((short)(j->DataBuf[40]<<8|j->DataBuf[39]))*180/32768.0;
	j->az = ((short)(j->DataBuf[18]<<8|j->DataBuf[17]))*156.8f/32768.0;
}

