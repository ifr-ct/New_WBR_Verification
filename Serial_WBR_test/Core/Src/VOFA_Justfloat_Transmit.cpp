/*
 * vofa.c
 * 
 * Improved VOFA+ communication module for STM32
 * 
 * Author: WML & ChatGPT
 * Date  : 2025-11-05
 */

#include "VOFA_Justfloat_Transmit.h"
IFR_USART_ClassDef usart1;


float a = 1;
/**
 * @brief  初始化 VOFA 模块
 * @param  huart: 指向 UART 句柄的指针（例如 &huart3）
 */
void VOFA_Init(UART_HandleTypeDef *huart)
{
//    vofa_huart = huart;
//    vofa_DataNum = 0;
		usart1.Init(&huart1);
		usart1.Start_DMA_Receive();
		Upper_Computer_Init(&a);
}


void USART_Analysis_Function(uint8_t *pData, uint8_t len){}

/*
 * vofa.c
 *
 *  Created on: Nov 28, 2024
 *      Author: WML
 */
#include "stm32f4xx_hal.h"

#define MAX_CHANNEL 10

#define BYTE0(dwTemp)       (*(char *)(dwTemp))
#define BYTE1(dwTemp)       (*((char *)(dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(dwTemp) + 3))

float *UserData[MAX_CHANNEL]={0};//only transmit float
unsigned char Data_Number = 0;

void Upper_Computer_Init(float* addr)
{
    if(Data_Number < MAX_CHANNEL)UserData[Data_Number++]=addr;
}

unsigned char data_to_send[4*MAX_CHANNEL+4] = {0}; //使用DMA发送数据时，由于指向的是地址，需要是全局地址，否则真正发送数据时数据已经没了!!!!!!!!!!

void Upper_Computer_Show_Wave(void)
{
	unsigned char cnt = 0;

	for(int i=0;i<Data_Number;i++)
	{
		data_to_send[cnt++] = BYTE0(UserData[i]);
		data_to_send[cnt++] = BYTE1(UserData[i]);
		data_to_send[cnt++] = BYTE2(UserData[i]);
		data_to_send[cnt++] = BYTE3(UserData[i]);
	}

	data_to_send[cnt++] = 0x00;
	data_to_send[cnt++] = 0x00;
	data_to_send[cnt++] = 0x80;
	data_to_send[cnt++] = 0x7F;
	//-------------user change-----------------
	usart1.TransmitData(data_to_send , cnt);
//	extern UART_HandleTypeDef huart3;
//	while((&huart3)->gState != HAL_UART_STATE_READY); //如果上一次包没有发送完成，会导致这个包被丢弃
//	HAL_UART_Transmit_DMA(&huart3, data_to_send, cnt);
//     HAL_UART_Transmit(&huart3, data_to_send, cnt,100);
	//----------------end----------------------
}