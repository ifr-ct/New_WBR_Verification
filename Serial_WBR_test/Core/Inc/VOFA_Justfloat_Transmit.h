/*
 * vofa.h
 * 
 * Improved VOFA+ communication module for STM32
 * 
 * Author: CHENSIRUI
 * Date  : 2025-11-05
 */

#ifndef __VOFA_H__
#define __VOFA_H__

#include "stm32f4xx_hal.h"
#include "ifr_usart.h"
#include "Chassis_Controller.h"
#define VOFA_MAX_CHANNEL   10   // 最多通道数

extern UART_HandleTypeDef huart1;
void VOFA_Init(UART_HandleTypeDef *huart);
void VOFA_Register(float *addr);
void VOFA_Send(void);
void USART_Analysis_Function(uint8_t *pData, uint8_t len);
void VOFA_Send_Data();


/*----------example-------------
//init
    Upper_Computer_Init(&pitch_filter);
    Upper_Computer_Init(&car_V);
    Upper_Computer_Init(&rtU.L_ia);
    Upper_Computer_Init(&rtU.L_ib);
    Upper_Computer_Init(&rtU.L_ic);
    Upper_Computer_Init(&Roll_delta);
//loop transmit
    Upper_Computer_Show_Wave();
-----------------end-------------*/
extern void Upper_Computer_Show_Wave(void);
extern void Upper_Computer_Init(float*);//maximum 10 channels;can modify

#endif /* INC_VOFA_H_ */

