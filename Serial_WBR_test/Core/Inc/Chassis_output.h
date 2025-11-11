#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H
#include "main.h"
#include "ifr_can.h"
#include "ifr_motor.h"
#include "ifr_robstride.h"
#include "ifr_tim.h"
#include "Chassis_Controller.h"
#include "VOFA_Justfloat_Transmit.h"
#include "VMC_Cal.h"
#include "LQR_Cal.h"
extern IFR_RS_Motor L_motor_1;
extern IFR_RS_Motor L_motor_4;
extern IFR_RS_Motor R_motor_1;
extern IFR_RS_Motor R_motor_4;

class Chassis_Motor_output
{
	public:
	Chassis_Motor_output();
	
	
};
extern TIM_HandleTypeDef htim2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void Chassis_Init();
void Update(Single_Leg_Typedef * leg_info);
void TIM2_Callback();
#endif
