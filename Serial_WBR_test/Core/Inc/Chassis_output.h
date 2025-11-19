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

static inline float deg2rad(float d) { return d * 0.0174533f; }
static inline float rad2deg(float r) { return r * 57.2958f; }
/*
	ÍÓÂÝÒÇÐý×ª¾ØÕó
	|-1 0 0|
	| 0 0 1|
	|	0 1 0|
*/
const float IMU_SPIN_MATRIX[9] = {-1.0f , 0.0f , 0.0f , 0.0f , 0.0f , 1.0f , 0.0f , 1.0f , 0.0f };


class Chassis_Motor_output
{
	public:
		Chassis_Motor_output();
		void Chassis_Motor_Setouttput();
	
		float Turn_T;
		float yaw_error;

		float Theta_error;
		float d_Theta_error;
		float AntiSplit_Tp;
		float Tar_Yaw;
		float Now_Yaw;
		
		float Tar_X;
		float Tar_V;
		
		IFR_PID Anti_Split_PID;
		IFR_PID Yaw_PID;
		
		float Anti_Split_Cal();
		float Heading_Angle_Cal();
	
};
extern IFR_RS_Motor L_motor_1;
extern IFR_RS_Motor L_motor_4;
extern IFR_RS_Motor R_motor_1;
extern IFR_RS_Motor R_motor_4;
extern TIM_HandleTypeDef htim2;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void Chassis_Init();
void Update(Single_Leg_Typedef * leg_info);
//void Euler_Update();
float trans(float angle);
void TIM2_Callback();
#endif
