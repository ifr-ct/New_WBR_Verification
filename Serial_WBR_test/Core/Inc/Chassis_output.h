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
		float Euler_[3];
		float d_Euler_[3];
		float TrueEular_[3];
		float True_d_Eular_[3];
		arm_matrix_instance_f32 E_mat;
		arm_matrix_instance_f32 d_E_mat;
		arm_matrix_instance_f32 SPIN_MATRIX;
		arm_matrix_instance_f32 True_E_mat;
		arm_matrix_instance_f32 True_d_E_mat;
	
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
void Euler_Update();
float trans(float angle);
void TIM2_Callback();
#endif
