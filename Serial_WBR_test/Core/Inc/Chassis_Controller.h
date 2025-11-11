#ifndef __CHASSIS_CONTROLLER_H
#define __CHASSIS_CONTROLLER_H
#include "ifr_pid.h"
#include "main.h"
//常量设置
const float K = 2.1904f;
const float l1 = 0.105f;
const float l2 = 0.125f;
const float l3 = 0.125f;
const float l4 = 0.105f;
const float W_R = 0.08f;

const float G = 9.81f;
const float pi = 3.1416f;
const float DEG_2_RAD = 0.0174533f;
const float RAD_2_DEG = 57.2958f;
const float GEAR_REDUCTION_RATIO_M2L = 0.3846f;//  MOTER->LEG 齿轮减速比
const float GEAR_REDUCTION_RATIO_L2M = 2.6f;	 //  LEG->MOTER 齿轮减速比
const float G_compensation = 10;
/*
		单腿各点角度结构体
*/
typedef struct Angle
{
	float phi1;
	float phi2;
	float phi3;
	float phi4;
	float phi0;
}Angle_;

/*
		单腿各点角速度结构体
*/
typedef struct Speed
{
	float d_phi1;
	float d_phi2;
	float d_phi3;
	float d_phi4;
	float d_phi0;
}Speed_;

/*
		单腿各点角加速度结构体
*/
typedef struct Accel
{
	float d_d_phi1;
	float d_d_phi4;
}Accel_;

/*
		末端点输出需求
*/
typedef struct Tip_output_require
{
	float F0;
	float Tp;
}Tip_output_require;

/*
		末端点输出需求
*/
typedef struct Output
{
	float T1;//电机1输出
	float T2;//电机2输出
	float T;//轮毂电机输出
}Output_;

/*
		末端点输出需求
*/
typedef struct VMC_result
{
	float phi0;
	float l0;
	float l0_target;
}VMC_;
/*
		电机状态结构体
*/
typedef struct Wheel_Info
{
	float wheel_pos;
	float wheel_speed;
	float wheel_accel;
}Wheel_;

/*
		机体状态结构体
*/
typedef struct Body_Info
{
	float pitch;
	float yaw;
	float roll;
	
	float d_pitch;
	float d_yaw;
	float d_roll;
}Body_;
/*
		@单腿模型状态类：

		@角度&角速度&角加速度信息：
			Angle_def Angle_state
			Speed_def Speed_state
			Accel_def Accel_state

		@腿长&目标腿长
			l0 & l0_target

		@末端点输出需求
			Tp & F0

		@关节电机输出力矩 & 轮电机输出力矩
			T1 & T2 & T
		
*/
class Single_Leg_Typedef
{
	public:
		Single_Leg_Typedef(uint8_t Legid):Leg_id(Legid)
		{
			Leg_Length_PID.PID_Init(0 , 0 , 0 , 100 , 1 , 0 , 0 , 1000 , 0.1);
		}
	
	Angle_ Angle_state;
	Speed_ Speed_state;
	Accel_ Accel_state;
	Wheel_ Wheel_state;
	Body_  Body_state;
	
	Tip_output_require Tip_Require;
	Output_ Final_Output;
	VMC_ VMC_Result;
	IFR_PID Leg_Length_PID;
	uint8_t Leg_id;
	void L_Control();
};

extern Single_Leg_Typedef Left_Leg;
extern Single_Leg_Typedef Right_Leg;
#endif
