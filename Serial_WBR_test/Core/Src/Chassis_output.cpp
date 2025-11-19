#include "Chassis_output.h"
Chassis_Motor_output Chassis;
IFR_TIM_ClassDef Tim2;
CanMsgQueue Can1_Msg;
CanMsgQueue Can2_Msg;
IFR_CAN_ClassDef Can1;
IFR_CAN_ClassDef Can2;

IFR_DJI_Motor L_motor_Wheel(0x204 , DJI_3508 , &Can2_Msg);
IFR_RS_Motor L_motor_1(1, RobStride_01, &Can2_Msg);
IFR_RS_Motor L_motor_4(4, RobStride_01, &Can2_Msg);

IFR_DJI_Motor R_motor_Wheel(0x202 , DJI_3508 , &Can1_Msg);
IFR_RS_Motor R_motor_1(1, RobStride_01, &Can1_Msg);
IFR_RS_Motor R_motor_4(4, RobStride_01, &Can1_Msg);
Chassis_Motor_output::Chassis_Motor_output()
{
}
/**
 * @brief  地盘初始化函数
 * @param  void
 * @param  void
 * @return void
 * @note   main函数中调用
 */
void Chassis_Init()
{
	Tim2.TIM_ITStart(&htim2,TIM2_Callback);
	Can1.CAN_Init(&hcan1,&Can1_Msg);
	Can2.CAN_Init(&hcan2,&Can2_Msg);
	Can2.RegisterMotor(&L_motor_1);
	Can2.RegisterMotor(&L_motor_4);
	Can2.RegisterMotor(&L_motor_Wheel);
	Can1.RegisterMotor(&R_motor_1);
	Can1.RegisterMotor(&R_motor_4);
	Can1.RegisterMotor(&R_motor_Wheel);
	L_motor_1.set_zero_pos();
	L_motor_4.set_zero_pos();
	R_motor_1.set_zero_pos();
	R_motor_4.set_zero_pos();
	VOFA_Init(&huart1);
	chassis_PID_init();
	Chassis.Anti_Split_PID.PID_Init(1.5,0,0.5,5,1,0,0,1,0.1);
	Chassis.Yaw_PID.PID_Init(0.11,0,0.01,5,2*pi,0,0,0.5,0.3);
}


/**
 * @brief  电机输出值计算函数
 * @param  void
 * @param  void
 * @return void
 * @note   tim2定时器中断中调用
 */
void Chassis_Motor_output::Chassis_Motor_Setouttput()
{
	L_motor_1.set_torque(-Left_Leg.Final_Output.T1);
	L_motor_4.set_torque(-Left_Leg.Final_Output.T2);
	L_motor_Wheel.set_electric_offset(Left_Leg.Final_Output.T - Heading_Angle_Cal());
//	L_motor_Wheel.set_electric_offset(0);
	R_motor_1.set_torque(Right_Leg.Final_Output.T1);
	R_motor_4.set_torque(Right_Leg.Final_Output.T2);
	R_motor_Wheel.set_electric_offset(-Right_Leg.Final_Output.T - Heading_Angle_Cal());
//	R_motor_Wheel.set_electric_offset(0);
}
/**
 * @brief  底盘信息更新函数
 * @param  Single_Leg_Typedef * leg_info 单腿模型类
 * @return void
 * @note   tim2定时器中断中调用
 */
void Update(Single_Leg_Typedef * leg_info)
{
	if(leg_info->Leg_id == 1)//id = 1代表左腿
	{
		static float L_Phi1_Start_State = L_motor_1.Motordata.Angle;
		static float L_Phi4_Start_State = L_motor_4.Motordata.Angle;
		leg_info->Angle_state.phi1 = -(L_motor_1.Motordata.Angle - L_Phi1_Start_State) * GEAR_REDUCTION_RATIO_M2L + 207.0f * DEG_2_RAD;
		leg_info->Angle_state.phi4 = -(L_motor_4.Motordata.Angle - L_Phi4_Start_State) * GEAR_REDUCTION_RATIO_M2L + 112.0f * DEG_2_RAD;
		leg_info->Speed_state.d_phi1 = - L_motor_1.Motordata.Speed;
		leg_info->Speed_state.d_phi4 = - L_motor_4.Motordata.Speed;
		leg_info->Body_state.x = ((W_R * L_motor_Wheel.Motordata.abs_angle) + (- W_R * R_motor_Wheel.Motordata.abs_angle)) / (2.0f * 19.0f) ; //3508电机减速箱为17：1
		leg_info->Body_state.d_x = W_R * L_motor_Wheel.Motordata.Speed / 17.0f;
	}
	if(leg_info->Leg_id == 2)//id = 2代表右腿
	{
		static float R_Phi1_Start_State = R_motor_1.Motordata.Angle;
		static float R_Phi4_Start_State = R_motor_4.Motordata.Angle;
		leg_info->Angle_state.phi1 = (R_motor_1.Motordata.Angle - R_Phi1_Start_State) * GEAR_REDUCTION_RATIO_M2L + 207.0f * DEG_2_RAD;
		leg_info->Angle_state.phi4 = (R_motor_4.Motordata.Angle - R_Phi4_Start_State) * GEAR_REDUCTION_RATIO_M2L + 112.0f * DEG_2_RAD;
		leg_info->Speed_state.d_phi1 = R_motor_1.Motordata.Speed;
		leg_info->Speed_state.d_phi4 = R_motor_4.Motordata.Speed;
		leg_info->Body_state.x = ((W_R * L_motor_Wheel.Motordata.abs_angle) + (- W_R * R_motor_Wheel.Motordata.abs_angle)) / (2.0f * 19.0f);//3508电机减速箱为17：1
		leg_info->Body_state.d_x = - W_R * R_motor_Wheel.Motordata.Speed / 17.0f;
	}
	Left_Leg.Body_state.yaw = deg2rad(IMU_Info.Angle.Yaw);
	Right_Leg.Body_state.yaw = deg2rad(IMU_Info.Angle.Yaw);
	Left_Leg.Body_state.pitch = deg2rad(IMU_Info.Angle.Pitch);
	Right_Leg.Body_state.pitch = deg2rad(IMU_Info.Angle.Pitch);
	Left_Leg.Body_state.roll = deg2rad(IMU_Info.Angle.Roll);
	Right_Leg.Body_state.roll = deg2rad(IMU_Info.Angle.Roll);
	
	Left_Leg.Body_state.d_yaw = deg2rad(IMU_Info.Gyro.Yaw);
	Right_Leg.Body_state.d_yaw = deg2rad(IMU_Info.Gyro.Yaw);
	Left_Leg.Body_state.d_pitch = deg2rad(IMU_Info.Gyro.Pitch);
	Right_Leg.Body_state.d_pitch = deg2rad(IMU_Info.Gyro.Pitch);
	Left_Leg.Body_state.d_roll = deg2rad(IMU_Info.Gyro.Roll);
	Right_Leg.Body_state.d_roll = deg2rad(IMU_Info.Gyro.Roll);
}


/**
 * @brief  IMU坐标系转换函数
 * @param  传入陀螺仪角度
 * @return 回传角度
 * @note   
 */
//float trans(float angle)
//{
//	float true_angle;
//	true_angle = angle + 180;
//	
//	if(true_angle > 180)true_angle -= 360;
//	return true_angle;
//}

float Chassis_Motor_output::Anti_Split_Cal()
{
	Theta_error = Left_LQR.Get_theta() - Right_LQR.Get_theta();
	d_Theta_error = Left_LQR.Get_d_theta() - Right_LQR.Get_d_theta();
	AntiSplit_Tp = Anti_Split_PID.Kp * Theta_error + Anti_Split_PID.Kd * d_Theta_error;
//	Last_Theta_error = Theta_error;
	return AntiSplit_Tp;
}


float Chassis_Motor_output::Heading_Angle_Cal()
{
	static float Target_angle = IMU_Info.Angle.Yaw;
	Chassis.yaw_error = IMU_Info.Angle.Yaw - Target_angle;
	Chassis.Tar_Yaw = Target_angle;
	if(yaw_error > 180)yaw_error -= 360;
	if(yaw_error < -180)yaw_error += 360;
	Turn_T = Chassis.Yaw_PID.Kp * yaw_error + Chassis.Yaw_PID.Kd * IMU_Info.Gyro.Yaw;
	return Turn_T;
}


void TIM2_Callback()
{						
	static uint8_t prep_flag = 0;
	static uint8_t lqr_flag = 0;
	static int timetick = 0;
	timetick++;
	if(timetick == 3000)prep_flag = 1;
	if(timetick == 6000)lqr_flag = 1; 
	if(!(timetick % 2) && prep_flag == 1)
	{
		Update(&Left_Leg);
		
		Left_VMC.Jmat_Update(&Left_Leg.Angle_state , &Left_Leg.VMC_Result);
		Left_LQR.LQR_Total_Cal();
		if(lqr_flag == 1)Left_LQR.LQR_Output_Cal();
		Left_Leg.L_Control();
		Left_VMC.Torque_Cal(&Left_Leg.Tip_Require , &Left_Leg.Final_Output , -Chassis.Anti_Split_Cal());
		Chassis.Chassis_Motor_Setouttput();
	}
	if((timetick % 2) && prep_flag == 1)
	{
		Update(&Right_Leg);
		
		Right_VMC.Jmat_Update(&Right_Leg.Angle_state , &Right_Leg.VMC_Result);
		Right_LQR.LQR_Total_Cal();
		if(lqr_flag == 1)Right_LQR.LQR_Output_Cal();
		Right_Leg.L_Control();
		Right_VMC.Torque_Cal(&Right_Leg.Tip_Require , &Right_Leg.Final_Output , Chassis.Anti_Split_Cal());
		Chassis.Chassis_Motor_Setouttput();
	}
	Can1.SendQueuedMsgs();//L
	Can2.SendQueuedMsgs();//R
	
	if(!(timetick % 10))Upper_Computer_Show_Wave();
}
