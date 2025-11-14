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
	arm_mat_init_f32(&Chassis.SPIN_MATRIX , 3 , 3 , (float *)IMU_SPIN_MATRIX);
	arm_mat_init_f32(&Chassis.True_E_mat , 3 , 1 , Chassis.TrueEular_);
	arm_mat_init_f32(&Chassis.True_d_E_mat , 3 , 1 , Chassis.True_d_Eular_);
}
float speed = 0;

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
	L_motor_Wheel.set_electric_offset(speed);
	R_motor_1.set_torque(Right_Leg.Final_Output.T1);
	R_motor_4.set_torque(Right_Leg.Final_Output.T2);
	R_motor_Wheel.set_electric_offset(speed);
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
		leg_info->Body_state.x = W_R * L_motor_Wheel.Motordata.abs_angle / 17.0f; //3508电机减速箱为17：1
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
		leg_info->Body_state.x = - W_R * R_motor_Wheel.Motordata.abs_angle / 17.0f;//3508电机减速箱为17：1
		leg_info->Body_state.d_x = - W_R * R_motor_Wheel.Motordata.Speed / 17.0f;
	}
	Chassis.Euler_[0] = IMU_Info.Angle.Yaw;
	Chassis.Euler_[1] = IMU_Info.Angle.Pitch;
	Chassis.Euler_[2] = IMU_Info.Angle.Roll;
	
	Chassis.d_Euler_[0] = IMU_Info.Gyro.Yaw;
	Chassis.d_Euler_[1] = IMU_Info.Gyro.Pitch;
	Chassis.d_Euler_[2] = IMU_Info.Gyro.Roll;
	
	arm_mat_init_f32(&Chassis.E_mat , 3, 1, Chassis.Euler_);
	arm_mat_init_f32(&Chassis.d_E_mat , 3, 1, Chassis.d_Euler_);
	arm_mat_mult_f32(&Chassis.SPIN_MATRIX , &Chassis.E_mat , &Chassis.True_E_mat);
	arm_mat_mult_f32(&Chassis.SPIN_MATRIX , &Chassis.d_E_mat , &Chassis.True_d_E_mat);
	for(int i ; i < 3 ; i++)
	{
		Chassis.TrueEular_[i] = Chassis.True_E_mat.pData[i];
		Chassis.True_d_Eular_[i] = Chassis.True_d_E_mat.pData[i];
	}
	
	Left_Leg.Body_state.yaw = deg2rad(Chassis.TrueEular_[0]);
	Right_Leg.Body_state.yaw = deg2rad(Chassis.TrueEular_[0]);
	Left_Leg.Body_state.pitch = deg2rad(trans(Chassis.TrueEular_[1]));
	Right_Leg.Body_state.pitch = deg2rad(trans(Chassis.TrueEular_[1]));
	Left_Leg.Body_state.roll = deg2rad(Chassis.TrueEular_[2]);
	Right_Leg.Body_state.roll = deg2rad(Chassis.TrueEular_[2]);
	
	Left_Leg.Body_state.d_yaw = - deg2rad(Chassis.True_d_Eular_[0]);
	Right_Leg.Body_state.d_yaw = - deg2rad(Chassis.True_d_Eular_[0]);
	Left_Leg.Body_state.d_pitch = deg2rad(Chassis.True_d_Eular_[1]);
	Right_Leg.Body_state.d_pitch = deg2rad(Chassis.True_d_Eular_[1]);
	Left_Leg.Body_state.d_roll = - deg2rad(Chassis.True_d_Eular_[2]);
	Right_Leg.Body_state.d_roll = - deg2rad(Chassis.True_d_Eular_[2]);
}


/**
 * @brief  IMU坐标系转换函数
 * @param  传入陀螺仪角度
 * @return 回传角度
 * @note   
 */
float trans(float angle)
{
	float true_angle;
	true_angle = angle + 180;
	
	if(true_angle > 180)true_angle -= 360;
	return true_angle;
}

void TIM2_Callback()
{						
	static uint8_t prep_flag = 0;
	static int timetick = 0;
	timetick++;
	
	if(timetick == 3000)prep_flag = 1;
	if(!(timetick % 2) && prep_flag == 1)
	{
		Update(&Left_Leg);
		
		Left_VMC.Jmat_Update(&Left_Leg.Angle_state , &Left_Leg.VMC_Result);
//		Left_LQR.LQR_Total_Cal();
		Left_Leg.L_Control();
		Left_VMC.Torque_Cal(&Left_Leg.Tip_Require , &Left_Leg.Final_Output);
		Chassis.Chassis_Motor_Setouttput();
	}
	if((timetick % 2) && prep_flag == 1)
	{
		Update(&Right_Leg);
		
		Right_VMC.Jmat_Update(&Right_Leg.Angle_state , &Right_Leg.VMC_Result);
//		Right_LQR.LQR_Total_Cal();
		Right_Leg.L_Control();
		Right_VMC.Torque_Cal(&Right_Leg.Tip_Require , &Right_Leg.Final_Output);
		Chassis.Chassis_Motor_Setouttput();
	}
	Can1.SendQueuedMsgs();//L
	Can2.SendQueuedMsgs();//R
	
	if(!(timetick % 10))Upper_Computer_Show_Wave();
}
