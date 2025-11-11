#include "Chassis_output.h"


Chassis_Motor_output Chassis;
IFR_TIM_ClassDef Tim2;
CanMsgQueue Can1_Msg;
CanMsgQueue Can2_Msg;
IFR_CAN_ClassDef Can1;
IFR_CAN_ClassDef Can2;
IFR_RS_Motor L_motor_1(1, RobStride_01, &Can2_Msg);
IFR_RS_Motor L_motor_4(4, RobStride_01, &Can2_Msg);
IFR_RS_Motor R_motor_1(1, RobStride_01, &Can1_Msg);
IFR_RS_Motor R_motor_4(4, RobStride_01, &Can1_Msg);
Chassis_Motor_output::Chassis_Motor_output()
{
}

void Chassis_Init()
{
	Tim2.TIM_ITStart(&htim2,TIM2_Callback);
	Can1.CAN_Init(&hcan1,&Can1_Msg);
	Can2.CAN_Init(&hcan2,&Can2_Msg);
	Can2.RegisterMotor(&L_motor_1);
	Can2.RegisterMotor(&L_motor_4);
	Can1.RegisterMotor(&R_motor_1);
	Can1.RegisterMotor(&R_motor_4);
	L_motor_1.Enable(); 
	L_motor_4.Enable();
	R_motor_1.Enable();
	R_motor_4.Enable();	
	VOFA_Init(&huart1);
	
}

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
	}
	if(leg_info->Leg_id == 2)//id = 2代表右腿
	{
		static float R_Phi1_Start_State = R_motor_1.Motordata.Angle;
		static float R_Phi4_Start_State = R_motor_4.Motordata.Angle;
		leg_info->Angle_state.phi1 = (R_motor_1.Motordata.Angle - R_Phi1_Start_State) * GEAR_REDUCTION_RATIO_M2L + 207.0f * DEG_2_RAD;
		leg_info->Angle_state.phi4 = (R_motor_4.Motordata.Angle - R_Phi4_Start_State) * GEAR_REDUCTION_RATIO_M2L + 112.0f * DEG_2_RAD;
		leg_info->Speed_state.d_phi1 = R_motor_1.Motordata.Speed;
		leg_info->Speed_state.d_phi4 = R_motor_4.Motordata.Speed;
	}
}

void TIM2_Callback()
{						
	static uint8_t prep_flag = 0;
	static int timetick = 0;
	timetick++;
	
	if(timetick == 3000)prep_flag = 1;
	if(!(timetick % 2) && prep_flag == 1)
	{
		L_motor_1.Enable(); 
		L_motor_4.Enable();
		Update(&Left_Leg);
		
		Left_VMC.Jmat_Update(&Left_Leg.Angle_state , &Left_Leg.VMC_Result);
		Left_LQR.LQR_Total_Cal();
		Left_Leg.L_Control();
		Left_VMC.Torque_Cal(&Left_Leg.Tip_Require , &Left_Leg.Final_Output);
		
	}
	if((timetick % 2) && prep_flag == 1)
	{
		R_motor_1.Enable();
		R_motor_4.Enable();
		Update(&Right_Leg);
		
		Right_VMC.Jmat_Update(&Right_Leg.Angle_state , &Right_Leg.VMC_Result);
		Right_LQR.LQR_Total_Cal();
		Right_Leg.L_Control();
		Right_VMC.Torque_Cal(&Right_Leg.Tip_Require , &Right_Leg.Final_Output);
	}
	Can2.SendQueuedMsgs();
	Can1.SendQueuedMsgs();
	if(!(timetick % 10))Upper_Computer_Show_Wave();
}
