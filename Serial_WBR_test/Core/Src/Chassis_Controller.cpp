#include "Chassis_Controller.h"
Single_Leg_Typedef Left_Leg(1);
Single_Leg_Typedef Right_Leg(2);

/**
 * @brief  腿部支持力计算函数
 * @param  void
 * @return void
 * @note    
 */
void Single_Leg_Typedef::L_Control()
{
	Tip_Require.F0 = G_compensation + Leg_Length_PID.Separate_Positional_PID(VMC_Result.l0_target , VMC_Result.l0);
}

/**
 * @brief  PID初始化函数（腿长分段pid）
 * @param  void
 * @param  void
 * @return void
 * @note   
 */
void chassis_PID_init()
{
	Left_Leg.Leg_Length_PID.PID_Init(360 , 1 , 90 , 100 , 1 , 9 , 0 , 0.05 , 0.1);
	Right_Leg.Leg_Length_PID.PID_Init(300 , 1 , 40 , 100 , 1 , 9 , 0 , 0.05 , 0.1);
	Left_Leg.Leg_Length_PID.Separate_PID_Init(750 , 0 , 10);
	Right_Leg.Leg_Length_PID.Separate_PID_Init(700 , 0 , 10);
}