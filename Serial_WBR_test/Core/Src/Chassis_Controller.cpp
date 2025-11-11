#include "Chassis_Controller.h"
Single_Leg_Typedef Left_Leg(1);
Single_Leg_Typedef Right_Leg(2);


void Single_Leg_Typedef::L_Control()
{
	Tip_Require.F0 = G_compensation + Leg_Length_PID.Positional_PID(VMC_Result.l0_target , VMC_Result.l0);
}

