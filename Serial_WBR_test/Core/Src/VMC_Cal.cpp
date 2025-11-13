#include "VMC_Cal.h"
VMC_CAL_ Left_VMC(&Left_Leg);
VMC_CAL_ Right_VMC(&Right_Leg);
VMC_CAL_::VMC_CAL_(Single_Leg_Typedef * leg_info)
{
//	T[0] = 0;
//	T[1] = 0;
	arm_mat_init_f32(&Tmat , 2 , 1 , T);
	Leg_info = leg_info;
}

void VMC_CAL_::Torque_Cal(Tip_output_require * require , Output_ * output)
{
	F[0] = require->F0;
	F[1] = require->Tp;
	arm_mat_init_f32(&Fmat , 2 , 1 , F);
	arm_mat_mult_f32(&Jmat , &Fmat ,&Tmat);
	
	output->T1 = Tmat.pData[0];
	output->T2 = Tmat.pData[1];
} 

void VMC_CAL_::Jmat_Update(Angle_ * angle , VMC_ * result)
{
	float xb = l1 * arm_cos_f32(angle->phi1);
	float yb = l1 * arm_sin_f32(angle->phi1);
	float xd = l4 * arm_cos_f32(angle->phi4);
	float yd = l4 * arm_sin_f32(angle->phi4);
	
	float lbd2 = (xd - xb)*(xd - xb) + (yd - yb)*(yd - yb);
	float a = 2 * l2 * (xd - xb);
	float b = 2 * l2 * (yd - yb);
	float c = l2 * l2 - l3 * l3 + lbd2;
	
	float temp_phi2;
	arm_atan2_f32((b + sqrt(a*a + b*b - c*c)) , a + c , &temp_phi2);
	angle->phi2 = 2 * temp_phi2;

	arm_atan2_f32(yb - yd + l2 * arm_sin_f32(angle->phi2) , xb - xd + l2 * arm_cos_f32(angle->phi2) , &angle->phi3);
	
	float xc = xb + l2 * arm_cos_f32(angle->phi2);
	float yc = yb + l2 * arm_sin_f32(angle->phi2);
	float xf = K * xc;
	float yf = K * yc;
	
	result->l0 = sqrt(xf*xf + yf*yf);
	arm_atan2_f32(yf , xf , &result->phi0);
	
	float J11 = -(K * l1 * arm_sin_f32(result->phi0 - angle->phi3) * arm_sin_f32(angle->phi1 - angle->phi2)) / arm_sin_f32(angle->phi2 - angle->phi3);
	float J12 = -(K * l1 * arm_cos_f32(result->phi0 - angle->phi3) * arm_sin_f32(angle->phi1 - angle->phi2)) / (result->l0 * arm_sin_f32(angle->phi2 - angle->phi3));
	float J21 = -(K * l4 * arm_sin_f32(result->phi0 - angle->phi2) * arm_sin_f32(angle->phi3 - angle->phi4)) / sin(angle->phi2 - angle->phi3);
	float J22 = -(K * l4 * arm_cos_f32(result->phi0 - angle->phi2) * arm_sin_f32(angle->phi3 - angle->phi4)) / (result->l0 * arm_sin_f32(angle->phi2 - angle->phi3));

	J[0] = J11;
	J[1] = J12;
	J[2] = J21;
	J[3] = J22;
	
	arm_mat_init_f32(&Jmat , 2 , 2 , J);
}

void VMC_CAL_::VMC_CAL()
{
	Jmat_Update(&Leg_info->Angle_state , &Leg_info->VMC_Result);
	Torque_Cal(&Leg_info->Tip_Require , &Leg_info->Final_Output);
}
