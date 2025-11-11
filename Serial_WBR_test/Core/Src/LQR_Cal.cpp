#include "LQR_Cal.h"
LQR_CAL_ Left_LQR(&Left_Leg);
LQR_CAL_ Right_LQR(&Right_Leg);
void LQR_CAL_::State_Update(Angle_ angle , Speed_ speed , Wheel_ wheel_info , Body_ body_info , VMC_ result)
{
	float phi;
	float d_phi;
	float x;
	float d_x;
	float theta;
	float d_theta;
	
	d_x = wheel_info.wheel_speed * W_R;
	x += d_x;
	
	phi = body_info.pitch;
	d_phi = body_info.d_pitch;
	
	theta = result.phi0 - (pi / 2) - body_info.pitch;
	d_theta = speed.d_phi0 - body_info.d_pitch;
	
	this->x[0] = theta;
	this->x[1] = d_theta;
	this->x[2] = x;
	this->x[3] = d_x;
	this->x[4] = phi;
	this->x[5] = d_phi;
	
	arm_mat_init_f32(&Xmat , 6 , 1 , this->x);
}

void LQR_CAL_::d_phi0_Update(Speed_ * speed , Angle_ angle)
{
	speed->d_phi0 = ((l1*speed->d_phi1*arm_cos_f32(angle.phi1) + l2*speed->d_phi4*arm_cos_f32(angle.phi4))/
									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)) + ((l1*speed->d_phi1*arm_sin_f32(angle.phi1) + l2*speed->d_phi4*arm_sin_f32(angle.phi4))*(l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4)))/
									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4))*(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)))/
									((l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4))*(l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4))/
									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4))*(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)) + 1);
}

void LQR_CAL_::Kmat_Simplify(VMC_ vmc_result)
{
		static float K[12] = {
    -47.2149f, -12.5821f, -9.4121f, -10.8986f, 23.8871f, 0.9300f,
     31.0719f,  8.8542f,  6.7563f,  7.7364f, 133.1076f, 3.4700f
		};

		float list[12][4];
		for(int i = 0; i < 12; i++)
		{
//			k[i] = -(list[2*i][0] * (vmc_result.l0*vmc_result.l0*vmc_result.l0) + 
//							list[2*i][1] * (vmc_result.l0*vmc_result.l0) + 
//							list[2*i][2] * vmc_result.l0 + list[2*i][3]);
//				
//			k[i+6] = -(list[2*i+1][0] * (vmc_result.l0*vmc_result.l0*vmc_result.l0) + 
//								list[2*i+1][1] * (vmc_result.l0*vmc_result.l0) + 
//								list[2*i+1][2] * vmc_result.l0 + list[2*i+1][3]);
			k[i] = -K[i];
		}
		arm_mat_init_f32(&Kmat , 2 , 6 , k);
		arm_mat_mult_f32(&Kmat , &Xmat , &Umat);
}

void LQR_CAL_::LQR_Output_Cal()
{
	
	u[0] = Umat.pData[0];
//	Leg_info->Final_Output.T = Umat.pData[0];
	u[1] = Umat.pData[1];
//	Leg_info->Tip_Require.Tp = Umat.pData[1];
}

void LQR_CAL_::LQR_Total_Cal()
{
	d_phi0_Update(&Leg_info->Speed_state , Leg_info->Angle_state);
	State_Update(Leg_info->Angle_state , Leg_info->Speed_state , Leg_info->Wheel_state , Leg_info->Body_state , Leg_info->VMC_Result);
	Kmat_Simplify(Leg_info->VMC_Result);
	LQR_Output_Cal();
}