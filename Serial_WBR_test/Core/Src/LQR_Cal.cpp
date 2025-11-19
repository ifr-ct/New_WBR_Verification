#include "LQR_Cal.h"
LQR_CAL_ Left_LQR(&Left_Leg);
LQR_CAL_ Right_LQR(&Right_Leg);

/**
 * @brief  状态矩阵更新函数
 * @param  髋关节电机角度结构体指针
 * @param  髋关节电机速度结构体指针
 * @param  底盘位姿结构体指针
 * @param  VMC结果结构体指针
 * @return void
 * @note  none
 */
void LQR_CAL_::State_Update(Angle_ angle , Speed_ speed , Body_ body_info , VMC_ result)
{
	float phi;
	float d_phi;
	float x;
	float d_x;
	float theta;
	float d_theta;
	
	phi = body_info.pitch;
	d_phi = body_info.d_pitch;
	
	x = body_info.x;
	d_x = body_info.d_x;
	
	theta = result.phi0 - (pi / 2) - body_info.pitch - 0.2f;
	d_theta = speed.d_phi0 - body_info.d_pitch;
	
	this->x[0] = theta;
	this->x[1] = d_theta;
	this->x[2] = x;
	this->x[3] = d_x;
	this->x[4] = phi;
	this->x[5] = d_phi;
	
	arm_mat_init_f32(&Xmat , 6 , 1 , this->x);
}

/**
 * @brief  d_phi0计算函数
 * @param  髋关节电机速度结构体指针
 * @param  髋关节电机角度结构体指针
 * @return void
 * @note   none
 */
void LQR_CAL_::d_phi0_Update(Speed_ * speed , Angle_ angle)
{
//	speed->d_phi0 = ((l1*speed->d_phi1*arm_cos_f32(angle.phi1) + l2*speed->d_phi4*arm_cos_f32(angle.phi4))/
//									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)) + ((l1*speed->d_phi1*arm_sin_f32(angle.phi1) + l2*speed->d_phi4*arm_sin_f32(angle.phi4))*(l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4)))/
//									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4))*(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)))/
//									((l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4))*(l1*arm_sin_f32(angle.phi1) + l2*arm_sin_f32(angle.phi4))/
//									(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4))*(l1*arm_cos_f32(angle.phi1) + l2*arm_cos_f32(angle.phi4)) + 1);
	float cos_phi1 = arm_cos_f32(angle.phi1);
	float cos_phi4 = arm_cos_f32(angle.phi4);
	float sin_phi1 = arm_sin_f32(angle.phi1);
	float sin_phi4 = arm_sin_f32(angle.phi4);

	float l1_cos_phi1 = l1 * cos_phi1;
	float l2_cos_phi4 = l2 * cos_phi4;
	float l1_sin_phi1 = l1 * sin_phi1;
	float l2_sin_phi4 = l2 * sin_phi4;

	float denominator_cos = l1_cos_phi1 + l2_cos_phi4;
	float denominator_sin = l1_sin_phi1 + l2_sin_phi4;

	// 计算平方项
	float denominator_cos_sq = denominator_cos * denominator_cos;
	float denominator_sin_sq = denominator_sin * denominator_sin;

	// 分子第一部分
	float numerator_part1 = (l1 * speed->d_phi1 * cos_phi1 + l2 * speed->d_phi4 * cos_phi4) / denominator_cos;

	// 分子第二部分  
	float numerator_part2 = ((l1 * speed->d_phi1 * sin_phi1 + l2 * speed->d_phi4 * sin_phi4) * denominator_sin) / denominator_cos_sq;

	// 分母
	float full_denominator = denominator_sin_sq / denominator_cos_sq + 1.0f;

	// 最终结果
	speed->d_phi0 = (numerator_part1 + numerator_part2) / full_denominator;
}

/**
 * @brief  依照腿长反求k矩阵
 * @param  VMC结果结构体指针
 * @return void
 * @note   none
 */
void LQR_CAL_::Kmat_Simplify(VMC_ vmc_result)
{
		static float K[12] = {
    -20.8092f, -3.3121f, -1.1759f, -7.8777f, 10.6572f, 1.0096f,
    14.9451f, 2.6283f, 1.1031f, 7.3330f, 48.6866f, 2.0882f
};

		float k_list[12][4];
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
		
}

/**
 * @brief  lqr输出计算函数
 * @param  void
 * @return void
 * @note   none
 */
void LQR_CAL_::LQR_Output_Cal()
{
	arm_mat_mult_f32(&Kmat , &Xmat , &Umat);
	u[0] = Umat.pData[0];
	Leg_info->Final_Output.T = 1.0f * Umat.pData[0];
	u[1] = Umat.pData[1];
	Leg_info->Tip_Require.Tp =  1.0f * Umat.pData[1];
}

/**
 * @brief  lqr计算函数
 * @param  void
 * @param  void
 * @return void
 * @note   lqr计算仅调用本函数即可
 */
void LQR_CAL_::LQR_Total_Cal()
{
	d_phi0_Update(&Leg_info->Speed_state , Leg_info->Angle_state);
	State_Update(Leg_info->Angle_state , Leg_info->Speed_state , Leg_info->Body_state , Leg_info->VMC_Result);
	Kmat_Simplify(Leg_info->VMC_Result);
//	LQR_Output_Cal();
}

float LQR_CAL_::Get_theta()
{
	return x[0];
}

float LQR_CAL_::Get_d_theta()
{
	return x[1];
}
