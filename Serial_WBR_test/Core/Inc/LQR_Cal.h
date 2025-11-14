#ifndef __LQR_CAL_H
#define __LQR_CAL_H
#include "arm_math.h"
#include "Chassis_Controller.h"


class LQR_CAL_
{
	public:
		LQR_CAL_(Single_Leg_Typedef * leg_info):Leg_info(leg_info)
		{
			arm_mat_init_f32(&Umat ,2 ,1 ,u);
		}
			
		void State_Update(Angle_ angle , Speed_ speed , Body_ body_info , VMC_ result);
		void d_phi0_Update(Speed_ * speed , Angle_ angle);
		void Kmat_Simplify(VMC_ vmc_result);
		void LQR_Output_Cal();
		void LQR_Total_Cal();
	private:
		arm_matrix_instance_f32 Umat;
		arm_matrix_instance_f32 Xmat;
		arm_matrix_instance_f32 Kmat;
		float x[6];
		float k_coeff[12*4];
		float k[2*6];
		float u[2];
		Single_Leg_Typedef * Leg_info;
};

extern LQR_CAL_ Left_LQR;
extern LQR_CAL_ Right_LQR;
#endif
