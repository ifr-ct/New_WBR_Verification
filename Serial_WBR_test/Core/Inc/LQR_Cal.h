#ifndef __LQR_CAL_H
#define __LQR_CAL_H
#include "Chassis_Controller.h"
#include "arm_math.h"

class LQR_CAL_
{
	public:
		LQR_CAL_(Single_Leg_Typedef * leg_info):Leg_info(leg_info){}
			
		void State_Update(Angle_ angle , Speed_ speed , Wheel_ wheel_info , Body_ body_info);
		void Kmat_Simplify(VMC_ vmc_result);
		void LQR_Output_Cal();
	private:
		arm_matrix_instance_f32 Umat;
		arm_matrix_instance_f32 Xmat;
		arm_matrix_instance_f32 Kmat;
		float X[6];
		float K_coeff[12*4];
		float K[2*6];
	
		Single_Leg_Typedef * Leg_info;
	
};

#endif