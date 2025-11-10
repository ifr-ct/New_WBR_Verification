#ifndef __VMC_CAL_H
#define __VMC_CAL_H
#include "arm_math.h"
#include "Chassis_Controller.h"


class VMC_CAL_
{
	public:
		VMC_CAL_(Single_Leg_Typedef * Leg_info);
		void Torque_Cal(Tip_output_require * require , Output_ * output);
		void Jmat_Update(Angle_ * angle , VMC_ * result);
		void VMC_CAL();
	private:
		arm_matrix_instance_f32 Jmat;
		arm_matrix_instance_f32 Fmat;
		arm_matrix_instance_f32 Tmat;
		float J[4];
		float F[2];
		float T[2];
		Single_Leg_Typedef * Leg_info;
};


extern VMC_CAL_ Left_VMC;
extern VMC_CAL_ Right_VMC;
#endif
