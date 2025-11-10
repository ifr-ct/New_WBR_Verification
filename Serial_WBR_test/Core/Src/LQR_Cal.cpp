#include "LQR_Cal.h"
void LQR_CAL_::State_Update(Angle_ angle , Speed_ speed , Wheel_ wheel_info , Body_ body_info)
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
	
	
	
	
}