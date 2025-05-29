#include "motor_control.h"


void MotorControl_OpenLoop_Process(MotorControl_t *mc)
{
	OpenLoopControl_t * olc = &(mc->open_loop);

	float speed_err  = olc->target_speed - olc->mech_speed;
	float max_step   = olc->max_accel * mc->delta_time;

	if (MOTOR_CONTROL_ABS(speed_err) <= max_step) {
			olc->mech_speed = olc->target_speed;
	} else {
			olc->mech_speed += (speed_err > 0 ? max_step : -max_step);
	}

	const float elec_speed = olc->mech_speed * (float)mc->pole_pairs;
	const float d_angle    = elec_speed * mc->delta_time;

	mc->angle_elec += d_angle;

    while (mc->angle_elec >= MOTOR_CONTROL_TWO_PI)
    	mc->angle_elec -= MOTOR_CONTROL_TWO_PI;
    while (mc->angle_elec < 0.0f)
    	mc->angle_elec += MOTOR_CONTROL_TWO_PI;

    MotorControl_SetPhasePWM(&mc->hw, mc->voltage_limit, mc->angle_elec);
}
