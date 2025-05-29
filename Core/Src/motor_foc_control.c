#include "motor_control.h"
#include "mt6835.h"
#include "fast_sin.h"

static inline float MotorControl_NormalizeAngle(float angle)
{
    while (angle >= MOTOR_CONTROL_TWO_PI) angle -= MOTOR_CONTROL_TWO_PI;
    while (angle < 0.0f)    angle += MOTOR_CONTROL_TWO_PI;
    return angle;
}

void MotorFOC_Calibrate(MotorControl_t *mc, uint32_t calib_delay, float calib_voltage)
{

	MotorControl_Enable(mc);

	MotorControl_SetPhasePWM(&(mc->hw), calib_voltage, 0.0f);

	HAL_Delay(calib_delay);

	float theta_mech = MT6835_ReadAngle(mc->sensor);
	float theta_e_meas = 0.0f - theta_mech * mc->pole_pairs;
	theta_e_meas = MotorControl_NormalizeAngle(theta_e_meas);
	mc->foc.offset_angle = theta_e_meas;

	MotorControl_Disable(mc);
}


void MotorControl_FOC_Torque_Process(MotorControl_t *mc)
{
    float mech  = MT6835_ReadAngle(mc->sensor);
    float elec  = mech * mc->pole_pairs + MOTOR_CONTROL_PI_2 + mc->foc.offset_angle;
    mc->angle_elec = MotorControl_NormalizeAngle(elec);
}


void MotorControl_FOC_Position_Process(MotorControl_t *mc)
{
	// Механический угол
	float mech = MT6835_ReadAngle(mc->sensor);

    // Ошибка в диапазоне [-π, +π]
    float delta = mc->foc.target_angle - mech;
    if (delta >  MOTOR_CONTROL_PI)  delta -= MOTOR_CONTROL_TWO_PI;
    else if (delta < -MOTOR_CONTROL_PI) delta += MOTOR_CONTROL_TWO_PI;

    // P-компонента
    float Pout = mc->foc.pid->kp * delta;

    // I-компонента
    mc->foc.pid->integral += mc->foc.pid->ki * delta * mc->delta_time;
    if (mc->foc.pid->integral > 1.0f)  mc->foc.pid->integral = 1.0f;
    else if (mc->foc.pid->integral < -1.0f) mc->foc.pid->integral = -1.0f;

    // D-компонента
    float derivative = (delta - mc->foc.pid->prev_error) / mc->delta_time;
    float Dout       = mc->foc.pid->kd * derivative;
    mc->foc.pid->prev_error = delta;


    float voltage = Pout + mc->foc.pid->integral + Dout;

    if (voltage >  1.0f) voltage =  1.0f;
    if (voltage < -1.0f) voltage = -1.0f;

    float elec = mech * mc->pole_pairs +
    		mc->foc.offset_angle +
			((voltage >= 0) ? MOTOR_CONTROL_PI_2 : -MOTOR_CONTROL_PI_2);

    mc->voltage_limit = (voltage >= 0) ? voltage : -voltage;
    mc->angle_elec = elec;

}
