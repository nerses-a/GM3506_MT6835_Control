#include "motor_control.h"
#include "mt6835.h"
#include "stm32f1xx_hal.h"
#include "fast_sin.h"

// Инициализация структуры MotorControl_t
void MotorControl_Init(MotorControl_t *mc,
                       MotorHardware_t *hw,
                       MT6835_Sensor_t    *sensor,
                       uint8_t             pole_pairs,
					   uint32_t            calib_delay,
                       float               calib_voltage,
                       float               delta_time)
{
    // Аппаратная часть
    mc->hw            = *hw;            // Копируем конфиг таймеров/пинов
    mc->sensor        = sensor;         // Указываем связанный энкодер
    mc->pole_pairs    = pole_pairs;     // Пара пар полюсов
    mc->delta_time    = delta_time;     // Интервал обновления [с]
    mc->voltage_limit = 0.0f;
    mc->mode          = MODE_IDLE;      // Режим по умолчанию

    // Open-loop: обнуляем параметры
    mc->open_loop.max_accel    = 0.0f;
    mc->open_loop.target_speed = 0.0f;
    mc->open_loop.mech_speed   = 0.0f;
    mc->angle_elec      = 0.0f;

    // FOC: обнуляем параметры
    mc->foc.foc_torque      = 0.0f;
    mc->foc.target_angle    = 0.0f;
    mc->foc.pid = MOTOR_CONTROL_NULL;

   // калибровка для FOC
    MotorFOC_Calibrate(mc, calib_delay, calib_voltage);
}

// Включение двигателя
void MotorControl_Enable(MotorControl_t *mc)
{
    HAL_GPIO_WritePin(mc->hw.en_port, mc->hw.en_pin, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(mc->hw.htim, mc->hw.channelA);
    HAL_TIM_PWM_Start(mc->hw.htim, mc->hw.channelB);
    HAL_TIM_PWM_Start(mc->hw.htim, mc->hw.channelC);

}

// Выключение двигателя
void MotorControl_Disable(MotorControl_t *mc)
{
    HAL_GPIO_WritePin(mc->hw.en_port, mc->hw.en_pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Stop(mc->hw.htim, mc->hw.channelA);
    HAL_TIM_PWM_Stop(mc->hw.htim, mc->hw.channelB);
    HAL_TIM_PWM_Stop(mc->hw.htim, mc->hw.channelC);
}


// Установка Open-loop режима с параметрами
void MotorControl_SetOpenLoop(MotorControl_t *mc, float target_speed, float max_accel, float voltage)
{
    mc->open_loop.target_speed = target_speed;   // задаём целевую скорость
    mc->open_loop.max_accel    = max_accel;      // задаём максимальное ускорение
    mc->voltage_limit 		   = voltage;
    mc->mode                   = MODE_OPEN_LOOP_SPEED;
    // Сбрасываем счётчик скорости
    mc->open_loop.mech_speed = 0.0f;
}

// Установка FOC Torque режима с параметрами
void MotorControl_SetFocTorque(MotorControl_t *mc, float torque)
{
    mc->foc.foc_torque    = torque;
    mc->voltage_limit 	  = torque;
    mc->mode              = MODE_FOC_TORQUE;
}

// Установка FOC Position режима с параметрами
void MotorControl_SetFocPosition(MotorControl_t *mc, float target_angle, PID_t* pid)
{
    mc->foc.target_angle  = target_angle;
    mc->foc.pid           = pid;
    mc->mode              = MODE_FOC_POSITION;

    mc->foc.pid->integral  = 0.0f;
    mc->foc.pid->prev_error = 0.0f;
}


void MotorControl_SetPhasePWM(MotorHardware_t *hw, float voltage, float angle)
{

	 // Генерация трехфазного сигнала
	 float ua = voltage * fastSin(angle);
	 float ub = voltage * fastSin(angle + MOTOR_CONTROL_TWO_PI_OVER_3);
	 float uc = voltage * fastSin(angle - MOTOR_CONTROL_TWO_PI_OVER_3);

	 // Преобразование в регистры сравнения
	 uint32_t period = hw->htim->Init.Period + 1;
	 uint32_t cmpA   = (uint32_t)((ua * 0.5f + 0.5f) * period);
	 uint32_t cmpB   = (uint32_t)((ub * 0.5f + 0.5f) * period);
	 uint32_t cmpC   = (uint32_t)((uc * 0.5f + 0.5f) * period);


	 __HAL_TIM_SET_COMPARE(hw->htim, hw->channelA, cmpA);
	 __HAL_TIM_SET_COMPARE(hw->htim, hw->channelB, cmpB);
	 __HAL_TIM_SET_COMPARE(hw->htim, hw->channelC, cmpC);
}

void MotorControl_Process(MotorControl_t *mc)
{
	if(mc->mode == MODE_OPEN_LOOP_SPEED) {
		MotorControl_OpenLoop_Process(mc);
	} else if(mc->mode == MODE_FOC_TORQUE) {
		MotorControl_FOC_Torque_Process(mc);
	} else if(mc->mode == MODE_FOC_POSITION) {
		MotorControl_FOC_Position_Process(mc);
	}

    while (mc->angle_elec >= MOTOR_CONTROL_TWO_PI) mc->angle_elec -= MOTOR_CONTROL_TWO_PI;
    while (mc->angle_elec < 0.0f) mc->angle_elec += MOTOR_CONTROL_TWO_PI;

    MotorControl_SetPhasePWM(&mc->hw, mc->voltage_limit, mc->angle_elec);
}
