#include "motor_torque_map.h"
#include "motor_control.h"


void MotorControl_GenerateTorqueMap(MotorControl_t *mc, float start, float end,
									int steps,	uint32_t settle_ms,
									float *out_angles,	float *out_torques)
{
    // Предварительно настраиваем P-регулятор и обнуляем накопители
    mc->foc.pid->integral  = 0.0f;
    mc->foc.pid->prev_error = 0.0f;

    float Kp = mc->foc.pid->kp;
    float delta = (end - start) / (float)(steps - 1);

    for (int i = 0; i < steps; ++i) {
        float target = start + delta * i;
        // Установка целевого угла
        mc->foc.target_angle = target;
        // Обнуляем интегральную и дифференциальную составляющие
        // (Ki и Kd уже равны нулю)
        mc->foc.pid->integral  = 0.0f;
        mc->foc.pid->prev_error = 0.0f;

        // Ждём для стабилизации момента
        HAL_Delay(settle_ms);

        // Читаем управляющее напряжение
        float Um = mc->voltage_limit;
        // Сохраняем угол и относительный момент
        out_angles[i]  = target;
        out_torques[i] = Um / Kp;
    }
}
