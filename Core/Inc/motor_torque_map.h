#ifndef INC_MOTOR_TORQUE_MAP_H_
#define INC_MOTOR_TORQUE_MAP_H_


#include "motor_control.h"
#include <stdint.h>


/**
 * Генерирует карту момента двигателя по углу.
 *
 * @param mc           Указатель на структуру MotorControl_t
 * @param start        Начальный угол [рад]
 * @param end          Конечный угол [рад] (обычно 2π)
 * @param steps        Число точек на интервале [start, end]
 * @param settle_ms    Время ожидания после установки каждого угла [мс]
 * @param out_angles   Массив из steps для записи углов [рад]
 * @param out_torques  Массив из steps для записи относительного момента (Vq/Kp)
 *
 * Описание алгоритма:
 * 1) Режим работы: P-регулятор (Ki=0, Kd=0) для чистой статической характеристики.
 * 2) Для каждого i от 0 до steps-1:
 *    - Вычисляем target = start + i*(end-start)/(steps-1)
 *    - Устанавливаем mc->foc.target_angle = target
 *    - Сбрасываем интеграл и предыдущую ошибку (Ki=0, Kd=0)
 *    - Ждём settle_ms миллисекунд для стабилизации
 *    - Считываем Vq через mc->voltage_limit (положительное значение)
 *    - Записываем угол и момент = Vq / mc->foc.pid.kp
 *
 * Результат позволяет получить массивы углов и соответствующих моментов,
 * которые можно использовать для анализа cogging, статики и неравномерности.
  */

void MotorControl_GenerateTorqueMap(
    MotorControl_t *mc,
    float            start,
    float            end,
    int              steps,
    uint32_t         settle_ms,
    float           *out_angles,
    float           *out_torques
);



#endif /* INC_MOTOR_TORQUE_MAP_H_ */
