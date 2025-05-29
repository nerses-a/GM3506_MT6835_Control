#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "mt6835.h"
#include "stm32f1xx_hal.h"

#define MOTOR_CONTROL_NULL ((void *)0)

#define MOTOR_CONTROL_TWO_PI_OVER_3  	2.0943951023931953f
#define MOTOR_CONTROL_PI				3.1415926535897932f
#define MOTOR_CONTROL_PI_2				1.5707963267948966f
#define MOTOR_CONTROL_TWO_PI			6.2831853071795865f

#define MOTOR_CONTROL_ABS(x) ((x) < 0 ? -(x) : (x))

// Режимы работы мотора
typedef enum {
    MODE_IDLE = 0,           // Остановлено
    MODE_OPEN_LOOP_SPEED,    // Open-loop: постоянная скорость
    MODE_FOC_TORQUE,         // FOC: управление моментом
    MODE_FOC_POSITION        // FOC: удержание позиции через PI-регулятор

} MotorMode;

// Аппаратная часть: таймеры, каналы, пины
typedef struct {
    TIM_HandleTypeDef *htim;  // HAL-таймер для ШИМ
    uint32_t channelA;        // PWM-фаза A
    uint32_t channelB;        // PWM-фаза B
    uint32_t channelC;        // PWM-фаза C
    GPIO_TypeDef *en_port;    // Порт EN-пина
    uint16_t en_pin;          // Номер EN-пина
} MotorHardware_t;

// Параметры для Open-loop управления
typedef struct {
    float max_accel;          // Максимальное ускорение
    float target_speed;       // Целевая скорость [рад/с]
    float mech_speed;         // Текущая механическая скорость [рад/с]
} OpenLoopControl_t;


// Структура PID-регулятора
typedef struct {
    float kp;         // Пропорциональный коэффициент
    float ki;         // Интегральный коэффициент
    float kd;         // Дифференциальный коэффициент
    float integral;   // Накопленная интегральная составляющая
    float prev_error;  // Ошибка на предыдущем шаге (для d-члена)
} PID_t;


// Параметры для FOC управления
typedef struct {
    float offset_angle;       // Калибровочный угол (нулевой)
    float foc_torque;         // Целевой момент для режима TORQUE
    float target_angle;       // Целевой угол (позиция) для режима POSITION
    PID_t *pid;                // PID-регулятор положения
} FOCControl_t;



// Основная структура управления мотором
typedef struct {
    MotorHardware_t hw;         	// Аппаратная часть
    MT6835_Sensor_t *sensor;    	// Энкодер (MT6835)
    uint8_t pole_pairs;         	// Количество пар полюсов
    float delta_time;           	// Интервал между вызовами прерываний [сек]
    float voltage_limit;
    float angle_elec;         // Текущий электрический угол [рад]
    MotorMode mode;             	// Текущий режим работы
    OpenLoopControl_t open_loop; 	// Open-loop управление
    FOCControl_t foc;           	// FOC управление

} MotorControl_t;


// Инициализация структуры MotorControl_t
void MotorControl_Init(MotorControl_t *mc,
                       MotorHardware_t *hw,
                       MT6835_Sensor_t    *sensor,
                       uint8_t             pole_pairs,
					   uint32_t            calib_delay,
                       float               voltage_limit,
                       float               delta_time);


// Включение двигателя
void MotorControl_Enable(MotorControl_t *mc);
// Выключение двигателя
void MotorControl_Disable(MotorControl_t *mc);

// Калибровка двигателя(для FOC)
void MotorFOC_Calibrate(MotorControl_t *mc, uint32_t calib_delay, float calib_voltage);

void MotorControl_SetPhasePWM(MotorHardware_t *hw, float voltage, float angle);

// Установка Open-loop режима с параметрами
void MotorControl_SetOpenLoop(MotorControl_t *mc, float target_speed, float max_accel, float voltage);

void MotorControl_SetFocTorque(MotorControl_t *mc, float torque);

void MotorControl_SetFocPosition(MotorControl_t *mc, float target_angle, PID_t* pid);

void MotorControl_Process(MotorControl_t *mc);
void MotorControl_OpenLoop_Process(MotorControl_t *mc);
void MotorControl_FOC_Torque_Process(MotorControl_t *mc);
void MotorControl_FOC_Position_Process(MotorControl_t *mc);


#endif /* INC_MOTOR_CONTROL_H_ */
