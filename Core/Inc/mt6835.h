#ifndef MT6835_H
#define MT6835_H

#include "stm32f1xx_hal.h"

#define MT6835_ABS(x) ((x) < 0 ? -(x) : (x))


#define MT6835_OK      					0
#define MT6835_ERROR  					-1
#define MT6835_ANGLE_READ_ERROR 		(1u << 31)
#define MT6835_SUCCESS_RESPONSE 		0x55

#define MT6835_AUTOCAL_IN_PROGRESS      0x01    // Калибровка в процессе
#define MT6835_AUTOCAL_FAILED           0x02    // Калибровка не удалась
#define MT6835_AUTOCAL_SUCCESS          0x03    // Калибровка успешно завершена

#define MT6835_CMD_READ_REG  			0b0011  // Команда чтения регистра
#define MT6835_CMD_WRITE_REG 			0b0110  // Команда записи регистра
#define MT6835_CMD_WRITE_EEPROM 		0b1100  // Команда записи в EEPROM
#define MT6835_CMD_SET_ZERO  			0b0101  // Команда установки нулевого положения
#define MT6835_CMD_READ_ANGLE			0b1010  // Команда чтения угла

#define MT6835_ADDR_REG_ANGLE_MSB   	0x003   // Адрес регистра старшего байта угла
#define MT6835_ADDR_REG_STATUS 			0x005   // Адрес регистра статуса mt6835
#define MT6835_ADDR_REG_AUTOCAL_FREQ 	0x00E   // Адрес регистра скорости
#define MT6835_ADDR_REG_AUTOCAL_STATUS  0x113   // Адрес регистра статуса во время автокалибровки

#define MT6835_MASK_STATUS				0x07
#define MT6835_MASK_ADDR    			0x0F    // Маска для старших 4 бит адреса
#define MT6835_MASK_AUTOCAL_FREQ_CLEAR 	0x8F    // Маска для очистки битов 6, 5, 4 (AUTOCAL_FREQ)
#define MT6835_MASK_AUTOCAL_STATUS      0x03    // Маска для выделения статуса

#define MT6835_MEASURE_DELAY_MS 		5  		// Задержка между измерениями для скорости
#define MT6835_AUTOCAL_TIMEOUT_MS 		20000  	// Таймаут авто-калибровки
#define MT6835_AUTOCAL_DELAY_MS 		10      // Задержка после активации CAL_EN
#define MT6835_AUTOCAL_STATUS_CHECK_MS 	500 	// Интервал проверки статуса
#define MT6835_AUTOCAL_STATUS_SHIFT     6       // Сдвиг для извлечения статуса

#define MT6835_MIN_DELTA_ANGLE 			0.01	// Минимальное изменение угла для детектирования движения (рад)
#define MT6835_MAX_SPEED 				1000.0 	// Максимальная ожидаемая скорость (рад/с)

#define MT6835_FILTER_SIZE 				10

#define MT6835_PI 						3.14159265358979323846f

typedef struct {
    SPI_HandleTypeDef *spi;  	// Указатель на SPI
    GPIO_TypeDef *cs_port;    	// Порт для Chip Select (CS)
    uint16_t cs_pin;          	// Пин Chip Select (CS)
    GPIO_TypeDef *cal_port;   	// Порт для CAL_EN
    uint16_t cal_pin;         	// Пин CAL_EN
} MT6835_Sensor_t;


// Функции для работы с датчиком

// Иницилизация mt6835
int MT6835_Init(MT6835_Sensor_t *sensor, SPI_HandleTypeDef *spi,
                GPIO_TypeDef *cs_port, uint16_t cs_pin,
                GPIO_TypeDef *cal_port, uint16_t cal_pin);

// Функция передачи SPI
int  MT6835_TransmitReceive(MT6835_Sensor_t *sensor, uint8_t *tx_data,
							uint8_t *rx_data, uint8_t length,
							uint32_t timeout);

// Чтение запсиь в регистры
int MT6835_ReadRegister(MT6835_Sensor_t *sensor, uint16_t address, uint8_t *value);
int MT6835_WriteRegister(MT6835_Sensor_t *sensor, uint16_t address, uint8_t data);

//
int MT6835_ProgramEEPROM(MT6835_Sensor_t *sensor);

// Чтение угла и скоросит
uint32_t MT6835_ReadRawAngle(MT6835_Sensor_t *sensor);
float MT6835_ReadAngle(MT6835_Sensor_t *sensor);
float MT6835_ReadAngleDegrees(MT6835_Sensor_t *sensor);
float MT6835_CalculateRotationSpeed(MT6835_Sensor_t *sensor, uint32_t timer_freq_hz, TIM_HandleTypeDef *htim);
float MT6835_CalculateRotationSpeedRPM(MT6835_Sensor_t *sensor, uint32_t timer_freq_hz, TIM_HandleTypeDef *htim);
float MovingAverage_FilterSpeed(float new_speed);

//Настройка датчика
uint8_t MT6835_CheckStatus(MT6835_Sensor_t *sensor);

int MT6835_SetZero(MT6835_Sensor_t *sensor);
int MT6835_SetZeroTemporary(MT6835_Sensor_t *sensor);

int MT6835_SetAutoCalFreq(MT6835_Sensor_t *sensor, uint8_t freqValue);
int MT6835_AutoCalibration(MT6835_Sensor_t *sensor, uint8_t freq_value);


int overwriting_registers(MT6835_Sensor_t *sensor, uint16_t reg_start, uint16_t reg_end);

#endif /* MT6835_H */
