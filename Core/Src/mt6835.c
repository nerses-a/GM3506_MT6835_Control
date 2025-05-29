#include "mt6835.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

// Иницилизация mt6835
int MT6835_Init(MT6835_Sensor_t *sensor, SPI_HandleTypeDef *spi,
                GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *cal_port,
                uint16_t cal_pin) {
  // Проверка входных параметров
  if (!sensor || !spi || !cs_port) {
    return MT6835_ERROR; // Недопустимые параметры
  }
  if (!cal_port && cal_pin != 0) {
    return MT6835_ERROR; // Если cal_port не используется, cal_pin должен быть 0
  }

  // Инициализация структуры
  sensor->spi = spi;
  sensor->cs_port = cs_port;
  sensor->cs_pin = cs_pin;
  sensor->cal_port = cal_port;
  sensor->cal_pin = cal_pin;

  // Если CAL_EN используется, выключаем его
  if (sensor->cal_port) {
    HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_RESET);
  }

  // Деактивируем CS по умолчанию
  HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

  return MT6835_OK;
}

// Функция передачи SPI
int  MT6835_TransmitReceive(MT6835_Sensor_t *sensor, uint8_t *tx_data, uint8_t *rx_data, uint8_t length, uint32_t timeout) {
    // Проверка входных параметров
    if (!sensor || !tx_data || !rx_data || length == 0) {
        return MT6835_ERROR;  // Недопустимые параметры
    }

    // Активировать CS (активный уровень — LOW)
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_RESET);

    // Передача и прием данных
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(sensor->spi, tx_data, rx_data, length, timeout);

    // Деактивировать CS
    HAL_GPIO_WritePin(sensor->cs_port, sensor->cs_pin, GPIO_PIN_SET);

    // Проверка результата
    return (result == HAL_OK) ? MT6835_OK : MT6835_ERROR;
}


int MT6835_ReadRegister(MT6835_Sensor_t *sensor, uint16_t address, uint8_t *value) {
    // Проверка входных параметров
    if (!sensor || !value) {
        return MT6835_ERROR;  // Недопустимые параметры
    }

    // Формируем команду: 4 бита команды (0b0011), 12 бит адреса, 8 бит данных
    uint8_t tx_data[3];
    tx_data[0] = (MT6835_CMD_READ_REG << 4) | ((address >> 8) & MT6835_MASK_ADDR);  // 0b0011xxxx, старшие 4 бита адреса
    tx_data[1] = address & 0xFF;  // Младшие 8 бит адреса
    tx_data[2] = 0x00;            // 8 бит данных (не используются при чтении)

    uint8_t rx_data[3] = {0};

    // Передача и прием данных
    int result = MT6835_TransmitReceive(sensor, tx_data, rx_data, 3, HAL_MAX_DELAY);
    if (result == MT6835_OK) {
        *value = rx_data[2];  // Получаем последний байт ответа
    }

    return result;
}

int MT6835_WriteRegister(MT6835_Sensor_t *sensor, uint16_t address, uint8_t data) {
    // Проверка входных параметров
    if (!sensor) {
        return MT6835_ERROR;  // Недопустимые параметры
    }

    // Формируем команду: 4 бита команды (0b0110), 12 бит адреса, 8 бит данных
    uint8_t tx_data[3];
    tx_data[0] = (MT6835_CMD_WRITE_REG << 4) | ((address >> 8) & MT6835_MASK_ADDR);  // 0b0110xxxx, старшие 4 бита адреса
    tx_data[1] = address & 0xFF;  // Младшие 8 бит адреса
    tx_data[2] = data;            // 8 бит данных

    uint8_t rx_data[3] = {0};

    // Передача и прием данных
    int result = MT6835_TransmitReceive(sensor, tx_data, rx_data, 3, HAL_MAX_DELAY);
    return (result == MT6835_OK) ? MT6835_OK : MT6835_ERROR;
}


uint8_t MT6835_CheckStatus(MT6835_Sensor_t *sensor) {
	uint8_t status_reg;
	if(MT6835_ReadRegister(sensor, MT6835_ADDR_REG_STATUS, &status_reg)) {
		return MT6835_ERROR;
	}

	return status_reg & MT6835_MASK_STATUS;  // Статус в последних 3 битах
}


// Функция программирования EEPROM
int MT6835_ProgramEEPROM(MT6835_Sensor_t *sensor) {
    uint8_t tx_data[3] = {MT6835_CMD_WRITE_EEPROM << 4, 0x00, 0x00};
    uint8_t rx_data[3] = {0};

    if (MT6835_TransmitReceive(sensor, tx_data, rx_data, 3, HAL_MAX_DELAY) != MT6835_OK) {
        return MT6835_ERROR;  // Ошибка SPI
    }

    if (rx_data[2] != MT6835_SUCCESS_RESPONSE) {
        return MT6835_ERROR;  // Ошибка программирования EEPROM
    }

    HAL_Delay(10000);  // Ожидание завершения программирования
    return MT6835_OK;
}


int MT6835_SetZeroTemporary(MT6835_Sensor_t *sensor) {
    if (!sensor) {
        return MT6835_ERROR;  // Проверка на NULL
    }

    uint8_t tx_data[3] = {(MT6835_CMD_SET_ZERO << 4), 0x00, 0x00};
    uint8_t rx_data[3] = {0};

    // Передача команды
    if (MT6835_TransmitReceive(sensor, tx_data, rx_data, 3, HAL_MAX_DELAY) != MT6835_OK) {
        return MT6835_ERROR;
    }

    // Проверка успешности установки нуля (датчик должен вернуть 0x55)
    return (rx_data[2] == MT6835_SUCCESS_RESPONSE) ? MT6835_OK : MT6835_ERROR;
}


// Основная функция установки нуля
int MT6835_SetZero(MT6835_Sensor_t *sensor) {

    // Временная установка нуля
    int result = MT6835_SetZeroTemporary(sensor);

    if (result != MT6835_OK) {
        return MT6835_ERROR;  // Ошибка установки нуля
    }

    HAL_Delay(10);  // Задержка перед записью в EEPROM

    // Программирование EEPROM
    return MT6835_ProgramEEPROM(sensor);
}


 // Чтение сырого значения угла
uint32_t MT6835_ReadRawAngle(MT6835_Sensor_t *sensor) {
    uint8_t tx_data[6] = {MT6835_CMD_READ_ANGLE << 4,  MT6835_ADDR_REG_ANGLE_MSB, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_data[6] = {0};

    if (MT6835_TransmitReceive(sensor, tx_data, rx_data, 6, HAL_MAX_DELAY) != MT6835_OK) {
            return MT6835_ANGLE_READ_ERROR;  // Ошибка SPI / чтения
    }

    // Получаем 21-битное значение угла
    uint32_t angle = ((uint32_t)rx_data[2] << 13) | ((uint32_t)rx_data[3] << 5) | ((uint32_t)rx_data[4] >> 3);

    return angle;
}


// Чтение угла в радианах
float MT6835_ReadAngle(MT6835_Sensor_t *sensor) {
    uint32_t raw_angle = MT6835_ReadRawAngle(sensor);

    if (raw_angle & MT6835_ANGLE_READ_ERROR) {
        return MT6835_ERROR;
    }

    return (raw_angle / (float)(1 << 21)) * 2.0 * MT6835_PI;
}

// Чтение угла в градусах
float MT6835_ReadAngleDegrees(MT6835_Sensor_t *sensor) {
    uint32_t raw_angle = MT6835_ReadRawAngle(sensor);

    if (raw_angle & MT6835_ANGLE_READ_ERROR) {
        return MT6835_ERROR ;
    }

    return (raw_angle / (float)(1 << 21)) * 360.0;
}

float MT6835_CalculateRotationSpeed(MT6835_Sensor_t *sensor,
                                   uint32_t timer_freq_hz,
                                   TIM_HandleTypeDef *htim)
{
    // 1. Проверка входных параметров
    if(sensor == NULL || htim == NULL || timer_freq_hz == 0) {
        return 0.0;
    }

    // 2. Первое измерение
    float angle1 = MT6835_ReadAngle(sensor);
    uint32_t time1 = __HAL_TIM_GET_COUNTER(htim);

    // 3. Задержка (оптимизированная вместо HAL_Delay)
    uint32_t start = HAL_GetTick();
    while((HAL_GetTick() - start) < MT6835_MEASURE_DELAY_MS) {
        __NOP();
    }

    // 4. Второе измерение
    float angle2 = MT6835_ReadAngle(sensor);
    uint32_t time2 = __HAL_TIM_GET_COUNTER(htim);

    // 5. Расчет разницы времени с обработкой переполнения
    uint32_t delta_time_ticks = (time2 >= time1) ?
                               (time2 - time1) :
                               (htim->Instance->ARR - time1 + time2 + 1);
    float delta_time = (float)delta_time_ticks / (float)timer_freq_hz;

    // 6. Коррекция угловой разницы
    float delta_angle = angle2 - angle1;

    // Обработка перехода через 0
    if(delta_angle > MT6835_PI) {
        delta_angle -= 2.0 * MT6835_PI;
    }
    else if(delta_angle < -MT6835_PI) {
        delta_angle += 2.0 * MT6835_PI;
    }

    // 7. Фильтрация шумов и проверка на движение
    if(MT6835_ABS(delta_angle) < MT6835_MIN_DELTA_ANGLE) {
        return 0.0;
    }

    // 8. Расчет скорости с защитой от деления на 0
    float speed = delta_angle / delta_time;

    // 9. Ограничение по максимальной скорости
    if(MT6835_ABS(speed) > MT6835_MAX_SPEED) {
        speed = (speed > 0) ? MT6835_MAX_SPEED : -MT6835_MAX_SPEED;
    }

    return speed;
}

float MovingAverage_FilterSpeed(float new_speed) {
    // Статический буфер и переменные состояния
    static float buffer[MT6835_FILTER_SIZE] = {0};
    static uint8_t index = 0;
    static uint8_t samples_collected = 0;
    float sum = 0;

    // 1. Добавляем новое значение в буфер
    buffer[index] = new_speed;

    // 2. Обновляем индекс (циклически)
    index = (index + 1) %  MT6835_FILTER_SIZE;

    // 3. Увеличиваем счетчик (пока буфер не заполнится)
    if (samples_collected <  MT6835_FILTER_SIZE) {
        samples_collected++;
    }

    // 4. Суммируем значения в буфере
    for (uint8_t i = 0; i < samples_collected; ++i) {
        sum += buffer[i];
    }

    // 5. Возвращаем среднее значение
    return sum / (float)samples_collected;
}

float MT6835_CalculateRotationSpeedRPM(MT6835_Sensor_t *sensor,
                                        uint32_t timer_freq_hz,
                                        TIM_HandleTypeDef *htim) {

  float speed_rpm =
      (MT6835_CalculateRotationSpeed(sensor, timer_freq_hz, htim) /
       (2 * MT6835_PI)) *
      60;

  return speed_rpm;
}



int MT6835_SetAutoCalFreq(MT6835_Sensor_t *sensor, uint8_t freq_value) {
    if (freq_value > 0x07) {
    	printf("Error: Invalid AUTOCAL_FREQ value.\r\n");
        return MT6835_ERROR;
    }

    uint8_t reg_value = 0;
    if (MT6835_ReadRegister(sensor, MT6835_ADDR_REG_AUTOCAL_FREQ, &reg_value) != MT6835_OK) {
    	printf("Error: Failed to read register 0x00E.\r\n");
        return MT6835_ERROR;
    }

    // Очистим биты AUTOCAL_FREQ[2:0] (биты 6, 5, 4) и установим новое значение
    reg_value &= MT6835_MASK_AUTOCAL_FREQ_CLEAR;               // 0b10001111 — сброс бит 6, 5, 4
    reg_value |= (freq_value << 4);   // Установка нового значения

    // Запись обновленного значения обратно
    if (MT6835_WriteRegister(sensor, MT6835_ADDR_REG_AUTOCAL_FREQ, reg_value) != MT6835_OK) {
    	 printf("Error: Failed to write register 0x00E.\r\n");
        return MT6835_ERROR;
    }

    printf("AUTOCAL_FREQ successfully set to 0x%X.\r\n", freq_value);
    return 0;
}

int MT6835_AutoCalibration(MT6835_Sensor_t *sensor, uint8_t freq_value) {
  // Устанавливаем скорость вращения
  if (MT6835_SetAutoCalFreq(sensor, freq_value) != MT6835_OK) {
    printf("Error: Failed to set auto-calibration frequency.\r\n");
    return MT6835_ERROR;
  }

  printf("Starting auto-calibration...\r\n");

  // Активируем CAL_EN (Pin 4) для начала калибровки
  HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_SET);
  HAL_Delay(10);

  printf("CAL_EN activated and reset.\r\n");

  uint8_t status;
  uint32_t timeout = HAL_GetTick() + MT6835_AUTOCAL_TIMEOUT_MS;

  // Ожидаем завершения калибровки или истечения таймаута
  while (HAL_GetTick() < timeout) {
    // Читаем регистр статуса калибровки
    if (MT6835_ReadRegister(sensor, MT6835_ADDR_REG_AUTOCAL_STATUS, &status) !=
        MT6835_OK) {
      printf("Error: Failed to read calibration status.\r\n");
      HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_RESET);
      return MT6835_ERROR;
    }

    // Извлекаем статус калибровки
    uint8_t cal_status =
        (status >> MT6835_AUTOCAL_STATUS_SHIFT) & MT6835_MASK_AUTOCAL_STATUS;

    switch (cal_status) {
    case MT6835_AUTOCAL_IN_PROGRESS:
      printf("Calibration in progress...\r\n");
      break;
    case MT6835_AUTOCAL_FAILED:
      printf("Calibration failed.\r\n");
      HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_RESET);
      return MT6835_ERROR;
    case MT6835_AUTOCAL_SUCCESS:
      printf("Calibration successful.\r\n");
      HAL_Delay(10);
      HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_RESET);
      return MT6835_ProgramEEPROM(sensor); // Сохранение настроек в EEPROM
    }

    // Ожидаем перед следующей проверкой статуса
    HAL_Delay(MT6835_AUTOCAL_STATUS_CHECK_MS);
  }

  printf("Calibration timeout reached.\r\n");
  HAL_GPIO_WritePin(sensor->cal_port, sensor->cal_pin, GPIO_PIN_RESET);
  return MT6835_ERROR;
}
