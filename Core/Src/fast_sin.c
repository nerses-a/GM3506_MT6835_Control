/*
 * fast_sin.c
 *
 *  Created on: May 12, 2025
 *      Author: sonno
 */

#include "fast_sin.h"


float fastSin(float x) {

    // Нормализация x в диапазон [0, 2π)
    while (x < 0) x += FAST_SIN_TWO_PI;
    while (x >= FAST_SIN_TWO_PI) x -= FAST_SIN_TWO_PI;

    // Перевод угла в индекс
    int id_x = (int)(x * (FAST_SIN_TABLE_SIZE / FAST_SIN_TWO_PI) + 0.5f); // округление до ближайшего
    if (id_x >= FAST_SIN_TABLE_SIZE) id_x = 0; // защита от переполнения

    return sin_table[id_x];
}


