/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, Leon, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "common.h"

int16_t map_ui16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    // if input min is smaller than output min, return the output min value
    if (x < in_min) {
        return out_min;
    }

    // if input max is bigger than output max, return the output max value
    else if (x > in_max) {
        return out_max;
    }

    // map the input to the output range, round up if mapping bigger ranges to smaller ranges
    else if ((in_max - in_min) > (out_max - out_min)) {
        return (int16_t)(((int32_t)(x - in_min) * (out_max - out_min + 1)) / (in_max - in_min + 1)) + out_min;
    }

    // map the input to the output range, round down if mapping smaller ranges to bigger ranges
    else {
        return (int16_t)(((int32_t)(x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
    }
}

uint8_t map_ui8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
    // if input min is smaller than output min, return the output min value
    if (x <= in_min) {
        return out_min;
    }

    // if input max is bigger than output max, return the output max value
    if (x >= in_max) {
        return out_max;
    }

    if (out_max < out_min)
        return (uint16_t)out_min - (uint16_t)((uint8_t)(x - in_min) * (uint8_t)(out_min - out_max)) / (uint8_t)(in_max - in_min);
    else
        return (uint16_t)out_min + (uint16_t)((uint8_t)(x - in_min) * (uint8_t)(out_max - out_min)) / (uint8_t)(in_max - in_min);
}

uint8_t ui8_min(uint8_t value_a, uint8_t value_b) {
    if (value_a < value_b) {
        return value_a;
    } else {
        return value_b;
    }
}

uint8_t ui8_max(uint8_t value_a, uint8_t value_b) {
    if (value_a > value_b)
        return value_a;
    else
        return value_b;
}

uint16_t filter(uint16_t ui16_new_value, uint16_t ui16_old_value, uint8_t ui8_alpha) {
    if (ui8_alpha < 11) {
        uint16_t ui16_filtered_value = (uint16_t)(((uint8_t)(10U - ui8_alpha) * ui16_new_value) + (uint16_t)(ui8_alpha * ui16_old_value) + 5U) / 10U;

        if (ui16_filtered_value == ui16_old_value) {
            if (ui16_filtered_value < ui16_new_value)
				ui16_filtered_value++;
			else if (ui16_filtered_value > ui16_new_value)
				ui16_filtered_value--;
        }

        return ui16_filtered_value;
    } else {
        return 0;
    }
}

// from here: https://github.com/FxDev/PetitModbus/blob/master/PetitModbus.c
/*
 * Function Name        : CRC16
 * @param[in]           : ui8_data  - Data to Calculate CRC
 * @param[in/out]       : ui16_crc   - Anlik CRC degeri
 * @How to use          : First initial data has to be 0xFFFF.
 */
void crc16(uint8_t ui8_data, uint16_t *ui16_crc) {
    unsigned int i;

    *ui16_crc = *ui16_crc ^ (uint16_t) ui8_data;

    for (i = 8; i > 0; i--) {
        if (*ui16_crc & 0x0001) {
            *ui16_crc = (*ui16_crc >> 1) ^ 0xA001;
        } else {
            *ui16_crc >>= 1;
        }
    }
}
