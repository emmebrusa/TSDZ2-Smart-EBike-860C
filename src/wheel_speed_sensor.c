/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "pins.h"
#include "main.h"
#include "interrupts.h"
#include "motor.h"

void wheel_speed_sensor_init(void) {
    //wheel speed sensor pin as input pull-up, no external interrupt
    GPIO_Init(WHEEL_SPEED_SENSOR__PORT, WHEEL_SPEED_SENSOR__PIN, GPIO_MODE_IN_PU_NO_IT);
}

