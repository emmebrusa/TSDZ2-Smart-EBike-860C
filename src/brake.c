/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "pins.h"
#include "main.h"
#include "interrupts.h"
#include "brake.h"
#include "motor.h"

void brake_init(void) {
    // brake pin as external input pin interrupt
    GPIO_Init(BRAKE__PORT, BRAKE__PIN, GPIO_MODE_IN_FL_NO_IT); // with external interrupt
}

