//
// Created by ysuho on 08-Mar-23.
//

#ifndef THECUBE_HUE_SHIFTER_H
#define THECUBE_HUE_SHIFTER_H

#include <stdint-gcc.h>
#include "stm32f4xx.h"
#include "arm_math.h"
#include "leds.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint16_t old_hue;
    float32_t matrix [3][3];
} Shifter;

static Shifter shift = {0};

void force_calculate_matrix();
void calculate_matrix();
Shifter * init_shifter(TIM_HandleTypeDef *timer);
uint8_t clip(int16_t d);
void shift_hue(LED *src_led, LED *tgt_led);



#endif //THECUBE_HUE_SHIFTER_H
