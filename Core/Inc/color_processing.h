//
// Created by ysuho on 08-Mar-23.
//

#ifndef THECUBE_COLOR_PROCESSING_H
#define THECUBE_COLOR_PROCESSING_H

#include <stdint-gcc.h>
#include "stm32f4xx.h"
#include "arm_math.h"
#include "leds.h"
#define MAX_BRIGHTNESS 255

typedef struct {
    volatile uint32_t *hue_source;
    volatile uint32_t *brightness_source;
    uint16_t old_hue;
    float32_t matrix [3][3];
} ColorProcessor;

void force_calculate_matrix();
void calculate_matrix();
ColorProcessor * init_processor(volatile uint32_t *hue_source, volatile uint32_t *brightness_source);
uint8_t clip(int16_t d);
void process(const uint8_t src_led[3], const uint8_t tgt_led[3]);



#endif //THECUBE_COLOR_PROCESSING_H
