//
// Created by kotik on 2/26/23.
//

#include <stdint-gcc.h>
#include "program.h"
#include "stm32f4xx.h"

#ifndef THECUBE_WS2812_BLASTER_H
#define THECUBE_WS2812_BLASTER_H

#endif //THECUBE_WS2812_BLASTER_H

typedef struct{
    Program* prog;
    TIM_HandleTypeDef* timer;
    uint16_t size;
    uint16_t * buffer;
    uint16_t * buffer_shadow;
    uint32_t channel;
} LedBlaster;

void blast(LedBlaster *blaster, int16_t red_increment, int16_t green_increment, int16_t blue_increment);
void delete_blaster(LedBlaster * blaster);
LedBlaster * create_blaster(Program* prog, TIM_HandleTypeDef* timer, uint32_t channel);

