//
// Created by ysuho on 08-Mar-23.
//

#ifndef THECUBE_LEDS_H
#define THECUBE_LEDS_H

#include <stdint-gcc.h>

typedef struct {
    uint8_t green;
    uint8_t red;
    uint8_t blue;
} LED;

typedef struct {
    LED *leds;
    uint8_t is_interpolated;
    uint8_t interpolation_steps;
    uint16_t time;
} Frame;
#endif //THECUBE_LEDS_H
