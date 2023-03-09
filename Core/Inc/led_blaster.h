//
// Created by kotik on 3/3/23.
//

#ifndef THECUBE_LED_BLASTER_H
#define THECUBE_LED_BLASTER_H

#include <malloc.h>
#include <memory.h>
#include <limits.h>
#include "stm32f4xx.h"
#include "error_codes.h"
#include "leds.h"
#include "color_processing.h"

#define TIMER_CNT_WIDTH uint16_t
#define MAX_ALLOCATABLE_MEMORY (128*1024)
#define RESET_PULSE_COUNT 50

/**
 *  @struct This struct holds all the data needed to perform DMA requests as well as interpolations
 */
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t timer_chanel;
    DMA_HandleTypeDef dma_handle;

    Frame *frames;
    TIMER_CNT_WIDTH *buffer;
    TIMER_CNT_WIDTH *shadow_buffer;
    uint16_t dma_buffer_len;
    uint16_t program_len;
    uint16_t led_count;

    LED *interpolation_buffer;
    LED *this_hue_shift_buffer;
    LED *next_hue_shift_buffer;
    ColorProcessor *hue_shifter;
    uint8_t desired_fps;
} Program;

/**
 * @note This function is supposed to be called only once to prevent frequent memory allocations and de-allocations
 * It won't break if called multiple times, though. Just beware of heap fragmentation and the fact that it will destroy previous program
 * @param timer handle for STM32 timer of choice
 * @param channel channel number
 * @param led_count how many LEDs there is in the line
 * @param dma_handle handle for DMA
 * @param program_length how many frames there is in the program
 * @param bits_per_led number of bits one LED unit consumes. In most cases (RGB leds) it is 24 bits, 8 for each color
 * @param interpolation_steps interpolate between this and next frames in this number of steps. Allows better FPS and smaller program size
 * @param is_error check whether the initialization was a success
 * @return initialized program struct
 */
Program *init_program(TIM_HandleTypeDef *timer, uint32_t channel, DMA_HandleTypeDef dma_handle, uint16_t led_count,
                      uint16_t program_length, uint8_t bits_per_led, uint8_t desired_fps, uint8_t * is_error, ColorProcessor * hue_shifter);

uint8_t safe_add(int16_t a, int16_t b);

uint8_t send_leds_to_dma(LED *leds);

uint8_t blast();
uint8_t blast_one_frame(uint16_t frame);

#endif //THECUBE_LED_BLASTER_H