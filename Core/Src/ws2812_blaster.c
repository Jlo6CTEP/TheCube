//
// Created by kotik on 2/26/23.
//

#include "ws2812_blaster.h"
#include <malloc.h>
#include <string.h>
#include "program.h"

#define RESET_CYCLE_COUNT 50

volatile uint8_t data_sent_flag = 0;
LedBlaster * create_blaster(Program* prog, TIM_HandleTypeDef* timer, uint32_t channel) {
    LedBlaster* blaster = (LedBlaster*)malloc(sizeof(LedBlaster));
    blaster->prog=prog;
    blaster->timer=timer;
    blaster->channel = channel;
    blaster->size = blaster->prog->led_count*sizeof(LED)*8+RESET_CYCLE_COUNT;
    blaster->buffer = (uint16_t *)malloc(blaster->size);
    blaster->buffer_shadow = (uint16_t *)malloc(blaster->size);

    memset(blaster->buffer, 0, blaster->size);
    memset(blaster->buffer_shadow, 0, blaster->size);
    return blaster;
}

void delete_blaster(LedBlaster * blaster) {
    free(blaster->buffer);
    free(blaster->buffer_shadow);
    free(blaster);
}

void blast(LedBlaster *blaster, int16_t red_increment, int16_t green_increment, int16_t blue_increment) {
    uint32_t buffer_prt;
    for (uint32_t frame = 0; frame < blaster->prog->prog_length; frame++) {
        buffer_prt = 0;
        for (uint32_t led_value = 0; led_value < blaster->prog->led_count; led_value++) {
            LED value = blaster->prog->led_values[frame][led_value];
            uint32_t bit_values = (
                    ((value.red + red_increment) << 16) |
                    ((value.green + green_increment) << 8) |
                    ((value.blue + blue_increment)));

            for (uint8_t bit = 0; bit < 24; bit++) {
                if ((bit_values & (1 << bit)) == 0) {
                    blaster->buffer[buffer_prt] = 30;
                } else {
                    blaster->buffer[buffer_prt] = 60;
                }
                buffer_prt++;
            }
        }

        while (data_sent_flag) {};
        data_sent_flag = 1;

        HAL_TIM_PWM_Start_DMA(
                blaster->timer,
                blaster->channel,
                (uint32_t *) blaster->buffer,
                blaster->size
        );

        uint16_t *temp = blaster->buffer;
        blaster->buffer = blaster->buffer_shadow;
        blaster->buffer_shadow = temp;
        HAL_Delay(300);
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
    }
}

void blast_line(LedBlaster *blaster, LED *line, int16_t red_increment, int16_t green_increment, int16_t blue_increment) {
    uint32_t buffer_prt = 0;
    for (uint32_t led_value = 0; led_value < blaster->prog->led_count; led_value++) {
        LED value = line[led_value];
        uint32_t bit_values = (
                ((value.red + red_increment) << 16) |
                ((value.green + green_increment) << 8) |
                ((value.blue + blue_increment)));

        for (uint8_t bit = 0; bit < 24; bit++) {
            if ((bit_values & (1 << bit)) == 0) {
                blaster->buffer[buffer_prt] = 30;
            } else {
                blaster->buffer[buffer_prt] = 60;
            }
            buffer_prt++;
        }
    }
}



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim != NULL) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        data_sent_flag = 0;
    }
}