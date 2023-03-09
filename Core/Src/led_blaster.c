#include "led_blaster.h"
//
// Created by ysuho on 04-Mar-23.
//

static TIMER_CNT_WIDTH *buffer_ptr;
static Program prog = {0};

/**
 * @note This function is supposed to be called only once to prevent frequent memory allocations and de-allocations
 * It won't break if called multiple times, though. Just beware of heap fragmentation and the fact that it will destroy previous program
 * @param timer handle for STM32 timer of choice
 * @param channel channel number
 * @param led_count how many LEDs there is in the line
 * @param program_length how many frames there is in the program
 * @param bits_per_led number of bits one LED unit consumes. In most cases (RGB leds) it is 24 bits, 8 for each color
 * @param interpolation_steps interpolate between this and next frames in this number of steps. Allows better FPS and smaller program size
 * @param desired_fps target fps to change frames at
 * @return
 */
Program *init_program(TIM_HandleTypeDef * timer, uint32_t channel, DMA_HandleTypeDef dma_handle, uint16_t led_count,
                      uint16_t program_length, uint8_t bits_per_led, uint8_t desired_fps, uint8_t * is_error, ColorProcessor * hue_shifter) {
    uint8_t safe_is_error = 0;
    if (is_error == NULL) {
        is_error = &safe_is_error;
    }

    if (timer == NULL) {
        *is_error = PROGRAM_INIT_ERROR;
        return NULL;
    }

    prog.timer = timer;
    prog.timer_chanel = channel;
    prog.program_len = program_length;
    prog.led_count = led_count;
    prog.desired_fps = desired_fps;
    prog.dma_handle = dma_handle;

    // handle the repeated call case
    prog.dma_buffer_len = led_count * bits_per_led * sizeof(TIMER_CNT_WIDTH) + RESET_PULSE_COUNT;
    prog.buffer = realloc(prog.buffer, prog.dma_buffer_len);
    prog.shadow_buffer = realloc(prog.shadow_buffer, prog.dma_buffer_len);
    prog.interpolation_buffer = realloc(prog.interpolation_buffer, led_count * sizeof(LED));
    prog.this_hue_shift_buffer = realloc(prog.interpolation_buffer, led_count * sizeof(LED));
    prog.next_hue_shift_buffer = realloc(prog.interpolation_buffer, led_count * sizeof(LED));
    prog.hue_shifter = hue_shifter;

    if (prog.buffer == NULL || prog.shadow_buffer == NULL || prog.interpolation_buffer == NULL) {
        *is_error = PROGRAM_INIT_ERROR;
        return NULL;
    }

    memset(prog.buffer, 0, prog.dma_buffer_len);
    memset(prog.shadow_buffer, 0, prog.dma_buffer_len);
    memset(prog.interpolation_buffer, 0, led_count * sizeof(LED));
    memset(prog.this_hue_shift_buffer, 0, led_count * sizeof(LED));
    memset(prog.next_hue_shift_buffer, 0, led_count * sizeof(LED));

    if ((program_length * sizeof(LED) + sizeof(Frame) + sizeof(Frame *)) * led_count > MAX_ALLOCATABLE_MEMORY) {
        *is_error = PROGRAM_INIT_ERROR;
        return NULL;
    }
    prog.frames = realloc(prog.frames, sizeof (Frame) * program_length);
    memset(prog.frames, 0, sizeof (Frame) * program_length);
    for (uint16_t i=0; i < program_length; i++) {
        prog.frames[i].leds = (LED *) realloc(prog.frames[i].leds, led_count * sizeof(LED));
        if (prog.frames[i].leds == NULL) {
            *is_error = PROGRAM_INIT_ERROR;
            return NULL;
        }

        memset(prog.frames[i].leds, 0, led_count * sizeof(LED));
    }
    *is_error = NO_ERROR;
    return &prog;
}

uint8_t safe_add(int16_t a, int16_t b)
{
    if (a >= 0) {
        if (b > (UINT8_MAX - a)) {
            return UINT8_MAX;
        }
    } else {
        if (b < (0 - a)) {
            return 0;
        }
    }
    return a + b;
}

uint8_t send_leds_to_dma(LED *leds) {
    buffer_ptr = prog.buffer;

    while (prog.dma_handle.State != HAL_DMA_STATE_READY) {}

    for (uint32_t led_value = 0; led_value < prog.led_count; led_value++) {
        LED value = leds[led_value];
        uint32_t bit_values = (
                (value.red << 16) |
                (value.green << 8) |
                (value.blue));

        for (int8_t bit = 23; bit >= 0; bit--) {
            if ((bit_values & (1 << bit)) == 0) {
                *buffer_ptr = 30;
            } else {
                *buffer_ptr = 60;
            }
            buffer_ptr++;
        }
    }

    uint8_t result = HAL_TIM_PWM_Start_DMA(
            prog.timer,
            prog.timer_chanel,
            (uint32_t *) prog.buffer,
            prog.dma_buffer_len / 2
    );
    if (result == HAL_ERROR) {
        return result;
    }

    uint16_t *temp = prog.buffer;
    prog.buffer = prog.shadow_buffer;
    prog.shadow_buffer = temp;
    return NO_ERROR;
}

uint8_t blast_one_frame(uint16_t frame) {
    calculate_matrix();

    if (frame == 0) {
        // on the first iteration, both this and next buffers should be generated
        // they are needed for interpolation
        for (uint32_t led=0; led < prog.led_count; led++){
            process((uint8_t*)&prog.frames[frame].leds[led], (uint8_t*)&prog.this_hue_shift_buffer[led]);
        }
    } else {
        // when the iteration advances, next becomes this
        // and next should be generated
        prog.this_hue_shift_buffer = prog.next_hue_shift_buffer;
    }

    if (frame + 1 != prog.program_len) {
        for (uint32_t led = 0; led < prog.led_count; led++) {
            process((uint8_t*)&prog.frames[frame + 1].leds[led], (uint8_t*)&prog.next_hue_shift_buffer[led]);
        }
    }

    if (!prog.frames[frame].is_interpolated || (frame + 1 == prog.program_len)) {
        // easy case, interpolation is not needed, just send it right away
        uint8_t result = send_leds_to_dma(prog.this_hue_shift_buffer);
        if (result != NO_ERROR) {
            return result;
        }
        HAL_Delay(1000 / (prog.desired_fps));
    } else {
        // here we will calculate interpolation on the fly
        // and send em to the DMA
        uint16_t interpolation_steps = prog.frames[frame].interpolation_steps;
        for (uint16_t step = 0; step < interpolation_steps; step++) {
            for (uint16_t led_value = 0; led_value < prog.led_count; led_value++) {
                LED *this = prog.this_hue_shift_buffer;
                LED *next = prog.next_hue_shift_buffer;

                int16_t red_diff = (int16_t) next[led_value].red - (int16_t) this[led_value].red;
                int16_t green_diff = (int16_t) next[led_value].green - (int16_t) this[led_value].green;
                int16_t blue_diff = (int16_t) next[led_value].blue - (int16_t) this[led_value].blue;

                prog.interpolation_buffer[led_value].red = safe_add(
                        this[led_value].red,
                        (float) red_diff * step / interpolation_steps);
                prog.interpolation_buffer[led_value].green = safe_add(
                        this[led_value].green,
                        (float) green_diff * step / interpolation_steps);
                prog.interpolation_buffer[led_value].blue = safe_add(
                        this[led_value].blue,
                        (float) blue_diff * step / interpolation_steps);
            }
            HAL_Delay(1000. / (prog.desired_fps * interpolation_steps));
            uint8_t result = send_leds_to_dma(
                    prog.interpolation_buffer);
            if (result != NO_ERROR) {
                return result;
            }
        }
    }
    return NO_ERROR;
}

uint8_t blast() {
    for (uint32_t frame = 0; frame < prog.program_len; frame++) {
        uint8_t result = blast_one_frame(frame);
        if (result != NO_ERROR) {
            return result;
        }
    }
    return NO_ERROR;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim != NULL) {
        HAL_TIM_PWM_Stop_DMA(htim, prog.timer_chanel);
    }
}