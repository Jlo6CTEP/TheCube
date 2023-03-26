//
// Created by ysuho on 08-Mar-23.
//

#include "color_processing.h"
static ColorProcessor processor = {0};

uint8_t clip(int16_t d) {
    const uint16_t t = d < 0 ? 0 : d;
    return t > UINT8_MAX ? UINT8_MAX : t;
}

void calculate_matrix() {
    if (*processor.hue_source == processor.old_hue) {
        return;
    }
    force_calculate_matrix();
}

void force_calculate_matrix() {
    uint32_t hue = *processor.hue_source;
    float32_t cosA = arm_cos_f32((float32_t)hue / 360 * M_TWOPI);
    float32_t sinA = arm_sin_f32((float32_t)hue / 360 * M_TWOPI);

    processor.matrix[0][0] = cosA + (1.0 - cosA) / 3.0;
    processor.matrix[0][1] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    processor.matrix[0][2] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    processor.matrix[1][0] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    processor.matrix[1][1] = cosA + 1. / 3. * (1.0 - cosA);
    processor.matrix[1][2] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    processor.matrix[2][0] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    processor.matrix[2][1] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    processor.matrix[2][2] = cosA + 1. / 3. * (1.0 - cosA);
}


ColorProcessor * init_processor(volatile uint32_t *hue_source, volatile uint32_t *brightness_source) {
    processor.hue_source = hue_source;
    processor.brightness_source = brightness_source;
    force_calculate_matrix();
    processor.old_hue = *processor.hue_source;
    return &processor;
}

void process(const uint8_t src_led[3], const uint8_t tgt_led[3]) {

    uint8_t brightness = 0;
    // Very big overflow, probably rollover from 0 to timer max, clamp to 0
    uint32_t brightness_shadow = *processor.brightness_source >> 2;
    if (brightness_shadow > UINT8_MAX * 2) {
        *processor.brightness_source = 0;
        brightness = 0;
    } else if (brightness_shadow > UINT8_MAX) {
        // overflow, so clamp to max UINT8
        *processor.brightness_source = UINT8_MAX << 2;
        brightness = UINT8_MAX;
    } else {
        brightness = brightness_shadow;
    }
    uint8_t temp_red = ((LED*)src_led)->red;
    uint8_t temp_green = ((LED*)src_led)->green;
    uint8_t temp_blue = ((LED*)src_led)->blue;
    float32_t red = (
            processor.matrix[0][0] * temp_red +
            processor.matrix[0][1] * temp_green +
            processor.matrix[0][0] * temp_blue);
    float32_t green = (
            processor.matrix[1][0] * temp_red +
            processor.matrix[1][1] * temp_green +
            processor.matrix[1][0] * temp_blue);
    float32_t blue = (
            processor.matrix[2][0] * temp_red +
            processor.matrix[2][1] * temp_green +
            processor.matrix[2][0] * temp_blue);

    uint8_t red_scaled = clip(red * ((float32_t)brightness / MAX_BRIGHTNESS));
    uint8_t green_scaled = clip(green * ((float32_t)brightness / MAX_BRIGHTNESS));
    uint8_t blue_scaled = clip(blue * ((float32_t)brightness / MAX_BRIGHTNESS));

    ((LED*)tgt_led)->red = (red != 0 && red_scaled == 0 && brightness != 0) ? 1 : red_scaled;
    ((LED*)tgt_led)->green = (green != 0 && green_scaled == 0 && brightness != 0) ? 1 : green_scaled;
    ((LED*)tgt_led)->blue = (blue != 0 && blue_scaled == 0 && brightness != 0) ? 1 : blue_scaled;

}
