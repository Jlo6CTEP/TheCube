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
    uint8_t brightness = (uint8_t)*processor.brightness_source;
    uint8_t red = ((LED*)src_led)->red;
    uint8_t green = ((LED*)src_led)->green;
    uint8_t blue = ((LED*)src_led)->blue;
    ((LED*)tgt_led)->red = clip((
            processor.matrix[0][0] * red +
            processor.matrix[0][1] * green +
            processor.matrix[0][0] * blue) * ((float32_t)brightness / MAX_BRIGHTNESS));
    ((LED*)tgt_led)->green = clip((
            processor.matrix[1][0] * red +
            processor.matrix[1][1] * green +
            processor.matrix[1][0] * blue) * ((float32_t)brightness / MAX_BRIGHTNESS));
    ((LED*)tgt_led)->blue = clip((
            processor.matrix[2][0] * red +
            processor.matrix[2][1] * green +
            processor.matrix[2][0] * blue) * ((float32_t)brightness / MAX_BRIGHTNESS));
}
