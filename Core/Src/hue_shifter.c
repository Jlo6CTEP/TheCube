//
// Created by ysuho on 08-Mar-23.
//

#include "hue_shifter.h"

uint8_t clip(int16_t d) {
    const uint16_t t = d < 0 ? 0 : d;
    return t > UINT8_MAX ? UINT8_MAX : t;
}

void calculate_matrix() {
    if (shift.timer->Instance->CNT == shift.old_hue) {
        return;
    }
    force_calculate_matrix();
}

void force_calculate_matrix() {
    uint32_t hue = shift.timer->Instance->CNT;
    float32_t cosA = arm_cos_f32((float32_t)hue / 360 * M_TWOPI);
    float32_t sinA = arm_sin_f32((float32_t)hue / 360 * M_TWOPI);

    shift.matrix[0][0] = cosA + (1.0 - cosA) / 3.0;
    shift.matrix[0][1] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    shift.matrix[0][2] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    shift.matrix[1][0] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    shift.matrix[1][1] = cosA + 1. / 3. * (1.0 - cosA);
    shift.matrix[1][2] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    shift.matrix[2][0] = 1. / 3. * (1.0 - cosA) - sqrt(1. / 3.) * sinA;
    shift.matrix[2][1] = 1. / 3. * (1.0 - cosA) + sqrt(1. / 3.) * sinA;
    shift.matrix[2][2] = cosA + 1. / 3. * (1.0 - cosA);
}


Shifter * init_shifter(TIM_HandleTypeDef *timer) {
    shift.timer = timer;
    force_calculate_matrix();
    shift.old_hue = shift.timer->Instance->CNT;
    return &shift;
}

void shift_hue(LED *src_led, LED *tgt_led) {
    tgt_led->red = clip(shift.matrix[0][0] * src_led->red + shift.matrix[0][1] * src_led->green + shift.matrix[0][0] * src_led->blue);
    tgt_led->green = clip(shift.matrix[1][0] * src_led->red + shift.matrix[1][1] * src_led->green + shift.matrix[1][0] * src_led->blue);
    tgt_led->blue = clip(shift.matrix[2][0] * src_led->red + shift.matrix[2][1] * src_led->green + shift.matrix[2][0] * src_led->blue);
}