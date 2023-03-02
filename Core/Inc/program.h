//
// Created by kotik on 2/26/23.
//

#ifndef THECUBE_PROGRAM_H
#define THECUBE_PROGRAM_H
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LED;

typedef struct {
    uint32_t prog_length;
    uint32_t led_count;
    LED** led_values;
    uint8_t interpolate;
} Program;

Program* create_program(uint32_t prog_length, uint32_t led_count, uint8_t interpolate);
void delete_program(Program * prog);

#endif //THECUBE_PROGRAM_H
