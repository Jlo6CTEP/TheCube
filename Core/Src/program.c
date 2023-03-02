//
// Created by kotik on 2/26/23.
//

#include <stdint-gcc.h>
#include <malloc.h>
#include "program.h"
Program* create_program(uint32_t prog_length, uint32_t led_count, uint8_t interpolate) {
    Program * prog = (Program *)malloc(sizeof(Program));
    prog->prog_length=prog_length;
    prog->led_count=led_count;
    prog->interpolate = interpolate;
    prog->led_values = (LED**)malloc(sizeof(LED*)*prog->prog_length);
    for (uint32_t i = 0; i < prog->prog_length; i++) {
        prog->led_values[i] = (LED*)malloc(sizeof(LED)*prog->led_count);
    }
    return prog;
}

void delete_program(Program * prog) {
    for (uint32_t i = 0; i < prog->prog_length; i++) {
        free(prog->led_values[i]);
    }
    free(prog->led_values);
    free(prog);
}