//
// Created by Shin on 2023/4/30.
//

#ifndef MAIN_C_TEST_FUNCTIONS_H
#define MAIN_C_TEST_FUNCTIONS_H

#include "main.h"
#include "lcd.h"
#include "touch.h"

void load_draw_dialog(void);
void lcd_draw_bline(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t size, uint16_t color);
void ctp_test(void);
void rtp_test(void);

void key_init(void);

#endif //MAIN_C_TEST_FUNCTIONS_H
