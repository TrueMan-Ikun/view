#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <stdint.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT   64

void OLED_Init(void);
void OLED_Clear(void);
void OLED_Fill(uint8_t data);
void OLED_Refresh(void);

void OLED_SetCursor(uint8_t x, uint8_t page);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, int num);

#endif /* __OLED_H */
