#ifndef __ST7789_H
#define __ST7789_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32h7xx_hal.h"

void LCD_Init(void);
void LCD_Clear(uint16_t color);
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//������Ļ��ʾ����
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);
void LCD_WR_REG(uint8_t dat);//��������
void LCD_WR_DATA(uint16_t dat);//����16λ����
void LCD_WR_DATA8(uint8_t dat);//��������
#endif /* RP2040_MOUSE_ST7789_ */
