/*
 * Copyright (c) 2017 - 2018 , NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LCD_H__
#define __LCD_H__

/* -------------------------------------------------------------------------- */
/*                                   Includes                                 */
/* -------------------------------------------------------------------------- */
#include <stdlib.h>
#include "stdint.h"
#include "fsl_gpio.h"
#include "fsl_spi.h"

/* -------------------------------------------------------------------------- */
/*                                   Defines                                  */
/* -------------------------------------------------------------------------- */
#define LCD_WIDTH    240
#define LCD_HEIGHT   320

/*+--------- X ------------> LCD_WIDTH 
  |
  Y         SCREEN
  |
  v
LCD_HEIGHT
*/

/* LCD Font type */
#define LCD_FONT_1206            12
#define LCD_FONT_1608            16
/* LCD Color Data */
#define LCD_COLOR_WHITE          0xFFFF
#define LCD_COLOR_BLACK          0x0000     
#define LCD_COLOR_BLUE           0x001F  
#define LCD_COLOR_BRED           0XF81F
#define LCD_COLOR_GRED           0XFFE0
#define LCD_COLOR_GBLUE          0X07FF
#define LCD_COLOR_RED            0xF800
#define LCD_COLOR_MAGENTA        0xF81F
#define LCD_COLOR_GREEN          0x07E0
#define LCD_COLOR_CYAN           0x7FFF
#define LCD_COLOR_YELLOW         0xFFE0
#define LCD_COLOR_BROWN          0XBC40 
#define LCD_COLOR_BRRED          0XFC07 
#define LCD_COLOR_GRAY           0X8430 
#define LCD_COLOR_CUSTOM1        0xFCC6
/* LCD command or data define */
#define LCD_CMD                  0
#define LCD_DATA                 1

/* -------------------------------------------------------------------------- */
/*                          Hardware Pin configuration                        */
/* -------------------------------------------------------------------------- */

#define LCD_SPI                  SPI8

#define LCD_CS_PORT              1u
#define LCD_CS_PIN               1u

#define LCD_BKL_PORT             1u
#define LCD_BKL_PIN              5u

#define LCD_DC_PORT              1u
#define LCD_DC_PIN               9u

#define LCD_CLK_PORT             1u
#define LCD_CLK_PIN              2u

#define LCD_TX_PORT              0u
#define LCD_TX_PIN               26u

#define LCD_TP_PORT              1u
#define LCD_TP_PIN               7u

#define LCD_CS_SET()             GPIO_PinWrite(GPIO, LCD_CS_PORT, LCD_CS_PIN, 1u)    /* LCD spi select set high */
#define LCD_CS_CLR()             GPIO_PinWrite(GPIO, LCD_CS_PORT, LCD_CS_PIN, 0u)    /* LCD spi select set low */

#define LCD_BKL_SET()            GPIO_PinWrite(GPIO, LCD_BKL_PORT, LCD_BKL_PIN, 1u)  /* LCD backlight enable */
#define LCD_BKL_CLR()            GPIO_PinWrite(GPIO, LCD_BKL_PORT, LCD_BKL_PIN, 0u)  /* LCD backlight disable */

#define LCD_DC_SET()             GPIO_PinWrite(GPIO, LCD_DC_PORT, LCD_DC_PIN, 1u)    /* LCD command set high */
#define LCD_DC_CLR()             GPIO_PinWrite(GPIO, LCD_DC_PORT, LCD_DC_PIN, 0u)    /* LCD command set low */         

#define LCD_TP_SET()             GPIO_PinWrite(GPIO, LCD_TP_PORT, LCD_TP_PIN, 1u)    /* LCD command set high */
#define LCD_TP_CLR()             GPIO_PinWrite(GPIO, LCD_TP_PORT, LCD_TP_PIN, 0u)    /* LCD command set low */         


/* -------------------------------------------------------------------------- */
/*                            functions extenal claim                         */
/* -------------------------------------------------------------------------- */

extern volatile uint16_t g_LCDDispBuf[];

extern uint8_t lcd_hardware_init(void);
extern uint8_t lcd_init(void);

extern void    lcd_set_cursor(uint16_t xpos, uint16_t ypos);
extern void    lcd_clear_screen(uint16_t color);
extern void    lcd_draw_point(uint16_t xpos, uint16_t ypos, uint16_t color);
extern void    lcd_clear_block(uint16_t xpos, uint16_t ypos, uint16_t color);
extern void    lcd_display_char(uint16_t xpos, uint16_t ypos, uint8_t chr, uint8_t font, uint16_t color) ;
extern void    lcd_display_num(uint16_t xpos, uint16_t ypos, uint32_t num, uint8_t len, uint8_t size, uint16_t color) ;
extern void    lcd_display_string(uint16_t xpos, uint16_t ypos, const uint8_t *string, uint8_t size, uint16_t color);

extern void    lcd_refresh_init(uint16_t color);
extern void    lcd_refresh_icon(uint16_t x, uint16_t y);
extern void    lcd_write_byte(uint8_t data, uint8_t cmd);
extern void    lcd_write_word(uint16_t data);
extern void    lcd_write_reg(uint8_t reg, uint8_t val);
extern void    lcd_refresh(void);
extern void    lcd_refresh(void);
#endif
/* End file __LCD_H__ */
