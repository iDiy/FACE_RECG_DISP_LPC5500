/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _EMWIN_SUPPORT_H_
#define _EMWIN_SUPPORT_H_

#define BOARD_LCD_READABLE 0

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

//#define BOARD_TOUCH_I2C Driver_I2C4
//#define BOARD_TOUCH_I2C_IRQ FLEXCOMM4_IRQn

#define GUI_NUMBYTES 0x8000 /*! Amount of memory assigned to the emWin library */

extern int BOARD_Touch_Poll(void);

#endif
