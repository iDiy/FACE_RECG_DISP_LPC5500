/*
 * Copyright (c) 2017 - 2018 , NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __APP_H__
#define __APP_H__

// C Standard Lib includes
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

// LPC55xx Chip peripherals & Boards includes
#include "board.h"

#include "fsl_usart.h"
#include "fsl_gpio.h"
#include "board.h"
#include "mcmgr.h"

#include "pin_mux.h"
#include "lcd.h"

#include "emwin_support.h"

#include "GUI.h"
#include "BUTTON.h"
#include "CHECKBOX.h"
#include "SLIDER.h"
#include "DROPDOWN.h"
#include "RADIO.h"
#include "MULTIPAGE.h"

#include "app_printf.h"
#include "app_interrupt.h"
#include "lpc_ring_buffer.h"

extern volatile uint32_t g_SysTicks;
extern volatile uint32_t g_TestID;

extern WM_HWIN g_EnrollFaceWMHWIN;
extern WM_HWIN g_StartPageWMHWIN;

extern GUI_CONST_STORAGE GUI_BITMAP bmimage_nxplogo;
extern GUI_CONST_STORAGE GUI_BITMAP bmui_face_little;
extern GUI_CONST_STORAGE GUI_BITMAP bmui_face_mid;
extern GUI_CONST_STORAGE GUI_BITMAP bmui_face_big;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FACE_SIZE 70
 
 
#define DEBUG_USART_ENABLE         1

/* Debug USART port */
#define DEBUG_UART                 USART0
#define DEBUG_UART_TYPE            kSerialPort_Uart

//#define BOARD_NAME "SmartElock_LPC5500"

#define DEBUG_UART_CLK_FREQ        12000000U
#define DEBUG_UART_CLK_ATTACH      kFRO12M_to_FLEXCOMM0
#define DEBUG_UART_RST             kFC0_RST_SHIFT_RSTn
#define DEBUG_UART_CLKSRC          kCLOCK_Flexcomm0
#define DEBUG_UART_IRQ_HANDLER     FLEXCOMM0_IRQHandler
#define DEBUG_UART_IRQNUM          FLEXCOMM0_IRQn
#define DEBUG_UART_BAUDRATE        115200U

#define DEBUGTX_PORT               0u
#define DEBUGTX_PIN                29u
#define DEBUGTX_FUNC               IOCON_FUNC1

#define DEBUGRX_PORT               0u
#define DEBUGRX_PIN                30u
#define DEBUGRX_FUNC               IOCON_FUNC1

#define PRINTF                     debug_printf

#endif
