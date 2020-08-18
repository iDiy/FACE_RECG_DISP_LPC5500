/*
 * Copyright (c) 2017 - 2018 , NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __APP_PRINTF_H__
#define __APP_PRINTF_H__

#include "stdint.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
extern void     debug_init(uint32_t baudrate);
extern uint32_t debug_printf(const char *formatString, ...);

extern uint8_t  uart_getc(void);
extern void     uart_putc(uint8_t c);
extern void     uart_puts(char *str);

#endif

