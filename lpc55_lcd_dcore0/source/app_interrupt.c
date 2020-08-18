/*
 * Copyright (c) 2017 - 2018 , NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
 
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/**
 * @brief   
 * @param   
 * @return  
 */
void GINT0_IRQHandler(void)
{
    /* Clear interrupt before callback */
    GINT0->CTRL |= GINT_CTRL_INT_MASK;
}

/**
 * @brief   
 * @param   
 * @return  
 */
void GINT1_IRQHandler(void)
{
    /* Clear interrupt before callback */
    GINT1->CTRL |= GINT_CTRL_INT_MASK;
}

/**
 * @brief   
 * @param   
 * @return  
 */
void MRT0_IRQHandler(void)
{
    //MRT0->CHANNEL[0].STAT = kMRT_TimerInterruptFlag;      /* Clean-up MRT0 Interrupt Status */
}

uint8_t g_RecvBuf[256];
volatile uint32_t g_RecvCnt = 0;
/**
 * @brief   Flexcomm0 Interrupt Handler
 * @param   NULL
 * @return  NULL
 */
void DEBUG_UART_IRQ_HANDLER(void)
{
    uint8_t data;
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEBUG_UART))
    {
        data = USART_ReadByte(DEBUG_UART);
        g_RecvBuf[g_RecvCnt] = data;
        g_RecvCnt++;
        if(g_RecvCnt>=256) g_RecvCnt = 0;
    }
}

// end file