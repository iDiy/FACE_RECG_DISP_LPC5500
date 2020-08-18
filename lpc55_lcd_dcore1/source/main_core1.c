/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "mcmgr.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_pint.h"
#include "lcd.h"
#include "fsl_inputmux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_INIT() GPIO_PinInit(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, &led_config);
#define LED_TOGGLE() GPIO_PortToggle(GPIO, BOARD_LED_RED_GPIO_PORT, 1u << BOARD_LED_RED_GPIO_PIN);
volatile uint16_t*  g_TouchValueBuf;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function to create delay for Led blink.
 */
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 10; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*!
 * @brief Application-specific implementation of the SystemInitHook() weak function.
 */
void SystemInitHook(void)
{
    /* Initialize MCMGR - low level multicore management library. Call this
       function as close to the reset entry as possible to allow CoreUp event
       triggering. The SystemInitHook() weak function overloading is used in this
       application. */
    MCMGR_EarlyInit();
}

/*!
 * @brief Call back for GINT0 event
 */
volatile uint8_t g_TouchedFlag = 0;
/*!
 * @brief Call back for PINT Pin interrupt 0-7.
 */
void pint_intr_callback(pint_pin_int_t pintr, uint32_t pmatch_status)
{
    g_TouchedFlag = 1;
}

uint16_t xpt2046_read_ad_value(uint8_t chCmd)
{
    uint16_t hwData = 0;

    LCD_TP_CLR();    
    // Send Read Command
    LCD_SPI->FIFOWR = 0x07300000|chCmd;
    while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0) {
    }
    hwData = (LCD_SPI->FIFORD)&0x000000FF;
    //delay(); // delay 7uS
            
    // Read High 
    LCD_SPI->FIFOWR = 0x07300000;
    while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0) {
    }
    hwData = (LCD_SPI->FIFORD)&0x000000FF;
    hwData <<= 8;
    
    // Read Low 
    LCD_SPI->FIFOWR = 0x07300000;
    while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0) {
    }
    hwData |= ((LCD_SPI->FIFORD)&0x000000FF);
    hwData >>= 4;
            
    LCD_TP_SET();
    return hwData;
}

#define READ_TIMES  5
#define LOST_NUM    1
uint16_t xpt2046_read_average(uint8_t chCmd)
{
    uint8_t i, j;
    uint16_t hwbuffer[READ_TIMES], hwSum = 0, hwTemp;

    for (i = 0; i < READ_TIMES; i ++) {
        hwbuffer[i] = xpt2046_read_ad_value(chCmd);
    }
    for (i = 0; i < READ_TIMES - 1; i ++) {
        for (j = i + 1; j < READ_TIMES; j ++) {
            if (hwbuffer[i] > hwbuffer[j]) {
                hwTemp = hwbuffer[i];
                hwbuffer[i] = hwbuffer[j];
                hwbuffer[j] = hwTemp;
            }
        }
    }
    for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i ++) {
        hwSum += hwbuffer[i];
    }
    hwTemp = hwSum / (READ_TIMES - 2 * LOST_NUM);

    return hwTemp;
}

void xpt2046_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	*phwXpos = xpt2046_read_average(0xD0);
	*phwYpos = xpt2046_read_average(0x90);
}

#define ERR_RANGE 50
bool xpt2046_twice_read_xy(uint16_t *phwXpos, uint16_t *phwYpos)
{
	volatile uint16_t hwXpos1, hwYpos1, hwXpos2, hwYpos2;

	xpt2046_read_xy(&hwXpos1, &hwYpos1);
	xpt2046_read_xy(&hwXpos2, &hwYpos2);

	if (((hwXpos2 <= hwXpos1 && hwXpos1 < hwXpos2 + ERR_RANGE) || (hwXpos1 <= hwXpos2 && hwXpos2 < hwXpos1 + ERR_RANGE))
	&& ((hwYpos2 <= hwYpos1 && hwYpos1 < hwYpos2 + ERR_RANGE) || (hwYpos1 <= hwYpos2 && hwYpos2 < hwYpos1 + ERR_RANGE))) {
		*phwXpos = (hwXpos1 + hwXpos2) >> 1;
		*phwYpos = (hwYpos1 + hwYpos2) >> 1;
		return true;
	}
	return false;
}

void xpt2046_init(void)
{
    uint16_t hwXpos, hwYpos;
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, kINPUTMUX_GpioPort1Pin6ToPintsel);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    /* Initialize PINT */
    PINT_Init(PINT);
    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableFallEdge, pint_intr_callback);
    /* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);
    
    g_TouchValueBuf = (uint16_t *)0x20043000;
    
    LCD_CS_SET();
    xpt2046_read_xy(&hwXpos, &hwYpos);
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t startupData, i;
    uint8_t  temp;
    mcmgr_status_t status;

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };
    
    /* Initialize MCMGR, install generic event handlers */
    MCMGR_Init();

    /* Get the startup data */
    do
    {
        status = MCMGR_GetStartupData(&startupData);
    } while (status != kStatus_MCMGR_Success);
   
    SPI_MasterSetBaud(SPI8, 1000000, 1500000000);
    LCD_CS_SET();
    xpt2046_init();
    SPI_MasterSetBaud(SPI8, 50000000, 1500000000);
    lcd_set_cursor(0, 0);
    lcd_write_byte(0x22, LCD_CMD);
    
    LCD_DC_SET();
    LCD_CS_CLR();
    while (1)
    {
        if(*(uint16_t *)0x20043004 != 0x55AA)
        {
            *(uint16_t *)0x20043008 = 0x55AA;
            lcd_refresh();
            *(uint16_t *)0x20043008 = 0xA55A;
        }
        GPIO_PortToggle(GPIO, 1, 1<<4);
        if( (g_TouchedFlag == 1) && (GPIO_PinRead(GPIO, 1, 6) == 0) )
        {
            __disable_irq();
            g_TouchedFlag = 0;
            SPI_MasterSetBaud(SPI8, 1000000, 1500000000);
            LCD_CS_SET();
            xpt2046_read_xy(&g_TouchValueBuf[0], &g_TouchValueBuf[1]);
            SPI_MasterSetBaud(SPI8, 50000000, 1500000000);
            lcd_set_cursor(0, 0);
            lcd_write_byte(0x22, LCD_CMD);
            LCD_DC_SET();
            LCD_CS_CLR();
            __enable_irq();
            g_TouchedFlag = 0;
            while(GPIO_PinRead(GPIO, 1, 6) == 0)
            {
                ;
            }
            /* Signal the other core we are ready by triggering the event and passing the APP_READY_EVENT_DATA */
            (void)MCMGR_TriggerEvent(kMCMGR_RemoteApplicationEvent, (1U) );
        }
    }
}
