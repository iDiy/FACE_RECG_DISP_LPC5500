/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "GUI.h"
#include "WM.h"
#include "GUIDRV_FlexColor.h"
#include "emwin_support.h"
#include "GUIDRV_Lin.h"

#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#ifndef GUI_MEMORY_ADDR
static uint32_t s_gui_memory[(GUI_NUMBYTES + 3) / 4]; /* needs to be word aligned */
#define GUI_MEMORY_ADDR ((uint32_t)s_gui_memory)
#endif

/*******************************************************************************
 * Implementation of PortAPI for emWin LCD driver
 ******************************************************************************/

static void APP_pfWrite8_A0(U8 Data)
{
 //   BOARD_LCD_SPI.Send(&Data, 1);
}

static void APP_pfWrite8_A1(U8 Data)
{
//    BOARD_LCD_SPI.Send(&Data, 1);
}

static void APP_pfWriteM8_A1(U8 *pData, int NumItems)
{
 //   BOARD_LCD_SPI.Send(pData, NumItems);
}

static U8 APP_pfRead8_A1(void)
{
    uint8_t Data;
//   BOARD_LCD_SPI.Receive(&Data, 1);
    return Data;
}

static void APP_pfReadM8_A1(U8 *pData, int NumItems)
{
//    BOARD_LCD_SPI.Receive(pData, NumItems);
}


/*******************************************************************************
 * Implementation of communication with the touch controller
 ******************************************************************************/
//static ft6x06_handle_t touch_handle;
static void BOARD_Touch_InterfaceInit(void)
{
//    NVIC_SetPriority(BOARD_TOUCH_I2C_IRQ, 0);

//    /*Init I2C */
//    BOARD_TOUCH_I2C.Initialize(I2C_MasterSignalEvent);
//    BOARD_TOUCH_I2C.PowerControl(ARM_POWER_FULL);
//    BOARD_TOUCH_I2C.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
}

void BOARD_Touch_InterfaceDeinit(void)
{
//    BOARD_TOUCH_I2C.PowerControl(ARM_POWER_OFF);
//    BOARD_TOUCH_I2C.Uninitialize();
}

//int BOARD_Touch_Poll(void)
//{
////    touch_event_t touch_event;
////    int touch_x;
////    int touch_y;
////    GUI_PID_STATE pid_state;

////    if (kStatus_Success != FT6X06_GetSingleTouch(&touch_handle, &touch_event, &touch_x, &touch_y))
////    {
////        return 0;
////    }
////    else if (touch_event != kTouch_Reserved)
////    {
////        pid_state.x       = LCD_WIDTH - touch_y;
////        pid_state.y       = touch_x;
////        pid_state.Pressed = ((touch_event == kTouch_Down) || (touch_event == kTouch_Contact));
////        pid_state.Layer   = 0;
////        GUI_TOUCH_StoreStateEx(&pid_state);
////        return 1;
////    }
//    return 0;
//}

/*******************************************************************************
 * Application implemented functions required by emWin library
 ******************************************************************************/
void LCD_X_Config(void)
{
    GUI_DEVICE *pDevice;
    GUI_PORT_API PortAPI;
//    CONFIG_FLEXCOLOR Config = {0, 0, GUI_SWAP_XY, 0, 1};
 //   CONFIG_FLEXCOLOR Config = {0, 0, GUI_SWAP_XY, 0, 1};
    GUI_MULTIBUF_Config(1);
    pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_LIN_16, GUICC_M565, 0, 0); //GUICC_565, 0, 0);
    //pDevice = GUI_DEVICE_CreateAndLink(GUIDRV_FLEXCOLOR, GUICC_565, 0, 0);
//    GUIDRV_FlexColor_Config(pDevice, &Config);

    LCD_SetSizeEx (0, LCD_WIDTH, LCD_HEIGHT);
    LCD_SetVSizeEx(0, LCD_WIDTH, LCD_HEIGHT);
#if 0
    PortAPI.pfWrite8_A0  = APP_pfWrite8_A0;
    PortAPI.pfWrite8_A1  = APP_pfWrite8_A1;
    PortAPI.pfWriteM8_A1 = APP_pfWriteM8_A1;
    PortAPI.pfRead8_A1   = APP_pfRead8_A1;
    PortAPI.pfReadM8_A1  = APP_pfReadM8_A1;
    GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);
#else
    LCD_SetVRAMAddrEx(0, (void *)0x20010000);
#endif
//    BOARD_Touch_InterfaceInit();
//    FT6X06_Init(&touch_handle, &BOARD_TOUCH_I2C);
}

int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void *p)
{
    int result = 0;
    uint32_t addr;
    LCD_X_SHOWBUFFER_INFO *pData;
    switch (Cmd)
    {
        case LCD_X_INITCONTROLLER:
        {
            //BOARD_LCD_InterfaceInit();
            GUI_X_Delay(50); /* settle down delay after reset */
            //FT9341_Init(APP_pfWrite8_A1, APP_pfWrite8_A0);
            break;
        }
        case LCD_X_SHOWBUFFER:
        {

            return 0;
        }
        default:
            result = -1;
            break;
    }

    return result;
}

void GUI_X_Config(void)
{
    /* Assign work memory area to emWin */
    GUI_ALLOC_AssignMemory((void *)GUI_MEMORY_ADDR, GUI_NUMBYTES);

    /* Select default font */
    GUI_SetDefaultFont(GUI_FONT_6X8);
}

void GUI_X_Init(void)
{
}

/* Dummy RTOS stub required by emWin */
void GUI_X_InitOS(void)
{
}

/* Dummy RTOS stub required by emWin */
void GUI_X_Lock(void)
{
}

/* Dummy RTOS stub required by emWin */
void GUI_X_Unlock(void)
{
}

/* Dummy RTOS stub required by emWin */
U32 GUI_X_GetTaskId(void)
{
    return 0;
}

void GUI_X_ExecIdle(void)
{
}

GUI_TIMER_TIME GUI_X_GetTime(void)
{
    return 0;
}

//int  GUI_TOUCH_X_MeasureX (void)
//{
//    int x, y;
////    x = *(uint16_t *)0x20043000;
////    y = *(uint16_t *)0x20043004;
////    GUI_TOUCH_StoreState(x,y);
////    *(uint16_t *)0x20043000 = 0;
////    *(uint16_t *)0x20043004 = 0;
//    return x;
//}

void GUI_X_Delay(int Period)
{
    volatile uint32_t per;
    volatile uint32_t i;
    for (per=0; per < Period; per++)
    {
        for (i = 0; i < 15000; i++)
        {
            ;
        }
    }
}

void *emWin_memcpy(void *pDst, const void *pSrc, long size)
{
    return memcpy(pDst, pSrc, size);
}