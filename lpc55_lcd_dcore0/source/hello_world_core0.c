/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "app.h"

extern GUI_CONST_STORAGE GUI_BITMAP bmimage_zhiwen150150;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Address of RAM, where the image for core1 should be copied */
#define CORE1_BOOT_ADDRESS (void *)0x04000000

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif (defined(__GNUC__)) && (!defined(__MCUXPRESSO))
extern const char m0_image_start[];
extern const char *m0_image_end;
extern int m0_image_size;
#define CORE1_IMAGE_START ((void *)m0_image_start)
#define CORE1_IMAGE_SIZE ((void *)m0_image_size)
#endif

WM_HWIN Createlpc55sxx_imx7ulp_faceid(void);
WM_HWIN CreateEnroll_face(void);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void)
{
    uint32_t core1_image_size;
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    core1_image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__sec_core"
    core1_image_size = (uint32_t)__section_end("__sec_core") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    core1_image_size = (uint32_t)m0_image_size;
#endif
    return core1_image_size;
}
#endif

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

volatile uint32_t g_SysTicks = 0;
volatile uint8_t  g_LCDDataUpdate = 0;
WM_HWIN g_EnrollFaceWMHWIN;

void SysTick_Handler(void)
{
    g_SysTicks++;    
    if( (g_SysTicks%100) == 01)
    {
        if(g_LCDDataUpdate == 0) g_LCDDataUpdate = 1;
        else if(g_LCDDataUpdate == 1) g_LCDDataUpdate = 2;
        else if(g_LCDDataUpdate == 2) g_LCDDataUpdate = 3;
        else g_LCDDataUpdate = 0;
    }
}

volatile uint16_t RemoteReadyEventData = 0U;
GUI_PID_STATE pid_state;
static void RemoteReadyEventHandler(uint16_t eventData, void *context)
{
    uint16_t temp_x, temp_y;
    RemoteReadyEventData = eventData;
    RemoteReadyEventData = 0;
    temp_x       = *(uint16_t *)0x20043000;//LCD_WIDTH - touch_y;
    temp_y       = *(uint16_t *)0x20043002;//touch_x;
    #if 1
    if(temp_x >= 1930) temp_x = 1930;
    if(temp_y >= 1930) temp_y = 1930;
    if(temp_x <= 120)  temp_x = 120;
    if(temp_y <= 170)  temp_y = 170;
    
    // pid_state.x = LCD_WIDTH  - (temp_x-120)*LCD_WIDTH/(1930-120);
    // pid_state.y = LCD_HEIGHT - (temp_y-170)*LCD_HEIGHT/(1930-170);
    pid_state.x = (temp_x-120)*LCD_WIDTH/(1930-120);
    pid_state.y = (temp_y-170)*LCD_HEIGHT/(1930-170);
    #endif
    
    if ((pid_state.x >= 0) && (pid_state.x < LCD_WIDTH) && (pid_state.y >= 0) && (pid_state.y < LCD_HEIGHT))
    {
    pid_state.Pressed = 1;
    pid_state.Layer   = 0;
    GUI_TOUCH_StoreStateEx(&pid_state);
    }
}


int BOARD_Touch_Poll(void)
{
    int touch_x;
    int touch_y;

    if(RemoteReadyEventData == 1)
    {
        RemoteReadyEventData = 0;
        return 1;
    }
    return 0;
}

volatile uint32_t g_TestID = 0;
volatile uint32_t g_top = 0;
volatile uint32_t g_bottom = 0;
volatile uint32_t g_left = 0;
volatile uint32_t g_right= 0;
volatile uint32_t g_facesize = 0;
volatile uint16_t g_DispTop = 0;
volatile uint16_t g_DispLeft = 0;
WM_HWIN hOldItem;

/*!
 * @brief Main function
 */
static uint16_t idx = 0;
int main(void)
{
    volatile uint8_t *g_BLECmdCmp  = NULL; 
    uint32_t i =0;
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        1,
    };	
    
    /* Initialize MCMGR, install generic event handlers */
    MCMGR_Init();

    /* Init board hardware.*/
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    BOARD_InitPins_Core0();
    BOARD_BootClockPLL150M();
    
    debug_init(115200);
    
    CLOCK_EnableClock(kCLOCK_InputMux);                        /* Enables the clock for the kCLOCK_InputMux block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);                           /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    GPIO_PortInit(GPIO, 0);
    GPIO_PortInit(GPIO, 1);
    //lcd_hardware_init();
    lcd_init();
    /* Print the initial banner from Primary core */
    PRINTF("\r\nHello World from the Primary Core!\r\n\n");
    *(uint16_t *)0x20043004 = 0x0000;
    *(uint16_t *)0x20043008 = 0x0000;
#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* Calculate size of the image  - not required on MCUXpresso IDE. MCUXpresso copies the secondary core
       image to the target memory during startup automatically */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    PRINTF("Copy Secondary core image to address: 0x%x, size: %d\n", CORE1_BOOT_ADDRESS, core1_image_size);

    /* Copy Secondary core application from FLASH to the target memory. */
    memcpy(CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif
    /* Register the application event before starting the secondary core */
    (void)MCMGR_RegisterEvent(kMCMGR_RemoteApplicationEvent, RemoteReadyEventHandler, ((void *)0));

    /* Boot Secondary core application */
    PRINTF("Starting Secondary core.\r\n");
    MCMGR_StartCore(kMCMGR_Core1, CORE1_BOOT_ADDRESS, 5, kMCMGR_Start_Asynchronous);
    PRINTF("The secondary core application has been started.\r\n");
    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO, 1, 4, &led_config);
    uint16_t *g_LCDImageBuf;
    g_LCDImageBuf = (uint16_t *)0x20010000;
    // LCD_DC_SET();
    // LCD_CS_CLR();
    
    SysTick_Config(SystemCoreClock / 100);
    
    GUI_Init();
    Createlpc55sxx_imx7ulp_faceid();
    
    GUI_MULTIBUF_Begin();
    GUI_Exec();
    GUI_MULTIBUF_End();
    GUI_PID_STATE State;
    while(1)
    {
        GUI_TOUCH_GetState(&State);
        #if 1
        if (State.Pressed == 0)
        {
        // GUI_X_Delay(1000);
        if (idx == 0)
        {
            idx++;
            CreateEnroll_faceTrigger();
                    while(*(volatile uint16_t *)0x20043008 != 0xA55A)
                    {
                        ;
                    }
                    *(volatile uint16_t *)0x20043008 = 0x0000;
                    *(volatile uint16_t *)0x20043004 = 0x55AA;
            GUI_X_Delay(10);
            GUI_MULTIBUF_Begin();
            GUI_Exec();
            GUI_MULTIBUF_End();
                    *(volatile uint16_t *)0x20043004 = 0x0000;
        }
        else if (idx == 1)
        {
            idx++;
            DemoTrigger();
                    while(*(volatile uint16_t *)0x20043008 != 0xA55A)
                    {
                        ;
                    }
                    *(volatile uint16_t *)0x20043008 = 0x0000;
                    *(volatile uint16_t *)0x20043004 = 0x55AA;
            GUI_X_Delay(10);
            GUI_MULTIBUF_Begin();
            GUI_Exec();
            GUI_MULTIBUF_End();
                    *(volatile uint16_t *)0x20043004 = 0x0000;
            
        }
        }
        #endif
        /* Poll touch controller for update */
        if (State.Pressed == 1)
        {
                    while(*(volatile uint16_t *)0x20043008 != 0xA55A)
                    {
                        ;
                    }
                    *(volatile uint16_t *)0x20043008 = 0x0000;
                    *(volatile uint16_t *)0x20043004 = 0x55AA;
            GUI_X_Delay(10);
            State.Pressed = 0;
            GUI_TOUCH_StoreStateEx(&State);
            GUI_MULTIBUF_Begin();
            GUI_Exec();
            GUI_MULTIBUF_End();
                    *(volatile uint16_t *)0x20043004 = 0x0000;
        }
        
        if(g_RecvCnt >= 10)
        {
            g_BLECmdCmp = (volatile uint8_t *)strstr((const char *)g_RecvBuf, (const char *)"END");
            if(g_BLECmdCmp != NULL)
            {
                sscanf(g_RecvBuf, "START%d,%d,%d,%d,%dEND", &g_top, &g_left, &g_bottom, &g_right, &g_facesize);
                memset(g_RecvBuf, 0x00, 256);
                PRINTF("HAHA%dEND\r\n", g_facesize);
                g_RecvCnt = 0;
                if(g_TestID == 1)
                {
                    *(volatile uint16_t *)0x20043008 = 0x0000;
                    *(volatile uint16_t *)0x20043004 = 0x55AA;
                    GUI_X_Delay(10);
                    while(*(volatile uint16_t *)0x20043008 != 0xA55A)
                    {
                        ;
                    }
                    hOldItem= WM_SelectWindow(WM_GetClientWindow(g_EnrollFaceWMHWIN));
                    GUI_SetColor(GUI_WHITE);
                    GUI_FillRect(0, 105, 235, 310);
                    
                    if(g_facesize != 0)
                    {
                        if(g_facesize>=220)
                        {
                            GUI_DrawBitmap(&bmui_face_big, 0, 107);
                        }
                        if ( (g_facesize>0) && (g_facesize<=160) )
                        {
                            g_DispTop  = (g_top+g_facesize/2)*240/640-50;
                            g_DispLeft = 110+(g_left+(g_right-g_left)/2)*180/480-39;
                            GUI_DrawBitmap(&bmui_face_little, g_DispTop, g_DispLeft);
                        }
                        if ( (g_facesize>160) && (g_facesize<220) )
                        {
                            g_DispTop  = (g_top+g_facesize/2)*240/640-90;
                            g_DispLeft = 110+(g_left+(g_right-g_left)/2)*180/480-80;
                            GUI_DrawBitmap(&bmui_face_mid, g_DispTop, g_DispLeft);
                        }
                    }
                    else
                    {
                        GUI_SetColor(GUI_WHITE);
                        GUI_FillRect(0, 105, 235, 310);
                    }
//                    //GUI_DrawBitmap(&bmui_face_big, 5, 107);
//                    GUI_DrawBitmap(&bmui_face_big, -55, 107);
                    GUI_SetColor(GUI_RED);
                    GUI_DrawCircle(120, 202, FACE_SIZE+1);
                    GUI_DrawCircle(120, 202, FACE_SIZE);
                    GUI_DrawCircle(120, 202, FACE_SIZE-1);
                    WM_SelectWindow(hOldItem);
                    *(volatile uint16_t *)0x20043004 = 0x0000;
                    //g_TestID = 1;
                }
            }
        }
    }
}
