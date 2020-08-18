/*
 * Copyright (c) 2017 - 2018 , NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Includes ------------------------------------------------------------------*/
#include "lcd_fonts.h"
#include "lcd.h"

/* -------------------------------------------------------------------------- */
/*                       Public display buffer                                */
/* -------------------------------------------------------------------------- */
volatile uint16_t g_LCDDispBuf[240*96];

/* -------------------------------------------------------------------------- */
/*                       Private function prototypes                          */
/* -------------------------------------------------------------------------- */

static void     lcd_delayms(uint32_t count);
static uint32_t lcd_pow(uint8_t m, uint8_t n);
static uint8_t  LCD_BYTE_WRITE(uint8_t data);
static uint16_t LCD_WORD_WRITE(uint16_t data);

/* -------------------------------------------------------------------------- */
/*                             Private functions                              */
/* -------------------------------------------------------------------------- */
/**
 * @brief       lcd_write_byte  Write a byte data to LCD driver chip
 * @param[0]    data     write data content.
 * @param[1]    cmd      command or data.
 * @ret         NULL
*/
void lcd_write_byte(uint8_t data, uint8_t cmd)
{
    if (cmd) {
        LCD_DC_SET();
    } else {
        LCD_DC_CLR();
    }

    LCD_CS_CLR();
    LCD_BYTE_WRITE(data);
    LCD_CS_SET();
}

/**
 * @brief       lcd_write_word    Write a word data to LCD driver chip
 * @param[0]    data              write data content.
 * @ret         NULL
*/
void lcd_write_word(uint16_t data)
{
    LCD_DC_SET();
    LCD_CS_CLR();
    LCD_WORD_WRITE(data);
    LCD_CS_SET();
}

/**
 * @brief       lcd_write_reg     Write a word data to LCD register
 * @param[0]    reg               target lcd register.
 * @param[1]    val               data content.
 * @ret         NULL
*/
void lcd_write_reg(uint8_t reg, uint8_t val)
{
    lcd_write_byte(reg, LCD_CMD);
    lcd_write_byte(val, LCD_DATA);
}

/**
 * @brief       lcd_hardware_init   lcd hardware initialize
 * @param[0]    NULL
 * @ret         NULL
*/
uint8_t lcd_hardware_init(void)
{
    spi_master_config_t LCDSpiConfig;
    gpio_pin_config_t   gpioPinConfig;
    
    CLOCK_AttachClk(kMAIN_CLK_to_HSLSPI);           /* attach 50 MHz clock to HSLSPI */
    RESET_PeripheralReset(kHSLSPI_RST_SHIFT_RSTn);  /* reset FLEXCOMM for SPI */
    
    /* SPI init */
    SPI_MasterGetDefaultConfig(&LCDSpiConfig);
    LCDSpiConfig.sselNum = (spi_ssel_t)0;
    LCDSpiConfig.sselPol = (spi_spol_t)kSPI_SpolActiveAllLow;
    LCDSpiConfig.baudRate_Bps = 50000000U;
    SPI_MasterInit(LCD_SPI, &LCDSpiConfig, CLOCK_GetHsLspiClkFreq());
    
     /* GPIO Pins. */
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    gpioPinConfig.pinDirection = kGPIO_DigitalOutput;
    gpioPinConfig.outputLogic  = 1u; /* output high as default. */
    GPIO_PinInit(GPIO, LCD_CS_PORT,  LCD_CS_PIN,  &gpioPinConfig);
    GPIO_PinInit(GPIO, LCD_DC_PORT,  LCD_DC_PIN,  &gpioPinConfig);
    
    gpioPinConfig.outputLogic  = 0u; /* output low as default. */
    GPIO_PinInit(GPIO, LCD_BKL_PORT, LCD_BKL_PIN, &gpioPinConfig);    
    return true;
}

/**
 * @brief       LCD_BYTE_WRITE   LCD write a 8bit data through SPI
 * @param[0]    data             data content
 * @ret         spi return read data
*/
static uint8_t LCD_BYTE_WRITE(uint8_t data)
{
    uint32_t temp;
    /* clear tx/rx errors and empty FIFOs */
    LCD_SPI->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
    LCD_SPI->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;
    LCD_SPI->FIFOWR = data | 0x07300000;
    /* wait if TX FIFO of previous transfer is not empty */
    while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0) {
    }
    temp = (LCD_SPI->FIFORD)&0x000000FF;
    return temp;
}

/**
 * @brief       LCD_WORD_WRITE   LCD write a 16bit data through SPI
 * @param[0]    data             data content
 * @ret         spi return read data
*/
static uint16_t LCD_WORD_WRITE(uint16_t data)
{
    uint32_t temp;
    /* clear tx/rx errors and empty FIFOs */
//    LCD_SPI->FIFOCFG |= SPI_FIFOCFG_EMPTYTX_MASK | SPI_FIFOCFG_EMPTYRX_MASK;
//    LCD_SPI->FIFOSTAT |= SPI_FIFOSTAT_TXERR_MASK | SPI_FIFOSTAT_RXERR_MASK;
    LCD_SPI->FIFOWR = data | 0x0F300000;
    /* wait if TX FIFO of previous transfer is not empty */
    while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_RXNOTEMPTY_MASK) == 0) {
    }
    temp = (LCD_SPI->FIFORD)&0x0000FFFF;
    return temp;
}

/**
 * @brief       lcd_delayms      lcd delay micro seconds
 * @param[0]    count            delay ms
 * @ret         NULL
*/
volatile uint32_t g_LCDDelayMsCnt;
static void lcd_delayms(uint32_t count)
{
    for (g_LCDDelayMsCnt = 0u; g_LCDDelayMsCnt < (150 * count); g_LCDDelayMsCnt++)
    {
        ;
    }
}

/**
 * @brief       lcd_pow
 * @param[0]    m
 * @param[1]    n
 * @ret         result
*/
static uint32_t lcd_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while(n --) result *= m;
    return result;
}

/* -------------------------------------------------------------------------- */
/*                              Public function                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief       lcd_set_cursor   Set diaply position of cursor on LCD
 * @param[0]    x position
 * @param[1]    y position
 * @ret         NULL
*/
void lcd_set_cursor(uint16_t xpos, uint16_t ypos)
{
    lcd_write_reg(0x02, xpos >> 8);
    lcd_write_reg(0x03, xpos & 0xFF);   /* Column Start */
    lcd_write_reg(0x06, ypos >> 8);
    lcd_write_reg(0x07, ypos & 0xFF);   /* Row Start */
}

/**
 * @brief       lcd_clear_screen   clear the lcd with color
 * @param[0]    color              displayed color
 * @ret         NULL
*/
void lcd_clear_screen(uint16_t color)
{
    uint32_t i, cnt = 0;

    cnt = LCD_WIDTH * LCD_HEIGHT;

    lcd_set_cursor(0, 0);
    lcd_write_byte(0x22, LCD_CMD);

    LCD_DC_SET();
    LCD_CS_CLR();
    for (i = 0; i < cnt; i ++)
    {
        LCD_WORD_WRITE(color);
    }
    LCD_CS_SET();
}

/**
 * @brief       lcd_draw_point     draw a point on the lcd with the color.
 * @param[0]    xpos               x position
 * @param[1]    ypos               y position
 * @param[2]    color              displayed color
 * @ret         NULL
*/
void lcd_draw_point(uint16_t xpos, uint16_t ypos, uint16_t color)
{
    lcd_set_cursor(xpos, ypos);
    lcd_write_byte(0x22, LCD_CMD);
    lcd_write_word(color);
}

/**
 * @brief       lcd_clear_block     clear a part of lcd with color
 * @param[0]    xpos                x position
 * @param[1]    ypos                y position
 * @param[2]    color               displayed color
 * @ret         NULL
*/
void lcd_clear_block(uint16_t xpos, uint16_t ypos, uint16_t color)
{
    uint32_t i;
    lcd_set_cursor(xpos, ypos);
    lcd_write_byte(0x22, LCD_CMD);

    LCD_DC_SET();
    LCD_CS_CLR();
    for (i = 0; i < 240*32; i ++) {
        LCD_WORD_WRITE(color);
    }
    LCD_CS_SET();
}

/**
 * @brief       lcd_clear_block     display a char at the position on lcd
 * @param[0]    xpos                x position
 * @param[1]    ypos                y position
 * @param[2]    chr                 a char
 * @param[3]    font                font type
 * @param[4]    color               displayed color
 * @ret         NULL
*/
void lcd_display_char(uint16_t xpos, uint16_t ypos, uint8_t chr, uint8_t font, uint16_t color) 
{
    uint8_t i, j, temp;
    uint16_t ypos_tmp = ypos;
    for (i = 0; i < font; i ++)
    {
        if (font == LCD_FONT_1206 )
        {
            temp = Font1206[chr - 0x20][i];
        }
        else if (LCD_FONT_1608 == font)
        {
            temp = Font1608[chr - 0x20][i];
        }

        for (j = 0; j < 8; j ++)
        {
            if (temp & 0x80)
            {
                lcd_draw_point(xpos, ypos, color);
            }
            temp <<= 1;
            ypos ++;
            if ((ypos - ypos_tmp) == font)
            {
                ypos = ypos_tmp;
                xpos ++;
                break;
            }
        }
    }
}

/**
 * @brief       lcd_display_num     display a number at the position on lcd.
 * @param[0]    xpos                x position
 * @param[1]    ypos                y position
 * @param[2]    num                 number
 * @param[3]    font                length of the number
 * @param[4]    color               displayed color
 * @ret         NULL
*/
void lcd_display_num(uint16_t xpos, uint16_t ypos, uint32_t num, uint8_t len, uint8_t size, uint16_t color) 
{
    uint8_t i, j = 0;
    uint8_t temp;
    for(i = 0; i < len; i ++)
    {
        temp = (num/lcd_pow(10, len-i-1)) % 10;
        if( (j == 0) && (i < (len-1)) )
        {
            if(temp == 0)
            {
                lcd_display_char(xpos+(size/2)*i, ypos, ' ', size, color);
                continue;
            }
            else
            {
                j = 1;
            }
        }
        lcd_display_char(xpos+(size/2)*i, ypos, '0'+temp, size, color);
    }
}

/**
 * @brief       lcd_display_string     display a string at the position on lcd.
 * @param[0]    xpos                   x position
 * @param[1]    ypos                   y position
 * @param[2]    *string                string to display
 * @param[3]    size                   the size of the string
 * @param[4]    color                  displayed color
 * @ret         NULL
*/
void lcd_display_string(uint16_t xpos, uint16_t ypos, const uint8_t *string, uint8_t size, uint16_t color)  
{
    while (*string != '\0')
    {
        if (xpos > (LCD_WIDTH - size/2))
        {
          xpos = 0;
          ypos += size;
          if (ypos > (LCD_HEIGHT - size))
          {
            ypos = xpos = 0;
            lcd_clear_screen(0x00);
          }
        }
        lcd_display_char(xpos, ypos, (uint8_t)*string, size, color);
        xpos += size / 2;
        string ++;
    }
}

/**
 * @brief       LCD initialize
 * @param       NULL
 * @retval      true
*/
uint8_t lcd_init(void)
{
    //lcd_hardware_init();
    //Driving ability Setting
    lcd_write_reg(0xEA,0x00);     // PTBA[15:8]
    lcd_write_reg(0xEB,0x20);     // PTBA[7:0]
    lcd_write_reg(0xEC,0x0C);     // STBA[15:8]
    lcd_write_reg(0xED,0xC4);     // STBA[7:0]
    lcd_write_reg(0xE8,0x38);     // OPON[7:0]
    lcd_write_reg(0xE9,0x10);     // OPON1[7:0]
    lcd_write_reg(0xF1,0x01);     // OTPS1B
    lcd_write_reg(0xF2,0x10);     // GEN
    //Gamma 2.2 Setting
    lcd_write_reg(0x40,0x01);     //
    lcd_write_reg(0x41,0x00);     //
    lcd_write_reg(0x42,0x00);     //
    lcd_write_reg(0x43,0x10);     //
    lcd_write_reg(0x44,0x0E);     //
    lcd_write_reg(0x45,0x24);     //
    lcd_write_reg(0x46,0x04);     //
    lcd_write_reg(0x47,0x50);     //
    lcd_write_reg(0x48,0x02);     //
    lcd_write_reg(0x49,0x13);     //
    lcd_write_reg(0x4A,0x19);     //
    lcd_write_reg(0x4B,0x19);     //
    lcd_write_reg(0x4C,0x16);     //
    lcd_write_reg(0x50,0x1B);     //
    lcd_write_reg(0x51,0x31);     //
    lcd_write_reg(0x52,0x2F);     //
    lcd_write_reg(0x53,0x3F);     //
    lcd_write_reg(0x54,0x3F);     //
    lcd_write_reg(0x55,0x3E);     //
    lcd_write_reg(0x56,0x2F);     //
    lcd_write_reg(0x57,0x7B);     //
    lcd_write_reg(0x58,0x09);     //
    lcd_write_reg(0x59,0x06);     //
    lcd_write_reg(0x5A,0x06);     //
    lcd_write_reg(0x5B,0x0C);     //
    lcd_write_reg(0x5C,0x1D);     //
    lcd_write_reg(0x5D,0xCC);     //
    //Power Voltage Setting
    lcd_write_reg(0x1B,0x1B);     //  VRH=4.65V
    lcd_write_reg(0x1A,0x01);     //  BT (VGH~15V,VGL~-10V,DDVDH~5V)
    lcd_write_reg(0x24,0x2F);     //  VMH(VCOM High voltage ~3.2V)
    lcd_write_reg(0x25,0x57);     //  VML(VCOM Low voltage -1.2V)
    //****VCOM offset**///
    lcd_write_reg(0x23,0x88);     //  for Flicker adjust //can reload from OTP
    //Power on Setting
    lcd_write_reg(0x18,0x34);     //  I/P_RADJ,N/P_RADJ, Normal mode 60Hz
    lcd_write_reg(0x19,0x01);     //  OSC_EN='1', start Osc
    lcd_write_reg(0x01,0x00);     //  DP_STB='0', out deep sleep
    lcd_write_reg(0x1F,0x88);     //  GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
    lcd_delayms(5);
    lcd_write_reg(0x1F,0x80);     //  GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
    lcd_delayms(5);
    lcd_write_reg(0x1F,0x90);     //  GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
    lcd_delayms(5);
    lcd_write_reg(0x1F,0xD0);     //  GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
    lcd_delayms(5);
    //262k/65k color selection
    lcd_write_reg(0x17,0x05);     //  default 0x06 262k color // 0x05 65k color
    //SET PANEL
    lcd_write_reg(0x36,0x00);     //  SS_P, GS_P,REV_P,BGR_P
    //Display ON Setting
    lcd_write_reg(0x28,0x18);     //  GON=1, DTE=1, D=1000
    lcd_delayms(40);
    lcd_write_reg(0x28,0x3F);     //  GON=1, DTE=1, D=1100

    lcd_write_reg(0x16,0xB8);
    //Set GRAM Area
#if 0
    lcd_write_reg(0x02,0x00);
    lcd_write_reg(0x03,0x00);     //  Column Start
    lcd_write_reg(0x04,0x00);
    lcd_write_reg(0x05,0xEF);     //  Column End
    lcd_write_reg(0x06,0x00);
    lcd_write_reg(0x07,0x00);     //  Row Start
    lcd_write_reg(0x08,0x01);
    lcd_write_reg(0x09,0x3F);     //  Row End
#else
    lcd_write_reg(0x02,0x00);
    lcd_write_reg(0x03,0x00);     //  Column Start
    lcd_write_reg(0x04,0x01);
    lcd_write_reg(0x05,0x3F);     //  Column End
    lcd_write_reg(0x06,0x00);
    lcd_write_reg(0x07,0x00);     //  Row Start
    lcd_write_reg(0x08,0x00);
    lcd_write_reg(0x09,0xEF);     //  Row End 
#endif
    lcd_clear_screen(LCD_COLOR_WHITE);  /* Clean up lcd screen */
    LCD_BKL_SET();                      /* Enable LCD backlight */

    return true;
}

/**
 * @brief       initialize lcd refresh color
 * @param       color -- display color
 * @retval      NULL
*/
void lcd_refresh_init(uint16_t color)
{
    uint32_t i;
    for(i=0; i<240*96; i++)
    {
        g_LCDDispBuf[i] = color;
    }
}

/**
 * @brief       lcd_refresh_icon    refresh lcd display colors
 * @param[0]    xpos                x position
 * @param[1]    ypos                y position
 * @ret         NULL
*/
void lcd_refresh_icon(uint16_t x, uint16_t y)
{
    uint32_t i;
    //lcd_set_cursor(x, y);
    //lcd_write_byte(0x22, LCD_CMD);
    //LCD_DC_SET();
    LCD_DC_CLR();
    delay(); // 10mS
    
    LCD_CS_CLR();
    for (i = 0; i < 240*320; i ++) {
      lcd_write_word(g_LCDDispBuf[i]);
    }
    LCD_CS_SET();
    
    LCD_DC_SET();
}

volatile uint16_t*  g_LCDImageBuf;


void lcd_refresh(void)
{
    uint32_t i;
    uint32_t temp;
    g_LCDImageBuf = (uint16_t *)0x20010000;

    for (i = 0; i < 76800; i ++) {
        /* wait if TX FIFO of previous transfer is not empty */
        while ((LCD_SPI->FIFOSTAT & SPI_FIFOSTAT_TXNOTFULL_MASK) == 0) {
        }
        
        LCD_SPI->FIFOWR = g_LCDImageBuf[i] | 0x0F400000;

        temp = (LCD_SPI->FIFORD);
    }
}

/* File End */