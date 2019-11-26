/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_common.h"
#include "fsl_elcdif.h"
#include "fsl_debug_console.h"
#include "fsl_lpi2c.h"
#include "fsl_ft5406_rt.h"

#include "pin_mux.h"
#include "board.h"
#include "fsl_gpio.h"
#include "clock_config.h"

#include "base/g2d.h"
#include "base/lcd.h"
#include "tkc/mem.h"

extern int gui_app_start(int lcd_w, int lcd_h);
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_ELCDIF LCDIF

#define APP_IMG_HEIGHT 272
#define APP_IMG_WIDTH 480
#define APP_HSW 41
#define APP_HFP 4
#define APP_HBP 8
#define APP_VSW 10
#define APP_VFP 4
#define APP_VBP 2
#define APP_POL_FLAGS \
    (kELCDIF_DataEnableActiveHigh | kELCDIF_VsyncActiveLow | kELCDIF_HsyncActiveLow | kELCDIF_DriveDataOnRisingClkEdge)

/* Display. */
#define LCD_DISP_GPIO GPIO1
#define LCD_DISP_GPIO_PIN 8
/* Back light. */
#define LCD_BL_GPIO GPIO1
#define LCD_BL_GPIO_PIN 10

/* Frame buffer data alignment, for better performance, the LCDIF frame buffer should be 64B align. */
#define FRAME_BUFFER_ALIGN 64

#ifndef APP_LCDIF_DATA_BUS
#define APP_LCDIF_DATA_BUS kELCDIF_DataBus16Bit
#endif



/* Macros for the touch touch controller. */
#define BOARD_TOUCH_I2C LPI2C1

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (5U)

#define BOARD_TOUCH_I2C_CLOCK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))
#define BOARD_TOUCH_I2C_BAUDRATE 100000U

#define LPI2C_DATA_LENGTH 32U

#define LPI2C_MASTER_IRQ LPI2C1_IRQn
#define LPI2C_MASTER_IRQHandler LPI2C1_IRQHandler
#define BOARD_TOUCH_INT_PORT PORTA
#define BOARD_TOUCH_INT_PIN 2

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_EnableLcdInterrupt(void);


/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool s_frameDone = false;

AT_NONCACHEABLE_SECTION_ALIGN(static uint16_t s_frameBuffer[2][APP_IMG_HEIGHT][APP_IMG_WIDTH], FRAME_BUFFER_ALIGN);


uint8_t *online_fb_addr = (uint8_t*)s_frameBuffer[0];
uint8_t *offline_fb_addr = (uint8_t*)s_frameBuffer[1];

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void APP_LCDIF_IRQHandler(void);

void LCDIF_IRQHandler(void)
{
    APP_LCDIF_IRQHandler();
}



/* Touch driver handle. */
static ft5406_rt_handle_t touchHandle;

void BOARD_Touch_Init(void)
{
    /*Clock setting for LPI2C*/
    CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);
    
    lpi2c_master_config_t masterConfig = {0};
    /*
    * masterConfig.debugEnable = false;
    * masterConfig.ignoreAck = false;
    * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
    * masterConfig.baudRate_Hz = 100000U;
    * masterConfig.busIdleTimeout_ns = 0;
    * masterConfig.pinLowTimeout_ns = 0;
    * masterConfig.sdaGlitchFilterWidth_ns = 0;
    * masterConfig.sclGlitchFilterWidth_ns = 0;
    */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = BOARD_TOUCH_I2C_BAUDRATE;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(BOARD_TOUCH_I2C, &masterConfig, BOARD_TOUCH_I2C_CLOCK_FREQ);

    /* Initialize the touch handle. */
    FT5406_RT_Init(&touchHandle, BOARD_TOUCH_I2C);
}

void BOARD_Touch_Deinit(void)
{
    LPI2C_MasterDeinit(BOARD_TOUCH_I2C);
}

int BOARD_Touch_Poll(int *pX, int *pY, int *pPressFlg)
{
    touch_event_t touch_event;
    int touch_x = -1;
    int touch_y = -1;

    if (kStatus_Success != FT5406_RT_GetSingleTouch(&touchHandle, &touch_event, &touch_x, &touch_y))
    {
        BOARD_Touch_Init();
        return 0;
    }
    else if (touch_event != kTouch_Reserved)
    {
        *pX = touch_y;
        *pY = touch_x;
        *pPressFlg = ((touch_event == kTouch_Down) || (touch_event == kTouch_Contact));
        return 1;
    }
    return 0;
}

/* Enable interrupt. */
void BOARD_EnableLcdInterrupt(void)
{
    EnableIRQ(LCDIF_IRQn);
}

/* Initialize the LCD_DISP. */
void BOARD_InitLcd(void)
{
    volatile uint32_t i = 0x100U;

    gpio_pin_config_t config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Reset the LCD. */
    GPIO_PinInit(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, &config);

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 0);

    while (i--)
    {
    }

    GPIO_PinWrite(LCD_DISP_GPIO, LCD_DISP_GPIO_PIN, 1);

    /* Backlight. */
    config.outputLogic = 1;
    GPIO_PinInit(LCD_BL_GPIO, LCD_BL_GPIO_PIN, &config);
}

void BOARD_InitLcdifPixelClock(void)
{
    /*
     * The desired output frame rate is 60Hz. So the pixel clock frequency is:
     * (480 + 41 + 4 + 18) * (272 + 10 + 4 + 2) * 60 = 9.2M.
     * Here set the LCDIF pixel clock to 9.3M.
     */

    /*
     * Initialize the Video PLL.
     * Video PLL output clock is OSC24M * (loopDivider + (denominator / numerator)) / postDivider = 93MHz.
     */
    clock_video_pll_config_t config = {
        .loopDivider = 31, .postDivider = 8, .numerator = 0, .denominator = 0,
    };

    CLOCK_InitVideoPll(&config);

    /*
     * 000 derive clock from PLL2
     * 001 derive clock from PLL3 PFD3
     * 010 derive clock from PLL5
     * 011 derive clock from PLL2 PFD0
     * 100 derive clock from PLL2 PFD1
     * 101 derive clock from PLL3 PFD1
     */
    CLOCK_SetMux(kCLOCK_LcdifPreMux, 2);

    CLOCK_SetDiv(kCLOCK_LcdifPreDiv, 4);

    CLOCK_SetDiv(kCLOCK_LcdifDiv, 1);
}


void APP_LCDIF_IRQHandler(void)
{
    uint32_t intStatus;

    intStatus = ELCDIF_GetInterruptStatus(APP_ELCDIF);

    ELCDIF_ClearInterruptStatus(APP_ELCDIF, intStatus);

    if (intStatus & kELCDIF_CurFrameDone)
    {
        s_frameDone = true;
    }
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void APP_ELCDIF_Init(void)
{
    const elcdif_rgb_mode_config_t config = {
        .panelWidth = APP_IMG_WIDTH,
        .panelHeight = APP_IMG_HEIGHT,
        .hsw = APP_HSW,
        .hfp = APP_HFP,
        .hbp = APP_HBP,
        .vsw = APP_VSW,
        .vfp = APP_VFP,
        .vbp = APP_VBP,
        .polarityFlags = APP_POL_FLAGS,
        .bufferAddr = (uint32_t)s_frameBuffer[0],
        .pixelFormat = kELCDIF_PixelFormatRGB565,
        .dataBus = APP_LCDIF_DATA_BUS,
    };

    ELCDIF_RgbModeInit(APP_ELCDIF, &config);
}

uint32_t g_systick_count = 0;

void SysTick_Handler (void)
{
    g_systick_count++;
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t frameBufferIndex = 0;

    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitSemcPins();
    BOARD_InitI2C1Pins();
    BOARD_BootClockRUN();
    BOARD_InitLcdifPixelClock();
    BOARD_InitDebugConsole();
    BOARD_InitLcd();
    
    SysTick_Config(SystemCoreClock/1000);

    PRINTF("LCDIF RGB example start...\r\n");

    APP_ELCDIF_Init();
    BOARD_Touch_Init();

    BOARD_EnableLcdInterrupt();

    /* Clear the frame buffer. */
    memset(s_frameBuffer, 0, sizeof(s_frameBuffer));


    ELCDIF_EnableInterrupts(APP_ELCDIF, kELCDIF_CurFrameDoneInterruptEnable);
    ELCDIF_RgbModeStart(APP_ELCDIF);
    
    return gui_app_start(APP_IMG_WIDTH, APP_IMG_HEIGHT);
    
    while (1)
    {
//        frameBufferIndex ^= 1U;

//        ELCDIF_SetNextBufferAddr(APP_ELCDIF, (uint32_t)s_frameBuffer[frameBufferIndex]);

//        s_frameDone = false;
//        /* Wait for previous frame complete. */
//        while (!s_frameDone)
//        {
//        }
    }
}
