/*******************************************************************************
* File Name: eink_shield.c
*
* Description: This file contains task and functions related to EInk display
* shield board to show texts and graphics using the EmWin Graphics Library. 
* Also reads temperature from thermistor.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mtb_e2271cs021.h"
#include "mtb_thermistor_ntc_gpio.h"
#include "GUI.h"
#include "images.h"
#include "LCDConf.h"
#include "eink_shield.h"
#include "bt_app.h"
#include "board.h"

/*******************************************************************************
* Macros
*******************************************************************************/
// Thermistor Constants
/** Resistance of the reference resistor */
#define THERM_R_REF             (float)(10000)
/** Beta constant of the (NCP18XH103F03RB) thermistor (3380 Kelvin).See the
 * thermistor datasheet for more details. */
#define THERM_B_CONST           (float)(3380)
/** Resistance of the thermistor is 10K at 25 degrees C (from datasheet)
 * Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
 * R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define THERM_R_INFINITY        (float)(0.1192855)

/** Pin for the Thermistor VDD signal */
#define THERM_PIN_VDD           (CYBSP_A0)
/** Pin for the Thermistor Output option1 signal */
#define THERM_PIN_OUT1          (CYBSP_A1)
/** Pin for the Thermistor Output option2 signal */
#define THERM_PIN_OUT2          (CYBSP_A2)
/** Pin for the Thermistor Ground signal */
#define THERM_PIN_GND           (CYBSP_A3)

/* Macros used to convert from IEEE-754 single precision floating
   point format to IEEE-11073 FLOAT with two decimal digits of precision */
#define IEEE_11073_MANTISSA_SCALER  (uint8_t) (100u)

#define IEEE_11073_EXPONENT_VALUE   (int8_t)  (-2)

#define IEEE_11073_EXPONENT_INDEX   (uint8_t) (3u)

/*******************************************************************************
* Global variables
*******************************************************************************/
/* HAL SPI object to interface with display driver */
cyhal_spi_t spi; 

/* HAL ADC object to interface with thermistor */
cyhal_adc_t adc;

/* Structure for thermistor configuration */
mtb_thermistor_ntc_gpio_t thermistor;
mtb_thermistor_ntc_gpio_cfg_t thermistor_cfg = {
    .r_ref = THERM_R_REF,              // CY8CKIT_028_EPD_THERM_R_REF
    .b_const = THERM_B_CONST,          // CY8CKIT_028_EPD_THERM_B_CONST
    .r_infinity = THERM_R_INFINITY,    //CY8CKIT_028_EPD_THERM_R_INFINITY
};

/* Configuration structure defining the necessary pins to communicate with
 * the E-ink display */
const mtb_e2271cs021_pins_t pins =
{
    .spi_mosi  = CYBSP_D11,    // CY8CKIT_028_EPD_PIN_DISPLAY_SPI_MOSI
    .spi_miso  = CYBSP_D12, // CY8CKIT_028_EPD_PIN_DISPLAY_SPI_MISO
    .spi_sclk  = CYBSP_D13, // CY8CKIT_028_EPD_PIN_DISPLAY_SPI_SCLK
    .spi_cs    = CYBSP_D10, // CY8CKIT_028_EPD_PIN_DISPLAY_CS
    .reset     = CYBSP_D2,  // CY8CKIT_028_EPD_PIN_DISPLAY_RST
    .busy      = CYBSP_D3,  // CY8CKIT_028_EPD_PIN_DISPLAY_BUSY
    .discharge = CYBSP_D5,  // CY8CKIT_028_EPD_PIN_DISPLAY_DISCHARGE
    .enable    = CYBSP_D4,  // CY8CKIT_028_EPD_PIN_DISPLAY_EN
    .border    = CYBSP_D6,  // CY8CKIT_028_EPD_PIN_DISPLAY_BORDER
    .io_enable = CYBSP_D7   // CY8CKIT_028_EPD_PIN_DISPLAY_IOEN
};

/* Buffer to the previous frame written on the display */
uint8_t previous_frame[PV_EINK_IMAGE_SIZE] = {0};

/* Pointer to the new frame that need to be written */
uint8_t *current_frame;

float temperature_value;

/* Data-type used to store temperature as an IEEE-11073 FLOAT value as well as
   access it as an array of bytes for BLE operations */
typedef union
{
    int32_t temeratureValue;
    int8_t  temperatureArray[5u];
} temperature_data_t;

/*******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_AFTER_STARTUP_SCREEN_MS       (2000)
#define AMBIENT_TEMPERATURE_C               (20)
#define SPI_FREQUENCY_HZ                    (20000000)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void show_main_screen(void);
void wait_for_switch_press_and_release(void);
void clear_screen(void);

/*******************************************************************************
* Function Name: void show_main_screen(void)
********************************************************************************
*
* Summary: This function shows the main UI screen with instructions
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void show_main_screen(void)
{
    uint8_t temp[] = "00.0";
    /* Set font size, background color and text mode */
    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLACK);
    GUI_SetTextMode(GUI_TM_NORMAL);

    /* Clear the display */
    GUI_Clear();

    GUI_DrawBitmap(&bmifx_logo, 172, 2);
    /* Horizontal lines */
    GUI_SetPenSize(2);
    GUI_DrawLine(0, 48, 262, 48);
    GUI_SetFont(GUI_FONT_24B_1);
    GUI_DispStringAt("BLUETOOTH UI", 2, 0);
    GUI_DispStringAt("DEMO", 2, 24);
    GUI_DrawBitmap(&bmtemp_logo, 112, 56);
    GUI_SetFont(GUI_FONT_32B_1);

    temp[0] = ((uint8_t) temperature_value/10) + '0';
    temp[1] = ((uint8_t) temperature_value % 10) + '0';
    temp[3] = ((uint8_t) (temperature_value*100) % 10) + '0';

    GUI_DispStringAt((const char*)temp, 170, 72);
    GUI_DispChar(176);
    GUI_DispStringAt("C", 236, 72);

    GUI_SetFont(GUI_FONT_16B_1);
    GUI_SetTextStyle(GUI_TS_UNDERLINE);
    GUI_DispStringAt("FEATURES:", 4, 58);

    GUI_SetTextStyle(GUI_TS_NORMAL);
    GUI_DispCharAt(187, 8, 76);
    GUI_DispStringAt("CAPSENSE", 20, 78);

    GUI_DispCharAt(187, 8, 96);
    GUI_DispStringAt("RGB LED", 20, 98);

    GUI_DispCharAt(187, 8, 116);
    GUI_DispStringAt("HEALTH THERMOMETER", 20, 118);

    GUI_SetFont(GUI_FONT_13_1);
    GUI_DispStringAt("[USE WITH AROC BLUETOOTH CONNECT APP]", 20, 135);

    GUI_SetFont(GUI_FONT_13B_1);
    GUI_DispStringAt("PRESS SW2 TO UPDATE THE SCREEN", 4, 158);

}

/*******************************************************************************
* Function Name: void clear_screen(void)
********************************************************************************
*
* Summary: This function clears the screen
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void clear_screen(void)
{
    GUI_SetColor(GUI_BLACK);
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
}

/*******************************************************************************
* Function Name: void eInk_task(void *arg)
********************************************************************************
*
* Summary:
*  Task that handles Eink initialization and temperature reading.
*
* Parameters:
*  void *param : Task parameter defined during task creation (unused)
*
* Return:
*  None
*
*******************************************************************************/
void eink_task(void *param)
{
    cy_rslt_t result;
    temperature_data_t tempData;

    /* Initialize the ADC */
    result = cyhal_adc_init(&adc, CYBSP_A1, NULL); /* CY8CKIT_028_EPD_PIN_THERM_OUT1 */

    if(CY_RSLT_SUCCESS != result)
    {
        printf("ADC init failed with error code: %lu\r\n", (unsigned long) result);
    }

    /* Initialize thermistor */
    result = mtb_thermistor_ntc_gpio_init(&thermistor, &adc, THERM_PIN_GND, THERM_PIN_VDD,
                                                THERM_PIN_OUT1, &thermistor_cfg,
                                                MTB_THERMISTOR_NTC_WIRING_VIN_R_NTC_GND);

    /* Configure Switch and LEDs*/
    cyhal_gpio_init( CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 
                     CYBSP_BTN_OFF);
    cyhal_gpio_init( CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG,
                     CYBSP_LED_STATE_OFF);
    
    /* Initialize SPI and EINK display */
    result = cyhal_spi_init(&spi, CYBSP_D11, CYBSP_D12, CYBSP_D13, NC, NULL, 8,
                                                        CYHAL_SPI_MODE_00_MSB, false);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to init the spi!\r\n");
        CY_ASSERT(0u);
    }

    result = cyhal_spi_set_frequency(&spi, SPI_FREQUENCY_HZ);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to set frequency for spi!\r\n");
        CY_ASSERT(0u);
    }

    result = mtb_e2271cs021_init(&pins, &spi);

    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize the eink display!\r\n");
        CY_ASSERT(0u);
    }
    printf("Eink display initialization done!\r\n");
    /* Set ambient temperature, in degree C, in order to perform temperature
     * compensation of E-INK parameters */
    mtb_e2271cs021_set_temp_factor(AMBIENT_TEMPERATURE_C);

    current_frame = (uint8_t*)LCD_GetDisplayBuffer();

    /* Initialize EmWin driver*/
    GUI_Init();


    for(;;)
    {

        /* Read temperature value */
        temperature_value = mtb_thermistor_ntc_gpio_get_temp(&thermistor);

        /* Convert from IEEE-754 single precision floating point format to
           IEEE-11073 FLOAT, which is mandated by the health thermometer
           characteristic */

        tempData.temeratureValue = (int32_t) (temperature_value * IEEE_11073_MANTISSA_SCALER);
        tempData.temperatureArray[IEEE_11073_EXPONENT_INDEX] =
                                             IEEE_11073_EXPONENT_VALUE;

        memcpy(&app_hts_temperature_measurement[1u], &tempData, 4u);

        xTaskNotify(bt_task_handle, 2 ,eSetValueWithoutOverwrite);

        cyhal_gpio_write( CYBSP_USER_LED, CYBSP_LED_STATE_ON);

        /* Show the instructions screen */
        show_main_screen();

        /* Update the display */
        mtb_e2271cs021_show_frame(previous_frame, current_frame,
                                  MTB_E2271CS021_FULL_4STAGE, true);

        cyhal_gpio_write( CYBSP_USER_LED, CYBSP_LED_STATE_OFF);

        /* Wait for a switch press event */
        board_switch_press_and_release();

    }
}


