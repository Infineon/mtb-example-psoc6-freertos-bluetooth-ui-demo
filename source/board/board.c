/*******************************************************************************
* File Name: board.c
*
* Description: This file contains board supported API's.
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

/*******************************************************************************
* Header file includes
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "board.h"
#include "capsense.h"
#include "cy_rgb_led.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define PWM_FREQUENCY_2KHZ              (2000u)

#define PWM_DUTY_CYCLE_0                (0.0)
#define PWM_DUTY_CYCLE_50               (50.0)
#define PWM_DUTY_CYCLE_100              (100.0)

#define BOARD_TASK_PRIORITY             (configMAX_PRIORITIES - 1u)
#define BOARD_TASK_STACK_SIZE           (256u)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void board_led_init(void);
/*******************************************************************************
* Extern Variables
*******************************************************************************/

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* FreeRTOS task handle for board task. Button task is used to handle button
 * events */
TaskHandle_t  board_task_handle;

/* Queue handle used for LED data */
QueueHandle_t led_command_data_q;

/* PWM object */
cyhal_pwm_t pwm_obj[USER_LED_MAX];

/*******************************************************************************
* Function Name: board_init
********************************************************************************
*
* Summary:
*   Initialize the board with LED's and Buttons
*
* Parameters:
*   None
*
* Return:
*   cy_rslt_t  Result status
*
*******************************************************************************/
cy_rslt_t board_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    BaseType_t rtos_result;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    board_led_init();

    /* Create Button Task for processing board events */
    rtos_result = xTaskCreate(board_task,"Board Task", BOARD_TASK_STACK_SIZE,
                            NULL, BOARD_TASK_PRIORITY, &board_task_handle);
    if( pdPASS != rtos_result)
    {
        printf("Failed to create board task.\r\n");
        CY_ASSERT(0u);
    }

    return result;
}

/*******************************************************************************
* Function Name: board_led_init
********************************************************************************
*
* Summary:
*   Initialize the leds with PWM
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
void board_led_init(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the PWM for USER_LED1 */
    result = cyhal_pwm_init_adv(&pwm_obj[USER_LED1], CYBSP_USER_LED1, NC,
                                     CYHAL_PWM_RIGHT_ALIGN, true, 0u, true, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM init failed with error: %lu\r\n", (unsigned long) result);
    }

    /* Initialize the RGB LEDs */
    result = cy_rgb_led_init(CYBSP_LED_RGB_RED, CYBSP_LED_RGB_GREEN, 
                                        CYBSP_LED_RGB_BLUE, CY_RGB_LED_ACTIVE_LOW);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("RGB led init failed with error: %lu\r\n", (unsigned long) result);
    }

    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_obj[USER_LED1]);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM start failed with error: %lu\r\n", (unsigned long) result);
    }

    cy_rgb_led_off();

}

/*******************************************************************************
* Function Name: board_led_set_color
********************************************************************************
*
* Summary:
*   Set the RGB led color and brightness
*
* Parameters:
*   rgb_data: color value
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_color(uint8_t *rgb_data)
{
    uint32_t color;
    uint8_t intensity;

    color = cy_rgb_led_create_color(rgb_data[RED_INDEX],
                                    rgb_data[GREEN_INDEX],
                                    rgb_data[BLUE_INDEX]);

    intensity = rgb_data[INTENSITY_INDEX];

    cy_rgb_led_on(color, intensity);
}

/*******************************************************************************
* Function Name: board_led_set_brightness
********************************************************************************
*
* Summary:
*   Set the led brightness over PWM
*
* Parameters:
*   index: index of LED
*   value: PWM duty cycle value
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_brightness(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index], value, PWM_FREQUENCY_2KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_led_set_state
********************************************************************************
*
* Summary:
*   Set the led state over PWM
*
* Parameters:
*   index: index of LED
*   value: ON/OFF state
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_state(uint8_t index, bool value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index],
                                value?PWM_DUTY_CYCLE_0:PWM_DUTY_CYCLE_100, PWM_FREQUENCY_2KHZ);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_led_set_blink
********************************************************************************
*
* Summary:
*   Set the led frequency for PWM
*
* Parameters:
*   index: index of LED
*   value: value of PWM frequency
*
* Return:
*   None
*
*******************************************************************************/
void board_led_set_blink(uint8_t index, uint8_t value)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cyhal_pwm_set_duty_cycle(&pwm_obj[index], PWM_DUTY_CYCLE_50, value);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("PWM set duty cycle failed with error: %lu\r\n", (unsigned long) result);
        CY_ASSERT(false);
    }
}

/*******************************************************************************
* Function Name: board_switch_press_and_release
********************************************************************************
*
* Summary: This implements a simple "Wait for button press and release"
*           function.  It first waits for the button to be pressed and then
*           waits for the button to be released.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This is a blocking function and exits only on a button press and release
*
*******************************************************************************/
void board_switch_press_and_release(void)
{
    /* Wait for SW2 to be pressed */
    while( CYBSP_BTN_PRESSED != cyhal_gpio_read(CYBSP_USER_BTN));

    /* Wait for SW2 to be released */
    while( CYBSP_BTN_PRESSED == cyhal_gpio_read(CYBSP_USER_BTN));
}

/*******************************************************************************
* Function Name: board_task
********************************************************************************
*
* Summary:
*   This task initialize the board and button events
*
* Parameters:
*   void *param: Not used
*
* Return:
*   None
*
*******************************************************************************/
void board_task(void *param)
{
    led_command_data_t led_cmd_data;

    /* Suppress warning for unused parameter */
    (void)param;

    for(;;)
    {
        /* Block until a command has been received over queue */
        xQueueReceive(led_command_data_q, &led_cmd_data, portMAX_DELAY);
        switch(led_cmd_data.command)
        {
        case LED_TURN_ON:
            board_led_set_state(USER_LED1, LED_ON);
            break;
        case LED_TURN_OFF:
            board_led_set_state(USER_LED1, LED_OFF);
            break;
        case LED_SET_BRIGHTNESS:
            board_led_set_brightness(USER_LED1, led_cmd_data.brightness);
            break;

        }
    }
}


/* [] END OF FILE */
