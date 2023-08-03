/*******************************************************************************
* File Name: board.h
*
* Description: This file is the public interface of board.c
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
 * Include guard
 ******************************************************************************/
#ifndef BOARD_H_
#define BOARD_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define DELAY_MS(X)            cyhal_system_delay_ms(X)
#define DELAY_US(X)            cyhal_system_delay_us(X)

/* User LED states */
enum
{
    LED_ON,
    LED_OFF,
};

/* Available User LEDs */
enum
{
    USER_LED1,
    USER_LED2,
    USER_LED_MAX
};

/* Available RGB LED data indexes */
enum
{
    RED_INDEX,
    GREEN_INDEX,
    BLUE_INDEX,
    INTENSITY_INDEX
};

/* Available Blink commands */
enum
{
    BLINK_SLOW = 2u,
    BLINK_MEDIUM = 5u,
    BLINK_FAST = 8u,
};

/* Available LED commands */
typedef enum
{
    LED_TURN_ON,
    LED_TURN_OFF,
    LED_SET_BRIGHTNESS,
} led_command_t;


/* Structure used for handling LED */
typedef struct
{
    led_command_t command;
    uint8_t brightness;
} led_command_data_t;


/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void board_led_set_brightness(uint8_t index, uint8_t value);
void board_led_set_state(uint8_t index, bool value);
void board_led_set_blink(uint8_t index, uint8_t value);
void board_led_set_color(uint8_t *rgb_data);
void board_switch_press_and_release(void);
void board_task(void *param);
cy_rslt_t board_init(void);

/*******************************************************************************
* Extern Variables
*******************************************************************************/
extern TaskHandle_t  board_task_handle;
extern QueueHandle_t led_command_data_q;

#endif /* BOARD_H_ */
