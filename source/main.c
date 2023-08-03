/*******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the FreeRTOS Bluetooth LE
*              UI Demo Example for ModusToolbox.
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
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "wiced_bt_stack.h"
#include "cybsp_bt_config.h"
#include "cycfg_bt_settings.h"
#include "cybt_platform_config.h"
#include "bt_app.h"
#include "board.h"
#include "capsense.h"
#include "eink_shield.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
/* Task parameters for Bluetooth Task. */
#define BT_TASK_PRIORITY                            (2u)
#define BT_TASK_STACK_SIZE                          (512u)

/* Task parameters for CapSense Task. */
#define CAPSENSE_TASK_PRIORITY                      (2u)
#define CAPSENSE_TASK_STACK_SIZE                    (256u)

/* Task parameters for eInk Task. */
#define EINK_TASK_PRIORITY                          (2u)
#define EINK_TASK_STACK_SIZE                        (1024u)

/* Queue lengths of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE                        (1u)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* FreeRTOS task handle for board task. Button task is used to handle button
 * events */
TaskHandle_t  bt_task_handle;
TaskHandle_t  eink_task_handle;

/******************************************************************************
 * Function Definitions
 ******************************************************************************/

/*******************************************************************************
 * Function Name : main
 * *****************************************************************************
 * Summary :
 *   Entry point to the application. Set device configuration and start BT
 *  freeRTOS tasks and initialization.
 *
 * Parameters:
 *    None
 *
 * Return:
 *    None
 ******************************************************************************/
int main()
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    wiced_result_t result = WICED_BT_SUCCESS;

    /* Initialize the board support package */
    cy_result = cybsp_init();

    if (CY_RSLT_SUCCESS != cy_result)
    {
        CY_ASSERT(0);
    }

    /* Initialize the board components */
    cy_result = board_init();

    if( CY_RSLT_SUCCESS != cy_result)
    {
        printf("Board initialization failed!\r\n");
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("==============================================\r\n");
    printf("PSoC™ 6 Bluetooth LE UI Demo\n");
    printf("===============================================\r\n\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Register call back and configuration with stack */
    result = wiced_bt_stack_init(bt_app_management_cb, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( CY_RSLT_SUCCESS == result)
    {
        printf("Bluetooth stack initialization successful!\r\n");
    }
    else
    {
        printf("Bluetooth stack initialization failed!\r\n");
        CY_ASSERT(0);
    }

    /* Create the queues. See the respective data-types for details of queue
    * contents
    */
    led_command_data_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                     sizeof(led_command_data_t));
    if(NULL == led_command_data_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }
    capsense_command_q  = xQueueCreate(SINGLE_ELEMENT_QUEUE,
                                     sizeof(capsense_command_t));
    if(NULL == capsense_command_q)
    {
        printf("Failed to create the queue!\r\n");
        CY_ASSERT(0u);
    }

    /* Create the BT task */
    if (pdPASS != xTaskCreate(bt_task, "BT Task", BT_TASK_STACK_SIZE,
                                                NULL, BT_TASK_PRIORITY, &bt_task_handle))
    {
        printf("Failed to create the BT task!\r\n");
        CY_ASSERT(0u);
    }

    /* Create the CapSense task */
    if (pdPASS != xTaskCreate(capsense_task, "CapSense Task", CAPSENSE_TASK_STACK_SIZE,
                                                NULL, CAPSENSE_TASK_PRIORITY, NULL))
    {
        printf("Failed to create the CapSense task!\r\n");
        CY_ASSERT(0u);
    }

    /* Create the E-Ink task */
    if (pdPASS != xTaskCreate(eink_task, "Eink Task", EINK_TASK_STACK_SIZE,
                                                NULL, EINK_TASK_PRIORITY, NULL))
    {
        printf("Failed to create the Eink task!\r\n");
        CY_ASSERT(0u);
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0) ;
}

/* END OF FILE [] */