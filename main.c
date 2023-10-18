/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the T2G MCU: FreeRTOS Blinky
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define BLINKY_TASK_NAME            ("Blinky")
#define BLINKY_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)
#define BLINKY_TASK_PRIORITY        (tskIDLE_PRIORITY + 1)
#define MAIN_TASK_NAME              ("Main")
#define MAIN_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE)
#define MAIN_TASK_PRIORITY          (tskIDLE_PRIORITY + 1)

/* USER LED toggle period in milliseconds */
#define USER_LED_TOGGLE_PERIOD_MS   1000u

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* RTOS semaphore */
static SemaphoreHandle_t xSemaphore;


/*******************************************************************************
* Function Name: blinky_task
********************************************************************************
* Summary:
*  This RTOS task toggles the User LED each time the semaphore is obtained.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  The RTOS task never returns.
*
*******************************************************************************/
static void blinky_task(void *pvParameters)
{
    (void) pvParameters;

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    for(;;)
    {
        /* Block until the semaphore is given */
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        /* Toggle the USER LED state */
        cyhal_gpio_toggle(CYBSP_USER_LED);
    }
}

/*******************************************************************************
* Function Name: main_task
********************************************************************************
* Summary:
*  This RTOS task releases the semaphore every USER_LED_TOGGLE_PERIOD_MS.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  The RTOS task never returns.
*
*******************************************************************************/
static void main_task(void *pvParameters)
{
    (void) pvParameters;

    for(;;)
    {
        /* Block task for USER_LED_TOGGLE_PERIOD_MS. */
        vTaskDelay(USER_LED_TOGGLE_PERIOD_MS);

        /* Release semaphore */
        xSemaphoreGive(xSemaphore);
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It creates two tasks, initializes the semaphore
*  for synchronization between tasks, and starts the FreeRTOS scheduler.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    BaseType_t retval;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Create the Semaphore for synchronization between Blinky and Main task */
    xSemaphore = xSemaphoreCreateBinary();
    if( xSemaphore == NULL )
    {
        CY_ASSERT(0);
    }

    /* Create the RTOS tasks */
    retval = xTaskCreate(blinky_task, BLINKY_TASK_NAME, BLINKY_TASK_STACK_SIZE, NULL, BLINKY_TASK_PRIORITY, NULL );
    if (retval != pdPASS)
    {
        CY_ASSERT(0);
    }

    retval = xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, NULL );
    if (retval != pdPASS)
    {
        CY_ASSERT(0);
    }

    /* Start the scheduler */
    vTaskStartScheduler();

    for(;;)
    {
        /* vTaskStartScheduler never returns */
    }
}

/* [] END OF FILE */
