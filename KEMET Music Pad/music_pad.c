 /**
 * File name: sensor_hub.c
 *
 * Description: Main file which is the entry point to the application.
 *              The application_start function initializes UART for trace
 *              messages and initializes the BT stack.
 *
 * Features demonstrated:
 *              I2C communication, ADC Reading, Timers, BLE advertisement,
 *              BLE connection, and notifications.
 *
 *  Controls:
 *  - Remove bond data with button press
 *
 */

/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 */

#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "app_bt_cfg.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"
#include <music_pad_ble.h>

/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Sets trace to PUART and register BLE management event callback
*
* Parameters:
*   None
*
* Return:
*  None
*
*******************************************************************************/
void application_start(void)
{
    /* Route Trace messages to PUART */
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen */
    WICED_BT_TRACE("\x1b[2J\x1b[;H");
    WICED_BT_TRACE("\r\n-------------------------------------------------\r\n\n"
                       "                  Music Pad                     \r\n\n"
                       "-------------------------------------------------\r\n");

    /* Register call back and configuration with stack */
     if(WICED_BT_SUCCESS != wiced_bt_stack_init(music_pad_management_cback ,
                     &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools))
     {
         WICED_BT_TRACE("Stack Init failed\r\n");
     }
}
