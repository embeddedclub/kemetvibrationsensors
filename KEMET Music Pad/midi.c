 /** motion_sensor_hw.c
 *
 * This file contains the APIs related to the lsm9ds1 sensor
 * The lsm9ds1 sensor libraries can be found at:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm9ds1_STdC
 * Commit ID: bb6d11f6fea38a08f680cb9f6cb5ed3f11f6e060
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

#include "midi.h"


/******************************************************************************
 *                                Structures
 ******************************************************************************/
uint8_t drum_1[8][4]= {{36u,37u,38u,40u}, // Bass Drum 1, Side Stick, Acoustic Snare, 	Electric Snare
					   {41u,36u,45u,44u}, //Low Floor Tom, Bass Drum 1, Low Tom, Pedal Hi-Hat
					   {40u,48u,37u,51u}, //Electric Snare, Hi Mid Tom, Side Stick, Ride Cymbal 1
					   {38u,39u,42u, 49u}, //Acoustic Snare, Hand Clap, 	Closed Hi Hat, Crash Cymbal 1
					   {60u,61u,64u,66u},
					   {60u,61u,64u,66u},
					   {60u,61u,64u,66u},
					   {60u,61u,64u,66u},

};



/******************************************************************************
 *                            Functions prototype
 ******************************************************************************/


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/


 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/


