/** music_pad_ble.h
 *
 * This file provides definitions and function prototypes for btstack.
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

#ifndef _music_pad_BLE_H_
#define _music_pad_BLE_H_

#include "cycfg_gatt_db.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define MOTION_SENSOR_TIME_MS       (800u)  /* 100 ms interval for notification */

#define music_pad_CONFIG_SIZE      (1u)    /* No of available client services */

#define music_pad_CCCD_LENGTH      (2u)   /* length in bytes */

/* VSIDs for NVRAM storage */
#define music_pad_VS_ID            (WICED_NVRAM_VSID_START)

/* PUART related macro */
#define NO_ACTION                   (0u)

#define HEADER_DEFAULT 0x80
#define TIMESTAMP_DEFAULT 0x80
#define NOTE_ON_STATUS 0x90  //0xB0
#define NOTE_OFF_STATUS 0x80
#define VELOCITY_DEFAULT 0x3F

#define HEADER_INDEX 0
#define TIMESTAMP_INDEX 1
#define STATUS_INDEX 2
#define NOTE_INDEX 3
#define VELOCITY_INDEX 4

#define NOTE_C 0x3C
#define NOTE_D 0x3E
#define NOTE_E 0x40
#define NOTE_F 0x42
#define NOTE_G 0x43
#define NOTE_A 0x45
#define NOTE_B 0x47
#define NOTE_C1 0x49

/* User Button */
#define USER_BUTTON                 (SW3)   /* Pin P0_0 used as button interrupt*/

/* Connection parameters related macro */
#define CONN_INTERVAL               (80u)   /* (80 * 1.25) = 100ms */
#define CONN_LATENCY                (0u)    /* 0 ms for connection latency */
#define SUP_TIMEOUT                 (512u)  /* supervision timout mutiplier value */
#define CONN_INTERVAL_MAJOR(a)      (a / 100)
#define CONN_INTERVAL_MINOR(a)      (a % 100)
#define CONN_INTERVAL_MULTIPLIER    (125)
#define TIME_OUT_MULTIPLIER         (10u)
#define MAX_SECURITY_KEY_SIZE       (0x10)

/******************************************************************************
 *                                Typedefs
 ******************************************************************************/
#pragma pack(1)
/* Host information saved in  NVRAM */
typedef PACKED struct
{
    uint16_t   connection_id;      /* Connection ID */
    uint16_t  sensor_value_cc_config[music_pad_CONFIG_SIZE];  /* Current value of the client configuration high byte descriptor */
    wiced_bt_device_link_keys_t link_keys; /* Value of the link keys */
    wiced_bt_local_identity_keys_t local_keys; /* value of local keys */
} host_info_t;
#pragma pack()
/******************************************************************************
*                             Function prototypes
******************************************************************************/
wiced_result_t music_pad_management_cback(wiced_bt_management_evt_t event,
                                    wiced_bt_management_evt_data_t *p_event_data);

static wiced_bt_gatt_status_t music_pad_gatts_callback(wiced_bt_gatt_evt_t event,
                                        wiced_bt_gatt_event_data_t *p_event_data);

static wiced_bt_gatt_status_t music_pad_gatts_conn_status_cb(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t music_pad_gatts_connection_up(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t music_pad_gatts_connection_down(
                                    wiced_bt_gatt_connection_status_t *p_status);

static wiced_bt_gatt_status_t music_pad_gatts_req_cb( uint16_t conn_id,
                                            wiced_bt_gatt_request_type_t type,
                                            wiced_bt_gatt_request_data_t *p_data);

static wiced_bt_gatt_status_t music_pad_gatts_get_value( uint16_t attr_handle,
                                            uint16_t conn_id, uint8_t *p_val,
                                            uint16_t max_len, uint16_t *p_len );

static wiced_bt_gatt_status_t music_pad_gatts_set_value( uint16_t attr_handle,
                                                uint16_t conn_id, uint8_t *p_val,
                                                uint16_t len );

static wiced_bt_gatt_status_t music_pad_gatts_req_read_handler(uint16_t conn_id,
                                                wiced_bt_gatt_read_t * p_read_data);

static wiced_bt_gatt_status_t music_pad_gatts_req_write_handler(uint16_t conn_id,
                                                    wiced_bt_gatt_write_t * p_data);

static void music_pad_application_init(void);

static void music_pad_set_advertisement_data(void);

static void temp_timer_callback(uint32_t arg);

static void button_interrupt_cb(void* user_data, uint8_t port_pin);

#endif /* _music_pad_BLE_H_ */
