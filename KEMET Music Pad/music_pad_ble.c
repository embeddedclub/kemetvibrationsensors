 /** music_pad_ble.c
*
* This file contains the btstack implementation that handles
* ble events and notifications.
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


#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"
#include "cycfg_pins.h"
#include "app_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_adc.h"
#include "wiced_hal_i2c.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "bt_types.h"
#include "wiced_rtos.h"
#include "wiced_bt_l2c.h"
#include "music_pad_ble.h"
#include "afe_shield_hw.h"
#include "midi.h"

/******************************************************************************
 *                   External Variables and Functions
 ******************************************************************************/

/******************************************************************************
 *                                Constants
 ******************************************************************************/
/* CCCD Index values of Sensors */
enum
{
    MOTION_SENSOR_CCCD_INDEX
};

#define BAR_LED1 WICED_P15
#define BAR_LED2 WICED_P06
#define BAR_LED3 WICED_P17
#define BAR_LED4 WICED_P09
/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
/* Timer handles */
static wiced_timer_t hal_gpio_app_timer, led_app_timer;
volatile uint32_t pulse_count = 0;
volatile uint32_t pulse_count_last = 0;
volatile uint32_t presence_count = 0;
/* Holds the host information*/
host_info_t midi_controller_hostinfo;

extern volatile uint8_t proximity_value;
extern volatile uint8_t swbutton_value;

static uint8_t prv_cap_value=0;
static uint8_t prv_sw_value=0;
static uint8_t prv_prox_value = 0;
static uint8_t kit_count =0;
static uint8_t vib_key_index[2]={1,2};
static uint8_t vib_i =0;
static uint8_t prv_vib_i =0;
static uint8_t pulse_off =0;
static uint8_t vsense_off =1;
void play_notes(uint8_t status, uint8_t channel,uint8_t note, uint8_t velocity);
void hal_gpio_app_interrrupt_handler(void *data, uint8_t pin);
void hal_gpio_app_timer_cb(uint32_t arg);
void app_led_timer_cb(uint32_t arg);

/******************************************************************************
*                                Function Definitions
******************************************************************************/
/**
 * Function         music_pad_management_cback
 *
 *                  This function is invoked after the controller's
 *                  initialization is complete
 *
 * @param[in] event                : Callback Event number
 * @param[in] p_event_data         : Event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_result_t music_pad_management_cback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    uint16_t conn_interval = 0;

    WICED_BT_TRACE("Sensor hub management cback: %d\r\n", event);

    switch(event)
    {
        case BTM_ENABLED_EVT:
            /* Initialize the application */
            music_pad_application_init();
            break;

        case BTM_DISABLED_EVT:
            /* Bluetooth Controller and Host Stack Disabled */
            WICED_BT_TRACE("Bluetooth Disabled \r\n");
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* No IO capabilities on this platform */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data     = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req     = BTM_LE_AUTH_REQ_NO_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_SECURITY_KEY_SIZE;
            p_event_data->pairing_io_capabilities_ble_request.init_keys    = 0;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys    = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
            break;

         case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

             if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
             {
                 WICED_BT_TRACE("Advertisement State Change: OFF\r\n");
             }
             else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_HIGH)
             {
                 WICED_BT_TRACE("Advertisement State Change: Undirected High\r\n");
             }
             else if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_UNDIRECTED_LOW)
             {
                 WICED_BT_TRACE("Advertisement State Change: Undirected Low\r\n");
             }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            /* Connection parameters updated */
            if(WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                conn_interval = (p_event_data->ble_connection_param_update.conn_interval) * CONN_INTERVAL_MULTIPLIER;
                WICED_BT_TRACE("New connection parameters:"
                               "\n\rConnection interval: %d.%dms\r\n",
                                  CONN_INTERVAL_MAJOR(conn_interval),
                                 CONN_INTERVAL_MINOR(conn_interval));
            }
            else
            {
                WICED_BT_TRACE("Connection parameters update failed\r\n");
            }
            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: %d\r\n", event);
            break;
    }

    return result;
}

/**
 * Function         music_pad_application_init
 *
 * @brief           This function is invoked after ENABLED event from controller
 *
 * @return    None
 */
void music_pad_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_device_address_t  local_device_bd_addr;

    /* Initialize I2C */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);

    /* Configure the Button GPIO as an input with a resistive pull up and falling edge interrupt */
    wiced_hal_gpio_register_pin_for_interrupt( SW3, button_interrupt_cb, NULL );
    wiced_hal_gpio_configure_pin( USER_BUTTON,
                                ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ),
                                GPIO_PIN_OUTPUT_HIGH );

    wiced_hal_gpio_register_pin_for_interrupt(WICED_P14, hal_gpio_app_interrrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(WICED_P14, GPIO_EN_INT_FALLING_EDGE , GPIO_PIN_OUTPUT_LOW);

	/* Initilize the OLED */
	afe_shield_init_oled();

	/* Clear the OLED screen */
	afe_shield_clear_screen();

	/* Init OLED with BLE information */
	afe_shield_print_largeText(0, 5,"KEMET PAD");
	afe_shield_print_smallText(3, 10, "DRUM KIT - 1 ");
	afe_shield_print_smallText(5, 10, "ACTIVE - [1]");
	afe_shield_print_smallText(5, 80, " - [2]");
	afe_shield_print_smallText(7, 10, "VSENSE - OFF");

    // Configure all the selected pins to be output
 //   wiced_hal_gpio_configure_pin(BAR_LED1, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
 //   wiced_hal_gpio_configure_pin(BAR_LED2, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
 //  wiced_hal_gpio_configure_pin(BAR_LED3, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
 //  wiced_hal_gpio_configure_pin(BAR_LED4, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    // Initialize timer to control the pin toggle frequency
    if (wiced_init_timer(&hal_gpio_app_timer, &hal_gpio_app_timer_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&hal_gpio_app_timer, 10) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("Seconds Timer Error\n");
        }


    }

    if (wiced_init_timer(&led_app_timer, &app_led_timer_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        if (wiced_start_timer(&led_app_timer, 80) != WICED_SUCCESS)
        {
            WICED_BT_TRACE("LED Timer Error\n");
        }
    }




    wiced_bt_dev_read_local_addr(local_device_bd_addr);

    WICED_BT_TRACE("Bluetooth Device Address: [ %B] \r\n\n",
                    local_device_bd_addr);

    /* Not Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_FALSE, 0u);

    WICED_BT_TRACE("\r\nDiscover this device with the name: \"%s\"\r\n",
                    app_gap_device_name);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(music_pad_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\r\n", gatt_status);

    /*  Tell stack to use our GATT database */
    WICED_BT_TRACE("GATT DB length: %d\r\n", gatt_database_len);
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len);
    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\r\n", gatt_status);

    /* Set the advertising params and make the device discoverable */
    music_pad_set_advertisement_data();
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         0u, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);


}

/**
 * Function         music_pad_gatts_callback
 *
 * @brief           This function is invoked on GATT event
 *
 * @param[in] event                : GATT event
 * @param[in] p_event_data         : GATT event data
 *
 * @return    WICED_BT_SUCCESS : on success;
 *            WICED_BT_FAILED : if an error occurred
 */
wiced_bt_gatt_status_t music_pad_gatts_callback(wiced_bt_gatt_evt_t event,
                                             wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_PDU;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = music_pad_gatts_conn_status_cb(&p_event_data->connection_status);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        p_attr_req = &p_event_data->attribute_request;
        status = music_pad_gatts_req_cb(p_attr_req->conn_id, p_attr_req->request_type, &p_attr_req->data);
        break;

    default:
        break;
    }
    return status;
}

/**
 * Function         music_pad_gatts_conn_status_cb
 *
 * @brief           This function is invoked on GATT connection event
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : Connection status from connection up/down function
 */
wiced_bt_gatt_status_t music_pad_gatts_conn_status_cb(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    if (p_status->connected)
    {
        return music_pad_gatts_connection_up(p_status);
    }

    return music_pad_gatts_connection_down(p_status);
}

/**
 * Function         music_pad_gatts_connection_up
 *
 * @brief           This function is invoked when connection is established
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t music_pad_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result = WICED_BT_ERROR;

    WICED_BT_TRACE("Sensor hub connected with [ %B] id:%d\r\n", p_status->bd_addr,
                                                             p_status->conn_id);


    /* Update the connection handler.  Save address of the connected device. */
    midi_controller_hostinfo.connection_id= p_status->conn_id;

    /* Stop advertising */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, BLE_ADDR_PUBLIC,
                                                                          NULL);

    WICED_BT_TRACE("Stopping Advertisements %d\r\n", result);

    /* Update connection parameters to 100 ms for SDS */
    wiced_bt_l2cap_update_ble_conn_params(p_status->bd_addr, CONN_INTERVAL,
                                      CONN_INTERVAL, CONN_LATENCY, SUP_TIMEOUT);

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function         music_pad_gatts_connection_down
 *
 * @brief           This function is invoked when connection is lost
 *
 * @param[in] p_status               : GATT connection status
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t music_pad_gatts_connection_down(
                                    wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    BD_ADDR bdaddr;
    memcpy(bdaddr, midi_controller_hostinfo.link_keys.bd_addr, sizeof(bdaddr));

    WICED_BT_TRACE("Sensor hub disconnected from [ %B] conn_id:%d reason:%d\r\n",
            bdaddr, p_status->conn_id, p_status->reason);

    /* Resetting the device info */
    midi_controller_hostinfo.connection_id = 0;

    /*Undirected advertisement */

    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                                     0u, NULL);

    WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);


    return WICED_BT_SUCCESS;
}

/**
 * Function         music_pad_gatts_req_cb
 *
 * @brief           Process GATT request from the peer
 *
 * @param[in] p_data                 : GATT attribute request data
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t music_pad_gatts_req_cb( uint16_t conn_id, wiced_bt_gatt_request_type_t type,
                                                                wiced_bt_gatt_request_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_ERROR;

    /* Check the type of request and service it by function calls */
    switch (type)
    {
    case GATTS_REQ_TYPE_READ:
        WICED_BT_TRACE("GATT read request. connection handle is %d\r\n", p_data->handle);
        result = music_pad_gatts_req_read_handler(conn_id, &p_data->read_req);
        break;
    case GATTS_REQ_TYPE_WRITE:
        WICED_BT_TRACE("GATT write request. connection handle is %d\r\n", p_data->handle);
        result = music_pad_gatts_req_write_handler(conn_id, &p_data->write_req);
        break;
    default:
        WICED_BT_TRACE("GATT request unhandled..\r\n");
        break;
    }
    return result;
}

/**
 * Function         music_pad_gatts_get_value
 *
 * @brief           This function handles reading of the attribute value from the GATT database and passing the
 *                   data to the BT stack.
 *
 * @param[in] attr_handle               : Attribute handle for read operation
 * @param[in] conn_id                     : Connection ID
 * @param[in] *p_val                     : Pointer to the buffer to store read data
 * @param[in] max_len                     : Maximum buffer length available to store
 * @param[in] *p_len                     : Actual length of data copied to the buffer
 *
 * @return                               : wiced_bt_gatt_status_e
 */
wiced_bt_gatt_status_t music_pad_gatts_get_value( uint16_t attr_handle, uint16_t conn_id, uint8_t *p_val,
                                                                         uint16_t max_len, uint16_t *p_len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= max_len)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                // Value to read will not fit within the buffer
                status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // Add code to read value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully read then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
        default:
            // The read operation was not performed for the indicated handle
            WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\r\n", attr_handle);
            status = WICED_BT_GATT_READ_NOT_PERMIT;
            break;
        }
    }

    return status;
}

/**
 * Function         music_pad_gatts_set_value
 *
 * @brief           This function handles writing to the attribute value in the GATT database
 *                   using the data passed from the BT stack.
 *
 * @param[in] attr_handle               : Attribute handle for write operation
 * @param[in] conn_id                     : Connection ID
 * @param[in] *p_val                     : Pointer to the buffer to store the data to be written
 * @param[in] len                         : Length of data to be written
 *
 * @return                               : wiced_bt_gatt_status_e
 */

wiced_bt_gatt_status_t music_pad_gatts_set_value( uint16_t attr_handle, uint16_t conn_id,
                                                            uint8_t *p_val, uint16_t len )
{
    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Verify that size constraints have been met
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                status = WICED_BT_GATT_SUCCESS;

                switch ( attr_handle )
                {
                case HDLD_MIDI_CONTROLLER_MUSIC_NOTES_CLIENT_CHAR_CONFIG:
                    if ( len != 2 )
                    {
                        return WICED_BT_GATT_INVALID_ATTR_LEN;
                    }
                    midi_controller_hostinfo.sensor_value_cc_config[0] = p_val[0] | ( p_val[1] << 8 );


                    break;

                default:
                    WICED_BT_TRACE("Write is not supported \r\n");
                }
            }
            else
            {
                // Value to write does not meet size constraints
                status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        switch ( attr_handle )
        {
        default:
            // The write operation was not performed for the indicated handle
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\r\n", attr_handle);
            status = WICED_BT_GATT_WRITE_NOT_PERMIT;
            break;
        }
    }

    return status;
}

/**
 * Function         music_pad_gatts_req_read_handler
 *
 * @brief           Process Read request or command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from GATT read request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t music_pad_gatts_req_read_handler(uint16_t conn_id,
                                             wiced_bt_gatt_read_t * p_read_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = music_pad_gatts_get_value(p_read_data->handle, conn_id, p_read_data->p_val,
                                        *p_read_data->p_val_len, p_read_data->p_val_len);

    if(WICED_BT_GATT_SUCCESS == status)
    {


    }
    return status;;
}

/**
 * Function         music_pad_gatts_req_write_handler
 *
 * @brief           Process write request or write command from peer device
 *
 * @param[in] conn_id                : Connection ID
 * @param[in] p_read_data            : Data from GATT write request
 *
 * @return                           : WICED_BT_GATT_SUCCESS
 */
wiced_bt_gatt_status_t music_pad_gatts_req_write_handler(uint16_t conn_id,
                                                 wiced_bt_gatt_write_t * p_data)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = music_pad_gatts_set_value(p_data->handle, conn_id,
                                        p_data->p_val, p_data->val_len);

    return status;
}

/**
 * Function         motion_sensor_set_advertisement_data
 *
 * @brief           Set Advertisement data
 *
 * @return                          : None
 */
void music_pad_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3] = {0};
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t midi_controller_service_uuid[LEN_UUID_128] = { __UUID_SERVICE_MIDI_CONTROLLER };

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)app_gap_device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)app_gap_device_name;
    num_elem++;

    /* Advertisement Element for Sensor Hub Service */
    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_128;
    adv_elem[num_elem].p_data       = midi_controller_service_uuid;
    num_elem++;

    if(WICED_BT_SUCCESS != wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem))
    {
        WICED_BT_TRACE("Set advertisement data failed\r\n");
    }
}

void set_bar_led(uint8_t value)
{
	   wiced_hal_gpio_set_pin_output(BAR_LED1, value & 0x1);
	   wiced_hal_gpio_set_pin_output(BAR_LED2, value>>1 & 0x1);
	   wiced_hal_gpio_set_pin_output(BAR_LED3, value>>2 & 0x1);
	   wiced_hal_gpio_set_pin_output(BAR_LED4, value>>3 & 0x1);
}

void app_led_timer_cb(uint32_t arg)
{


	static uint8_t strength = 0;

   // wiced_seconds++;
    WICED_BT_TRACE("Pulse Count is %d \n\r", swbutton_value);

	if(swbutton_value != prv_sw_value && swbutton_value == 2)
	{
		vsense_off = 1- vsense_off;
		if(vsense_off==1)
		{
			afe_shield_print_smallText(7, 10, "VSENSE - OFF");
		}
		else
		{
			afe_shield_print_smallText(7, 10, "VSENSE - ON ");
		}
	}

    if(swbutton_value != prv_sw_value && swbutton_value == 1)
    {
    	kit_count++;

    	if(kit_count>7) kit_count =0;
    	switch(kit_count)
    	{
    	case 0:
    		afe_shield_print_smallText(3, 10, "DRUM KIT - 1 ");

    	break;
    	case 1:
    		afe_shield_print_smallText(3, 10, "DRUM KIT - 2 ");
    	break;
    	case 2:
    		afe_shield_print_smallText(3, 10, "DRUM KIT - 3 ");
    	break;
    	case 3:
    		afe_shield_print_smallText(3, 10, "DRUM KIT - 4 ");
    	break;
    	case 4:
    		afe_shield_print_smallText(3, 10, "KEY BOARD - 1");

    	break;
    	case 5:
    		afe_shield_print_smallText(3, 10, "KEY BOARD - 2");
    	break;
    	case 6:
    		afe_shield_print_smallText(3, 10, "STRINGS - 1  ");
    	break;
    	case 7:
    		afe_shield_print_smallText(3, 10, "STRINGS - 2");
    	break;
    	default:
    		break;
    	}
    }

    if(vib_i != prv_vib_i){

	if(vib_key_index[0]== 1) afe_shield_print_smallText(5, 10, "ACTIVE - [1]");
	if(vib_key_index[0]== 2) afe_shield_print_smallText(5, 10, "ACTIVE - [2]");
	if(vib_key_index[0]== 3) afe_shield_print_smallText(5, 10, "ACTIVE - [3]");
	if(vib_key_index[0]== 4) afe_shield_print_smallText(5, 10, "ACTIVE - [4]");

	if(vib_key_index[1]== 1) afe_shield_print_smallText(5, 80, " - [1]");
	if(vib_key_index[1]== 2) afe_shield_print_smallText(5, 80, " - [2]");
	if(vib_key_index[1]== 3) afe_shield_print_smallText(5, 80, " - [3]");
	if(vib_key_index[1]== 4) afe_shield_print_smallText(5, 80, " - [4]");

    }
	prv_vib_i = vib_i;

    if(pulse_count == 0 && pulse_off == 1)
    {
 	   strength = 10;
 	   if(proximity_value == 1)
 	   {
	   play_notes(0, 1, drum_1[kit_count][vib_key_index[0]-1], strength);
 	   }
 	   else
 	   {
 		  play_notes(0, 2, drum_1[kit_count][vib_key_index[1]-1], strength);
 	   }
	   pulse_off = 0;
		if((GATT_CLIENT_CONFIG_NOTIFICATION ==
			   midi_controller_hostinfo.sensor_value_cc_config[0])
											  && (0 != midi_controller_hostinfo.connection_id ))
		   {


			   if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
										  midi_controller_hostinfo.connection_id ,
										  HDLC_MIDI_CONTROLLER_MUSIC_NOTES_VALUE,
										  app_gatt_db_ext_attr_tbl[2].cur_len,
										  app_gatt_db_ext_attr_tbl[2].p_data))
			   {
				   WICED_BT_TRACE("Sending motion sensor value notification failed\r\n");
			   }

		   }
    }

	if(pulse_count !=0)
	{

	   if(pulse_count > 2 && pulse_count <= 25)
	       {
		   strength  = 0x40;
	       }
	       if(pulse_count > 25 && pulse_count <= 50)
	       {
	    	   strength  = 0x40 + 5;
	       }
	       if(pulse_count > 50 && pulse_count <= 100)
	       {
	    	   strength  = 0x40 + 10;
	       }
	       if(pulse_count > 100 && pulse_count <= 200 )
	       {
	    	   strength  = 0x40 + 15;
	       }
	       if(pulse_count > 200 )
	       {
	    	   strength  = 0x40 + 20;
	       }


	 	   if(proximity_value == 1)
	 	   {
	 		   play_notes(1, 1, drum_1[kit_count][vib_key_index[0]-1], strength);
	 	   }
	 	   else
	 	   {
	 		   play_notes(1, 2, drum_1[kit_count][vib_key_index[1]-1], strength);
	 	   }


	   	if((GATT_CLIENT_CONFIG_NOTIFICATION ==
	   		   midi_controller_hostinfo.sensor_value_cc_config[0])
	   										  && (0 != midi_controller_hostinfo.connection_id )&& pulse_off == 0)
	   	   {


	           if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
	                                      midi_controller_hostinfo.connection_id ,
	                                      HDLC_MIDI_CONTROLLER_MUSIC_NOTES_VALUE,
	                                      app_gatt_db_ext_attr_tbl[2].cur_len,
	                                      app_gatt_db_ext_attr_tbl[2].p_data))
	           {
	               WICED_BT_TRACE("Sending motion sensor value notification failed\r\n");
	           }

	           pulse_off = 1;

	       }
	       pulse_count = 0;
	}

}



/*
 * The function invoked on timeout of app. seconds timer.
 */
void hal_gpio_app_timer_cb(uint32_t arg)
{
    static uint32_t wiced_seconds = 0; /* number of seconds elapsed */
    uint8_t cap_value = 0;

    cap_value = afe_shield_get_val();
	if(cap_value != prv_cap_value)
	{
    switch(cap_value)
    {

    case 1:
    	play_notes(1, 1, drum_1[kit_count][0], 0x48);
    	break;
    case 3:
    	//play_notes(1, 1, drum_1[0][1], 0x40);
    	break;
    case 2:
    	play_notes(1, 2, drum_1[kit_count][1], 0x48);
    	break;
    case 6:
    	//play_notes(1, 11, 38, 0x40);
    	break;
    case 4:
    	play_notes(1, 3, drum_1[kit_count][2], 0x48);
    	break;
    case 8:
    	play_notes(1, 4, drum_1[kit_count][3], 0x48);
    	break;
    default:
    {
        	switch(prv_cap_value)
        	{
            case 1:
            	play_notes(0, 1, drum_1[kit_count][0], 0x10);
            	vib_key_index[vib_i]= 1;
            	break;
            case 3:
            	//play_notes(0, 1, 37, 0x40);
            	break;
            case 2:
            	play_notes(0, 2, drum_1[kit_count][1], 0x10);
            	vib_key_index[vib_i]= 2;
            	break;
            case 6:
            	//play_notes(0, 11, 38, 0x40);
            	break;
        	case 4:
            	play_notes(0, 3, drum_1[kit_count][2], 0x10);
            	vib_key_index[vib_i]= 3;
        		break;
        	case 8:
            	play_notes(0, 4, drum_1[kit_count][3], 0x10);
            	vib_key_index[vib_i]= 4;
        		break;
        	}
        	vib_i++;
        	if(vib_i>1){
        		vib_i=0;
        	}


    }
    break;

    }

    prv_cap_value = cap_value;

    if((GATT_CLIENT_CONFIG_NOTIFICATION ==
           midi_controller_hostinfo.sensor_value_cc_config[0])
                                          && (0 != midi_controller_hostinfo.connection_id ))
       {

        if(WICED_BT_GATT_SUCCESS != wiced_bt_gatt_send_notification(
                                   midi_controller_hostinfo.connection_id ,
                                   HDLC_MIDI_CONTROLLER_MUSIC_NOTES_VALUE,
                                   app_gatt_db_ext_attr_tbl[2].cur_len,
                                   app_gatt_db_ext_attr_tbl[2].p_data))
        {
            WICED_BT_TRACE("Sending motion sensor value notification failed\r\n");
        }

     }

    }






}

/*
 * Handle interrupt generated due to change in the GPIO state
 */
void hal_gpio_app_interrrupt_handler(void *data, uint8_t pin)
{

    static uint8_t index = 0;
    if(pulse_off==0 && vsense_off ==0)pulse_count++;

   // clear the interrupt status
    wiced_hal_gpio_clear_pin_interrupt_status(pin);
}

/**
 * Function         button_interrupt_cb
 *
 * @brief           Button callback
 * @param[in] user_data      : related to the interrupt, populated internally.
 * @param[in] port_pin       : pin number on which interrupt was received.
 *
 * @return                   : None
 */
void button_interrupt_cb(void* user_data, uint8_t port_pin)
{
    wiced_result_t result = WICED_BT_ERROR;
    uint16_t written_byte = 0u;

    if(0 == midi_controller_hostinfo.connection_id)
    {
        /* Set the advertising params and make the device discoverable */
        music_pad_set_advertisement_data();

        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                         0u, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements %d\r\n", result);
    }

    /* Clear the GPIO interrupt */
    wiced_hal_gpio_clear_pin_interrupt_status( port_pin );

}


void play_notes(uint8_t status, uint8_t channel,uint8_t note, uint8_t velocity)
{

	if(status==1)
	{
	     app_midi_controller_music_notes[STATUS_INDEX]= NOTE_ON_STATUS | channel;
	}
	else
	{
	     app_midi_controller_music_notes[STATUS_INDEX]= NOTE_OFF_STATUS | channel;
	}

     app_midi_controller_music_notes[HEADER_INDEX]= HEADER_DEFAULT;
     app_midi_controller_music_notes[TIMESTAMP_INDEX]= TIMESTAMP_DEFAULT;
     app_midi_controller_music_notes[NOTE_INDEX]= note;
     app_midi_controller_music_notes[VELOCITY_INDEX]= velocity;

}
