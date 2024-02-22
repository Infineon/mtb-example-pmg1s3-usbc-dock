/******************************************************************************
 * File Name: usb_hid.c
 *
 * Description: This is source code for the PMG1S3 Dock Solution.
 *              This file implements functions for handling of HID interface
 *              features
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cybsp.h"
#include "usb_hid.h"
#include "cy_app_status.h"
#if ETAG_SUPPORT_ENABLE
#include "etag.h"
#endif /* ETAG_SUPPORT_ENABLE  */
#if CY_APP_DMC_ENABLE
#include "cy_app_dmc_metadata.h"
#endif /* CY_APP_DMC_ENABLE */

#if CY_APP_USB_HID_INTF_ENABLE

/* Buffer used for sending HID system control information to host */
static uint8_t gl_sys_ctrl_report_buffer[HID_REPORT_MAX_SIZE];

/* Buffer that holds input reports */
static uint8_t gl_input_report_buffer[HID_REPORT_MAX_SIZE];

/* Buffer that holds output reports */
static uint8_t gl_output_report_buffer[HID_REPORT_MAX_SIZE];

/* Size of input report to be sent */
static uint32_t gl_input_report_size = 0;

/* Flag to check if there is a valid input report or not */
static bool gl_input_report_valid = false;

/* Flag to indicate that an output report was received. */
static bool gl_output_report_received = false;

/* Flag to check if a wakeup event should be signaled to host or not */
static bool gl_send_wake_to_host = false;

/* Flag to check if a sleep event should be signaled to host or not */
static bool gl_send_sleep_to_host = false;

/* Flag used in HID state machine for report exchange between host and dock through interrupt endpoints. */
static cy_en_usb_hid_state_t gl_hid_state = HID_EP_IDLE;

/* This function will prepare the input report that needs to be sent to the USB host. */
static void prepare_input_report(uint8_t report_id, uint8_t command_code, uint8_t command_status, uint8_t *buffer, uint8_t length);

/* This function will handle the output report received from the USB host. */
static void handle_output_report(uint8_t *report);

void hid_set_sleep_or_wake_flag(bool set_sleep)
{
    if(set_sleep)
    {
        gl_send_sleep_to_host = true;
    }
    else
    {
        gl_send_wake_to_host = true;
    }
}

static void prepare_input_report(uint8_t report_id, uint8_t command_code, uint8_t command_status, uint8_t *buffer, uint8_t length)
{
    memset(gl_input_report_buffer, 0x00, HID_REPORT_MAX_SIZE);
    gl_input_report_buffer[HID_COMMAND_REPORT_ID_INDEX] = report_id;
    gl_input_report_buffer[HID_COMMAND_CODE_INDEX] = command_code;
    gl_input_report_buffer[HID_COMMAND_STATUS_INDEX] = command_status;
    memcpy(&gl_input_report_buffer[HID_COMMAND_DATA_INDEX], buffer, length);
    gl_input_report_size = HID_REPORT_MAX_SIZE;
    gl_input_report_valid = true;
}

static void handle_output_report(uint8_t *report)
{
    uint8_t report_id = report[HID_COMMAND_REPORT_ID_INDEX];
    uint8_t command_code = report[HID_COMMAND_CODE_INDEX];
    uint8_t command_status = HID_STAT_SUCCESS;
    uint8_t buffer[HID_IP_REPORT_MAX_DATA_SIZE];
    cy_en_app_status_t retStatus;
#if CY_APP_DMC_ENABLE
    cy_stc_app_dmc_dock_status_t* dock_status = Cy_App_Dmc_GetDockStatus();
#endif /* CY_APP_DMC_ENABLE */

    memset(buffer, 0x00, HID_IP_REPORT_MAX_DATA_SIZE);

    /* Identify the report id */
    switch(report_id)
    {
        case HID_REPORT_ID_VENDOR:
            /* Identify the command code */
            switch(command_code)
            {
#if CY_APP_DMC_ENABLE
                case HID_READ_DOCK_STATUS:
                    /* Prepare the input report and do the task related to the command */
                    command_status = HID_STAT_SUCCESS;
                    buffer[0] = dock_status->dock_status;
                    prepare_input_report(report_id, command_code, command_status, buffer, 1);
                    break;

                case HID_READ_DOCK_COMPOSITE_VERSION:
                    /* Prepare the input report and do the task related to the command */
                    command_status = HID_STAT_SUCCESS;
                    buffer[0] = (uint8_t)(dock_status->composite_ver >> 24);
                    buffer[1] = (uint8_t)(dock_status->composite_ver >> 16);
                    buffer[2] = (uint8_t)(dock_status->composite_ver >> 8);
                    buffer[3] = (uint8_t)(dock_status->composite_ver);
                    prepare_input_report(report_id, command_code, command_status, buffer, 4);
                    break;
#endif /* CY_APP_DMC_ENABLE */
#if ETAG_SUPPORT_ENABLE
                case HID_CMD_SET_ETAG_DATA:
                    retStatus = (uint8_t)write_etag_information(report);
                    switch(retStatus)
                    {
                        case CY_APP_STAT_INVALID_COMMAND:
                            command_status = (uint8_t)HID_STAT_ETAG_ALREADY_PROGRAMMED;
                            break;

                        case CY_APP_STAT_SUCCESS:
                            command_status = (uint8_t)HID_STAT_SUCCESS;
                            break;

                        case CY_APP_STAT_FAILURE:
                            command_status = (uint8_t)HID_STAT_ETAG_PROG_FAILED;
                            break;

                        default:
                            /* Do Nothing */
                            break;
                    }

                    prepare_input_report(report_id, command_code, command_status, NULL, 0);
                    break;

                case HID_CMD_GET_ETAG_DATA:

                    if(read_etag_information(buffer) == CY_APP_STAT_SUCCESS)
                    {
                        if(buffer[ETAG_VALID_OFFSET] == ETAG_VALID)
                        {
                            command_status = (uint8_t)HID_STAT_SUCCESS;
                        }
                        else
                        {
                            command_status = (uint8_t)HID_STAT_ETAG_INVALID_DATA;
                        }
                    }
                    else
                    {
                        command_status = (uint8_t)HID_STAT_ETAG_FLASH_READ_FAILURE;
                    }

                    prepare_input_report(report_id, command_code, command_status, &buffer[2], ETAG_DATA_SIZE);
                    break;
#endif /* ETAG_SUPPORT_ENABLE  */
                default:
                    /* Prepare the input report with invalid command response */
                    command_status = HID_STAT_CMD_NOT_SUPPORTED;
                    prepare_input_report(report_id, command_code, command_status, NULL, 0);
                    break;
            }
            break;

        default:
            /* Prepare the input report with invalid report id response */
            command_status = HID_STAT_REPORT_ID_NOT_SUPPORTED;
            prepare_input_report(report_id, command_code, command_status, NULL, 0);
            break;
    }
    (void)retStatus;

}

cy_en_usb_dev_status_t dmc_usb_hid_get_report_callback(
                       uint32_t intf,
                       uint32_t type,
                       uint32_t id,
                       uint8_t **report,
                       uint32_t *size)
{
    if(gl_input_report_valid)
    {
        *report = gl_input_report_buffer;
        *size = gl_input_report_size;
        gl_input_report_valid = false;
        return CY_USB_DEV_SUCCESS;
    }
    return CY_USB_DEV_REQUEST_NOT_HANDLED;
}

cy_en_usb_dev_status_t dmc_usb_hid_set_report_callback(
                       uint32_t intf,
                       uint32_t type,
                       uint32_t id,
                       uint8_t *report,
                       uint32_t size)
{
    memcpy(gl_output_report_buffer, report, size);
    gl_output_report_received = true;
    return CY_USB_DEV_SUCCESS;
}

void hid_state_machine(cy_en_usb_hid_state_t *state)
{
    uint32_t actSize;
    cy_en_usb_dev_status_t status;
    cy_en_usb_hid_state_t cur_state = *state;

    switch(cur_state)
    {
        case HID_EP_IDLE:
        {
            status = Cy_USB_Dev_StartReadEp(HID_OUT_EP, &gl_usb_devContext);
            if(status == CY_USB_DEV_SUCCESS)
            {
                *state = HID_EP_CMD;
            }
        }
        break;

        case HID_EP_CMD:
            status = Cy_USB_Dev_ReadEpNonBlocking(HID_OUT_EP, gl_output_report_buffer, HID_REPORT_MAX_SIZE, &actSize, &gl_usb_devContext);
            if(status == CY_USB_DEV_SUCCESS)
            {
                gl_output_report_received = true;
                *state = HID_EP_RESP;
            }
         break;

        case HID_EP_RESP:
            if(gl_input_report_valid)
            {
                actSize = gl_input_report_size;
                status = Cy_USB_Dev_WriteEpNonBlocking(HID_IN_EP, gl_input_report_buffer, actSize, &gl_usb_devContext);
                if(status == CY_USB_DEV_SUCCESS)
                {
                    *state = HID_EP_IDLE;
                }
                gl_input_report_valid = false;
            }
         break;

        default:
            /* Do Nothing. */
         break;
    }
}

void hid_task(void)
{
    /* If wake up needs to be sent to host, then send the input report for triggering system wake to host. */
    if(gl_send_wake_to_host)
    {
        gl_sys_ctrl_report_buffer[0] = HID_REPORT_ID_VENDOR_2;
        gl_sys_ctrl_report_buffer[1] = HID_SYS_CTRL_WAKEUP;
        /* Send the report to the USB host */
        Cy_USB_Dev_WriteEpNonBlocking(HID_IN_EP, gl_sys_ctrl_report_buffer, 2, &gl_usb_devContext);
        gl_send_wake_to_host = false;
    }
    /* If sleep needs to be sent to host, then send the input report for triggering system sleep to host. */
    if(gl_send_sleep_to_host)
    {
        gl_sys_ctrl_report_buffer[0] = HID_REPORT_ID_VENDOR_2;
        gl_sys_ctrl_report_buffer[1] = HID_SYS_CTRL_SLEEP;
        /* Send the report to the USB host */
        Cy_USB_Dev_WriteEpNonBlocking(HID_IN_EP, gl_sys_ctrl_report_buffer, 2, &gl_usb_devContext);
        gl_send_sleep_to_host = false;
    }
    /* If output report is received, then handle it. */
    if(gl_output_report_received)
    {
        handle_output_report(gl_output_report_buffer);
        gl_output_report_received = false;
    }
    /* Run the HID state machine. */
    hid_state_machine(&gl_hid_state);
}

#endif /* CY_APP_USB_HID_INTF_ENABLE */
