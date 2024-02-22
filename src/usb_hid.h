/******************************************************************************
 * File Name: usb_hid.h
 *
 * Description: This file defines data structures and function prototypes
 *              associated with the HID interface
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

#ifndef _USB_HID_H_
#define _USB_HID_H_

#if CY_APP_USB_HID_INTF_ENABLE

#include "cy_app_usb.h"
#include "solution.h"

/* HID command report ID index */
#define HID_COMMAND_REPORT_ID_INDEX             0u
/* HID command code index */
#define HID_COMMAND_CODE_INDEX                  1u
/* HID command status index */
#define HID_COMMAND_STATUS_INDEX                2u
/* HID command data index in Input report */
#define HID_COMMAND_DATA_INDEX                  3u

/* Endpoint number for Interrupt IN endpoint. */
#define HID_IN_EP                               4u
/* Endpoint number for interrupt OUT endpoint. */
#define HID_OUT_EP                              3u

/* Maximum size of each reports. */
#define HID_REPORT_MAX_SIZE                     64u
/* Maximum response data size for an input report. */
#define HID_IP_REPORT_MAX_DATA_SIZE             HID_REPORT_MAX_SIZE - 3u

/* Data sent as part of input report for signaling system sleep. */
#define HID_SYS_CTRL_SLEEP                      0x01u
/* Data sent as part of input report for signaling system wakeup. */
#define HID_SYS_CTRL_WAKEUP                     0x02u

/* USBDEV context variables. These are used by the USB middleware and PDL for the implementation
 *  of USB block. */

/* Report IDs supported by the HID interface. */
typedef enum
{
    /* Report ID for custom report exchange. */
    HID_REPORT_ID_VENDOR = 0x01,

    /* Report ID for system control. */
    HID_REPORT_ID_VENDOR_2 = 0x02,

}cy_en_usb_hid_report_id_t;

/* Command Opcodes supported in HID custom reports. */
typedef enum
{
   /* Opcode for reading Dock Status. */
   HID_READ_DOCK_STATUS = 0x00,

   /* Opcode for reading Dock Composite version. */
   HID_READ_DOCK_COMPOSITE_VERSION,

#if ETAG_SUPPORT_ENABLE
   /* Opcode for setting ETAG information. */
   HID_CMD_SET_ETAG_DATA = 0x10,

   /* Opcode for reading ETAG information. */
   HID_CMD_GET_ETAG_DATA
#endif /* ETAG_SUPPORT_ENABLE */

}cy_en_usb_hid_command_code_t;

/* States involved in HID custom report exchange between host and PMG1S3 dock over interrupt endpoints. */
typedef enum
{
    /* Endpoint is idle. */
    HID_EP_IDLE = 0x0U,

    /* Endpoint receives the output report in this state. */
    HID_EP_CMD,

    /* Endpoint sends out the response (input report) in this state. */
    HID_EP_RESP
}cy_en_usb_hid_state_t;

/* HID command status that are sent as part of the input report. */
typedef enum
{
    HID_STAT_SUCCESS    = 0u,                    /**< Success status. */
    HID_STAT_CMD_NOT_SUPPORTED ,                 /**< Command not supported. */
    HID_STAT_REPORT_ID_NOT_SUPPORTED ,           /**< Report ID not supported. */

#if ETAG_SUPPORT_ENABLE
    HID_STAT_ETAG_PROG_FAILED = 0x40U,           /**< Failure in programming ETAG. */
    HID_STAT_ETAG_ALREADY_PROGRAMMED,            /**< ETAG already programmed. */
    HID_STAT_ETAG_INVALID_DATA,                  /**< Invalid ETAG DATA. */
    HID_STAT_ETAG_FLASH_READ_FAILURE             /**< Failure in reading ETAG from flash. */
#endif /* ETAG_SUPPORT_ENABLE */
}cy_en_usb_hid_cmd_status_t;

/*******************************************************************************
* Function name: hid_task
****************************************************************************//**
*
* This function will run infinitely from main loop. It checks if a system control
* report needs to be sent or not. Also, it runs the HID state machine for custom report
* exchange between host and PMG1S3 dock.
*
* \param
* None.
*
* \return
* None.
*
*******************************************************************************/
void hid_task(void);

/*******************************************************************************
* Function name: dmc_usb_hid_get_report_callback
****************************************************************************//**
*
* This is the callback function registered to the HID driver. It handles GET_REPORT
* request.
*
* \param intf
* Interface.
*
* \param type
* Report Type.
*
* \param id
* Report ID.
*
* \param report
* Report data to be sent to host.
*
* \param size
* Size of report.
*
* \return
* Status of request handling \ref cy_en_usb_dev_status_t
*
*******************************************************************************/
cy_en_usb_dev_status_t dmc_usb_hid_get_report_callback(uint32_t intf, uint32_t type, uint32_t id,
        uint8_t **report, uint32_t *size);

/*******************************************************************************
* Function name: dmc_usb_hid_set_report_callback
****************************************************************************//**
*
* This is the callback function registered to the HID driver. It handles SET_REPORT
* request.
*
* \param intf
* Interface.
*
* \param type
* Report Type.
*
* \param id
* Report ID.
*
* \param report
* Report data received from host.
*
* \param size
* Size of report.
*
* \return
* Status of request handling \ref cy_en_usb_dev_status_t
*
*******************************************************************************/
cy_en_usb_dev_status_t dmc_usb_hid_set_report_callback(uint32_t intf, uint32_t type, uint32_t id,
        uint8_t *report, uint32_t size);

/*******************************************************************************
* Function name: hid_set_sleep_or_wake_flag
****************************************************************************//**
*
* This function is used to determine if system sleep or wakeup needs to be sent
* to the host based on button press events.
*
* \param set_sleep
* If set to true, then system sleep is sent. Otherwise, system wake will be sent.
*
* \return
* None.
*
*******************************************************************************/
void hid_set_sleep_or_wake_flag(bool set_sleep);

#endif /* CY_APP_USB_HID_INTF_ENABLE */
#endif /* _USB_HID_H_ */
