/******************************************************************************
* File Name: solution.h
*
* Description: This header file defines the data structures and function
*              prototypes associated with PMG1S3 USBC Dock solution.
*
* Related Document: See README.md
*
******************************************************************************
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

#ifndef _SOLUTION_H_
#define _SOLUTION_H_

#include "cy_pdaltmode_defines.h"
#include "cy_pdstack_common.h"
#include "cy_app_vdm.h"
#include "cy_pdl.h"
#include "cybsp.h"
#if CY_APP_USB_ENABLE
#include "cy_usb_dev.h"
#include "cy_usb_dev_hid.h"
#endif /* CY_APP_USB_ENABLE */
#if CY_APP_SMART_POWER_ENABLE
#include "cy_app_smart_power.h"
#endif /* CY_APP_SMART_POWER_ENABLE */

/* Defines for voltage levels indicating power adapter size. */
#define PSU_ID_120W                     (1650u)
#define PSU_ID_135W                     (300u)
#define PSU_ID_150W                     (810u)
#define PSU_ID_180W                     (2480u)
#define PSU_ID_200W                     (2010u)
#define PSU_ID_230W                     (1010u)
#define PSU_ID_280W                     (1190u)

/* Define for error margin allowed for the voltage level. */
#define MARGIN                          (50u)

/* The adapter power watts definitions */
#define ADAPTER_POWER_120W              (120u)
#define ADAPTER_POWER_135W              (135u)
#define ADAPTER_POWER_150W              (150u)
#define ADAPTER_POWER_180W              (180u)
#define ADAPTER_POWER_200W              (200u)
#define ADAPTER_POWER_230W              (230u)
#define ADAPTER_POWER_280W              (280u)
#define ADAPTER_POWER_350W              (350u)

#if CY_APP_SMART_POWER_ENABLE
/*******************************************************************************
* Macros
*******************************************************************************/
/* SAR ADC can sample 8 channel inputs.
 * P2.6 is mapped to Channel-0 VPlus.
 * P3.4 is mapped to channel-1 VPlus.
 */
#define SYS_CURRENT_MEASURE_CHANNEL_NUM     (0u)
#define US_CURRENT_MEASURE_CHANNEL_NUM      (1u)

/* Range of value that can be measured using 12-bit SAR ADC in
 * Single Ended configuration.
 * Configured to measure 2 * VREF, hence Just multiply vrefMvValue by 2.
 * Range is available from:
 * Device Configurator -> Sampling -> Single-ended voltage range
 */
#define SINGLE_ENDED_RANGE                  (2u)

/* The SAR ADC configured for full [12-bit] resolution */
#define SAR_ADC_RESOULTION                  (0xFFFU)

#endif /* CY_APP_SMART_POWER_ENABLE */

/* LMP8640MKE-F/NOPB is used as CSA (current sense amplifier).
 * Gain = 50
 * Rsense = 5 mOhm
 * Vref = 0 V
 *
 * From LMP8640 datasheet:
 * Vout = (Iin * Rsense * GAIN)         ==> Iin = Vout / (Rsense * GAIN)
 * Iin = Vout / (5 * 10 ^ (-3) * 50)    ==> Iin = Vout * 4
 */
#define LMP8640_VOUT_TO_CURRENT_MULTIPLIER  (4u)

#if CY_APP_USB_ENABLE
/* USBDEV context variables */
extern cy_stc_usbfs_dev_drv_context_t  gl_usb_drvContext;
extern cy_stc_usb_dev_context_t        gl_usb_devContext;
#if CY_APP_USB_HID_INTF_ENABLE
extern cy_stc_usb_dev_hid_context_t    gl_usb_hidContext;
#endif /* CY_APP_USB_HID_INTF_ENABLE */
#endif /* #if CY_APP_USB_ENABLE */

/**
 * @brief Initialize Button Interrupt callback functions.
 *
 * These functions are called by pin interrups and handles different button states (press, relase and etc).
 *
 * @param None.
 *
 * @return None.
 */

#if EXTENDED_ALERT_EVENTS_SUPPORT || (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) || BUTTON_PRESS_FACTORY_RESET
void power_button_intr_cb (void);
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT || (CY_APP_USB_HID_INTF_ENABLE && CY_APP_USB_ENABLE) || BUTTON_PRESS_FACTORY_RESET */

#if EXTENDED_ALERT_EVENTS_SUPPORT
void lan_button_intr_cb (void);
#endif /* EXTENDED_ALERT_EVENTS_SUPPORT */

#if CY_APP_SMART_POWER_ENABLE
uint16_t measure_adapter_power(cy_stc_usbpd_context_t *context);

uint16_t measure_total_dock_current(cy_stc_app_smart_power_context_t *context);

uint16_t measure_us_current(cy_stc_app_smart_power_context_t *context);
#endif /* CY_APP_SMART_POWER_ENABLE */

void solution_init (cy_stc_pdstack_context_t* ptrPdStackContext);

void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data);

#if CY_APP_SMART_POWER_ENABLE
/**
 *  This function is used to  initialize the SAR ADC
 */
bool system_current_measure_init(void);
#endif /* CY_APP_SMART_POWER_ENABLE */
#endif /* _SOLUTION_H_ */
