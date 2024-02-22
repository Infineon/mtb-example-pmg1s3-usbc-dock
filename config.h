/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the
*              PMG1S3 Dock Solution.
*
* Related Document: See README.md
*
********************************************************************************
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

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "cybsp.h"
#include "cy_app_buck_boost.h"
#include "cy_pdutils_sw_timer.h"
#include "cy_pdstack_timer_id.h"
#include "cy_app_timer_id.h"
#include "cy_usbpd_config_table.h"


/* MPS buck boost controller I2C address */
#define MP4247_REG_I2C_ADDR_P1                      (0x67u)
#define MP4247_REG_I2C_ADDR_P2                      (0x68u)

/* Ratio between output voltage and feedback voltage. This needs to be updated
 * based on the feedback circuit used in the hardware. */
#define MP4247_REG_FB_RATIO                         (15u)

/* Richtek buck boost controller's I2C addresses. */
#define RT6190_REG_I2C_ADDR_P1                      (0x2Cu)
#define RT6190_REG_I2C_ADDR_P2                      (0x2Du)

/*
 * Macro defines additional delay in milliseconds before the PD stack starts
 * sending SRC_CAP message. This may be required to work with some
 * non-compliant sink devices which require more start up time for PD.
 */
#define DELAY_SRC_CAP_START_MS                      (100u)

/*******************************************************************************
 * PSOURCE controls for Port 2.
 ******************************************************************************/
/* ADC instance used for VBus voltage measurement. */
#define APP_VBUS_POLL_ADC_ID                        (CY_USBPD_ADC_ID_0)

/* Types of PD ADC input sources for VBus voltage measurement. */
#define APP_VBUS_POLL_ADC_INPUT                     (CY_USBPD_ADC_INPUT_AMUX_B)

/*******************************************************************************
 * Application Timer IDs and Periods
 ******************************************************************************/
/* Timer used to run smart power algorithm */
#define SMART_POWER_DELAY_TIMER                     (CY_PDUTILS_TIMER_USER_START_ID)

/* Timer used to re-try the source capabilities change command */
#define SMART_POWER_SRC_CAP_CHANGE_RETRY_TIMER      (CY_PDUTILS_TIMER_USER_START_ID + 1u)

/* Timer ID for Power Button Debounce */
#define APP_POWER_BUTTON_TIMER_ID                   (CY_PDUTILS_TIMER_USER_START_ID + 2u)

/* Debounce time for power button in milliseconds. */
#define APP_POWER_BUTTON_TIMER_PERIOD               (20u)

/* Dock reset delay timer. This timer is used to delay initiating the soft
 * reset, so that host tool will receive the soft reset notification before DMC
 * disconnects from host.
 */
#define APP_DOCK_RESET_DELAY_TIMER                  (CY_PDUTILS_TIMER_USER_START_ID + 3u)

/* Timer to check the status of SPI Flash Erase or Write operation. */
#define APP_SPI_EEPROM_OPR_DELAY_TIMER              (CY_PDUTILS_TIMER_USER_START_ID + 4u)

/* Timer used to trigger factory reset. */
#define APP_BUTTON_FACTORY_RESET_TIMER              (CY_PDUTILS_TIMER_USER_START_ID + 5u)

/* Time in milliseconds to decide if factory reset button is pressed. */
#define APP_BUTTON_FACTORY_RESET_TIMER_PERIOD       (20000u)

/* Timer used for implementing blinking and breathing modes in LED control. */
#define APP_LED_CTRL_TIMER                          (CY_PDUTILS_TIMER_USER_START_ID + 6u)

/* Timer ID for LAN Button Debounce */
#define APP_LAN_BUTTON_TIMER_ID                     (CY_PDUTILS_TIMER_USER_START_ID + 7u)

/* Debounce time for LAN button in milliseconds. */
#define APP_LAN_BUTTON_TIMER_PERIOD                 (20u)

/* Timer ID for the flash log module */
#define FLASH_LOG_TIMER_ID                          (CY_PDUTILS_TIMER_USER_START_ID + 8u)

/*
 * Macro defines debounce time period in milliseconds from flash log event to
 * start of flash log data write, if VDDD is stable.
 */
#define FLASH_LOG_WRITE_DEFER_TIME                  (100u)

/* HX3 Hub, VBUS_US GPIO Control Timer on Connect event. */
#define APP_VBUS_DET_TIMER                          (CY_PDUTILS_TIMER_USER_START_ID + 9u)

/* Delay VBUS_US of HX3 Hub for 700ms. */
#define CY_APP_VBUS_DET_TIMER_PERIOD                (700u)

/* Timer to wait for the HPI response from CCGx device. */
#define CCGX_RESPONSE_WAIT_TIMER_ID                 (CY_PDUTILS_TIMER_USER_START_ID + 10u)

/* CCGx HPI response wait timer period. */
#define CCGX_RESPONSE_WAIT_TIMER_PERIOD             (1000u)

/* Timer to wait for HX3 boot up. */
#define HX3_BOOT_WAIT_TIMER_ID                      (CY_PDUTILS_TIMER_USER_START_ID + 11u)

/* HX3 boot up timer period. */
#define HX3_BOOT_WAIT_TIMER_PERIOD                  (1000u)

/******************************************************************************/

/******************************** DMC *****************************************/
/* ID to represent current KEY_ID used for representing  the public key
 * present inside DMC FW. It also represents the corresponding private key
 * used for signing the FWCT.
 *  Value = 0: No valid key
 *  Value = 1 - 0xFF: Unique ID corresponding to the key pair used for signing
 */
#define KEY_ID_EMBEDDED                             (1u)

#if CY_APP_DMC_ENABLE
/* Update the dock firmware with factory image on 20 second button press */
#define BUTTON_PRESS_FACTORY_RESET                  (1u)
#endif /* CY_APP_DMC_ENABLE */

/******************************** Composite version ***************************/
/* Major composite version of the composite dock firmware image */
#define COMPOSITE_VERSION_MAJOR_1                   (1)

/* Major composite version of the composite dock firmware image */
#define COMPOSITE_VERSION_MAJOR_0                   (0)

/* Minor composite version of the composite dock firmware image */
#define COMPOSITE_VERSION_MINOR_1                   (0)

/* Minor composite version of the composite dock firmware image */
#define COMPOSITE_VERSION_MINOR_0                   (0)

/* Composite firmware version value for the composite dock firmware image.
 *
 *  Composite firmware stack version value. This is a 4 byte value with the following format:
 *  Byte 1-0: Minor Version
 *  Byte 3-2: Major Version
 */
#define COMPOSITE_FW_VERSION                                                   \
    ((COMPOSITE_VERSION_MAJOR_1 << 24) | (COMPOSITE_VERSION_MAJOR_0 << 16) |   \
     (COMPOSITE_VERSION_MINOR_1 << 8) | (COMPOSITE_VERSION_MINOR_0))

/******************************** SPI master *********************************/
/* Enable this macro if the SPI CS line is manually controlled using GPIO. */
#define SPI_CS_MANUAL                                (1u)

/******************************* SPI Flash specific Macros ********************/
/* SPI flash factory package address. */
#define FACTORY_PACKAGE_SPI_ADDRESS                  (0x340000)

/* SPI flash primary package address */
#define PRIMARY_PACKAGE_SPI_ADDRESS                  (0x40000)

/************************* Power throttling related Macros ********************/

#if CY_APP_SMART_POWER_ENABLE
/* Default adapter voltage in milli volts*/
#define DEFAULT_ADAPTER_VOLTAGE                      (19500u)
#endif /* CY_APP_SMART_POWER_ENABLE */

/******************************** other *********************************/
#define CySoftwareReset()                            NVIC_SystemReset()

/* When set to 1, UFP, DFP Mux will be configured.
 * DP Controller and USB Hub control will be controlled on Upstream
 * connect and disconnect events.
 */
#define MUX_CHANGES_ENABLE                           (1u)

/* Number of GPIO ports in PMG1S3. */
#define NUM_OF_GPIO_PORTS                            (9u)

#endif /* _CONFIG_H_ */

/* End of file [] */
