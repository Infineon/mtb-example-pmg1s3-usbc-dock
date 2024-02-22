/******************************************************************************
* File Name: pericom_pi3usb31532.h
*
* Description: Pericom USB31532 Header file.
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

#ifndef _PI3USB31532_H__
#define _PI3USB31532_H__

#include "cy_pdaltmode_defines.h"

/* Fixed part of I2C slave address for Pericom PI3USB31532 Mux. */
#define PERICOM_FIXED_ADDRESS       (0xA8u)

/* Bit-2 is A1 and Bit-1 is A0 which are being strapped to pull-down in Hardware. */
#define PERICOM_CFG_ADDRESS         (0x00u)

/* Effective Slave address. The i2c API's expects slave address to be 7-bit value
 * hence the left shift by 1.
 */
#define USB31532_SLAVE_ADDR         ((PERICOM_FIXED_ADDRESS | PERICOM_CFG_ADDRESS) >> 1)

/* Number of retries to be performed when there is a I2C failure. */
#define PERICOM_I2C_RETRY_COUNT     (2u)

/**
 * @brief Pericom USB31532 Type-C Mux DFP enumeration.
 *
 * The enumeration for the DFP Mux Config codes.
 */
typedef enum
{
    PI3_CONF_SWITCH_OPEN_POWER_DOWN,    /**< Open and power-down. */
    PI3_CONF_SWITCH_OPEN,               /**< Open and no power-down. */
    PI3_CONF_DP_4_LANE,                 /**< 4-Lane DP. */
    PI3_CONF_DP_4_LANE_FLIP,            /**< 4-Lane DP, flip. */
    PI3_CONF_USB_3_1_GEN1_2,            /**< USB3.1 Gen 1 / Gen 2  */
    PI3_CONF_USB_3_1_GEN1_2_FLIP,       /**< USB3.1 Gen 1 / Gen 2, flip. */
    PI3_CONF_DP_2_LANE,                 /**< USB3.1 Gen 1 / Gen 2 + 2 Lane DP */
    PI3_CONF_DP_2_LANE_FLIP,            /**< USB3.1 Gen 1 / Gen 2 + 2 Lane DP, flip */
    PI3_CONF_SWITCH_VAL_MAX             /**< Max allowed values. */
}pi3usb31532_src_cfg_t;

/* Typedef for Application provided I2C Master write callback function. */
typedef bool (*mux_i2c_master_write)(uint8_t slave_addr, uint8_t *wr_buf, uint8_t write_count);

/* DFP (DS1) Port Mux configuration based on cfg and Type-C Polarity. */
bool pericom_mux_cfg(uint8_t slave_addr, cy_en_pdaltmode_mux_select_t cfg, uint8_t polarity, mux_i2c_master_write i2c_write_cbk);

#endif /* _PI3USB31532_H__ */
