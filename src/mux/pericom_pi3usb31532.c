/******************************************************************************
* File Name: pericom_pi3usb31532.c
*
* Description: Pericom USB31532 source file.
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

#include "pericom_pi3usb31532.h"

/* DFP (DS1) Port Mux configuration based on cfg and Type-C Polarity. */
bool pericom_mux_cfg (uint8_t slave_addr, 
        cy_en_pdaltmode_mux_select_t cfg, 
        uint8_t polarity, 
        mux_i2c_master_write i2c_write_cbk
        )
{
    uint8_t switch_val = PI3_CONF_SWITCH_VAL_MAX;

    bool retval = 0;

    if(i2c_write_cbk == NULL)
    {
        return retval;
    }

    switch(cfg)
    {
        case CY_PDALTMODE_MUX_CONFIG_ISOLATE:
            switch_val = PI3_CONF_SWITCH_OPEN_POWER_DOWN;
        break;
        case CY_PDALTMODE_MUX_CONFIG_SS_ONLY:
            switch_val = PI3_CONF_USB_3_1_GEN1_2 | polarity;
        break;
        case CY_PDALTMODE_MUX_CONFIG_DP_2_LANE:
            switch_val = PI3_CONF_DP_2_LANE | polarity;
        break;
        case CY_PDALTMODE_MUX_CONFIG_DP_4_LANE:
            switch_val = PI3_CONF_DP_4_LANE | polarity;
        break;
        default:
            return retval;
        break;
    }

    uint8_t reg_addr = 2;
    uint8_t wr_buf[] = {reg_addr, switch_val};

    retval = (*i2c_write_cbk)(slave_addr, wr_buf, sizeof(wr_buf));

    return retval;
}
