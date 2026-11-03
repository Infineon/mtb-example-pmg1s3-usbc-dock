/******************************************************************************
* File Name: pericom_pi3usb31532.c
*
* Description: Pericom USB31532 source file.
*
* Related Document: See README.md
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
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
