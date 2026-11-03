/******************************************************************************
* File Name: anx7443.h
*
* Description: Analogix ANX7443 function prototypes and data structures.
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

#ifndef _ANX7443_H_
#define _ANX7443_H_

#include "cy_pdaltmode_defines.h"

/* 7-Bit Slave Address, 8-Bit Address for Chip top. */
#define ANX7443_I2C_SLAVE_ADDR_TOP          (0x20U >> 1)

/* Typedef for Application provided I2C Master write callback function. */
typedef bool (*mux_i2c_master_write)(uint8_t slave_addr,
                                     uint8_t *wr_buf,
                                     uint8_t write_count);

/**
 * @brief Handler for ANX Mux configuration based on Type-C Polarity.
 *
 * @param slave_addr 7-Bit slave address to which I2C Write to be performed.
 * @param cfg enum type cy_en_pdaltmode_mux_select_t.
 * @param polarity 0-for CC1 polarity, 1-for CC2 polarity.
 * @param i2c_write_cbk Pointer to application specific I2C Write.
 *
 * @return true if MUX configuration is success, false otherwise.
 */
bool anx7443_mux_cfg(uint8_t slave_addr,
                    cy_en_pdaltmode_mux_select_t cfg,
                    uint8_t polarity,
                    mux_i2c_master_write i2c_write_cbk);

#endif /* _ANX7443_H_ */
