/******************************************************************************
* File Name: anx7443.h
*
* Description: Analogix ANX7443 function prototypes and data structures.
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
