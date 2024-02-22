/******************************************************************************
* File Name: dmc_flashing.h
*
* Description: DMC firmware update interface header file.
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

#ifndef _DMC_FLASHING_H_
#define _DMC_FLASHING_H_

#include <stdint.h>
#include "cy_app_dmc_common.h"

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/

/**
 * @brief Structure based layout to access base firmware version
 * 
 * The base firmware version is stored in a 32-bit value as given below
 * major_version[31:28], minor_version[27:24]
 * patch_version [23:16], build_num [15:0]
 */
typedef struct
{
    uint16_t build_num;
    uint8_t  patch_version;
    uint8_t  minor_version:4;
    uint8_t  major_version:4;

}base_fw_version_t;


/**
 * @brief Structure based layout to access version defined in app_version.h
 * 
 * The application version is stored in a 32-bit value as given below
 * major_version[31:28], minor_version[27:24]
 * ckt_num [23:16], type_string [15:0]
 */
typedef struct
{
    uint16_t type_string;       /**< This indicates the application type */
    uint8_t  ckt_num;           /**< Member to access 8-bit circular version number */
    uint8_t  minor_version:4;   /**< Member to access 4-bit application minor version */
    uint8_t  major_version:4;   /**< Member to access 4-bit application major version */

}app_fw_version_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/
/**
 * @brief Gets the pointer to the structure of function pointers related to 
 *        various DMC operation.
 * @param None
 *
 * @return Pointer to the cy_stc_app_dmc_update_opern_t structure.
 */
const cy_stc_app_dmc_update_opern_t *get_dmc_operation(void);

#endif  /* _DMC_FLASHING_H_ */

/* [] END OF FILE */
