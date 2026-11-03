/******************************************************************************
* File Name: dmc_flashing.h
*
* Description: DMC firmware update interface header file.
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
