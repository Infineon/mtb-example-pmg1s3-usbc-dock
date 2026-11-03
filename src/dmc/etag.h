/******************************************************************************
 * File Name: etag.h
 *
 * Description: This file defines data structures and function prototypes
 *              associated with the storage and retrieval of ETAG information.
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

#ifndef _ETAG_H_
#define _ETAG_H_

#include "cy_app_status.h"

#if ETAG_SUPPORT_ENABLE

/* Actual size of ETAG Data in bytes*/
#define ETAG_DATA_SIZE                             10u

/* Total size allocated for ETAG. This includes ETAG_DATA, ETAG_VALID and Factory_update_done fields */
#define ETAG_INFORMATION_SIZE                      ETAG_DATA_SIZE + 2u

/* ETAG_VALID field for a valid ETAG in flash */
#define ETAG_VALID                                 0x01u

/* Offset of ETAG_VALID field inside the flash page that holds SPI metadata */
#define ETAG_VALID_OFFSET                          0x01u

/*******************************************************************************
* Function name: read_etag_information
****************************************************************************//**
*
* This function is used to read the ETAG information from SPI metadata section of
* SPI flash.
*
* \param buffer
* The buffer that will be filled with the ETAG information.
*
* \return
* CY_APP_STAT_SUCCESS if the operation was successful
* CY_APP_STAT_FAILURE otherwise
*
*******************************************************************************/
cy_en_app_status_t read_etag_information(uint8_t *buffer);

/*******************************************************************************
* Function name: write_etag_information
****************************************************************************//**
*
* This function is used to write the ETAG information to SPI metadata section of
* SPI flash.
*
* \param report
* The output report received from USB host which contains the ETAG information.
*
* \return
* CY_APP_STAT_SUCCESS if the ETAG programming was successful
* CY_APP_STAT_INVALID_COMMAND if ETAG programming was already done
* CY_APP_STAT_FAILURE if the ETAG programming failed
*
*******************************************************************************/
cy_en_app_status_t write_etag_information(uint8_t *report);

#endif /* ETAG_SUPPORT_ENABLE */

#endif /* SRC_DMC_ETAG_H_ */
