/******************************************************************************
 * File Name: etag.h
 *
 * Description: This file defines data structures and function prototypes
 *              associated with the storage and retrieval of ETAG information.
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
