/******************************************************************************
 * File Name: app_version.h
 *
 * Description: Header file defining DMC app firmware version.
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

#ifndef __APP_VERSION_H__
#define __APP_VERSION_H__

#include "config.h"

/******************************************************************************
 * Constant definitions.
 *****************************************************************************/
/*! Signature used while creating the DMC Application Version. 
*/
#define APP_TYPE_DMC                   ( ('d' << 8) | 'm')        /* "dm" */       

#define APP_TYPE_STRING                 (APP_TYPE_DMC)

/*! Circular version number used from BITS[23:16] */    
#define APP_EXT_CIR_NUM                 (0)           /* 8 bit value */

/*! 4-Bit value indicating the minor version BITS[27:24] */    
#define APP_MINOR_VERSION               (0)            /* 4 bit value */

/*! 4-Bit value indicating the minor version BITS[31:28] */    
#define APP_MAJOR_VERSION               (0)            /* 4 bit value */

/*! Generating final DMC Application Version using APP_TYPE_STRING, APP_EXT_CIR_NUM 
 *  APP_MINOR_VERSION & APP_MAJOR_VERSION.
 */    
#define APP_VERSION                                             \
    ((APP_TYPE_STRING) | (APP_EXT_CIR_NUM << 16) |              \
     (APP_MINOR_VERSION << 24) | (APP_MAJOR_VERSION << 28))

#endif /* __APP_VERSION_H__ */

/* [] END OF FILE */

