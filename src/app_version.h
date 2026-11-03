/******************************************************************************
 * File Name: app_version.h
 *
 * Description: Header file defining DMC app firmware version.
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

