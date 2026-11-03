/******************************************************************************
* File Name: ccgx_ctrl.h
*
* Description: CCG firmware update interface header file.
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

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#ifndef _CCGX_CTRL_H_
#define _CCGX_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_app_status.h"
#include "config.h"
#include "cy_app_dmc_common.h"
#include "dmc_solution.h"
#include "cy_hpi_master.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/


/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/


 /*******************************************************************************
 * Function Declarations
 ******************************************************************************/

const cy_stc_app_dmc_update_opern_t* get_ccgx_operation(void);

bool ccg_handle_hpi_event(void *context, cy_hpi_master_event_t *event);

#endif /* _CCGX_CTRL_H_ */
