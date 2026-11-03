/******************************************************************************
 * File Name: custom_altmode_vid.h
 *
 * Description: This file implement Generic/Custom ALT mode for the PMG1S3 Dock
 *              Solution.
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

#ifndef _CUSTOM_ALTMODE_VID_H_
#define _CUSTOM_ALTMODE_VID_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/

#include "cy_pdaltmode_mngr.h"
#include "cy_pdaltmode_defines.h"
#include "cy_usbpd_defines.h"
#include "config.h"

/*******************************************************************************
 * MACRO Definition
 ******************************************************************************/

/**
  @brief Insert SVID value here. Customers shall replace this value with
 *       their required VID. This value shall not be used for production.
 */
#define CUSTOM_ALT_MODE_VID                 (0x04B4U)
/**
  @brief custom ALT mode ID.
 */
#define CUSTOM_ALT_MODE_ID                  (1U)
/**
   @bried Custom ALT Mode ID Mask.
 */
#define CUSTOM_ALT_MODE_ID_MASK             (1U)

/**
  @brief Represents maximum number of VDO which custom ALT mode uses to provide handling.
 */
#define MAX_CUSTOM_VDO_NUMB                 (1U)

#define CUSTOM_VDO_IDX                      (0U)

#define PRT_TYPE_UFP                        (0U)

#define NO_DATA                             (0U)

#define SOMEVDO                             (0U)

/*******************************************************************************
 * Enumerated Data Definition
 ******************************************************************************/
/**
  @typedef custom_state_t
  @brief This enumeration holds all possible custom states.
 */
typedef enum
{
    CUSTOM_AM_STATE_SOMESTATE = 1,
    CUSTOM_AM_STATE_IDLE = 0,                 /**< Idle state. */
    CUSTOM_AM_STATE_ENTER = 4,                /**< Enter mode state. */
    CUSTOM_AM_STATE_ATT = 6,                  /**< Attention state. */
    CUSTOM_AM_STATE_EXIT = 5,                 /**< Exit mode state. */
    /* Add here special custom ALT mode command if needed */

} custom_state_t;

typedef struct
{
    cy_stc_pdaltmode_mngr_info_t info;
    custom_state_t state;
    cy_pd_pd_do_t vdo[MAX_CUSTOM_VDO_NUMB];
    uint8_t max_sop_supp;
#if CUSTOM_ALT_MODE_DFP_SUPP
    const cy_stc_pdaltmode_atch_tgt_info_t* tgt_info_ptr;
#endif /* CUSTOM_ALT_MODE_DFP_SUPP */
} custom_am_status_t;

/**
 * @brief This function analyse's discovery information to find out
 * if further custom alternative mode processing is allowed.
 *
 * @param context index the function is performed for.
 * @param reg_info Pointer to structure which holds ALT mode register info.
 *
 * @return Pointer to custom alternative mode command structure if analyse's passed
 * successful. In other case function returns NULL pointer
 */
cy_stc_pdaltmode_mngr_info_t* custom_alt_mode_reg_modes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info); /* insert SVID name */

#endif /* _CUSTOM_ALTMODE_VID_H_ */

/* [] END OF FILE */
