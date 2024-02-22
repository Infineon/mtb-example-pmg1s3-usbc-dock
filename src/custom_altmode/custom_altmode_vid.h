/******************************************************************************
 * File Name: custom_altmode_vid.h
 *
 * Description: This file implement Generic/Custom ALT mode for the PMG1S3 Dock
 *              Solution.
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
