/******************************************************************************
 * File Name: custom_altmode_vid.c
 *
 * Description: This file implements Generic/Custom ALT mode for the PMG1S3
 * Dock Solution.
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

#include "custom_altmode_vid.h"

#if ((CUSTOM_ALT_MODE_DFP_SUPP) || (CUSTOM_ALT_MODE_UFP_SUPP))

custom_am_status_t customAMStatus[NO_OF_TYPEC_PORTS];

#if CUSTOM_ALT_MODE_DFP_SUPP
/* Init's custom DFP ALT mode */
static void custom_alt_mode_init(uint8_t port);
/* Main custom source handling functions */
static void custom_alt_mode_dfp_run(void *context);
/* Checks if cable supports custom ALT mode handling */
static bool custom_alt_mode_is_cable_capable(uint8_t port, const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);
/* Check if cable contains custom related SVID */
static bool custom_alt_mode_is_cable_support_custom_alt_mode_svid(const cy_stc_pdaltmode_atch_tgt_info_t *atch_tgt_info);
/* De-inits custom ALT mode */
static void custom_alt_mode_exit(uint8_t port);
/* Composes VDM for sending by ALT mode manager */
static void custom_alt_mode_send_cmd(uint8_t port);
/* Returns pointer to custom ALT mode VDO buffer */
static cy_pd_pd_do_t* custom_alt_mode_get_vdo(uint8_t port);
/* Evaluates the received attention and returns the custom state */
static custom_state_t custom_alt_mode_eval_attention(uint8_t port);
#endif /* CUSTOM_ALT_MODE_DFP_SUPP */

#if CUSTOM_ALT_MODE_UFP_SUPP
/* Main custom UFP ALT mode handling function */
static void custom_alt_mode_ufp_run(void *context);
#endif /* CUSTOM_ALT_MODE_UFP_SUPP */

/* Stores given VDO in custom ALT mode VDO buffer for sending */
void custom_alt_mode_assign_vdo(uint8_t port, uint32_t vdo);
/* Evaluates xx ALT mode command received from application */
static bool custom_alt_mode_eval_app_cmd(uint8_t port, cy_stc_pdaltmode_alt_mode_evt_t cmd_data);
/* Returns pointer to ALT mode info structure */
static cy_stc_pdaltmode_mngr_info_t* custom_alt_mode_info(uint8_t port);

/************************** Function definitions *****************************/
cy_stc_pdaltmode_mngr_info_t* custom_alt_mode_reg_modes(void *context, cy_stc_pdaltmode_alt_mode_reg_info_t* reg_info) /* insert SVID name */
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = ((cy_stc_pdaltmode_context_t *)context);

    uint8_t port = ptrAltModeContext->pdStackContext->port;

#if CUSTOM_ALT_MODE_UFP_SUPP
    /* If custom UFP alternate mode */
    if (reg_info->dataRole == PRT_TYPE_UFP)
    {
        /* Check if received ALT mode ID is custom ALT mode ID.
         * Each ALT mode should have bit fields in DISC MODE VDO response which
         * indicates Alternate Mode ID (see custom ALT mode related specification for the details)
         * For example Thunderbolt ALT mode ID is 0x1 and occupies first 16 bits
         * Intel DISC MODE VDO response */

        if (reg_info->svidVdo.val & CUSTOM_ALT_MODE_ID_MASK)
        {
            /* Insert function to reset internal variable if needed */

            /* Save ALT mode ID */
            reg_info->altModeId = CUSTOM_ALT_MODE_ID;
            /* Save main custom UFP ALT mode state machine structure */
            custom_alt_mode_info(port)->cbk = custom_alt_mode_ufp_run;
            /* Save custom ALT mode related custom SVID in the ALT mode info structure*/
            custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.svid = CUSTOM_ALT_MODE_VID; /* insert SVID name */
            /* Save pointer to the custom ALT mode SOP VDO buffer */
            custom_alt_mode_info(port)->vdo[CY_PD_SOP] = customAMStatus[port].vdo;
            /* Save maximum number VDOs which could be handled by ALT mode */
            custom_alt_mode_info(port)->vdoMaxNumb = MAX_CUSTOM_VDO_NUMB;
            /* Save function to evaluate received Application command  */
            custom_alt_mode_info(port)->evalAppCmd = (cy_pdaltmode_app_cbk_t)custom_alt_mode_eval_app_cmd;

            /* Insert internal processing if needed */

            /* return custom ALT mode info structure pointer */
            return &(customAMStatus[port].info);
        }

        /* If received ALT mode ID not related to custom ALT mode then reject */
        reg_info->altModeId = CY_PDALTMODE_MNGR_MODE_NOT_SUPPORTED;

        return NULL;
    }
#endif /* CUSTOM_ALT_MODE_UFP_SUPP */

#if CUSTOM_ALT_MODE_DFP_SUPP

    /* Check if received ALT mode ID is custom ALT mode ID.
     * Each ALT mode should have bit fields in DISC MODE VDO response which
     * indicates Alternate Mode ID (see custom ALT mode related specification for the details)
     * For example Thunderbolt ALT mode ID is 0x1 and occupies first 16 bits
     * Intel DISC MODE VDO response */

    if (reg_info->svidVdo.val & CUSTOM_ALT_MODE_ID_MASK)
    {
        /* During Discover Mode stage ALT modes manager could run Discover mode process
         * as for UFP target as for cable.
         * To inform custom DFP alternate mode what kind of the Discover mode response
         * (UFP or cable) is received reg_info struct uses atch_type variable.
         * ATCH_TGT - for the SOP Disc mode response; CABLE - for SOP'/SOP". */
        /* UFP Disc Mode analysis */
        if (reg_info->atchType == CY_PDALTMODE_MNGR_ATCH_TGT)
        {
            /* Save ALT mode ID */
            reg_info->altModeId = CUSTOM_ALT_MODE_ID;

            /* Reset pdaltmode mnrg info struct */
            Cy_PdAltMode_Mngr_ResetAltModeInfo(&(customAMStatus[port].info));

            /* This function should check if cable capabilities can ensure
             * properly custom alternate mode processing. If can then function should
             * return true; false - in the other case (Check atch_tgt_info->cbl_vdo).
             * This function could be ignored when cable check not needed */
            if (custom_alt_mode_is_cable_capable(port, reg_info->atchTgtInfo) == false)
            {
                /* If cable capabilities don't meet custom ALT mode then set
                 * appropriate application event and return NULL */
                reg_info->appEvt = CY_PDALTMODE_EVT_CBL_NOT_SUPP_ALT_MODE;
                return NULL;
            }
            /* If alternate modes potentially could be supported by the cable then
             * we need to run cable Disc mode process for current SVID.
             * This function should check if cable support custom ALT mode related SVID
             * (find related SVID among atch_tgt_info->cblSvid[] array)
             * If SVID is found then function should return true; false - in the other case.
             * Ignore this function if we not interested in cable Disc mode process
             * for current SVID.
             * NOTE: you MUST set cbl_sop_flag to SOP_INVALID if cable Disc mode
             * not needed and this function is ignored*/
            /* Check if cable has custom ALT mode related SVID */
            if (custom_alt_mode_is_cable_support_custom_alt_mode_svid(reg_info->atchTgtInfo) == false)
            {
                /* SOP' disc SVID is not needed */
                reg_info->cblSopFlag = CY_PD_SOP_INVALID;
                /* You can put your additional extra code here if needed when
                 * not contains appropriate SVID cable and notify App with appropriate
                 * event */

                /* If we don't care to cable SVID support then set Application event */
                reg_info->appEvt = CY_PDALTMODE_EVT_ALT_MODE_SUPP;
            }
            else
            {
                /* If SOP' Disc mode needed notify ALT modes manager with cbl_sop_flag variable*/
                reg_info->cblSopFlag = CY_PD_SOP_PRIME;
            }
            /* If all previous processing passed successfully and custom DFP ALT mode
             * could be run then set/save custom related information */
            /* Save cable, device/AMA info pointer; It could be used in future */
            customAMStatus[port].tgt_info_ptr = reg_info->atchTgtInfo;
            /* Set maximum SOP type supported variable to SOP. It means that
             * when custom alternate mode will send some command this command will
             * be sent to UFP only (not send to cable).
             * This done because custom ALT mode didn't get cable Disc mode response
             * yet and can't actually know could SOP'/SOP" command be used */
            customAMStatus[port].max_sop_supp = CY_PD_SOP;

            /* Save maximum number VDOs which could be handled by alt mode */
            custom_alt_mode_info(port)->vdoMaxNumb = MAX_CUSTOM_VDO_NUMB;

            /* NOTE: Place custom ALT mode DISC mode (SOP) related code here if needed */

            /* INIT custom ALT mode.
             * Set custom ALT mode to the INIT state and run main DFP ALT mode state
             * machine function. */
            custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_INIT;
            custom_alt_mode_dfp_run(context);

            /* Return custom ALT mode info structure pointer */
            return &customAMStatus[port].info;
        }
        /* This section is responsible for the cable Disc Mode success/failed
         * response analysis. All SOP'/SOP" Disc Mode processing should be placed here.
         * Ignore this section if custom ALT mode don't provide SOP'/SOP" messages handling */
        else if (reg_info->atchType == CY_PDALTMODE_MNGR_CABLE)
        {
            /* Check if SOP'/SOP" response received */
            if (reg_info->cblSopFlag != CY_PD_SOP_INVALID)
            {
                /* NOTE: Place custom ALT mode DISC mode cable response analysis
                 * related code here if needed */

                /* If SOP' Disc mode response received and custom ALT mode commands
                 * should be handled by ALT modes manager also for SOP then we need
                 * to set SOP' state in the SEND state (For example to send SOP'
                 * Enter mode command) */

                /* Set that custom ALT modes should handle as SOP as SOP' command */
                customAMStatus[port].max_sop_supp = CY_PD_SOP_PRIME;
                /* Allow send SOP' packets */
                custom_alt_mode_info(port)->sopState[CY_PD_SOP_PRIME] = CY_PDALTMODE_STATE_SEND;

                 /* If SOP" Disc mode needed notify ALT modes manager with
                  * cbl_sop_flag variable.
                  * In other case set cbl_sop_flag to SOP_INVALID */
                reg_info->cblSopFlag = CY_PD_SOP_DPRIME;

                /* If SOP'' controller present allow send SOP'' VDM.
                 * Check cable VDO for this */
                if ((customAMStatus[port].tgt_info_ptr->cblVdo->val != NO_DATA) &&
                        (customAMStatus[port].tgt_info_ptr->cblVdo->std_cbl_vdo.sopDp == 1))
                {
                    /* Set that custom ALT modes should handle SOP/SOP'/SOP" command */
                    custom_alt_mode_info(port)->sopState[CY_PD_SOP_DPRIME] = CY_PDALTMODE_STATE_SEND;
                    customAMStatus[port].max_sop_supp = CY_PD_SOP_DPRIME;
                }

                /* Set supported application event if custom ALT mode Disc mode process
                 * finished and ALT mode could be run */
                reg_info->appEvt = CY_PDALTMODE_EVT_ALT_MODE_SUPP;

                /* INIT custom ALT mode.
                 * Set custom ALT mode to the INIT state and run main DFP ALT mode state
                 * machine function. */
                custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_INIT;
                custom_alt_mode_dfp_run(context);
            }
            /* This section handles failed SOP'/SOP" Disc mode commands */
            else
            {
                /* NOTE: Place custom ALT mode DISC mode cable failed response
                 * analysis related code here if needed */

                /* If custom alternate mode can't be ran in some reason
                 * then set ALT mode state to DISABLE and set appropriate APP
                 * event if needed */

                /* Disable custom functionality */
                custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_DISABLE;
                return NULL;
            }

            /* Return custom ALT mode info structure pointer */
            return (&customAMStatus[port].info);
        }
    }
#endif /* CUSTOM_ALT_MODE_DFP_SUPP */

    return NULL;
}

#if CUSTOM_ALT_MODE_DFP_SUPP
static void custom_alt_mode_dfp_run(void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext  = ((cy_stc_pdaltmode_context_t *)context);

    uint8_t port = ptrAltModeContext ->pdStackContext->port;
    /* Get current ALT mode state which is detected by ALT modes manager */
    cy_en_pdaltmode_state_t custom_mode_state = custom_alt_mode_info(port)->modeState;

    /*
     * If ALT mode handles both Structure and Unstructured VDMs then two different
     * switch cases (handlers) should be provided in dependence of which VDM (S/U) is handled
     * Also in this case VDM Type field should be set as S/UVDM each time before sending VDM
     * In case if ALT mode uses only SVDMs then condition below to check VDM type not needed - in
     * in this VDM type field set as SVDM before sending VDM to ALT mode manager
     */
    switch (custom_mode_state)
    {
        /* INIT state called from registration function */
        case CY_PDALTMODE_STATE_INIT:
            /* Init's custom ALT mode related variables */
            custom_alt_mode_init(port);
            /* Prepare enter mode command for sending */
            custom_alt_mode_send_cmd(port);
            break;
        /* This section is responsible for handling of the sent VDMs
         * Enter/Exit command are common for all ALT modes. If custom ALT mode
         * operates with its own special commands then they processing should be
         * added here.
         * NOTE that this states should be defined in custom ALT mode *.h file and
         * their numerical value be in accordance with command numeric value */
        case CY_PDALTMODE_STATE_WAIT_FOR_RESP:
            /* Get custom ALT mode state from the received VDM header */
            customAMStatus[port].state = (custom_state_t)custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.cmd;
            /* Set ALT mode state as IDLE by default. If ALT mode state should be
             * changed its should be done in case handler */
            custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_IDLE;
            /* Process all possible custom ALT mode states */
            switch (customAMStatus[port].state)
            {
                case CUSTOM_AM_STATE_ENTER:

                    /* Place here analysis code part if needed, else ignore and
                     * set custom ALT mode state to IDLE */
                    customAMStatus[port].state = CUSTOM_AM_STATE_IDLE;
                    return;

                case CUSTOM_AM_STATE_EXIT:
                    custom_alt_mode_exit(port);
                    return;

                /* In case if after received VDM response custom DFP ALT mode need
                 * to send next specific command then use below format */
                case CUSTOM_AM_STATE_SOMESTATE:

                    /* Place here analysis code part if needed */

                    /* If custom ALT mode need to send next VDM after received VDM analysis
                     * then you need to assign appropriate VDOs for sending to
                     * ALT mode info structure, change custom ALT mode state to the
                     * state which corresponds to specific command which should
                     * be sent and use custom_alt_mode_send_cmd() function to form VDM.
                     * In case if sending next command vot needed then set custom
                     * ALT mode state to CUSTOM_AM_STATE_IDLE */

                    custom_alt_mode_assign_vdo(port, SOMEVDO);
                    customAMStatus[port].state = CUSTOM_AM_STATE_SOMESTATE;
                    custom_alt_mode_send_cmd(port);
                    return;
                default:
                    return;
            }

            break;

        /* This case handles received commands from UFP (Attention command for USB-PD revision 2.0)*/
        case CY_PDALTMODE_STATE_IDLE:
            /* Verify if input message is Attention */
            if ((custom_state_t)custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.cmd == CUSTOM_AM_STATE_ATT)
            {
                /* Evaluate received ATT message */
                customAMStatus[port].state = custom_alt_mode_eval_attention(port);
                /* If custom ALT mode should react to Attention message by sending any VDM
                 * then use custom_alt_mode_send_cmd() function to form custom ALT mode VDM.
                 * In the other case - ignore this */
                custom_alt_mode_send_cmd(port);
            }
            break;

        /* This section is responsible for handling of the failed VDM */
        case CY_PDALTMODE_STATE_FAIL:
            /* Put custom ALT modes processing in dependance of custom ALT mode
             * behavior predicted by custom ALT mode specification. Place custom ALT mode
             * specific commands case handling here if needed */
            switch (customAMStatus[port].state)
            {
                case CUSTOM_AM_STATE_ENTER:
                    /* If Enter mode command fails then Exit from ALT mode */
                    custom_alt_mode_exit(port);
                    break;

                case CUSTOM_AM_STATE_EXIT:
                    /* If Exit mode command fails then Exit from ALT mode */
                    custom_alt_mode_exit(port);
                    break;

                default:
                    break;
            }
            break;

        /* This case uses to send exit custom ALT mode if ALT modes manager requires */
        case CY_PDALTMODE_STATE_EXIT:
            customAMStatus[port].state = CUSTOM_AM_STATE_EXIT;
            custom_alt_mode_send_cmd(port);
            break;

        default:
            break;
    }
}

static custom_state_t custom_alt_mode_eval_attention(uint8_t port)
{
    /* Set return state to Idle */
    custom_state_t stat = CUSTOM_AM_STATE_IDLE;

    /* Get pointer to the received Attention VDM */
    cy_pd_pd_do_t* alt = custom_alt_mode_get_vdo(port);

    (void)alt;

    /* Place evaluation of the received attention VDM here */

    return stat;
}

static void custom_alt_mode_init(uint8_t port)
{
    cy_en_pd_sop_t idx;

    /* Set SOP (SOP'/SOP" if needed) VDM to the send */
    for (idx = 0; idx <= customAMStatus[port].max_sop_supp; idx++)
    {
        custom_alt_mode_info(port)->sopState[idx] = CY_PDALTMODE_STATE_SEND;
        custom_alt_mode_info(port)->vdo[idx] = customAMStatus[port].vdo;
    }

    /* Assign enter mode VDO if needed. Else - ignore */
    custom_alt_mode_assign_vdo(port, CY_PDALTMODE_MNGR_NONE_VDO);
    /* Save DFP custom ALT mode state machine function in the info structure */
    custom_alt_mode_info(port)->cbk = custom_alt_mode_dfp_run;
    /* Save APP command handler function in the info structure */
    custom_alt_mode_info(port)->evalAppCmd = (cy_pdaltmode_app_cbk_t)custom_alt_mode_eval_app_cmd;
    /* Set custom state as enter */
    customAMStatus[port].state = CUSTOM_AM_STATE_ENTER;
    /*
     * If during Enter and Exit alternate mode MUX should be set to isolate
     * then set_mux_isolate flag should be set as true. Else - igrore
     */
    /* xxx_info(port)->setMuxIsolate = true; */

    /*
     * If alternate mode supports UVDMs then uvdm_supp should be set as true
     * to notify alt mode manager that UVDMs also could handled. Else - igrore
     */
    /* xxx_info(port)->uvdmSupp = true; */

    /* Place xxx alt mode additional init code if needed */
}

static void custom_alt_mode_exit(uint8_t port)
{
    /* Place custom ALT mode additional de-init code if needed */

    custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_EXIT;
}

static bool custom_alt_mode_is_cable_capable(uint8_t port, const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    /* Place code to verify if cable/AMA capabilities are consistent with custom ALT
     * mode here */

    return true;
}

static bool custom_alt_mode_is_cable_support_custom_alt_mode_svid(const cy_stc_pdaltmode_atch_tgt_info_t* atch_tgt_info)
{
    uint8_t idx = 0;

    while (atch_tgt_info->cblSvid[idx] != 0)
    {
        /* If Cable CUSTOM SVID found */
        if (atch_tgt_info->cblSvid[idx] == CUSTOM_ALT_MODE_VID)
        {
            return true;
        }
        idx++;
    }

    return false;
}

static void custom_alt_mode_send_cmd(uint8_t port)
{
    if (customAMStatus[port].state != CUSTOM_AM_STATE_IDLE)
    {
        cy_en_pd_sop_t sop_idx;
        custom_alt_mode_info(port)->vdmHeader.val = NO_DATA;
        custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.cmd = (uint32_t)customAMStatus[port].state;
        custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.svid = CUSTOM_ALT_MODE_VID;
        if (customAMStatus[port].state == CUSTOM_AM_STATE_EXIT)
        {
            custom_alt_mode_assign_vdo(port, CY_PDALTMODE_MNGR_NONE_VDO);
        }
        for (sop_idx = CY_PD_SOP; sop_idx <= customAMStatus[port].max_sop_supp; sop_idx++)
        {
            custom_alt_mode_info(port)->sopState[sop_idx] = CY_PDALTMODE_STATE_SEND;
        }
        custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_SEND;
    }
}

cy_pd_pd_do_t* custom_alt_mode_get_vdo(uint8_t port)
{
    return &(customAMStatus[port].vdo[CUSTOM_VDO_IDX]);
}
#endif /* CUSTOM_ALT_MODE_DFP_SUPP */

void custom_alt_mode_assign_vdo(uint8_t port, uint32_t vdo)
{
    /* No custom VDOs needed while send VDM */
    if (vdo == CY_PDALTMODE_MNGR_NONE_VDO)
    {
        custom_alt_mode_info(port)->vdoNumb[CY_PD_SOP] = NO_DATA;
    }
    else
    {
        /* Include given VDO as part of VDM */
        customAMStatus[port].vdo[CUSTOM_VDO_IDX].val = vdo;
        custom_alt_mode_info(port)->vdoNumb[CY_PD_SOP] = MAX_CUSTOM_VDO_NUMB;
    }
}

static cy_stc_pdaltmode_mngr_info_t* custom_alt_mode_info(uint8_t port)
{
    return &(customAMStatus[port].info);
}

/**********************custom UFP ALT MODE FUNCTIONS****************************/
#if CUSTOM_ALT_MODE_UFP_SUPP
static void custom_alt_mode_ufp_run(void *context)
{
    cy_stc_pdaltmode_context_t *ptrAltModeContext = ((cy_stc_pdaltmode_context_t *)context);

    uint8_t port = ptrAltModeContext->pdStackContext->port;

    /* Get custom ALT mode state and VDM command */
    cy_en_pdaltmode_state_t custom_mode_state = custom_alt_mode_info(port)->modeState;
    customAMStatus[port].state = (custom_state_t)custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.cmd;
    /* If exit all modes CMD received */
    if (custom_alt_mode_info(port)->vdmHeader.std_vdm_hdr.cmd == CY_PDALTMODE_MNGR_EXIT_ALL_MODES)
    {
        customAMStatus[port].state = CUSTOM_AM_STATE_EXIT;
    }
    switch(custom_mode_state)
    {
        /* This case section is responsible for evaluation of the received VDMs
         * If received VDM command evaluated successfully and ACK response should
         * be sent then set custom ALT mode state to IDLE and then return; in other
         * case use break operator.
         * If custom ALT mode VDM response contains VDOs then fill VDO buffer before
         * return operator. If custom don't need assign any VDOs when responds to
         * the received VDM with ACK then set vdo_numb[SOP] value from ALT mode
         * info structure with NO_DATA value */
        case CY_PDALTMODE_STATE_IDLE:
            switch(customAMStatus[port].state)
            {
                /* CUSTOM_AM_STATE_SOMESTATE flow is applicable to all received VDM
                 * commands processing (include enter,exit,etc. states) */
                case CUSTOM_AM_STATE_SOMESTATE:

                    /* Place evaluation code here */

                    /* If evaluation passed successfully then see below: */
                    /* Set state to idle */
                    /* If VDO is not needed in response use next, else - use custom_am_assign_vdo(port, SOMEVDO); to add VDO response */
                    custom_alt_mode_info(port)->vdoNumb[CY_PD_SOP] = NO_DATA;
                    /* If VDO is not needed in response use next, else - use custom_am_assign_vdo(port, SOMEVDO); to add VDO response */
                    custom_alt_mode_assign_vdo(port, SOMEVDO);
                    /* use return to respond with ACK */
                    return;

                case CUSTOM_AM_STATE_ENTER:
                    /* If VDO is not needed in response use next, else - use custom_am_assign_vdo(port, SOMEVDO); to add VDO response */
                    custom_alt_mode_info(port)->vdoNumb[CY_PD_SOP] = NO_DATA;

                    return;
                case CUSTOM_AM_STATE_EXIT:
                    /* When custom ALT mode exits then custom ALT mode state should be
                     * changed to EXIT to notify ALT modes manager */
                    custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_EXIT;
                    return;
                default:
                    break;
            }
            break;
        /* This case is responsible for evaluation of received VDM responses.
         * This case behavior is the same as for the same state processing in the
         * DFP state machine function (see above) */
        case CY_PDALTMODE_STATE_WAIT_FOR_RESP:

            custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_IDLE;
            /* Place received VDM response evaluation code here */
            return;
        case CY_PDALTMODE_STATE_RUN:
            /* Required handling can be implemented here */
            custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_IDLE;
            return;
        default:
            break;
    }
    /* Send NACK */
    custom_alt_mode_info(port)->modeState = CY_PDALTMODE_STATE_FAIL;
}
#endif /* CUSTOM_ALT_MODE_UFP_SUPP */

/******************* custom application Related Functions ************************/
/* Activates only when application custom command is received */
static bool custom_alt_mode_eval_app_cmd(uint8_t port, cy_stc_pdaltmode_alt_mode_evt_t cmd_data)
{

    /* Place APP command evaluation code here */

    /* If received APP command processed successfully then return true, if not - false */

    return false;
}

#endif /* CUSTOM_ALT_MODE_UFP_SUPP || CUSTOM_ALT_MODE_DFP_SUPP */

/* [] END OF FILE */
