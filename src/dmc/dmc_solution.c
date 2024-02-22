/******************************************************************************
 * File Name: dmc_solution.c
 *
 * Description: Solution source file for DMC.
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

#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "cy_pdutils_sw_timer.h"
#include "dmc_solution.h"
#include "dmc_flashing.h"
#include "cy_app_dmc_vendor.h"
#include "cy_app_dmc_metadata.h"
#include "cy_app_spi_comp_update.h"
#include "cy_app_dmc_fwupdate.h"
#include "cy_pdstack_common.h"
#include "cy_pdstack_dpm.h"
#include "cy_app_system.h"
#include "ccgx_ctrl.h"
#include "etag.h"
#include "cy_app_debug.h"
#include "cy_app_led_ctrl.h"
#include "spi_eeprom_master.h"

/*****************************Macros*****************************/

/*************************Global Variables*************************/

extern cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx);
#if CY_APP_LED_CONTROL_ENABLE
extern cy_stc_led_ctrl_context_t gl_LedCtrlCtx;
#endif /* CY_APP_LED_CONTROL_ENABLE */
extern cy_stc_dmc_params_t gl_dmcParams;
extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;

/*******************************APIs*******************************/
/**
 * @brief Hardware interface initialization for DMC operation.
 * @param None
 * @return None.
 */
void dmc_init_hw_interface(cy_stc_dmc_params_t *params)
{
    const cdtt_config_t *cdtt = params->ptrCdttCfg;
    uint8_t dev_count;
    uint8_t comp_id;
    const dev_topology_t *topology;
    const cy_stc_app_dmc_update_opern_t **dev_opern = Cy_App_Dmc_GetDevUpdateOpern();

    if (cdtt->signature == CY_APP_DMC_CDTT_VALID_SIG)
    {
        dev_count = cdtt->dev_count;

        for (comp_id = 0; comp_id < dev_count; comp_id++)
        {
            topology = &cdtt->dev_info[comp_id];
            switch (topology->device_type)
            {
                case CY_APP_DMC_DEV_TYPE_DMC_PMG1S3:
                    dev_opern[comp_id] = get_dmc_operation();
                    break;

                case CY_APP_DMC_DEV_TYPE_SPI_COMPONENT:

                    Cy_App_Dmc_SpiSetCompId (comp_id);

                    spi_eeprom_init();
                    dev_opern[comp_id] =  Cy_App_Dmc_SpiGetCompOperation ();
                    if ((dev_opern[comp_id] != NULL) && (dev_opern[comp_id]->dev_config_hw_interface != NULL))
                    {
                        dev_opern[comp_id]->dev_config_hw_interface(topology, params);
                    }
                    break;
    
#if (CCGX_UPDATE == 1)
                case CY_APP_DMC_DEV_TYPE_CCG7SC:
                    dev_opern[comp_id] = get_ccgx_operation();
                    if ((dev_opern[comp_id] != NULL) && (dev_opern[comp_id]->dev_config_hw_interface != NULL))
                    {
                        dev_opern[comp_id]->dev_config_hw_interface(topology, params);
                    }
                    break;
#endif /* (CCGX_UPDATE ==1) */

                default:
                    dev_opern[comp_id] = NULL;
            }
        }
    }
    else
    {
        dev_opern[CY_APP_DMC_COMPONENT_ID] = get_dmc_operation();
    }
}

bool is_dmc_in_factory_condition (void)
{
    /* 1)If ETAG is supported:
     * DMC is in factory condition if a VALID ETAG is present.
     * If a VALID ETAG is not present, then DMC is not in factory condition.
     * 2)If ETAG is not supported:
     * DMC is in factory condition if Cy_App_Dmc_IsFactoryUpdateDone returns false.
     * Otherwise, DMC is not in factory condition.
    */

#if (ETAG_SUPPORT_ENABLE)
    uint8_t buffer[ETAG_INFORMATION_SIZE];
    if(read_etag_information(buffer) == CY_APP_STAT_SUCCESS)
    {
        if(buffer[ETAG_VALID_OFFSET] == ETAG_VALID)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return true;
    }
#else
    return !(Cy_App_Dmc_IsFactoryUpdateDone());
#endif /* ETAG_SUPPORT_ENABLE */
}

/**
 * @brief Initiates All devices reset: HX3, All CCGx reset; then initiates DMC soft reset.
 * If any device needs to be reset at the end of the firmware update, add the handler here.
 */
void init_dock_reset(void)
{
    Cy_App_Dmc_PrepareSoftReset (&gl_dmcParams);
    CySoftwareReset();
}

#if HX3_BOOT_WAIT_ENABLE
static void hx3_boot_wait_timer_cb(cy_timer_id_t id, void *callbackContext)
{
    (void)id;
    (void)callbackContext;

    Cy_App_Dmc_SetCurState (CY_APP_DMC_STATE_POWER_ON);
}
#endif /* HX3_BOOT_WAIT_ENABLE */

void dmc_init(const cdtt_config_t *ptrCdttCfg, const sec_config_t *ptrSecCfg,
              const cy_stc_dmc_app_cbk_t *ptrAppCbk, cy_stc_dmc_params_t *params)
{
    cy_en_app_status_t status;
    cy_stc_app_dmc_dock_metadata_t* dock_md = Cy_App_Dmc_GetDockMetadata();

    /* Initialize DMC structure and global status flags*/
    status = Cy_App_Dmc_PreInit((cy_en_dmc_fw_mode_t)Cy_App_Sys_GetDeviceMode(), ptrCdttCfg, ptrSecCfg, ptrAppCbk, params);
    if(status != CY_APP_STAT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize hardware interface for DMC operation. */
    dmc_init_hw_interface(params);

    /* Init dock metadata. */
    Cy_App_Dmc_InitMetadata(params);

    if (dock_md->soft_reset_enum != CY_APP_DMC_USB_ENUM_RQT_SIG)
    {
        Cy_App_Dmc_ResetState(params);
    }
    else
    {
        dock_md->soft_reset_enum = 0;
        /* Write dock metadata in RAM to flash. */
        Cy_App_Dmc_WriteMetadata(false, params);
    }

#if HX3_BOOT_WAIT_ENABLE
    /* Wait for HX3 to boot-up */
    Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, NULL, HX3_BOOT_WAIT_TIMER_ID,
                             HX3_BOOT_WAIT_TIMER_PERIOD, hx3_boot_wait_timer_cb);
#else
    Cy_App_Dmc_SetCurState (CY_APP_DMC_STATE_POWER_ON);
#endif /* HX3_BOOT_WAIT_ENABLE */

    Cy_App_Dmc_SetFactoryUpdateStatus( Cy_App_Dmc_ReadFactoryUpdateStatus (params) );
}

uint8_t Cy_App_Dmc_GetKeyId(void)
{
    return KEY_ID_EMBEDDED;
}

void dmc_internal_flash_enter_mode(bool is_enable, bool data_in_place)
{
    Cy_App_Flash_EnterMode(is_enable, CY_APP_FLASH_IF_USB_HID, data_in_place);
}

cy_en_app_status_t dmc_internal_flash_row_write(uint16_t row_num, uint8_t *data)
{
    cy_en_app_status_t status;
    status = Cy_App_Flash_RowWrite(row_num, data, NULL);

    return status;
}

cy_en_app_status_t dmc_spi_flash_write_enable (bool enable)
{
    cy_en_app_status_t status;
    status = spi_eeprom_write_enable(enable);

    return status;
}

cy_en_app_status_t dmc_spi_flash_write (uint8_t *buffer, uint16_t size, uint32_t page_addr, bool retry)
{
    cy_en_app_status_t status;

    if(retry)
    {
        status = spi_eeprom_write_flash_with_retry(buffer, size, page_addr);
    }
    else
    {
        status = spi_eeprom_write_flash(buffer, size, page_addr);
    }

    return status;
}

cy_en_app_status_t dmc_spi_flash_read (uint8_t *buffer, uint16_t size, uint32_t page_addr, bool retry)
{
    cy_en_app_status_t status;

    if(retry)
    {
        status = spi_eeprom_read_flash_with_retry(buffer, size, page_addr);
    }
    else
    {
        status = spi_eeprom_read_flash(buffer, size, page_addr);
    }

    return status;
}

cy_en_app_status_t dmc_spi_flash_erase (uint32_t flash_addr, cy_en_spi_flash_erase_t size)
{
    cy_en_app_status_t status;

    switch(size)
    {
        case CY_APP_DMC_SPI_FLASH_ERASE_4K:
            status = spi_eeprom_4k_sector_erase(flash_addr);
            break;

        case CY_APP_DMC_SPI_FLASH_ERASE_32K:
            status = spi_eeprom_32k_block_erase(flash_addr);
            break;

        case CY_APP_DMC_SPI_FLASH_ERASE_64K:
            status = spi_eeprom_64k_block_erase(flash_addr);
            break;

        case CY_APP_DMC_SPI_FLASH_ERASE_CHIP:
            status = spi_eeprom_chip_erase();
            break;

        default:
            status = CY_APP_STAT_INVALID_COMMAND;
            break;
    }
    return status;
}

bool dmc_spi_flash_is_write_in_progress (void)
{
    uint8_t flash_status = SPI_WIP;

    spi_eeprom_read_status_reg(&flash_status);
    if(SPI_EEPROM_IS_WRITE_IN_PROGRESS(flash_status))
    {
        return true;
    }

    return false;
}

void app_fw_update_complete_handler (void)
{
    Cy_PdStack_Dpm_Start(get_pdstack_context(0));
}

void dmc_led_set_mode(cy_en_dmc_led_mode_t mode)
{
#if CY_APP_LED_CONTROL_ENABLE
    Cy_App_LedCtrl_SwitchMode((cy_en_led_ctrl_mode_t)mode, &gl_LedCtrlCtx);
#else
    (void)mode;    
#endif /* CY_APP_LED_CONTROL_ENABLE */
}

void dmc_app_event_handler(cy_en_dmc_app_evt_t evt, uint8_t *data, uint8_t size)
{
    switch(evt)
    {
        case CY_APP_DMC_EVT_FW_UPDATE_FAIL:
            if(data[0] == CY_APP_DMC_AUTHENTICATION_FAILED)
            {
                CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_FW_UPD_P1_AUTH_FAILURE, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            }
            else if(data[0] == CY_APP_DMC_PHASE2_UPDATE_FAIL_AUTHENTICATION_FAILED)
            {
                CY_APP_DEBUG_LOG(0, CY_APP_DEBUG_FW_UPD_P2_AUTH_FAILURE, NULL, 0, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            }
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_FAILURE, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_UPDATE_STARTS:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_START, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_AUTHENTICATION_SUCCESS:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_AUTH_SUCCESS, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_IMAGE_WRITE_SUCCESS:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_IMG_WRITE_SUCCESS, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_IMAGE_WRITE_FAIL:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_IMG_WRITE_FAIL, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PENDING_UPDATES:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PENDING_UPDATES, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_IMG_UPDATE_START:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_IMG_UPDATE_START, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_UPDATE_ENDS:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_UPDATE_END, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_FACTORY_BACKUP_STARTED:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_FACTORY_BKUP_START, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_DOCK_RESET:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_DOCK_RESET, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_FACTORY_BACKUP_NOT_DONE:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_FACTORY_BKUP_NOT_DONE, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_PHASE2_FACTORY_UPDATE_STATUS_FAILED:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_FW_UPD_PHASE2_FACTORY_UPDATE_FAIL, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        case CY_APP_DMC_EVT_DMC_STATE_CHANGE:
            CY_APP_DEBUG_LOG(0u, CY_APP_DEBUG_OPCODE_DMC_STATE, data, size, CY_APP_DEBUG_LOGLEVEL_CRITICAL, true);
            break;

        default:
            break;
    }
}

/* [] END OF FILE */
