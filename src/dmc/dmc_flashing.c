/******************************************************************************
 * File Name: dmc_flashing.c
 *
 * Description: DMC firmware update interface source file.
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
#include "cy_app_boot.h"
#include "cy_app_flash_config.h"
#include "dmc_flashing.h"
#include "cy_app_dmc_fwupdate.h"
#include "cy_app_dmc_metadata.h"
#include "dmc_solution.h"
#include "cy_app_flash.h"

#if (__ARMCC_VERSION)
__attribute__ ((section(".bootloaderruntype"), zero_init))
#elif defined (__GNUC__)
__attribute__ ((section(".bootloaderruntype")))
#elif defined (__ICCARM__)
#pragma location=".bootloaderruntype"
#endif /* (__ARMCC_VERSION) */
volatile uint32_t cyBtldrRunType;

/*
 * Prepares DMC flash write operation by enabling flash write and clearing DMC metadata
 * for invaldating the alternate DMC image prior to starting the image update.
 */
static cy_en_app_status_t dmc_prepare_update(cy_en_dmc_image_type_t img_type, bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);

/* API initiates blocking flash write to the DMC flash. In case of last row write, finish SHA2 calculation. */
static cy_en_app_status_t dmc_flash_row_write(cy_en_dmc_image_type_t img_type, uint16_t row_id, uint8_t *buffer, uint16_t size, bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);

/* Update dock metadata with DMC current running image, image validity and FW version. */
static void dmc_init_dev_param (const dev_topology_t *topology, cy_stc_dmc_params_t *params);

/* Jump to alternate image handler. */
static void dmc_jump_to_alternate(const dev_topology_t *topology, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);

/*! API to defer device reset sequence */
static bool dmc_is_dev_query_deferred(void);

/*! API to determine the update logic specific for DMC flashing module. */
static bool dmc_update_logic (uint8_t comp_id, cy_en_dmc_image_type_t img_type, bool *jump_to_alt_request);

/**
 * @brief Structure holding function pointers for DMC device modules.
 */
static const cy_stc_app_dmc_update_opern_t dmc_operation = {
    dmc_init_dev_param,
    NULL,
    NULL,
    dmc_prepare_update,
    dmc_flash_row_write,
    NULL,
    dmc_check_fw_version,
    dmc_jump_to_alternate,
    dmc_is_dev_query_deferred,
    dmc_update_logic,
    dmc_skip_jump_to_alt_request,
    NULL
};

/* Gets the pointer to the strcuture of function pointers related to various DMC operation */
const cy_stc_app_dmc_update_opern_t *get_dmc_operation(void)
{
    return &dmc_operation;
}

/**
 * @brief: Prepares DMC flash write operation by enabling flash write and clearing DMC metadata for invalidating the alternate
 * DMC image prior to starting the image update.
 */
static cy_en_app_status_t dmc_prepare_update(cy_en_dmc_image_type_t img_type, bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    (void)img_type;
    (void)cb;
    *deferred = false;

    /* Erase the firmware metadata before starting the firmware update to invalidate the alternate image. */
    Cy_App_Flash_RowClear (Cy_App_Dmc_GetAltImgMdRowNum());

    return CY_APP_STAT_SUCCESS;
}

/* API initiates blocking flash write to the DMC flash. In case of last row write, finish SHA2 calculation. */
static cy_en_app_status_t dmc_flash_row_write(cy_en_dmc_image_type_t img_type, uint16_t row_id, uint8_t *buffer, uint16_t size, bool *deferred,
                                           cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    (void)cb;
    (void) size;
    (void) img_type;

    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    *deferred = false;

    if ((row_id >= CY_APP_SYS_DMC_METADATA_START_ROW_ID) && (row_id <= CY_APP_SYS_DMC_METADATA_END_ROW_ID))
    {
        /* Do not allow writes to the Dock metadata rows if a malicious tool attempts to. */
        return CY_APP_STAT_INVALID_ARGUMENT;
    }

    /* Enter flash mode. */
    Cy_App_Flash_EnterMode(true, CY_APP_FLASH_IF_USB_HID, false);

    /* Flash row write done only if (calculated SHA hash = received hash in FWCT).
     * Else metadata remains cleared, thus invalidating the firmware image
     */
    if (status == CY_APP_STAT_SUCCESS)
    {
        status = Cy_App_Flash_RowWrite(row_id, buffer, NULL);
    }

    /* Exit flash mode. */
    Cy_App_Flash_EnterMode(false, CY_APP_FLASH_IF_USB_HID, false);

    return status;
}

/* Gets current running image info for DMC. */
static cy_en_dmc_image_type_t dmc_get_cur_image(const dev_topology_t *topology)
{
    (void)topology;
    cy_stc_app_dmc_dock_status_t* dock_status = Cy_App_Dmc_GetDockStatus();

    return dock_status->devx_status[CY_APP_DMC_COMPONENT_ID].cur_img;
}

/**
 * @brief: Update dock metadata with DMC current running image, image validity and FW version.
 */
static void dmc_init_dev_param (const dev_topology_t *topology, cy_stc_dmc_params_t *params)
{
    cy_en_dmc_image_type_t img_type;
    uint8_t img_status = 0x00;
    uint32_t* volatile bl_ver = (uint32_t *)CY_APP_SYS_BOOT_VERSION_ADDRESS;
    uint32_t fw1_ver, fw2_ver;
    cy_stc_sys_fw_metadata_t *fw1_md, *fw2_md;
    cy_stc_app_dmc_dock_status_t* dock_status = Cy_App_Dmc_GetDockStatus();

    /* Set mode variables and flash access limits based on the active firmware. */
    if (Cy_App_Sys_GetDeviceMode() == CY_APP_SYS_FW_MODE_FWIMAGE_1)
    {
        img_type = CY_APP_DMC_IMAGE_TYPE_FWIMAGE_1;
        img_status = CY_APP_DMC_IMG_STATUS_VALID;

        /* Check if FW2 is valid. */
        if (Cy_App_Boot_ValidateFw ((cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG2_FW_METADATA_ADDR) != CY_APP_STAT_SUCCESS)
        {
            img_status |= (CY_APP_DMC_IMG_STATUS_INVALID << 4);
        }
        else
        {
            img_status |= (CY_APP_DMC_IMG_STATUS_VALID << 4);
        }

    }
    else
    {
        img_type = CY_APP_DMC_IMAGE_TYPE_FWIMAGE_2;
        img_status = (CY_APP_DMC_IMG_STATUS_VALID << 4);

        /* Check if FW1 is valid. */
        if (Cy_App_Boot_ValidateFw ((cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG1_FW_METADATA_ADDR) != CY_APP_STAT_SUCCESS)
        {
            img_status |= CY_APP_DMC_IMG_STATUS_INVALID;
        }
        else
        {
            img_status |= CY_APP_DMC_IMG_STATUS_VALID;
        }

    }

    /* Calculate the version address from the firmware metadata. */
    if ((img_status & 0x0F) == CY_APP_DMC_IMG_STATUS_VALID)
    {
        fw1_md  = (cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG1_FW_METADATA_ADDR;
        fw1_ver = ((uint32_t)fw1_md->fw_start) + CY_APP_SYS_FW_VERSION_OFFSET;
    }
    else
    {
        /* Retain last good FW version. */
        fw1_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img1_version;
    }

    if ((img_status >> 4) == CY_APP_DMC_IMG_STATUS_VALID)
    {
        fw2_md  = (cy_stc_sys_fw_metadata_t *)CY_APP_SYS_IMG2_FW_METADATA_ADDR;
        fw2_ver = ((uint32_t)fw2_md->fw_start) + CY_APP_SYS_FW_VERSION_OFFSET;
    }
    else
    {
        /* Retain last good FW version. */
        fw2_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img2_version;
    }

    Cy_App_Dmc_SetRamImageStatus (CY_APP_DMC_COMPONENT_ID, img_type, img_status);

    /* Update version information in the dock metadata for DMC */
    Cy_App_Dmc_UpdateRamVersions (CY_APP_DMC_COMPONENT_ID,
            (uint8_t *)bl_ver,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

#if (CY_APP_USR_DEFINED_SN_SUPPORT)
    /* Enable the SN Support in DMC firmware*/
    if (((dock_status->devx_status[topology->comp_id].dev_specific[CY_APP_DMC_DEV_SPECIFIC_BYTE0] & CY_APP_DMC_METADATA_SN_BIT_MASK) >> CY_APP_DMC_METADATA_SN_BIT_POS) == SN_NOT_IMPLEMENTED)
    {
        dock_status->devx_status[topology->comp_id].dev_specific[CY_APP_DMC_DEV_SPECIFIC_BYTE0] |= (SN_INVALID << CY_APP_DMC_METADATA_SN_BIT_POS);
    }
#endif

}

/**
 * @brief Checks if the DMC/CCGx image version from FWCT in RAM > dock metadata version for the alternate CCGx image.
 */
bool dmc_check_fw_version(uint8_t *img_version, cy_en_dmc_image_type_t img_type, uint8_t comp_id, bool check_last_valid)
{
    bool update_needed = true;
    cy_stc_app_dmc_dock_status_t* dock_status = Cy_App_Dmc_GetDockStatus();

    /* During factory condition - unconditionally update the DMC without performing version check. */
    if (!is_dmc_in_factory_condition () && Cy_App_Dmc_VersionCheckEnabled())
        {
            cy_stc_app_dmc_fw_version_t *ver = &dock_status->devx_status[comp_id].fw_version;
            base_fw_version_t *cur_base_ver_ptr = NULL, *fwct_base_ver_ptr = NULL;
            app_fw_version_t *cur_app_ver_ptr = NULL, *fwct_app_ver_ptr = NULL;
            uint32_t cur_base_ver, fwct_base_ver, cur_app_ver, fwct_app_ver;

            if (img_type == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_1)
            {
                cur_base_ver_ptr = (base_fw_version_t *)(ver->img1_version);
                cur_app_ver_ptr = (app_fw_version_t *)(ver->img1_version + 4);

            }
            else if (img_type == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_2)
            {
                cur_base_ver_ptr = (base_fw_version_t *)(ver->img2_version);
                cur_app_ver_ptr = (app_fw_version_t *)(ver->img2_version + 4);
            }

            fwct_base_ver_ptr = (base_fw_version_t *)(img_version);
            fwct_app_ver_ptr = (app_fw_version_t *)(img_version + 4);

            cur_base_ver = *((uint32_t *)cur_base_ver_ptr);
            fwct_base_ver = *((uint32_t *)fwct_base_ver_ptr);
            cur_app_ver = *((uint32_t *)cur_app_ver_ptr);
            fwct_app_ver = *((uint32_t *)fwct_app_ver_ptr);

            if (fwct_app_ver_ptr->type_string == cur_app_ver_ptr->type_string)
            {
                if (fwct_base_ver < cur_base_ver)
                {
                    update_needed = false;
                }
                else if (fwct_base_ver == cur_base_ver)
                {
                    if (check_last_valid)
                    {
                        /* Update image if new version >= last good version. */
                        if (fwct_app_ver < cur_app_ver)
                        {
                            update_needed = false;
                        }
                    }
                    else
                    {
                        /* Update image if new version > current running version. */
                        if (fwct_app_ver <= cur_app_ver)
                        {
                            update_needed = false;
                        }
                    }
                }
            }
            else
            {
                update_needed = false;
            }
        }
    return update_needed;
}

static void dmc_jump_to_alternate(const dev_topology_t *topology, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    (void)topology;
    (void)cb;
    if (Cy_App_Sys_GetDeviceMode() == CY_APP_SYS_FW_MODE_FWIMAGE_1)
    {
        cyBtldrRunType = CY_APP_SYS_FW2_BOOT_RQT_SIG;
    }
    else if (Cy_App_Sys_GetDeviceMode() == CY_APP_SYS_FW_MODE_FWIMAGE_2)
    {
        cyBtldrRunType = CY_APP_SYS_FW1_BOOT_RQT_SIG;
    }

    Cy_App_Dmc_PrepareSoftReset(params);
    CySoftwareReset();
}

static bool dmc_is_dev_query_deferred(void)
{
    return false;
}

static bool dmc_update_logic (uint8_t comp_id, cy_en_dmc_image_type_t img_type, bool *jump_to_alt_request)
{
    bool status = false;
    const dev_topology_t *topology = Cy_App_Dmc_GetDeviceTopology (comp_id);
    *jump_to_alt_request  = false;

    if (dmc_get_cur_image(topology) != img_type)
    {
        status = true;
    }
    else
    {
        *jump_to_alt_request  = true;
    }

    return status;
}

bool dmc_skip_jump_to_alt_request (uint8_t comp_id, cy_en_dmc_image_type_t cur_img)
{
    bool alt_img_valid = false;

    /* Check the image validity of the alternate image. */
    if (cur_img == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_2)
    {
        alt_img_valid = (Cy_App_Dmc_CheckImageValidity (comp_id, CY_APP_DMC_IMAGE_TYPE_FWIMAGE_1, NULL) == CY_APP_DMC_IMG_STATUS_VALID);
    }
    else
    {
        alt_img_valid = (Cy_App_Dmc_CheckImageValidity (comp_id, CY_APP_DMC_IMAGE_TYPE_FWIMAGE_2, NULL) == CY_APP_DMC_IMG_STATUS_VALID);
    }

    /* Skip the jump to alternate request if the alternate image is not valid.  */
    return (!alt_img_valid);
}


/* [] END OF FILE */
