/*******************************************************************************
* File Name: ccgx_ctrl.c
*
* Description: CCG device firmware update interface source file.
*
* Related Document: See README.md
*
********************************************************************************
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

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <stdint.h>
#include "cy_app_status.h"
#include "config.h"
#include "cy_hpi_master.h"
#include "dmc_solution.h"
#include "ccgx_ctrl.h"
#include "cy_app_dmc_fwupdate.h"
#include "cy_app_dmc_metadata.h"

#define CCG7_SILICON_FAMILY_ID                      (0x3Au)

#define CCG7SC_LAST_ROW_NUM                         (0x01FFu)

/* Maximum support flash row size. */
#define CCG_MAX_FLASH_ROW_SIZE                      (256u)

#define HPI_DEV_REG_RESET_ADDR_LEN                  (0x02u)
#define HPI_DEV_REG_INTR_ADDR_LEN                   (0x01u)
#define HPI_DEV_REG_DEVICE_MODE_LEN                 (0x01u)
#define HPI_DEV_REG_FLASH_READ_WRITE_LEN            (0x04u)
#define HPI_DEV_REG_SI_ID_LEN                       (0x02u)
#define HPI_DEV_REG_ENTER_FLASH_MODE_LEN            (0x01u)
#define HPI_DEV_REG_BOOT_MODE_REASON_LEN            (0x01u)
#define HPI_DEV_REG_PORT_ENABLE_LEN                 (0x01u)
#define HPI_DEV_REG_JUMP_TO_BOOT_LEN                (0x01u)
#define HPI_DEV_REG_RESPONSE_LEN                    (0x02u)

extern cy_hpi_master_context_t gl_HpiMasterCtx;
extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;

extern GPIO_PRT_Type* gpio_base_arr[NUM_OF_GPIO_PORTS];

/**
 * @brief Structure holding parameters needed for CCGx FW update. Parameters are
 *        initialized in ccgx_update_init().
 * Every ccgx_update_init() must be associated with a corresponding ccgx_update_deinit().
 */
typedef struct
{
    uint8_t intr_gpio;
    uint8_t scb_index;
    uint8_t slave_addr;
    volatile bool intr_pending;

    cy_stc_app_dmc_completion_cbk_t completion;
    const dev_topology_t *topology;
    uint16_t cur_hpi_cmd;
    uint16_t next_hpi_cmd;
    uint16_t md_row_num;

    uint8_t comp_id;
    uint8_t finish_update_reset_flag;
    uint8_t ccgx_update_started;

}dmc_ccgx_update_t;

/*******************************************************************************
 * Static Function Declarations
 ******************************************************************************/   

static void ccgx_update_init(const dev_topology_t *topology, cy_stc_dmc_params_t *params);
static void ccgx_update_deinit(cy_stc_dmc_params_t *params);
cy_en_app_status_t ccgx_prepare_update(cy_en_dmc_image_type_t img_type, bool *deferred,
                                    cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);
static cy_en_app_status_t ccgx_flash_row_write(cy_en_dmc_image_type_t img_type, uint16_t row_id,
                                            uint8_t *buffer, uint16_t size,
                                            bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);
static void ccgx_init_dev_param (const dev_topology_t *topology, cy_stc_dmc_params_t *params);
static void ccgx_jump_to_alternate(const dev_topology_t *topology, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);
static bool ccgx_is_dev_query_deferred(void);
static bool ccgx_update_logic (uint8_t comp_id, cy_en_dmc_image_type_t img_type, bool *jump_to_alt_request);
static cy_en_app_status_t ccgx_finish_update (cy_en_dmc_image_type_t img_type, bool flashing_status,
                                           bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params);
static bool ccgx_skip_jump_to_alt_request (uint8_t comp_id, cy_en_dmc_image_type_t cur_img);
static void ccgx_config_hw_interface(const dev_topology_t *topology, cy_stc_dmc_params_t *params);

static void report_cmd_completion(cy_en_app_status_t status);

/* Structure holding function pointers for CCGx device modules.. */
const  cy_stc_app_dmc_update_opern_t ccgx_operation ={
    ccgx_init_dev_param,
    ccgx_update_init,
    ccgx_update_deinit,
    ccgx_prepare_update,
    ccgx_flash_row_write,
    ccgx_finish_update,
    dmc_check_fw_version,
    ccgx_jump_to_alternate,
    ccgx_is_dev_query_deferred,
    ccgx_update_logic,
    ccgx_skip_jump_to_alt_request,
    ccgx_config_hw_interface,
};

/**
 * @brief Structure holding parameters needed for CCGx FW update. Parameters are
 *        initialized in ccgx_update_init().
 */
static dmc_ccgx_update_t gl_ccgx_update;

const cy_stc_app_dmc_update_opern_t* get_ccgx_operation(void)
{
    return &ccgx_operation;
}

void ccgx_response_timer_cbk(cy_timer_id_t id, void *ptrContext)
{
    report_cmd_completion(CY_APP_STAT_FAILURE);

    (void)id;
    (void)ptrContext;
}

static cy_en_app_status_t ccgx_device_register_write(uint8_t slaveAddr, uint8_t deviceReg, void *writeBuff, uint16_t writeLen)
{
    cy_hpi_master_status_t status;

    status = Cy_HPI_Master_DevRegWrite(&gl_HpiMasterCtx, slaveAddr, deviceReg, writeBuff, writeLen);

    Cy_PdUtils_SwTimer_Start(&gl_TimerCtx,
                             NULL,
                             CCGX_RESPONSE_WAIT_TIMER_ID,
                             CCGX_RESPONSE_WAIT_TIMER_PERIOD,
                             ccgx_response_timer_cbk);

    return (status == CY_HPI_MASTER_SUCCESS) ? CY_APP_STAT_SUCCESS : CY_APP_STAT_FAILURE;
}

/**
 * @brief Initiates CCGx soft reset.
 */
cy_en_app_status_t ccgx_soft_reset(cy_stc_app_dmc_completion_cbk_t cb)
{
    cy_en_app_status_t status;
    uint8_t buffer[2];
    gl_ccgx_update.completion = cb;

    buffer[0] = CY_HPI_MASTER_RESET_CMD_SIG;
    buffer[1] = CY_HPI_MASTER_RESET_DEVICE_RESET_CMD;
    status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                       CY_HPI_MASTER_DEV_REG_RESET_ADDR, &buffer,
                                       HPI_DEV_REG_RESET_ADDR_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        gl_ccgx_update.cur_hpi_cmd = CY_HPI_MASTER_DEV_REG_RESET_ADDR;
    }

    return status;
}

static void ccgx_clear_pending_hpi_intr(uint8_t intr_stat)
{
    uint8_t slave_index;

    slave_index = Cy_HPI_Master_GetSlaveIndexByAddr(&gl_HpiMasterCtx,
                                                    gl_ccgx_update.slave_addr);

    gl_HpiMasterCtx.intrMask &= ~(0x01 << slave_index);

    Cy_HPI_Master_DevRegWrite(&gl_HpiMasterCtx, gl_ccgx_update.slave_addr,
                              CY_HPI_MASTER_DEV_REG_INTR_ADDR, (uint8_t *)&intr_stat,
                              HPI_DEV_REG_INTR_ADDR_LEN);
}

/**
 * @brief Gets current running image info for CCGx device.
 */
static cy_en_dmc_image_type_t ccgx_get_cur_image(const dev_topology_t *topology)
{
    cy_en_app_status_t status;
    uint8_t cur_img_type = 0u;
    cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)&topology->access_param[0];

    status = Cy_HPI_Master_DevRegRead(&gl_HpiMasterCtx, access_param->hpi_i2c_param.slave_addr,
                                      CY_HPI_MASTER_DEV_REG_DEVICE_MODE, &cur_img_type,
                                      HPI_DEV_REG_DEVICE_MODE_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        cur_img_type &= 0x03u;
    }
    return cur_img_type;
}

static void ccgx_update_init(const dev_topology_t *topology, cy_stc_dmc_params_t *params)
{
    uint8_t temp_finish_update_flag = gl_ccgx_update.finish_update_reset_flag;
    cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)&topology->access_param[0];

    /* gl_ccgx_update parameters initialized from CDTT device topology. */
    memset(&gl_ccgx_update, 0, sizeof(gl_ccgx_update));
    
    gl_ccgx_update.intr_gpio = access_param->hpi_i2c_param.hpi_intr_gpio;
    gl_ccgx_update.slave_addr = access_param->hpi_i2c_param.slave_addr;
    gl_ccgx_update.comp_id = topology->comp_id;
    gl_ccgx_update.topology = topology;

    gl_ccgx_update.finish_update_reset_flag = temp_finish_update_flag;
}

/**
 * @brief Deinit CCGx module operation.
 */
static void ccgx_update_deinit(cy_stc_dmc_params_t *params)
{
    gl_ccgx_update.completion = NULL;
    gl_ccgx_update.ccgx_update_started = 0u;
}

/**
 * @brief Invokes completion callback passed in from DMC application layer.
 */
static void report_cmd_completion(cy_en_app_status_t status)
{
    if (gl_ccgx_update.completion != NULL)
    {
        gl_ccgx_update.completion(status);
    }
}

/**
 * @brief Clears the flash row of the specified row using HPI command.
 */
static cy_en_app_status_t ccgx_clear_flash_row(uint16_t row_id)
{
    cy_en_app_status_t status;
    uint8_t buffer[CCG_MAX_FLASH_ROW_SIZE] = {0};
    uint16_t size = CCG_MAX_FLASH_ROW_SIZE;

    /* Initiates CCGx flash write operation. */
    status = Cy_HPI_Master_FlashMemoryWrite(&gl_HpiMasterCtx, gl_ccgx_update.slave_addr, buffer, size);

    if (status == CY_APP_STAT_SUCCESS)
    {
        buffer[0] = CY_HPI_MASTER_FLASH_READ_WRITE_CMD_SIG;
        buffer[1] = CY_HPI_MASTER_FLASH_ROW_WRITE_CMD;
        buffer[2] = (uint8_t)row_id;
        buffer[3] = (uint8_t)(row_id >> 8u);
        status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                           CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE, buffer,
                                           HPI_DEV_REG_FLASH_READ_WRITE_LEN);
        if (status == CY_APP_STAT_SUCCESS)
        {
            gl_ccgx_update.cur_hpi_cmd = CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE;
        }
    }

    return status;
}

/**
 * @brief Return the last row number of the silicon.
 */
static uint16_t ccgx_get_last_row_num(uint16_t silicon_id)
{
    uint16_t last_row_num = 0;
    uint8_t silicon_family = (silicon_id >> 8);

    switch (silicon_family)
    {
        case CCG7_SILICON_FAMILY_ID:
            last_row_num = CCG7SC_LAST_ROW_NUM;
            break;
        default:
            break;
    }

    return last_row_num;
}

/**
 * @brief Calculate the metadata row number of the image to be updated.
 */
static void ccgx_calc_md_row_num(cy_en_dmc_image_type_t img_type)
{
    uint16_t silicon_id = 0;
    cy_en_app_status_t status;
    uint16_t last_row_num;

    /* Read the silicon id of CCGx devices to determine the metadata row number. */
    status = Cy_HPI_Master_DevRegRead(&gl_HpiMasterCtx, gl_ccgx_update.slave_addr,
                                      CY_HPI_MASTER_DEV_REG_SI_ID, (uint8_t *)&silicon_id,
                                      HPI_DEV_REG_SI_ID_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        last_row_num = ccgx_get_last_row_num(silicon_id);
        if (img_type == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_1)
        {
           gl_ccgx_update.md_row_num = last_row_num;
        }
        else
        {
           gl_ccgx_update.md_row_num = last_row_num - 1;
        }
    }
}


/**
* @brief: Complete firmware update.
*/
cy_en_app_status_t ccgx_finish_update (cy_en_dmc_image_type_t img_type, bool flashing_status,
                                    bool *deferred, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    gl_ccgx_update.completion = cb;

    if ((flashing_status == true) && (img_type == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_1))
    {
        *deferred = true;
        gl_ccgx_update.finish_update_reset_flag = true;

        /* Initiate CCG soft reset. */
        if(gl_ccgx_update.topology->image_mode != CY_APP_DMC_IMG_MODE_SINGLE_IMG)
        {
            ccgx_jump_to_alternate(gl_ccgx_update.topology, cb, params);
        }
        else
        {
            ccgx_soft_reset(cb);
        }
    }
    else if ((flashing_status == true) && (img_type == CY_APP_DMC_IMAGE_TYPE_FWIMAGE_2))
    {
        *deferred = false;
        gl_ccgx_update.ccgx_update_started = 0;
    }
    else
    {
        *deferred = false;
        gl_ccgx_update.ccgx_update_started = 0;
    }

    return status;
}

/**
 * @brief: Send Prepare Update command and Defer Writing Row data for 5s for
 *         FXVL to erase its NVM.
 */
cy_en_app_status_t ccgx_prepare_update(cy_en_dmc_image_type_t img_type, bool *deferred,
                                    cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;

    *deferred = false;
    uint8_t buffer;

    gl_ccgx_update.ccgx_update_started = 1u;
    gl_ccgx_update.completion = cb;

    /* Determine the metadata row number. */
    ccgx_calc_md_row_num(img_type);

    buffer = CY_HPI_MASTER_ENTER_FLASHING_CMD_SIG;
    /* Send CCGx HPI command to enable flashing in CCGx device. */
    status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                        CY_HPI_MASTER_DEV_REG_ENTER_FLASH_MODE,
                                        (uint8_t *)&buffer,
                                        HPI_DEV_REG_ENTER_FLASH_MODE_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        gl_ccgx_update.cur_hpi_cmd = CY_HPI_MASTER_DEV_REG_ENTER_FLASH_MODE;

        /* Marking deferred flag as true for the state machine to wait for HPI
         * response for this command. */
        *deferred = true;
    }

    return status;
}

static cy_en_app_status_t ccgx_flash_row_write(cy_en_dmc_image_type_t img_type, uint16_t row_id,
                                            uint8_t *buffer, uint16_t size,
                                            bool *deferred, cy_stc_app_dmc_completion_cbk_t cb,
                                            cy_stc_dmc_params_t *params)
{
    cy_en_app_status_t status;
    *deferred = false;
    uint8_t reg_data[4];

    /* Initiates CCGx flash write operation. */
    status = Cy_HPI_Master_FlashMemoryWrite(&gl_HpiMasterCtx,
                                           gl_ccgx_update.slave_addr, buffer, size);
    if (status == CY_APP_STAT_SUCCESS)
    {
        gl_ccgx_update.completion = cb;
        reg_data[0] = CY_HPI_MASTER_FLASH_READ_WRITE_CMD_SIG;
        reg_data[1] = CY_HPI_MASTER_FLASH_ROW_WRITE_CMD;
        reg_data[2] = (uint8_t)row_id;
        reg_data[3] = (uint8_t)(row_id >> 8u);
        status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                    CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE, reg_data,
                    HPI_DEV_REG_FLASH_READ_WRITE_LEN);
    }

    if (status == CY_APP_STAT_SUCCESS)
    {
        gl_ccgx_update.cur_hpi_cmd = CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE;

        /* Marking deferred flag as true for the state machine to wait for HPI
         * response for this command. */
        *deferred = true;
    }

    return status;
}

/**
 * @brief Gets validity of both CCGx images.
 */
static void ccgx_get_image_validity(const dev_topology_t *topology, uint8_t *img_validity)
{
    uint8_t boot_mode;
    cy_en_app_status_t status;
    cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)&topology->access_param[0];

    *img_validity = 0u;

    status = Cy_HPI_Master_DevRegRead (&gl_HpiMasterCtx, access_param->hpi_i2c_param.slave_addr,
                                        CY_HPI_MASTER_DEV_REG_BOOT_MODE_REASON, &boot_mode,
                                        HPI_DEV_REG_BOOT_MODE_REASON_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        if (boot_mode & 0x04u)
        {
            *img_validity |= CY_APP_DMC_IMG_STATUS_INVALID;
        }
        else
        {
            *img_validity |= CY_APP_DMC_IMG_STATUS_VALID;
        }

        if (boot_mode & 0x08u)
        {
            *img_validity |= (CY_APP_DMC_IMG_STATUS_INVALID << 4u);
        }
        else
        {
            *img_validity |= (CY_APP_DMC_IMG_STATUS_VALID << 4u);
        }
    }
}

/**
 * @brief: Update dock metadata with DMC current running image, image validity and FW version.
 */
static void ccgx_init_dev_param (const dev_topology_t *topology, cy_stc_dmc_params_t *params)
{
    uint8_t img_status = 0x00;
    cy_en_dmc_image_type_t img_type;
    uint8_t fw_ver[CY_HPI_MASTER_DEV_REG_ALL_VERSION_BYTES] = {0};
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;
    uint32_t bl_ver, fw1_ver, fw2_ver;
    cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)&topology->access_param[0];
    cy_stc_app_dmc_dock_status_t* dock_status = Cy_App_Dmc_GetDockStatus();

    uint8_t intr  = CY_HPI_MASTER_INTR_REG_CLEAR_DEV_INT;
    /* Clear any pending interrupt. */
    Cy_HPI_Master_DevRegWrite(&gl_HpiMasterCtx,
                              access_param->hpi_i2c_param.slave_addr,
                              CY_HPI_MASTER_DEV_REG_INTR_ADDR, &intr,
                              HPI_DEV_REG_INTR_ADDR_LEN);

    /* Get CCGx current running image. */
    img_type = ccgx_get_cur_image(topology);

    /* Get CCGx images' validity. */
    ccgx_get_image_validity(topology, &img_status);

    /* Get CCGx FW version. */
    status = Cy_HPI_Master_DevRegRead(&gl_HpiMasterCtx,
                                      access_param->hpi_i2c_param.slave_addr,
                                      CY_HPI_MASTER_DEV_REG_ALL_VERSION_BYTE, fw_ver,
                                      CY_HPI_MASTER_DEV_REG_ALL_VERSION_BYTES);
    if (status == CY_APP_STAT_SUCCESS)
    {
        /* Update dock metadata with CCGx FW version. */
        bl_ver = (uint32_t) fw_ver;

        if ((img_status & CY_APP_DMC_IMG1_STATUS_MASK) == CY_APP_DMC_IMG_STATUS_VALID)
        {
            fw1_ver = (uint32_t) (fw_ver + CY_APP_DMC_FW_VERSION_SIZE);
        }
        else
        {
            /* Retain last good FW version. */
            fw1_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img1_version;
        }

        if (((img_status & CY_APP_DMC_IMG2_STATUS_MASK) >> 4) == CY_APP_DMC_IMG_STATUS_VALID)
        {
            fw2_ver = (uint32_t) (fw_ver + CY_APP_DMC_FW_VERSION_SIZE * 2);
        }
        else
        {
            /* Retain last good FW version. */
            fw2_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img2_version;
        }

    }
    else
    {
        /* Update dock metadata with CCGx invalid version. */
        img_type = img_status = 0u;
        fw1_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img1_version;
        fw2_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.img2_version;
        bl_ver = (uint32_t)dock_status->devx_status[topology->comp_id].fw_version.bl_version;
    }

    /* Update dock metadata with CCGx current runing image and image validity. */
    Cy_App_Dmc_SetRamImageStatus(topology->comp_id, img_type, img_status);

    /* Update dock metadata with CCGx FW version. */
    Cy_App_Dmc_UpdateRamVersions(topology->comp_id, (uint8_t *)bl_ver, (uint8_t *)fw1_ver, (uint8_t *)fw2_ver);
}

static void ccgx_jump_to_alternate(const dev_topology_t *topology, cy_stc_app_dmc_completion_cbk_t cb, cy_stc_dmc_params_t *params)
{
    cy_en_app_status_t status;
    uint8_t buffer = 0u;

    ccgx_update_init (topology, params);

    gl_ccgx_update.ccgx_update_started = 1;
    gl_ccgx_update.completion = cb;
    status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                       CY_HPI_MASTER_DEV_REG_PORT_ENABLE, &buffer,
                                       HPI_DEV_REG_PORT_ENABLE_LEN);
    if (status == CY_APP_STAT_SUCCESS)
    {
        gl_ccgx_update.next_hpi_cmd = CY_HPI_MASTER_DEV_REG_JUMP_TO_BOOT;
        gl_ccgx_update.cur_hpi_cmd = CY_HPI_MASTER_DEV_REG_PORT_ENABLE;
    }
}

/*! API to defer device status query during initialization sequence. */
static bool ccgx_is_dev_query_deferred(void)
{
    return true;
}

static void ccgx_config_hw_interface(const dev_topology_t *topology, cy_stc_dmc_params_t *params)
{
    cy_stc_app_dmc_dev_access_param_t *access_param = (cy_stc_app_dmc_dev_access_param_t *)&topology->access_param[0];
    uint8_t gpio_port = access_param->hpi_i2c_param.hpi_intr_gpio >> 4u;
    uint8_t gpio_pin = access_param->hpi_i2c_param.hpi_intr_gpio & 0x0Fu;

    GPIO_PRT_Type* gpio_base = gpio_base_arr[gpio_port];
    Cy_GPIO_Pin_FastInit(gpio_base, gpio_pin, CY_GPIO_DM_HIGHZ, 1u, HSIOM_SEL_GPIO);
    Cy_GPIO_SetInterruptEdge(gpio_base, gpio_pin, CY_GPIO_INTR_FALLING);

    Cy_HPI_Master_SlaveDeviceInit(&gl_HpiMasterCtx,
                                access_param->hpi_i2c_param.slave_addr,
                                gpio_port,
                                gpio_pin,
                                1u);
}

static bool ccgx_skip_jump_to_alt_request (uint8_t comp_id, cy_en_dmc_image_type_t cur_img)
{
    return false;
}

static bool ccgx_update_logic (uint8_t comp_id, cy_en_dmc_image_type_t img_type, bool *jump_to_alt_request)
{
    bool status = false;
    const dev_topology_t *topology = Cy_App_Dmc_GetDeviceTopology (comp_id);
    *jump_to_alt_request  = false;

    if (ccgx_get_cur_image(topology) != img_type)
    {
        status = true;
    }
    else
    {
        *jump_to_alt_request  = true;
    }

    return status;
}

/**
 * @brief HPI response event handler based on the response code data returned
 * from response register of CCGx HPI interface.
 */
bool ccg_handle_hpi_event(void *context, cy_hpi_master_event_t * event)
{
    cy_en_app_status_t status;
    bool clear_intr = true;
    uint8_t buffer[2];

    if((gl_ccgx_update.ccgx_update_started == 1) && (gl_ccgx_update.slave_addr == event->slaveDev->slaveAddr))
    {
        Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, CCGX_RESPONSE_WAIT_TIMER_ID);

        switch (event->eventCode)
        {
            case CY_HPI_MASTER_RESPONSE_SUCCESS:
                switch (gl_ccgx_update.cur_hpi_cmd)
                {
                    case CY_HPI_MASTER_DEV_REG_ENTER_FLASH_MODE:
                        ccgx_clear_pending_hpi_intr(CY_HPI_MASTER_INTR_REG_CLEAR_DEV_INT);
                        clear_intr = false;
                        status = ccgx_clear_flash_row(gl_ccgx_update.md_row_num);
                        if (status != CY_APP_STAT_SUCCESS)
                        {
                            report_cmd_completion(CY_APP_STAT_FAILURE);
                        }
                        break;

                    case CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE:
                        /* Call prepare_update_cb/ flash_write_cb() to complete
                         * the post-preparation/post-flashing operations needed
                         * for DMC state machine.
                         */
                        report_cmd_completion(CY_APP_STAT_SUCCESS);
                        break;

                    case CY_HPI_MASTER_DEV_REG_PORT_ENABLE:
                        ccgx_clear_pending_hpi_intr(CY_HPI_MASTER_INTR_REG_CLEAR_DEV_INT);
                        clear_intr = false;

                        /* Send HPI command to initiate CCGx device reset/ Jump
                         * to alternate image as is appropriate. */
                        if (gl_ccgx_update.next_hpi_cmd == CY_HPI_MASTER_DEV_REG_RESET_ADDR)
                        {
                            buffer[0] = CY_HPI_MASTER_RESET_CMD_SIG;
                            buffer[1] = CY_HPI_MASTER_RESET_DEVICE_RESET_CMD;

                            status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                        CY_HPI_MASTER_DEV_REG_RESET_ADDR, &buffer,
                                        HPI_DEV_REG_RESET_ADDR_LEN);
                        }
                        else if (gl_ccgx_update.next_hpi_cmd == CY_HPI_MASTER_DEV_REG_JUMP_TO_BOOT)
                        {
                            if(gl_ccgx_update.topology->image_mode != CY_APP_DMC_IMG_MODE_SINGLE_IMG)
                            {
                                buffer[0] = CY_HPI_MASTER_JUMP_TO_ALT_FW_SIG;
                            }
                            else
                            {
                                buffer[0] = CY_HPI_MASTER_JUMP_TO_BOOT_CMD_SIG;
                            }
                            status = ccgx_device_register_write(gl_ccgx_update.slave_addr,
                                        CY_HPI_MASTER_DEV_REG_JUMP_TO_BOOT, &buffer,
                                        HPI_DEV_REG_JUMP_TO_BOOT_LEN);
                        }
                        else
                        {
                            status = CY_APP_STAT_FAILURE;
                        }

                        if (status == CY_APP_STAT_SUCCESS)
                        {
                            gl_ccgx_update.cur_hpi_cmd = gl_ccgx_update.next_hpi_cmd;
                        }
                        break;

                    default:
                        break;
                }
                break;

            case CY_HPI_MASTER_RESPONSE_INVALID_ARGUMENT:
            case CY_HPI_MASTER_RESPONSE_INVALID_COMMAND:
            case CY_HPI_MASTER_RESPONSE_NOT_SUPPORTED:
            case CY_HPI_MASTER_RESPONSE_UNDEFINED_ERROR:
            case CY_HPI_MASTER_RESPONSE_CMD_ABORTED:
            case CY_HPI_MASTER_RESPONSE_FLASH_UPDATE_FAILED:
            case CY_HPI_MASTER_RESPONSE_INVALID_FW:
                switch (gl_ccgx_update.cur_hpi_cmd)
                {
                    case CY_HPI_MASTER_DEV_REG_ENTER_FLASH_MODE:
                    case CY_HPI_MASTER_DEV_REG_FLASH_READ_WRITE:
                    case CY_HPI_MASTER_DEV_REG_JUMP_TO_BOOT:
                    case CY_HPI_MASTER_DEV_REG_RESET_ADDR:
                    case CY_HPI_MASTER_DEV_REG_PORT_ENABLE:
                        /* Call prepare_update_cb/ flash_write_cb() to complete
                         * the post-preparation/ post-flashing operations
                         * needed for DMC state machine. */
                        report_cmd_completion(CY_APP_STAT_FAILURE);
                        break;

                    default:
                        break;
                }
                break;

            case CY_HPI_MASTER_EVENT_RESET_COMPLETE:
                switch (gl_ccgx_update.cur_hpi_cmd)
                {
                    case CY_HPI_MASTER_DEV_REG_RESET_ADDR:
                        ccgx_clear_pending_hpi_intr(CY_HPI_MASTER_INTR_REG_CLEAR_DEV_INT);
                        clear_intr = false;
                        gl_ccgx_update.ccgx_update_started = 0;
                        report_cmd_completion(CY_APP_STAT_SUCCESS);
                        break;

                    case CY_HPI_MASTER_DEV_REG_JUMP_TO_BOOT:
                    {
                        uint8_t dev_mode;

                        ccgx_clear_pending_hpi_intr(CY_HPI_MASTER_INTR_REG_CLEAR_DEV_INT);
                        clear_intr = false;

                        /* CY_HPI_MASTER_EVENT_RESET_COMPLETE is s sent by CCGx (HPIv2) Jump_to_alternate
                         * mode command from the I2C bootloader of CCGx and from application image.
                         * Read device mode to understand if the CCGx device is bootloader. If in
                         * bootloader, do not initiate FW update.
                         */
                        status = Cy_HPI_Master_DevRegRead(&gl_HpiMasterCtx, gl_ccgx_update.slave_addr,
                                    CY_HPI_MASTER_DEV_REG_DEVICE_MODE, &dev_mode,
                                    HPI_DEV_REG_DEVICE_MODE_LEN);
                        if (status == CY_APP_STAT_SUCCESS)
                        {
                            if(gl_ccgx_update.topology->image_mode != CY_APP_DMC_IMG_MODE_SINGLE_IMG)
                            {
                                if (dev_mode & 0x03)
                                {
                                    /* Jump to alternate image complete. CCGx is in
                                     * firmware mode. Initiate FW update. */
                                    /* Call ccgx_jump_to_cb to complete the post-jump
                                     * to alternate image operation needed for DMC state machine. */
                                    if (gl_ccgx_update.finish_update_reset_flag == true)
                                    {
                                        gl_ccgx_update.finish_update_reset_flag = false;
                                        gl_ccgx_update.ccgx_update_started = 0;
                                        report_cmd_completion(CY_APP_STAT_SUCCESS);
                                    }
                                    else
                                    {
                                        gl_ccgx_update.ccgx_update_started = 0;
                                        report_cmd_completion(CY_APP_STAT_SUCCESS);
                                    }
                                }
                                else
                                {
                                    /* Jump to alternate image in process. CCGx is in
                                     * Bootloader mode. Disable and enable BB again. */
                                }
                            }
                            else
                            {
                                gl_ccgx_update.ccgx_update_started = 0;
                                report_cmd_completion(CY_APP_STAT_SUCCESS);
                            }
                        }
                        else
                        {
                            report_cmd_completion(CY_APP_STAT_FAILURE);
                        }
                        break;
                    }
                }
                break;

            default:
                break;
        }
    }

    return clear_intr;
}

/* [EOF] */
