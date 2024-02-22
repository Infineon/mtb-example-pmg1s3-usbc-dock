/******************************************************************************
* File Name: dmc_solution.h
*
* Description: Header file for DMC's solution layer.
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

#ifndef _DMC_SOLUTION_H_
#define _DMC_SOLUTION_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_app_status.h"
#include "config.h"
#include "cy_app_dmc_common.h"
#include "cy_app_flash.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/*****************************************************************************
 ********************************* Data Types ********************************
 *****************************************************************************/


/*****************************************************************************
 ************************ Global variable declarations ***********************
 *****************************************************************************/

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

/**
 * @brief Initializes the devices available in the Dock
 * @param None
 * @return None.
 */
void dmc_init(const cdtt_config_t *ptrCdttCfg, const sec_config_t *ptrSecCfg,
              const cy_stc_dmc_app_cbk_t *ptrAppCbk, cy_stc_dmc_params_t *params);

/**
 * @brief Determine if update is needed based on the FW version of image entry
 *        in FWCT and the existing image entry in dock metadata.
 *
 * @param img_version   Image version retrieved from the FWCT
 * @param img_type      Indicates whether the check needs to be done for the
 *                      cy_en_dmc_image_type_t
 * @param comp_id       Component Id in the Dock
 * @param check_last_valid
 *
 * @return True When the update is required
 * @return False When the update is not required
 */
bool dmc_check_fw_version(uint8_t *img_version, cy_en_dmc_image_type_t img_type, uint8_t comp_id, bool check_last_valid);

/**
 * @brief Initiates All devices reset: US CCGx reset; then initiates DMC soft
 *        reset.
 *
 * @param None.
 *
 * @return None.
 */
void init_dock_reset(void);

bool is_dmc_in_factory_condition (void);

/**
 * @brief Checks if the DMC/CCGx need to service the jump to alternate image
 *        request.
 *
 * @param comp_id Each device in dock has a unique component id
 * @param cur_img Type of the Image for a given component id
 *
 * @return True if the jump to alternate image request is to be serviced.
 * @return False if the jump to alternate image request is not to be serviced.
 */
bool dmc_skip_jump_to_alt_request (uint8_t comp_id, cy_en_dmc_image_type_t cur_img);

const cdtt_config_t * get_cdtt_config(void);

const smart_power_config_t* get_smart_power_config(void);

const sec_config_t * get_sec_config(void);

void dmc_internal_flash_enter_mode(bool is_enable, bool data_in_place);

cy_en_app_status_t dmc_internal_flash_row_write(uint16_t row_num, uint8_t *data);

cy_en_app_status_t dmc_spi_flash_write_enable (bool enable);

cy_en_app_status_t dmc_spi_flash_write (uint8_t *buffer, uint16_t size, uint32_t page_addr, bool retry);

cy_en_app_status_t dmc_spi_flash_read (uint8_t *buffer, uint16_t size, uint32_t page_addr, bool retry);

cy_en_app_status_t dmc_spi_flash_erase (uint32_t flash_addr, cy_en_spi_flash_erase_t size);

bool dmc_spi_flash_is_write_in_progress (void);

void app_fw_update_complete_handler (void);

void dmc_led_set_mode(cy_en_dmc_led_mode_t mode);

void dmc_app_event_handler(cy_en_dmc_app_evt_t evt, uint8_t *data, uint8_t size);

#endif  /* _DMC_SOLUTION_H_ */

/* [] END OF FILE */
