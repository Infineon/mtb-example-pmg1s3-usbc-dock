/******************************************************************************
 * File Name: etag.c
 *
 * Description: This is source code for the PMG1S3 Dock Solution.
 *              This file implements functions for handling the storage and retrieval
 *              of ETAG information.
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "etag.h"
#include "spi_eeprom_master.h"
#include "cy_app_spi_comp_update.h"

#if ETAG_SUPPORT_ENABLE

cy_en_app_status_t read_etag_information(uint8_t *buffer)
{
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;

    if(spi_eeprom_read_flash_with_retry (buffer, ETAG_INFORMATION_SIZE, CY_APP_DMC_SPI_METADATA_ADDRESS) == CY_APP_STAT_SUCCESS)
    {
           status = CY_APP_STAT_SUCCESS;
    }
    return status;
}

cy_en_app_status_t write_etag_information(uint8_t *report)
{
    uint8_t timeout = SPI_IDLE_WIP_TIMEOUT, eeprom_status = SPI_WIP;
    uint8_t buffer[ETAG_INFORMATION_SIZE], read_buffer[ETAG_INFORMATION_SIZE];
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;

    status = read_etag_information(buffer);

    if(buffer[ETAG_VALID_OFFSET] == ETAG_VALID)
    {
        /* ETAG is already programmed */
        return CY_APP_STAT_INVALID_COMMAND;
    }

    if(status != CY_APP_STAT_FAILURE)
    {
        buffer[ETAG_VALID_OFFSET] = ETAG_VALID;
        memcpy(&buffer[2], &report[2], ETAG_DATA_SIZE);

        /* Write enable command for 4k sector erase operation */
        if (spi_eeprom_write_enable (true) == CY_APP_STAT_SUCCESS)
        {
            /* Erase before writing data */
            if (spi_eeprom_4k_sector_erase(CY_APP_DMC_SPI_METADATA_ADDRESS) == CY_APP_STAT_SUCCESS)
            {
                /* Wait till erase is complete */
                while ((SPI_EEPROM_IS_WRITE_IN_PROGRESS(eeprom_status)) && (timeout != 0))
                {
                    Cy_SysLib_Delay (SPI_READ_DELAY_MS);
                    spi_eeprom_read_status_reg(&eeprom_status);
                    timeout--;
                }
                /* Write enable for page program command */
                if (spi_eeprom_write_enable (true) == CY_APP_STAT_SUCCESS)
                {
                    /* Write data to flash */
                    if (spi_eeprom_write_flash (buffer, ETAG_INFORMATION_SIZE, CY_APP_DMC_SPI_METADATA_ADDRESS) == CY_APP_STAT_SUCCESS)
                    {
                        /* Wait till the write is complete */
                        while ((SPI_EEPROM_IS_WRITE_IN_PROGRESS(eeprom_status)) && (timeout != 0))
                        {
                            Cy_SysLib_Delay (SPI_READ_DELAY_MS);
                            spi_eeprom_read_status_reg(&eeprom_status);
                            timeout--;
                        }
                        /* Check if the data written and read back are same */
                        if (read_etag_information(read_buffer) == CY_APP_STAT_SUCCESS)
                        {
                            if (memcmp(buffer, read_buffer, ETAG_INFORMATION_SIZE) == 0)
                            {
                                return CY_APP_STAT_SUCCESS;
                            }
                        }
                    }
                }
            }

        }

    }
    return CY_APP_STAT_FAILURE;
}
#endif /* ETAG_SUPPORT_ENABLE */
