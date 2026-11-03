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
