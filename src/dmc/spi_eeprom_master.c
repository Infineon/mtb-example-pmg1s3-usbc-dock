/******************************************************************************
 * File Name: spi_eeprom_master.c
 *
 * Description: Source file for SPI master interface to SPI EEPROM slave.
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
#include "cy_app_status.h"
#include "cy_sysint.h"
#include "cy_scb_spi.h"
#include "spi_eeprom_master.h"

/**
 * @brief Maximum size of SPI command.
 */
#define SPI_FLASH_CMD_MAX_SIZE                          (4)

#define SPI_IDLE_WAIT_TIMEOUT                           (0xFFFF)
#define SPIM_BUS_TIMEOUT                                (20000)

static cy_stc_scb_spi_context_t  flash_spi_context;

const cy_stc_sysint_t FLASH_SPI_IRQ_config = {
    .intrSrc = (IRQn_Type) FLASH_SPI_IRQ,
    .intrPriority = 3u
};

void flash_spi_InterruptHandler(void);

void flash_spi_InterruptHandler (void)
{
    Cy_SCB_SPI_Interrupt(FLASH_SPI_HW, &flash_spi_context);
}

void spi_eeprom_init(void)
{
    cy_rslt_t result;

    result = Cy_SCB_SPI_Init(FLASH_SPI_HW, &FLASH_SPI_config, &flash_spi_context);
    if( result != CY_SCB_SPI_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = Cy_SysInt_Init(&FLASH_SPI_IRQ_config, &flash_spi_InterruptHandler);
    if(result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(FLASH_SPI_IRQ);

    /* Enable the SPI Master block */
    Cy_SCB_SPI_Enable(FLASH_SPI_HW);
}

cy_en_app_status_t spi_eeprom_rdid_reg(uint8_t *data, uint8_t data_len)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    /* Create RDID command packet. */
    cmd_pkt[0] = FLASH_RDID;

    Cy_SCB_SPI_ClearRxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array (FLASH_SPI_HW, NULL, data, data_len, cmd_pkt, CMD_LEN_1BYTE);

    return status;
}

cy_en_app_status_t spi_eeprom_read_status_reg(uint8_t *eeprom_status)
{
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create READ_STATUS command packet. */
    cmd_pkt[0] = FLASH_READ_STATUS;

    Cy_SCB_SPI_ClearRxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array (FLASH_SPI_HW, NULL, eeprom_status, RD_STATUS_DATA_LEN, cmd_pkt, CMD_LEN_1BYTE);

    return status;
}

cy_en_app_status_t spi_eeprom_read_status_2_reg(uint8_t *eeprom_status)
{
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create READ_STATUS command packet. */
    cmd_pkt[0] = FLASH_READ_STATUS_2;

    Cy_SCB_SPI_ClearRxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array (FLASH_SPI_HW, NULL, eeprom_status, RD_STATUS_DATA_LEN, cmd_pkt, CMD_LEN_1BYTE);

    return status;
}

cy_en_app_status_t spi_eeprom_read_config_reg(uint8_t *config_value)
{
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create READ_CONFIG command packet. */
    cmd_pkt[0] = FLASH_READ_CONFIG;

    Cy_SCB_SPI_ClearRxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array (FLASH_SPI_HW, NULL, config_value, RD_CONFIG_DATA_LEN, cmd_pkt, CMD_LEN_1BYTE);

    return status;
}

cy_en_app_status_t spi_eeprom_write_status_reg(bool srwd_block_write_prot_en)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create WRSR (Write Status Register) command packet. */
    cmd_pkt[0] = FLASH_WRITE_STATUS_CFG;
    cmd_pkt[1] = 0;

    if(srwd_block_write_prot_en)
    {
        /* When write protect to be enabled perform the following in
         * the Status Register
         *
         * SRWD[7] = 1
         * BP[3:0]: bit[5:2] = b1111
         */
        cmd_pkt[1] |= (SPI_EEPROM_STAT_REG_WR_DISABLE |
                SPI_EEPROM_PROT_ALL_BLOCKS);
    }

    spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, WR_STATUS_DATA_LEN);
    spi_master_wait_for_idle(FLASH_SPI_HW);

    return CY_APP_STAT_SUCCESS;
}

cy_en_app_status_t spi_eeprom_write_enable (bool enable)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    if (enable)
    {
        /* Create WRITE_ENABLE command packet. */
        cmd_pkt[0] = FLASH_WRITE_ENABLE;
    }
    else
    {
        /* Create WRITE_DISABLE command packet. */
        cmd_pkt[0] = FLASH_WRITE_DISABLE;
    }

    status = spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, CMD_LEN_1BYTE);
    if (status == CY_APP_STAT_SUCCESS)
    {
        status = spi_master_wait_for_idle(FLASH_SPI_HW);
    }

    return status;
}

cy_en_app_status_t spi_eeprom_read_flash_with_retry(uint8_t *buffer, uint16_t size, uint32_t page_addr)
{
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;
    uint8_t retry_cnt = SPI_EEPROM_RETRY_COUNT;

    while ((status != CY_APP_STAT_SUCCESS) && (retry_cnt))
    {
        if (retry_cnt != SPI_EEPROM_RETRY_COUNT)
        {
            Cy_SysLib_Delay (SPI_READ_DELAY_MS);
        }
        status = spi_eeprom_read_flash (buffer, size, page_addr);
        retry_cnt--;
    }

    return status;
}

/**
 * @brief Reads data in SPI flash.
 *
 * This function reads data from SPI flash
 *
 * @param page_addr Page address of SPI flash row to be read
 * @param buffer Ponter to buffer holding data read from SPI flash.
 * @return CY_APP_STAT_SUCCESS if successfully reads flash row , CY_APP_STAT_FAILURE otherwise.
 */
cy_en_app_status_t spi_eeprom_read_flash(uint8_t *buffer, uint16_t size, uint32_t page_addr)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;
    /* Create READ_DATA command packet. */
    cmd_pkt[0] = FLASH_READ_DATA;
    cmd_pkt[1] = (page_addr >> 16) & 0xFF;
    cmd_pkt[2] = (page_addr >> 8) & 0xFF;
    cmd_pkt[3] = (page_addr & 0xFF);

    Cy_SCB_SPI_ClearRxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array (FLASH_SPI_HW, NULL, buffer, size, cmd_pkt, CMD_LEN_4BYTE);

    return status;
}

cy_en_app_status_t spi_eeprom_write_flash_with_retry(uint8_t *buffer, uint16_t size, uint32_t page_addr)
{
    cy_en_app_status_t status = CY_APP_STAT_FAILURE;
    uint8_t retry_cnt = SPI_EEPROM_RETRY_COUNT;

    while ((status != CY_APP_STAT_SUCCESS) && (retry_cnt))
    {
        if (retry_cnt != SPI_EEPROM_RETRY_COUNT)
        {
            Cy_SysLib_Delay (SPI_READ_DELAY_MS);
        }
        status = spi_eeprom_write_flash (buffer, size, page_addr);
        retry_cnt--;
    }

    return status;
}

cy_en_app_status_t spi_eeprom_write_flash(uint8_t *buffer, uint16_t size, uint32_t page_addr)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    /* Create WRITE_DATA command packet. */
    cmd_pkt[0] = FLASH_WRITE_DATA;
    cmd_pkt[1] = (page_addr >> 16) & 0xFF;
    cmd_pkt[2] = (page_addr >> 8) & 0xFF;
    cmd_pkt[3] = (page_addr & 0xFF);

    Cy_SCB_SPI_ClearTxFifo(FLASH_SPI_HW);

    status = spi_master_read_write_array(FLASH_SPI_HW, buffer, NULL, size, cmd_pkt, CMD_LEN_4BYTE);
    if (status == CY_APP_STAT_SUCCESS)
    {
        status = spi_master_wait_for_idle(FLASH_SPI_HW);
    }

    return status;
}

cy_en_app_status_t spi_eeprom_32k_block_erase(uint32_t flash_addr)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create 32K Block Erase command packet. */
    cmd_pkt[0] = FLASH_32K_BLOCK_ERASE;
    cmd_pkt[1] = (flash_addr >> 16) & 0xFF;
    cmd_pkt[2] = (flash_addr >> 8) & 0xFF;
    cmd_pkt[3] = (flash_addr & 0xFF);

    spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, CMD_LEN_4BYTE);
    spi_master_wait_for_idle(FLASH_SPI_HW);

    return CY_APP_STAT_SUCCESS;
}

cy_en_app_status_t spi_eeprom_4k_sector_erase(uint32_t flash_addr)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create 4K Block Erase command packet. */
    cmd_pkt[0] = FLASH_4K_SECTOR_ERASE;
    cmd_pkt[1] = (flash_addr >> 16) & 0xFF;
    cmd_pkt[2] = (flash_addr >> 8) & 0xFF;
    cmd_pkt[3] = (flash_addr & 0xFF);

    spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, CMD_LEN_4BYTE);
    spi_master_wait_for_idle(FLASH_SPI_HW);

    return CY_APP_STAT_SUCCESS;
}

cy_en_app_status_t spi_eeprom_64k_block_erase(uint32_t flash_addr)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;

    /* Create 4K Block Erase command packet. */
    cmd_pkt[0] = FLASH_64K_BLOCK_ERASE;
    cmd_pkt[1] = (flash_addr >> 16) & 0xFF;
    cmd_pkt[2] = (flash_addr >> 8) & 0xFF;
    cmd_pkt[3] = (flash_addr & 0xFF);

    status = spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, CMD_LEN_4BYTE);
    spi_master_wait_for_idle(FLASH_SPI_HW);

    return status;
}

cy_en_app_status_t spi_eeprom_chip_erase(void)
{
    uint8_t cmd_pkt[SPI_FLASH_CMD_MAX_SIZE];

    /* Create Chip Erase command packet. */
    cmd_pkt[0] = FLASH_CHIP_ERASE;

    spi_master_read_write_array(FLASH_SPI_HW, NULL, NULL, 0, cmd_pkt, CMD_LEN_1BYTE);
    spi_master_wait_for_idle(FLASH_SPI_HW);

    return CY_APP_STAT_SUCCESS;
}

cy_en_app_status_t spi_master_read_write_array(CySCB_Type* scb_num, uint8_t *wr_buf, uint8_t *rd_buf,
        uint16_t size, uint8_t *cmd_buf, uint8_t cmd_size)
{
    cy_en_app_status_t status = CY_APP_STAT_SUCCESS;
    cy_en_scb_spi_status_t masterStatus;
    uint32_t timeout;

    if ((cmd_size == 0) && (size == 0))
    {
        return CY_APP_STAT_INVALID_ARGUMENT;
    }

    timeout = SPIM_BUS_TIMEOUT;
    /* Wait for the data to be drained out. */
    while (Cy_SCB_SPI_IsBusBusy(scb_num) == true)
    {
        if (--timeout <= 0)
        {
            status = CY_APP_STAT_FAILURE;
            break;
        }
        Cy_SysLib_DelayUs(1);
    }

#if(SPI_CS_MANUAL == 1)
    if (status == CY_APP_STAT_SUCCESS)
    {
        Cy_GPIO_Write(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, 0u);
    }
#endif

    if(cmd_buf != NULL)
    {
        timeout = SPIM_BUS_TIMEOUT;
        masterStatus = Cy_SCB_SPI_Transfer(scb_num, cmd_buf, NULL, cmd_size, &flash_spi_context);
        if(masterStatus == CY_SCB_SPI_SUCCESS)
        {
            /* Blocking wait for transfer completion */
            while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(scb_num, &flash_spi_context)))
            {
                if (--timeout <= 0)
                {
                    Cy_SCB_SPI_AbortTransfer(scb_num, &flash_spi_context);
                    status = CY_APP_STAT_TIMEOUT;
                    break;
                }
                Cy_SysLib_DelayUs(1u);
            }
        }
        else
        {
            status = CY_APP_STAT_FAILURE;
        }
    }
    if((status == CY_APP_STAT_SUCCESS) && ((wr_buf != NULL) || (rd_buf != NULL)))
    {
        timeout = SPIM_BUS_TIMEOUT;
        masterStatus = Cy_SCB_SPI_Transfer(scb_num, wr_buf, rd_buf, size, &flash_spi_context);
        if(masterStatus == CY_SCB_SPI_SUCCESS)
        {
            /* Blocking wait for transfer completion */
            while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE & Cy_SCB_SPI_GetTransferStatus(scb_num, &flash_spi_context)))
            {
                if (--timeout <= 0)
                {
                    Cy_SCB_SPI_AbortTransfer (scb_num, &flash_spi_context);
                    status = CY_APP_STAT_TIMEOUT;
                    break;
                }
                Cy_SysLib_DelayUs(1u);
            }
        }
        else
        {
            status = CY_APP_STAT_FAILURE;
        }
    }

    timeout = SPIM_BUS_TIMEOUT;
    /* Wait for the data to be drained out. */
    while (Cy_SCB_SPI_IsBusBusy(scb_num) == true)
    {
        if (--timeout <= 0)
        {
            status = CY_APP_STAT_FAILURE;
            break;
        }
        Cy_SysLib_DelayUs(1);
    }

#if(SPI_CS_MANUAL == 1)
    Cy_GPIO_Write(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, 1u);
#endif

    return status;
}

cy_en_app_status_t spi_master_wait_for_idle(CySCB_Type* scb_num)
{
    uint16_t timeout;
    for (timeout = 0; timeout < SPI_IDLE_WAIT_TIMEOUT; timeout++)
    {
        if(Cy_SCB_SPI_IsBusBusy(scb_num) == false)
        {
            break;
        }
        Cy_SysLib_DelayUs(10);
    }
    
    Cy_SysLib_DelayUs(50);

    if(timeout < SPI_IDLE_WAIT_TIMEOUT)
    {
        return CY_APP_STAT_SUCCESS;
    }
    else
    {
        return CY_APP_STAT_TIMEOUT;
    }
}

/* [] END OF FILE */
