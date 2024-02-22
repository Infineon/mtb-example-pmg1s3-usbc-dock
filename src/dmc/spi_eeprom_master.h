/******************************************************************************
 * File Name: spi_eeprom_master.h
 *
 * Description: Header file for SPI master interface to SPI EEPROM slave.
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

#ifndef _SPI_EEPROM_MASTER_H_
#define _SPI_EEPROM_MASTER_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_app_status.h"
#include "config.h"

/*****************************************************************************
 ********************************* Macros ************************************
 *****************************************************************************/

/*
 * SPI EEPROM Status register bit mask
 */

/* Write In Progress bit. */
#define SPI_EEPROM_STAT_REG_WIP         (1 << 0)
/* Write Enable latch. */
#define SPI_EEPROM_STAT_REG_WEL         (1 << 1)

/* Macro which sets BIT7 in STATUS Register, so that the write to the SRWD
 * is disabled.
 */
#define SPI_EEPROM_STAT_REG_WR_DISABLE  ((1 << 7))

/* Macro which enables protection to all blocks of the SPI EEPROM
*/
#define SPI_EEPROM_PROT_ALL_BLOCKS      (0xF << 2)

#define SPI_EEPROM_IS_WRITE_IN_PROGRESS(status) (((status) & SPI_EEPROM_STAT_REG_WIP) != 0)

#define SPI_EEPROM_IS_WRITE_ENABLED(status)     (((status) & SPI_EEPROM_STAT_REG_WEL) != 0)

#define SPI_EEPROM_RETRY_COUNT          (3u)

#define SPI_READ_DELAY_MS               (2)

#define SPI_WIP                         (0x03)

#define SPI_IDLE_WIP_TIMEOUT            (100u)

/*****************************************************************************
 ********************************** Datatypes ********************************
 *****************************************************************************/

/* Helper Macros */
#define CMD_LEN_0BYTE                                   (0u)
#define CMD_LEN_1BYTE                                   (1u)
#define CMD_LEN_4BYTE                                   (4u)

#define RD_STATUS_DATA_LEN                              (1u)
#define RD_CONFIG_DATA_LEN                              (1u)
#define WR_STATUS_DATA_LEN                              (2u)

/**
 * @brief SPI FLASH Commands
 *
 * Enumeration to hold SPI flash instruction IDs.
 */
typedef enum spi_flash_cmd
{
    FLASH_WRITE_STATUS_CFG = 1,
    FLASH_WRITE_DATA = 2,
    FLASH_READ_DATA = 3,
    FLASH_WRITE_DISABLE = 4,
    FLASH_READ_STATUS = 5,
    FLASH_WRITE_ENABLE = 6,
    FLASH_READ_STATUS_2 = 7,
    FLASH_4K_SECTOR_ERASE = 0x20,
    FLASH_READ_CONFIG = 0x35,
    FLASH_32K_BLOCK_ERASE = 0x52,
    FLASH_64K_BLOCK_ERASE = 0xD8,
    FLASH_CHIP_ERASE = 0x60,
    FLASH_CHIP_ERASE_ALT = 0xC7,
    FLASH_RDID = 0x9F
}spi_flash_cmd_t;

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/
/**
 * @brief Initialize SPI communication.
 *
 * @param scb_num Base address of SCB.
 *
 * @return none
 */
void spi_eeprom_init(void);

/**
 * @brief Read SPI EEPROM RDID register.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param data Pointer to EEPROM RDID register value.
 * @param data_len Number of bytes to be read from teh register.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed
 */
cy_en_app_status_t spi_eeprom_rdid_reg(uint8_t *data, uint8_t data_len);

/**
 * @brief Read SPI EEPROM status register.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param eeprom_status Pointer to EEPROM status.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed
 */
cy_en_app_status_t spi_eeprom_read_status_reg(uint8_t *eeprom_status);

/**
 * @brief Read SPI EEPROM status register 2
 * Supported in Spansion flashes (S25FL064L). RDSR2 command is to be sent to read status register 2 Volatile (SR2V).
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param eeprom_status Pointer to EEPROM status.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed
 */
cy_en_app_status_t spi_eeprom_read_status_2_reg(uint8_t *eeprom_status);

/**
 * @brief Read SPI EEPROM configuration register
 * Supported in Spansion flashes (S25FL064L). RDCR1 command is to be sent to read Comfiguration register 1 Volatile (CR1V).
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param config_value Pointer to EEPROM configuration register value.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed
 */
cy_en_app_status_t spi_eeprom_read_config_reg(uint8_t *config_value);

/**
 * @brief Write SPI EEPROM status/configuration register.
 * Status/Configuration register is written to set/clear HPM in the EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 *
 * @param srwd_block_write_prot_en  This parameter indicates if SRWD needs to
 *                                  be set and all blocks needs to be
 *                                  protected in the SPI Flash. This will
 *                                  prepare the SPI Flash for Hardware
 *                                  Protection Mode.
 * Notes:
 * 1. EEPROM write protect GPIO is not handled in this function.
 * 2. The value of status register is different for different flashes.
 *    So this API needs to be changed accordingly to suite the EEPROM requirements.
 * 3. spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_write_status_reg(bool srwd_block_write_prot_en);

/**
 * @brief Enable/disable write access to EEPROM. This is software based write enable.
 * When called with enable param as "true", it sets the Write Enable Latch (WEL)
 * bit of the EEPROM Status Register. It must be so called to enable write, program and
 * erase commands.
 * When called with enable param as "false", it clears the Write Enable Latch (WEL)
 * bit of the EEPROM Status Register.
 * Note: WEL bit is cleared automatcially after successful erase or write or program operation.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param enable Indicates write enable/disable.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_write_enable(bool enable);

/**
 * @brief Reads data from SPI EEPROM with retries on a failure.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param buffer Buffer to store data.
 * @param size Size of data to be read.
 * @param page_addr Page address from where data is to be read.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed.
 */
cy_en_app_status_t spi_eeprom_read_flash_with_retry(uint8_t *buffer, uint16_t size, uint32_t page_addr);

/**
 * @brief Reads data from SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param buffer Buffer to store data.
 * @param size Size of data to be read.
 * @param page_addr Page address from where data is to be read.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success, CY_APP_STAT_FAILURE if failed.
 */
cy_en_app_status_t spi_eeprom_read_flash(uint8_t *buffer, uint16_t size, uint32_t page_addr);

/**
 * @brief Writes data to SPI EEPROM with retries on failure.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param buffer Buffer containing data to be written.
 * @param size Size of data to be written.
 * @param page_addr Page address from where data is to be written.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_write_flash_with_retry(uint8_t *buffer, uint16_t size, uint32_t page_addr);

/**
 * @brief Writes data to SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param buffer Buffer containing data to be written.
 * @param size Size of data to be written.
 * @param page_addr Page address from where data is to be written.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_write_flash(uint8_t *buffer, uint16_t size, uint32_t page_addr);

/**
 * @brief Erase 64 KB block of SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param flash_addr Address of the start of the block to be erased.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_64k_block_erase(uint32_t flash_addr);

/**
 * @brief Erase 32 KB block of SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param flash_addr Address of the start of the block to be erased.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_32k_block_erase(uint32_t flash_addr);

/**
 * @brief Erase 4 KB sector of SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 * @param flash_addr Address of the start of the sector to be erased.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_4k_sector_erase(uint32_t flash_addr);

/**
 * @brief Erase entire chip of SPI EEPROM.
 *
 * @param scb_index SCB block index. Caller should ensure parameter validity.
 *
 * Note: spi_eeprom_write_enable must be called before calling this function.
 *
 * @return cy_en_app_status_t: CY_APP_STAT_SUCCESS if success.
 */
cy_en_app_status_t spi_eeprom_chip_erase(void);

cy_en_app_status_t spi_master_read_write_array(CySCB_Type* scb_num, uint8_t *wr_buf, uint8_t *rd_buf,
        uint16_t size, uint8_t *cmd_buff, uint8_t cmd_size);

cy_en_app_status_t spi_master_wait_for_idle(CySCB_Type* scb_num);

#endif /* _SPI_EEPROM_MASTER_H_ */
