/*
 * File: c:\Users\Brandon\Desktop\OpenQuad2\fw\oq2\Drivers\QSPI_FLASH\qspi_flash.c                 /
 * Project: OQ2                                                                                    /
 * Created Date: Sunday, January 31st 2021, 7:07:06 am                                             /
 * Author: Brandon Riches                                                                          /
 * Email: richesbc@gmail.com                                                                       /
 * -----                                                                                           /
 *                                                                                                 /
 * Copyright (c) 2020 OpenQuad2.                                                                   /
 * All rights reserved.                                                                            /
 *                                                                                                 /
 * Redistribution and use in source or binary forms, with or without modification,                 /
 * are not permitted without express written approval of OpenQuad2                                 /
 * -----                                                                                           /
 * HISTORY:                                                                                        /
*/


#include "qspi_flash.h"
#include "main.h"
#include "S25FL256S.h"
#include "FreeRTOS.h"
#include "string.h"

static uint8_t TEST_BUFFER[] = "****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication**** ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication**** ****Memory-mapped OSPI communication****  ****Memory-mapped OSPI communication**** ";
static uint8_t rx_buffer[512] = { 0 };

#include "debug_log.h"
#define debug_error(fmt, ...)           debug_error(QSPI_FLASH_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_printf(fmt, ...)          debug_printf(QSPI_FLASH_MODULE_ID, fmt, ##__VA_ARGS__)
#define debug_print_buffer(fmt, ...)    debug_print_buffer(QSPI_FLASH_MODULE_ID, fmt, ##__VA_ARGS__)

static OSPI_HandleTypeDef* m_ospi = NULL;

static OSPI_RegularCmdTypeDef m_generic_cmd = { 0 };


void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef* hospi)
{
    debug_error("Error!");
}

// void HAL_OSPI_AbortCpltCallback(OSPI_HandleTypeDef* hospi);
// void HAL_OSPI_FifoThresholdCallback(OSPI_HandleTypeDef* hospi);

// /* OSPI indirect mode functions */
void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef* hospi)
{
    debug_printf("command complete.");
}
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef* hospi)
{
    debug_printf("Rx complete!");
}
// void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef* hospi);
// void HAL_OSPI_RxHalfCpltCallback(OSPI_HandleTypeDef* hospi);
// void HAL_OSPI_TxHalfCpltCallback(OSPI_HandleTypeDef* hospi);
/* OSPI status flag polling mode functions */
void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef* hospi)
{
    debug_printf("Match!");
}
/* OSPI memory-mapped mode functions */
void HAL_OSPI_TimeOutCallback(OSPI_HandleTypeDef* hospi)
{
    debug_printf("Timeout!");
}

/**
 * @brief Set default values in m_generic_cmd. Use at the begining of each transaction.
 *
 */
static void _reset_generic_cmd()
{
    m_generic_cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    m_generic_cmd.FlashId = HAL_OSPI_FLASH_ID_1;
    m_generic_cmd.Instruction = 0;
    m_generic_cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    m_generic_cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    m_generic_cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    m_generic_cmd.Address = 0;
    m_generic_cmd.AddressMode = HAL_OSPI_ADDRESS_NONE;
    m_generic_cmd.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
    m_generic_cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    m_generic_cmd.AlternateBytes = 0;
    m_generic_cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    m_generic_cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
    m_generic_cmd.AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
    m_generic_cmd.DataMode = HAL_OSPI_DATA_NONE;
    m_generic_cmd.NbData = 0;
    m_generic_cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    m_generic_cmd.DummyCycles = 0;
    m_generic_cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
    m_generic_cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

/**
 * @brief Configure auto polling for the Status Register 1.
 *
 * Checks for (SR1 & match_mask) == match_value
 *
 * @param match_value The desired value
 * @param match_mask The bit mask
 * @return bool
 */
static bool qf_poll_sr1_for_match(uint8_t match_value, uint8_t match_mask)
{
    _reset_generic_cmd();

    m_generic_cmd.Instruction = CMD_READ_STATUS_REG1;
    m_generic_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    m_generic_cmd.NbData = 1;

    HAL_StatusTypeDef stat = HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    if (stat != HAL_OK)
        return false;

    OSPI_AutoPollingTypeDef poll_config;
    poll_config.Match = match_value;
    poll_config.Mask = match_mask;
    poll_config.MatchMode = HAL_OSPI_MATCH_MODE_AND;
    poll_config.Interval = 0x5;
    poll_config.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

    stat = HAL_OSPI_AutoPolling(m_ospi, &poll_config, 100000);

    if (stat != HAL_OK)
        return false;
    else
    {
        s25fl_status1_reg_t* stat1 = (s25fl_status1_reg_t*)&m_ospi->Instance->DR;
        // debug_printf("SR1 value: 0x%02X", *((uint8_t*) stat1));
        return true;
    }
}


/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hospi: OSPI handle
  * @retval None
  */
static bool qf_write_enable()
{
    _reset_generic_cmd();

    /* Enable write operations ------------------------------------------ */
    m_generic_cmd.Instruction = CMD_WREN;

    HAL_StatusTypeDef stat = HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

    if (stat != HAL_OK)
    {
        debug_error("Couldn't send WREN command. stat = %u", stat);
        return false;
    }

    return qf_poll_sr1_for_match(MATCH_WRITE_ENABLED, BITMASK_WRITE_ENABLE_LATCH);
}

/**
 * @brief Read the JEDEC ID in the flash to verify communication and flash parameters
 *
 */
static bool qf_read_jedec(void)
{
    _reset_generic_cmd();

    /* Initialize the Write Enable cmd in single SPI mode */
    m_generic_cmd.Instruction = CMD_READ_JEDEC_ID;
    m_generic_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    m_generic_cmd.NbData = 80;
    /* Send Read Jedec command in single SPI mode */
    if (HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        debug_error("Could not set HAL_OSPI_Command().");
        return false;
    }

    uint8_t buf[80] = { 0 };

    /* Receive Data  */
    if (HAL_OSPI_Receive(m_ospi, buf, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        debug_error("Error with HAL_OSPI_Receive().");
        return false;
    }

    debug_printf("Success");
    // debug_print_buffer(buf, sizeof(buf), 0, 16);

    uint8_t device_size = buf[2];
    uint8_t sector_architecture = buf[4];

    uint32_t t_bp_typ = 1 << buf[31];
    uint32_t t_pp_typ = 1 << buf[32];
    uint32_t t_sect_erase_typ = 1 << buf[33];
    uint32_t t_full_erase_typ = 1 << buf[34]; // 22h

    uint32_t t_bp_max = t_bp_typ * (1 << buf[35]);
    uint32_t t_pp_max = t_pp_typ * (1 << buf[36]);
    uint32_t t_sect_erase_max = t_sect_erase_typ * (1 << buf[37]);
    uint32_t t_full_erase_max = t_full_erase_typ * (1 << buf[38]); // 26h

    uint8_t page_size = buf[42]; // 2Ah
    uint8_t tech = buf[69];

    if (device_size == 0x19)
        debug_printf("Device size: 256 Mb");
    // else if (device_size == 0x18)
    //     debug_printf("Device size: 128 Mb");
    else
        return false;

    if (sector_architecture == 0x01)
        debug_printf("Arch: 4 kB param sectors w/ uniform 64 kB sectors");
    else
        return false;
    

    debug_printf("Typ. timeout: BB %u / PP %u / SE %u / FE %u us",
        t_bp_typ, t_pp_typ, t_sect_erase_typ, t_full_erase_typ);
    debug_printf("Max timeout: BB %u / PP %u / SE %u / FE %u us",
        t_bp_max, t_pp_max, t_sect_erase_max, t_full_erase_max);

    if (page_size == 0x09)
        debug_printf("Write page size: 512 B");
    else if (page_size == 0x08)
        debug_printf("Write page size: 256 B");

    if ((tech >> 2) == 0x8)
        debug_printf("Tech: 0.065 um MirrorBit");
    else
        return false;

    return true;
}


/**
 * @brief Send the Sector Erase command to erase a flash sector. Then, wait for the operation to complete.
 *
 * @param sector_index
 * @return true success
 * @return false failed (possibly timeout)
 */
bool qf_erase_sector(uint16_t sector_index)
{
    if (!qf_write_enable())
        return false;

    _reset_generic_cmd();

    /* Write erase command ------------------------------------------ */
    m_generic_cmd.Instruction = CMD_ERASE_SECTOR;
    m_generic_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    m_generic_cmd.Address = sector_index * S25FL256S_UNIFORM_SECTOR_SIZE;

    HAL_StatusTypeDef stat = HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    if (stat != HAL_OK)
    {
        debug_error("Error with erase command %u", stat);
        return false;
    }

    debug_printf("Erase sector %u at address 0x%08X", sector_index, m_generic_cmd.Address);

    // Poll for erase to be completed (OP in )
    return qf_poll_sr1_for_match(MATCH_NO_OP_IN_PROGRESS, BITMASK_OP_IN_PROGRESS);

}


bool qf_write_data(uint32_t address, uint8_t* p_data, uint32_t length)
{
    uint32_t write_length = length;
    uint32_t bytes_written = 0;

    while (bytes_written < length)
    {

        // Check for writes longer than page
        if ((length - bytes_written) > S25FL256S_PAGE_SIZE)
            write_length = S25FL256S_PAGE_SIZE;
        else
            write_length = (length - bytes_written);

        if (!qf_write_enable())
            return false;

        _reset_generic_cmd();

        m_generic_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
        m_generic_cmd.Address = address + bytes_written;
        m_generic_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
        m_generic_cmd.NbData = write_length;
        m_generic_cmd.Instruction = CMD_PAGE_PGM_4PP;
        if (HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            return false;
        }

        // debug_printf("write addr %08X", m_generic_cmd.Address);

        HAL_StatusTypeDef stat = HAL_OSPI_Transmit(m_ospi, p_data + bytes_written, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
        if (stat != HAL_OK)
        {
            debug_error("Problem with transmit: %u", stat);
            return false;
        }

        // Poll for write to be completed
        if (!qf_poll_sr1_for_match(MATCH_NO_OP_IN_PROGRESS, BITMASK_OP_IN_PROGRESS))
            return false;

        bytes_written += write_length;
    }

    return true;
}


bool qf_read_data(uint32_t address, uint8_t* buffer, uint32_t length)
{
    _reset_generic_cmd();

    // debug_printf("Read data at 0x%08X, %u bytes", address, length);

    m_generic_cmd.Instruction = CMD_4READ;
    m_generic_cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    m_generic_cmd.Address = address;
    m_generic_cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    m_generic_cmd.NbData = length;
    if (HAL_OSPI_Command(m_ospi, &m_generic_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return false;
    }

    HAL_StatusTypeDef stat = HAL_OSPI_Receive(m_ospi, buffer, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
    if (stat != HAL_OK)
    {
        debug_error("Problem with Receive: %u", stat);
        return false;
    }

    return true;
}

bool qspi_flash_init_S25FL256S(OSPI_HandleTypeDef* hospi)
{
    bool op_result = true;

    m_ospi = hospi;
    debug_printf("Init S25FL256S flash");

    // Disable Reset of chip to allow commands. Not needed for JEDEC, but for r/w/e
    HAL_GPIO_WritePin(FLASH_NRESET_GPIO_Port, FLASH_NRESET_Pin, GPIO_PIN_SET);
    osDelay(10);

    debug_printf("Read Jedec ID");

    if (qf_read_jedec() == false)
    {
        debug_error("Couldn't read JEDEC ID... HW problem?");
        return false;
    }

    return true;

    /* Erase Sectors ----------------------------------------------- */
    for (int sect = 0; sect < 512; sect++)
    {
        osDelay(10);    // this is just to allow debug printing to continue.

        op_result = qf_erase_sector(sect);

        if (op_result == false)
            debug_error("Couldn't erase sector %u", sect);
    }

    return true;

    /* Writing Sequence ----------------------------------------------- */
    uint32_t address = 0x00000000;
    uint32_t data_len = sizeof(TEST_BUFFER) - 1;

    debug_printf("Write data.");
    op_result = qf_write_data(address, TEST_BUFFER, data_len);
    if (op_result == false)
    {
        debug_error("Couldn't write data.");
        return false;
    }

    /* Reading Sequence ----------------------------------------------- */
    debug_printf("Read data");

    memset(rx_buffer, 0x00, sizeof(rx_buffer));

    op_result = qf_read_data(address, rx_buffer, data_len);
    if (op_result == false)
    {
        debug_error("Couldn't receive data.");
        return false;
    }

    /* Compare Sequence ----------------------------------------------- */
    debug_printf("Compare data");
    if (memcmp(TEST_BUFFER, rx_buffer, data_len) != 0)
    {
        debug_error("Compare failed.");
        return false;
    }
    else
    {
        debug_printf("Compare success.");
        return true;
    }

    return true;
}
