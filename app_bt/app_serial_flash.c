/******************************************************************************
 * File Name: app_serial_flash.c
 *
 * Description: This file contains block device function implementations
 *              required by kv-store library
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
 *                              INCLUDES
 ******************************************************************************/

#include "cycfg_qspi_memslot.h"
#include "mtb_kvstore.h"
#include "app_serial_flash.h"
#include "app_handler.h"

/*******************************************************************************
 *                              Macro Definitions
 ******************************************************************************/
#define FLASH_POWER_DOWN_CMD    (0xB9)
#define FLASH_POWER_UP_CMD      (0xAB)

/*******************************************************************************
 *                              GLOBAL DECLARATIONS
 ******************************************************************************/

/*Kvstore block device*/
mtb_kvstore_bd_t block_device;
/* Object for kvstore library */
mtb_kvstore_t kvstore_obj;
/* Object for SMIF Context */
extern cy_stc_smif_context_t cybsp_smif_context;

uint32_t g_PORT_SEL0;
uint32_t g_PORT_SEL1;
uint32_t g_CFG;
uint32_t g_OUT;

/*******************************************************************************
 *                              FUNCTION DECLARATIONS
 ******************************************************************************/
static uint32_t app_flash_bd_read_size(void *context, uint32_t addr);
static uint32_t app_flash_bd_program_size(void *context, uint32_t addr);
static uint32_t app_flash_bd_erase_size(void *context, uint32_t addr);
static cy_rslt_t app_flash_bd_read(void *context, uint32_t addr, uint32_t length, uint8_t *buf);
static cy_rslt_t app_flash_bd_program(void *context, uint32_t addr, uint32_t length, const uint8_t *buf);
static cy_rslt_t app_flash_bd_erase(void *context, uint32_t addr, uint32_t length);

/*******************************************************************************
 *                              FUNCTION DEFINITIONS
 ******************************************************************************/

CY_SECTION_RAMFUNC_BEGIN
/**
 * Function Name:
 * app_flash_bd_get_flash_id
 *
 * Function Description:
 * @brief Function to get the unique flash id.
 *        This function should be present in RAM and no XPI transactions should be done in between Transmit command and receive.
 *
 * @param  context : Context object that is passed into mtb_kvstore_init
 * @param  addr: Address to read the data from the block device. This address is passed in as start_addr + offset
 * @param  length : Length of the data to be read
 * @param  buf : Data that needs to be read
 *
 *
 * @return cy_rslt_t: Result of the read operation.
 *
 */
cy_rslt_t app_flash_bd_get_flash_id(void *context, uint32_t addr, uint32_t length, uint8_t *buf)
{
    (void)context;
    cy_rslt_t result = 0;

    printf("bd_get_flash_id\r\n");

    /* Read the unique id register */
    /* The Transmit command and receive data should be done from RAM and not flash */
    result = Cy_SMIF_TransmitCommand(SMIF0, addr, CY_SMIF_WIDTH_SINGLE,
                                     NULL, CY_SMIF_CMD_WITHOUT_PARAM, CY_SMIF_WIDTH_NA,
                                     smifBlockConfig.memConfig[0]->slaveSelect,
                                     CY_SMIF_TX_NOT_LAST_BYTE,
                                     &cybsp_smif_context);

    /* No printfs should be added in between transmit command and receive data block */

    if (CY_SMIF_SUCCESS == result)
    {
        Cy_SMIF_SendDummyCycles(SMIF0, 32);
        result = Cy_SMIF_ReceiveDataBlocking(SMIF0, buf, length, CY_SMIF_WIDTH_SINGLE,
                                             &cybsp_smif_context);
    }

    return result;
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
/**
 * Function Name:
 * app_flash_memory_power_down
 *
 * Function Description:
 * @brief  This function puts the Flash to Power down state.
 *
 * @param  void
 *
 * @return void
 */
void app_flash_memory_power_down(void)
{
    Cy_SMIF_TransmitCommand(SMIF0, FLASH_POWER_DOWN_CMD, CY_SMIF_WIDTH_SINGLE, NULL, CY_SMIF_CMD_WITHOUT_PARAM,
                                    CY_SMIF_WIDTH_NA, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_TX_LAST_BYTE,
                                    &cybsp_smif_context);
    while(Cy_SMIF_BusyCheck(SMIF0));
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
/**
 * Function Name:
 * app_flash_memory_power_up
 *
 * Function Description:
 * @brief  This function recovers the flash from Power down state.
 *
 * @param  void
 *
 * @return void
 */
void app_flash_memory_power_up(void)
{
    Cy_SMIF_TransmitCommand(SMIF0, FLASH_POWER_UP_CMD, CY_SMIF_WIDTH_SINGLE, NULL, CY_SMIF_CMD_WITHOUT_PARAM,
                                     CY_SMIF_WIDTH_NA, CY_SMIF_SLAVE_SELECT_0, CY_SMIF_TX_LAST_BYTE,
                                     &cybsp_smif_context);
    while(Cy_SMIF_BusyCheck(SMIF0));

    Cy_SysLib_DelayUs(5);
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
/**
 *  Function name:
 *  app_flash_smif_disable
 *
 *  Function Description:
 *  @brief Function used to disable SMIF
 *
 *  @param  void
 *
 *  @return void
 */
void app_flash_smif_disable(void)
{
    SMIF0->CTL = SMIF0->CTL & ~SMIF_CTL_ENABLED_Msk;
    g_PORT_SEL0 = HSIOM_PRT2->PORT_SEL0;
    g_PORT_SEL1 = HSIOM_PRT2->PORT_SEL1;
    g_CFG = GPIO_PRT2->CFG;
    g_OUT = GPIO_PRT2->OUT;
    HSIOM_PRT2->PORT_SEL0 = 0x00;
    HSIOM_PRT2->PORT_SEL1 = 0x00;
    GPIO_PRT2->CFG = 0x600006;
    GPIO_PRT2->OUT = 0x1;
}
CY_SECTION_RAMFUNC_END

CY_SECTION_RAMFUNC_BEGIN
/**
 *  Function name:
 *  app_flash_smif_enable
 *
 *  Function Description:
 *  @brief Function used to enable SMIF
 *
 *  @param  void
 *
 *  @return void
 */
void app_flash_smif_enable(void)
{
    SMIF0->CTL = SMIF0->CTL | SMIF_CTL_ENABLED_Msk;
    HSIOM_PRT2->PORT_SEL0 = g_PORT_SEL0;
    HSIOM_PRT2->PORT_SEL1 = g_PORT_SEL1;
    GPIO_PRT2->CFG = g_CFG;
    GPIO_PRT2->OUT = g_OUT;
}
CY_SECTION_RAMFUNC_END

/**
 * Function Name:
 * app_flash_bd_read_size
 *
 * Function Description:
 * @brief Function to get the read size of the block device for a specific address.
 *
 * @param  context: Context object that is passed into mtb_kvstore_init
 * @param  addr: Address for which the read size is queried. This address is passed in as start_addr + offset.
 *
 * @return uint32_t: Read size of the memory device.
 *
 */
static uint32_t app_flash_bd_read_size(void *context, uint32_t addr)
{
    (void)context;
    (void)addr;
    return 1;
}

/**
 * Function Name:
 * app_flash_bd_program_size
 *
 * Function Description:
 * @brief Function to get the program size of the block device for a specific address.
 *
 * @param context: Context object that is passed into mtb_kvstore_init
 * @param addr: Address for which the program size is queried. This address is passed in as start_addr + offset.
 *
 *
 * @return uint32_t: Program size of the memory device.
 */
static uint32_t app_flash_bd_program_size(void *context, uint32_t addr)
{
    (void)context;
    CY_UNUSED_PARAMETER(addr);
    return (size_t)smifBlockConfig.memConfig[0]->deviceCfg->programSize;
}

/**
 * Function Name:
 * app_flash_bd_erase_size
 *
 * Function Description:
 * @brief Function to get the erase size of the block device for a specific address.
 *
 * @param context: Context object that is passed into mtb_kvstore_init
 * @param addr: Address for which the program size is queried. This address is passed in as start_addr + offset.
 *
 * @return uint32_t: Erase size of the memory device.
 */
static uint32_t app_flash_bd_erase_size(void *context, uint32_t addr)
{
    (void)context;
    size_t erase_sector_size;
    cy_stc_smif_hybrid_region_info_t *hybrid_info = NULL;

    cy_en_smif_status_t smif_status =
        Cy_SMIF_MemLocateHybridRegion(smifBlockConfig.memConfig[0], &hybrid_info, addr);

    if (CY_SMIF_SUCCESS != smif_status)
    {
        erase_sector_size = (size_t)smifBlockConfig.memConfig[0]->deviceCfg->eraseSize;
    }
    else
    {
        erase_sector_size = (size_t)hybrid_info->eraseSize;
    }

    return erase_sector_size;
}

/**
 * Function Name:
 * app_flash_bd_read
 *
 * Function Description:
 * @brief Function for reading data from the block device.
 *
 * @param  context: Context object that is passed into mtb_kvstore_init
 * @param  addr: Address to read the data from the block device. This address is passed in as start_addr + offset
 * @param  length: Length of the data to be read into the buffer
 * @param  buf: Buffer to read the data.
 *
 * @return cy_rslt_t: Result of the read operation.
 *
 */
static cy_rslt_t app_flash_bd_read(void *context, uint32_t addr, uint32_t length, uint8_t *buf)
{
    (void)context;
    cy_rslt_t result = 0;
    // Cy_SMIF_MemRead() returns error if (addr + length) > total flash size.
    result = (cy_rslt_t)Cy_SMIF_MemRead(SMIF0, smifBlockConfig.memConfig[0], addr, buf, length,
                                        &cybsp_smif_context);

    return result;
}

/**
 * Function Name:
 * app_flash_bd_program
 *
 * Function Description:
 * @brief Function to write/program data into the block device.
 *
 * @param  context: Context object that is passed into mtb_kvstore_init
 * @param  addr: Address to program the data into the block device. This address is passed in as start_addr + offset
 * @param  length: Length of the data to be written
 * @param  buf: Data that needs to be written
 *
 * @return cy_rslt_t: Result of the program operation.
 *
 */
static cy_rslt_t app_flash_bd_program(void *context, uint32_t addr, uint32_t length, const uint8_t *buf)
{
    (void)context;
    cy_rslt_t result = 0;
    // Cy_SMIF_MemWrite() returns error if (addr + length) > total flash size.
    result = (cy_rslt_t)Cy_SMIF_MemWrite(SMIF0, smifBlockConfig.memConfig[0], addr, (uint8_t*)buf,
                                         length, &cybsp_smif_context);

    return result;
}

/**
 * Function Name:
 * app_flash_bd_erase
 *
 * Function Description:
 * @brief Function to erase data from the device
 *
 * @param  context: Context object that is passed into mtb_kvstore_init
 * @param  addr: Address to erase the data from the device. This address is passed in as start_addr + offset
 * @param  length: Length of the data that needs to be erased
 *
 * @return cy_rslt_t : Result of the erase operation.
 *
 */
static cy_rslt_t app_flash_bd_erase(void *context, uint32_t addr, uint32_t length)
{
    (void)context;
    cy_rslt_t result = 0;
    // If the erase is for the entire chip, use chip erase command
    if ((addr == 0u) && (length == (size_t)smifBlockConfig.memConfig[0]->deviceCfg->memSize))
    {
        result = (cy_rslt_t)Cy_SMIF_MemEraseChip(SMIF0, smifBlockConfig.memConfig[0], &cybsp_smif_context);
    }
    else
    {
        // Cy_SMIF_MemEraseSector() returns error if (addr + length) > total flash size or if
        // addr is not aligned to erase sector size or if (addr + length) is not aligned to
        // erase sector size.
        result = (cy_rslt_t)Cy_SMIF_MemEraseSector(SMIF0, smifBlockConfig.memConfig[0], addr,
                                                   length, &cybsp_smif_context);
    }

    return result;
}

/**
 * Function Name:
 * app_flash_bd_init
 *
 * Function Description:
 * @brief  This function provides the pointer to the implemented prototype function for the block device.
 *
 * @param  device: Block device interface
 *
 * @return void
 */
void app_flash_bd_init(mtb_kvstore_bd_t *device)
{
    device->read         = app_flash_bd_read;
    device->program      = app_flash_bd_program;
    device->erase        = app_flash_bd_erase;
    device->read_size    = app_flash_bd_read_size;
    device->program_size = app_flash_bd_program_size;
    device->erase_size   = app_flash_bd_erase_size;
    device->context      = NULL;
}

/**
 * Function Name:
 * app_flash_kv_store_init
 *
 * Function Description:
 * @brief   This function initializes the kv-store library
 *
 * @param   void
 *
 * @return  void
 */
void app_flash_kv_store_init(void)
{
    cy_rslt_t rslt;
    uint32_t start_addr = 0;
    /* Define the space to be used for Bond Data Storage */
    uint32_t sector_size = (size_t)smifBlockConfig.memConfig[0]->deviceCfg->eraseSize;
    uint32_t length = sector_size * 2;

    /* Initialize the block device used by kv-store for performing read/write
     * operations to the flash
     */
    app_flash_bd_init(&block_device);

    start_addr = smifMemConfigs[0]->deviceCfg->memSize - sector_size * 2;

    printf("kvstore init..\r\n");
    /*Initialize kv-store library*/
    rslt = mtb_kvstore_init(&kvstore_obj, start_addr, length, &block_device);
    /*Check if the kv-store initialization was successfull*/
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("failed to initialize kv-store \r\n");
        CY_ASSERT(0);
    }
}
