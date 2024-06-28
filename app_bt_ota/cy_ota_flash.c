/*
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
 */

/*
 *  Flash operation callback implementation for OTA update.
 */

/* Header file includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_ota_flash.h"

#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
#include <cycfg_pins.h>
#endif

/**********************************************************************************************************************************
 * local defines
 **********************************************************************************************************************************/
/* This defines if External Flash (SMIF) will be used for Upgrade Slots */
#if (defined (CYW20829) || defined (CYW89829))
#define CY_FLASH_BASE                       CY_XIP_BASE /* Override value in /mtb-pdl-cat1/devices/COMPONENT_CAT1A/include/cy_device_common.h for CYW20829 and CYW89829 */
#define CY_FLASH_SIZEOF_ROW                 512u        /* Override value in /mtb-pdl-cat1/devices/COMPONENT_CAT1A/include/cy_device_common.h for CYW20829 and CYW89829 */
#endif /* CYW20829 or CYW89829 */

#if defined(XMC7200)
#define CY_XIP_BASE                         0x60000000UL
#define CY_FLASH_SIZE                       0x830000UL
#define CY_FLASH_BASE                       0x10000000UL
#endif /* XMC7200 */

#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
/* UN-comment to test the write functionality */
//#define READBACK_SMIF_WRITE_TEST

// SMIF slot from which the memory configuration is picked up - fixed to 0 as
// the driver supports only one device
#define MEM_SLOT                                    (0u)

/* Set it high enough for the sector erase operation to complete */
#define MEMORY_BUSY_CHECK_RETRIES                   (750ul)
#define _CYHAL_QSPI_DESELECT_DELAY                  (7UL)

/* cyhal_qspi_init() succeeded */
#define FLAG_HAL_INIT_DONE                          (0x01lu << 0)

#define IS_FLAG_SET(mask)                           (status_flags & (mask))
#define SET_FLAG(mask)                              (status_flags |= (mask))
#define CLEAR_FLAG(mask)                            (status_flags &= ~(mask))

/* QSPI bus frequency set to 50 Mhz */
#define QSPI_BUS_FREQUENCY_HZ                       (50000000lu)

#define TIMEOUT_1_MS                                (1000lu)

#define CY_SMIF_BASE_MEM_OFFSET                     CY_XIP_BASE

#ifdef CY_XIP_SMIF_MODE_CHANGE

/*
 * IMPORTANT NOTE. Do not add calls to non-RAM resident routines
 * between TURN_OFF_XIP and TURN_ON_XIP calls or you will break
 * XIP environments.
 */

#define PRE_SMIF_ACCESS_TURN_OFF_XIP \
                    uint32_t interruptState;                            \
                    interruptState = Cy_SysLib_EnterCriticalSection();  \
                    while(Cy_SMIF_BusyCheck(SMIF0));    \
                    (void)Cy_SMIF_SetMode(SMIF0, CY_SMIF_NORMAL);

#define POST_SMIF_ACCESS_TURN_ON_XIP \
                    while(Cy_SMIF_BusyCheck(SMIF0));    \
                    (void)Cy_SMIF_SetMode(SMIF0, CY_SMIF_MEMORY);   \
                    Cy_SysLib_ExitCriticalSection(interruptState);


#else
#define PRE_SMIF_ACCESS_TURN_OFF_XIP
#define POST_SMIF_ACCESS_TURN_ON_XIP
#endif

/**********************************************************************************************************************************
 * local variables & data
 **********************************************************************************************************************************/

static cy_stc_smif_context_t ota_QSPI_context;
static volatile uint32_t     status_flags;

/* Default QSPI configuration */
const cy_stc_smif_config_t ota_SMIF_config =
{
    .mode = (uint32_t)CY_SMIF_NORMAL,
    .deselectDelay = _CYHAL_QSPI_DESELECT_DELAY,
#if (CY_IP_MXSMIF_VERSION >= 2)
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INVERTED_FEEDBACK_CLK,
#else
    .rxClockSel = (uint32_t)CY_SMIF_SEL_INV_INTERNAL_CLK,
#endif
    .blockEvent = (uint32_t)CY_SMIF_BUS_ERROR,
};

//extern const cy_stc_smif_mem_config_t* smifMemConfigs[];
//extern const cy_stc_smif_block_config_t smifBlockConfig;

#ifdef READBACK_SMIF_WRITE_TEST
/* Used for testing the write functionality */
static uint8_t read_back_test[1024];
#endif
#endif /* CY_IP_MXSMIF & !XMC7200 */


/**********************************************************************************************************************************
 * Internal Functions
 **********************************************************************************************************************************/
#if defined(CY_IP_MXSMIF) && !defined(PSOC_062_1M) && !defined(XMC7200)
#if defined(OTA_USE_EXTERNAL_FLASH)
/*******************************************************************************
* Function Name: IsMemoryReady
****************************************************************************//**
*
* Polls the memory device to check whether it is ready to accept new commands or
* not until either it is ready or the retries have exceeded the limit.
*
* \param memConfig
* memory device configuration
*
* \return Status of the operation.
* CY_SMIF_SUCCESS        - Memory is ready to accept new commands.
* CY_SMIF_EXCEED_TIMEOUT - Memory is busy.
*
*******************************************************************************/
static cy_en_smif_status_t IsMemoryReady(cy_stc_smif_mem_config_t const *memConfig)
{
    uint32_t retries = 0;
    bool isBusy;

    do
    {
        isBusy = Cy_SMIF_Memslot_IsBusy(SMIF0, (cy_stc_smif_mem_config_t* )memConfig, &ota_QSPI_context);
        Cy_SysLib_Delay(5);
        retries++;
    }while(isBusy && (retries < MEMORY_BUSY_CHECK_RETRIES));

    return (isBusy ? CY_SMIF_EXCEED_TIMEOUT : CY_SMIF_SUCCESS);
}

/*******************************************************************************
* Function Name: IsQuadEnabled
****************************************************************************//**
*
* Checks whether QE (Quad Enable) bit is set or not in the configuration
* register of the memory.
*
* \param memConfig
* Memory device configuration
*
* \param isQuadEnabled
* This parameter is updated to indicate whether Quad mode is enabled (true) or
* not (false). The value is valid only when the function returns
* CY_SMIF_SUCCESS.
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
static cy_en_smif_status_t IsQuadEnabled(cy_stc_smif_mem_config_t const *memConfig, bool *isQuadEnabled)
{
    cy_en_smif_status_t status;
    uint8_t readStatus = 0;
    uint32_t statusCmd = memConfig->deviceCfg->readStsRegQeCmd->command;
    uint8_t maskQE = (uint8_t) memConfig->deviceCfg->stsRegQuadEnableMask;

    status = Cy_SMIF_Memslot_CmdReadSts(SMIF0, memConfig, &readStatus, statusCmd, &ota_QSPI_context);

    *isQuadEnabled = false;
    if(CY_SMIF_SUCCESS == status)
    {
        /* Check whether Quad mode is already enabled or not */
        *isQuadEnabled = (maskQE == (readStatus & maskQE));
    }

    return status;
}

/*******************************************************************************
* Function Name: EnableQuadMode
****************************************************************************//**
*
* This function sets the QE (QUAD Enable) bit in the external memory
* configuration register to enable Quad SPI mode.
*
* \param memConfig
* Memory device configuration
*
* \return Status of the operation. See cy_en_smif_status_t.
*
*******************************************************************************/
static cy_en_smif_status_t EnableQuadMode(cy_stc_smif_mem_config_t const *memConfig)
{
    cy_en_smif_status_t status;

    /* Send Write Enable to external memory */
    status = Cy_SMIF_Memslot_CmdWriteEnable(SMIF0, smifMemConfigs[0], &ota_QSPI_context);

    if(CY_SMIF_SUCCESS == status)
    {
        status = Cy_SMIF_Memslot_QuadEnable(SMIF0, (cy_stc_smif_mem_config_t* )memConfig, &ota_QSPI_context);

        if(CY_SMIF_SUCCESS == status)
        {
            /* Poll memory for the completion of operation */
            status = IsMemoryReady(memConfig);
        }
    }

    return status;
}
#endif /* OTA_USE_EXTERNAL_FLASH */
#endif /* CY_IP_MXSMIF & !PSOC_062_1M & !XMC7200 */

#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG) || defined (XMC7200))
static int psoc6_internal_flash_write(uint8_t data[], uint32_t address, size_t len)
{
    int retCode;
    cy_en_flashdrv_status_t rc = CY_FLASH_DRV_SUCCESS;

    uint32_t writeBuffer[CY_FLASH_SIZEOF_ROW / sizeof(uint32_t)];
    uint32_t rowId;
    uint32_t dstIndex;
    uint32_t srcIndex = 0u;
    uint32_t eeOffset;
    uint32_t byteOffset;
    uint32_t rowsNotEqual;
    uint8_t *writeBufferPointer;

    eeOffset = (uint32_t)address;
    writeBufferPointer = (uint8_t*)writeBuffer;

    bool cond1;

    /* Make sure, that varFlash[] points to Flash */
    cond1 = ((eeOffset >= CY_FLASH_BASE) && ((eeOffset + len) <= (CY_FLASH_BASE + CY_FLASH_SIZE)));

    if(cond1)
    {
        eeOffset -= CY_FLASH_BASE;
        rowId = eeOffset / CY_FLASH_SIZEOF_ROW;
        byteOffset = CY_FLASH_SIZEOF_ROW * rowId;

        while((srcIndex < len) && (rc == CY_FLASH_DRV_SUCCESS))
        {
            rowsNotEqual = 0u;
            /* Copy data to the write buffer either from the source buffer or from the flash */
            for(dstIndex = 0u; dstIndex < CY_FLASH_SIZEOF_ROW; dstIndex++)
            {
                if((byteOffset >= eeOffset) && (srcIndex < len))
                {
                    writeBufferPointer[dstIndex] = data[srcIndex];
                    /* Detect that row programming is required */
                    if((rowsNotEqual == 0u) && ( (CY_GET_REG8(CY_FLASH_BASE + byteOffset) != data[srcIndex]) ) )
                    {
                        rowsNotEqual = 1u;
                    }
                    srcIndex++;
                }
                else
                {
                    writeBufferPointer[dstIndex] = CY_GET_REG8(CY_FLASH_BASE + byteOffset);
                }
                byteOffset++;
            }

            if(rowsNotEqual != 0u)
            {
                /* Write flash row */
                rc = Cy_Flash_WriteRow((rowId * CY_FLASH_SIZEOF_ROW) + CY_FLASH_BASE, writeBuffer);
            }

            /* Go to the next row */
            rowId++;
        }
    }
    else
    {
        rc = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }

    /* Return error code */
    switch(rc)
    {
        case CY_FLASH_DRV_SUCCESS:
            retCode = 0;
            break;

        case CY_FLASH_DRV_INVALID_INPUT_PARAMETERS:
        case CY_FLASH_DRV_INVALID_FLASH_ADDR:
            retCode = 1;
            break;

        default:
            retCode = 2;
            break;
    }
    return(retCode);
}

static int psoc6_internal_flash_erase(uint32_t addr, size_t size)
{
    int rc = 0;

    uint32_t addrStart, addrEnd, address;
    uint32_t remStart, remEnd;
    uint32_t rowIdxStart, rowIdxEnd, rowNum;
    uint8_t  buff[CY_FLASH_SIZEOF_ROW];

    /* flash_area_write() uses offsets, we need absolute address here */
    addr += CY_FLASH_BASE;

    addrStart = addr;
    addrEnd   = addrStart + size;

    /* find if area bounds are aligned to rows */
    remStart = addrStart%CY_FLASH_SIZEOF_ROW;
    remEnd   = addrEnd%CY_FLASH_SIZEOF_ROW;

    /* find which row numbers are affected for full Erase */
    rowIdxStart = addrStart/CY_FLASH_SIZEOF_ROW;
    rowIdxEnd   = addrEnd/CY_FLASH_SIZEOF_ROW;

    if(remStart != 0)
    {/* first row is fragmented, move to next */
        rowIdxStart++;
    }

    /* total number of rows for full erase */
    rowNum = rowIdxEnd - rowIdxStart;
    address = rowIdxStart*CY_FLASH_SIZEOF_ROW;

    while(rowNum>0)
    {
        rc = Cy_Flash_EraseRow(address);
        assert(rc == 0);
        address += CY_FLASH_SIZEOF_ROW;
        rowNum--;
    }

    /* if Start of erase area is unaligned */
    if(remStart != 0)
    {
        /* first row is fragmented, shift left by one*/
        rowIdxStart--;

        /* find start address of fragmented row */
        address = rowIdxStart*CY_FLASH_SIZEOF_ROW;

        /* store fragmented row contents first */
        memcpy((void *)buff, (const void*)address, remStart);

        /* erase fragmented row */
        rc = Cy_Flash_EraseRow(address);
        assert(rc == 0);

        /* write stored back */
        rc = psoc6_internal_flash_write(buff, address, remStart);
        assert(rc == 0);
    }
    /* if End of erase area is unaligned */
    if(remEnd != 0)
    {
        /* find start address of fragmented row */
        address = rowIdxEnd*CY_FLASH_SIZEOF_ROW;

        /* store fragmented row contents first */
        memcpy((void *)buff, (const void*)addrEnd, CY_FLASH_SIZEOF_ROW-remEnd);

        /* erase fragmented row */
        rc = Cy_Flash_EraseRow(address);
        assert(rc == 0);

        /* write stored back */
        rc = psoc6_internal_flash_write(buff, addrEnd, CY_FLASH_SIZEOF_ROW-remEnd);
        assert(rc == 0);
    }
    return rc;
}
#endif

#if defined (XMC7200)
CY_SECTION_RAMFUNC_BEGIN
static int xmc_internal_flash_erase(uint32_t addr, size_t size)
{
    int rc                   = 0;
    uint32_t row_addr        = 0u;
    uint32_t erase_sz        = 0x8000U; //32KB
    cy_en_flashdrv_status_t flashEraseStatus;

    /* flash_area_write() uses offsets, we need absolute address here */
    addr += CY_FLASH_BASE;

    uintptr_t erase_end_addr = addr + size;
    uint32_t row_start_addr  = (addr / erase_sz) * erase_sz;
    uint32_t row_end_addr    = (erase_end_addr / erase_sz) * erase_sz;
    uint32_t row_number      = (row_end_addr - row_start_addr) / erase_sz;

    /* assume single row needs to be erased */
    if (row_start_addr == row_end_addr) {
        row_number = 1U;
    }

    Cy_Flash_Init();
    Cy_Flashc_MainWriteEnable();

    while (row_number != 0u) {
        row_number--;
        row_addr = row_start_addr + row_number * (uint32_t)erase_sz;

        flashEraseStatus = Cy_Flash_EraseSector((uint32_t) row_addr);
        // Wait for completion with counting
        while(Cy_Flash_IsOperationComplete() != CY_FLASH_DRV_SUCCESS)
        {
        }
        if (flashEraseStatus != CY_FLASH_DRV_SUCCESS)
        {
            rc = 1; /* BOOT_EFLASH */
            break;
        } else {
            rc = 0;
        }
    }

    return rc;
}
CY_SECTION_RAMFUNC_END

/*
 * Writes `len` bytes of flash memory at `off` from the buffer at `src`
 */
CY_SECTION_RAMFUNC_BEGIN
static int xmc_internal_flash_write(uint8_t data[], uint32_t address, size_t len)
{
    int retCode;
    cy_en_flashdrv_status_t rc = CY_FLASH_DRV_SUCCESS;

    uint32_t writeBuffer[CY_FLASH_SIZEOF_ROW / sizeof(uint32_t)];
    uint32_t rowId;
    uint32_t dstIndex;
    uint32_t srcIndex = 0u;
    uint32_t eeOffset;
    uint32_t byteOffset;
    uint32_t rowsNotEqual;
    uint8_t *writeBufferPointer;

    eeOffset = (uint32_t)address;
    writeBufferPointer = (uint8_t*)writeBuffer;

    bool cond1;

    /* Make sure, that varFlash[] points to Flash */
    cond1 = ((eeOffset >= CY_FLASH_BASE) && ((eeOffset + len) <= (CY_FLASH_BASE + CY_FLASH_SIZE)));

    if(cond1)
    {
        eeOffset -= CY_FLASH_BASE;
        rowId = eeOffset / CY_FLASH_SIZEOF_ROW;
        byteOffset = CY_FLASH_SIZEOF_ROW * rowId;

        while((srcIndex < len) && (rc == CY_FLASH_DRV_SUCCESS))
        {
            rowsNotEqual = 0u;
            /* Copy data to the write buffer either from the source buffer or from the flash */
            for(dstIndex = 0u; dstIndex < CY_FLASH_SIZEOF_ROW; dstIndex++)
            {
                if((byteOffset >= eeOffset) && (srcIndex < len))
                {
                    writeBufferPointer[dstIndex] = data[srcIndex];
                    /* Detect that row programming is required */
                    if((rowsNotEqual == 0u) && ( (CY_GET_REG8(CY_FLASH_BASE + byteOffset) != data[srcIndex]) ) )
                    {
                        rowsNotEqual = 1u;
                    }
                    srcIndex++;
                }
                else
                {
                    writeBufferPointer[dstIndex] = CY_GET_REG8(CY_FLASH_BASE + byteOffset);
                }
                byteOffset++;
            }

            if(rowsNotEqual != 0u)
            {
                rc = Cy_Flash_ProgramRow((rowId * CY_FLASH_SIZEOF_ROW) + CY_FLASH_BASE, writeBuffer);
                if(rc == CY_FLASH_DRV_SUCCESS)
                {
                    while(Cy_Flash_IsOperationComplete() != CY_FLASH_DRV_SUCCESS)
                    {
                    }
                }
            }

            /* Go to the next row */
            rowId++;
        }
    }
    else
    {
        rc = CY_FLASH_DRV_INVALID_INPUT_PARAMETERS;
    }

    /* Return error code */
    switch(rc)
    {
        case CY_FLASH_DRV_SUCCESS:
            retCode = 0;
            break;

        case CY_FLASH_DRV_INVALID_INPUT_PARAMETERS:
        case CY_FLASH_DRV_INVALID_FLASH_ADDR:
            retCode = 1;
            break;

        default:
            retCode = 2;
            break;
    }
    return(retCode);
}
CY_SECTION_RAMFUNC_END
#endif /* XMC7200 */

#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
static uint32_t ota_smif_get_memory_size(void)
{
    uint32_t size = 0;

    if (SMIF0 != NULL)
    {
        size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->memSize;
    }

    return size;
}
#endif /* CY_IP_MXSMIF & !XMC7200 */

/**********************************************************************************************************************************
 * External Functions
 **********************************************************************************************************************************/
/**
 * @brief Initializes flash, QSPI flash, or any other external memory type
 *
 * @return  CY_RSLT_SUCCESS on success
 *          CY_RSLT_TYPE_ERROR on failure
 */
cy_rslt_t cy_ota_mem_init( void )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
#if defined(OTA_USE_EXTERNAL_FLASH)
    cy_rslt_t smif_status = CY_SMIF_BAD_PARAM;    /* Does not return error if SMIF Quad fails */
    bool QE_status = false;

    /* pre-access to SMIF */
    PRE_SMIF_ACCESS_TURN_OFF_XIP;

#if (defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
    /* SMIF is already initialized for 20829 and 89829 so we are only initializing the
     * SMIF base address and the context variables.
     */
    smif_status = Cy_SMIF_Init(SMIF0, &ota_SMIF_config, TIMEOUT_1_MS, &ota_QSPI_context);
    if (smif_status != CY_SMIF_SUCCESS)
    {
        result = smif_status;
        goto _bail;
    }
#endif

    /* Set up SMIF */
    Cy_SMIF_SetDataSelect(SMIF0, smifMemConfigs[0]->slaveSelect, smifMemConfigs[0]->dataSelect);
    Cy_SMIF_Enable(SMIF0, &ota_QSPI_context);

    /* Map memory device to memory map */
    smif_status = Cy_SMIF_Memslot_Init(SMIF0, &smifBlockConfig, &ota_QSPI_context);
    if (smif_status != CY_SMIF_SUCCESS)
    {
        result = smif_status;
        goto _bail;
    }

#if (defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
    /* Even after SFDP enumeration QE command is not initialized */
    /* So, it should be 1.0 device */
    if ((smifMemConfigs[0]->deviceCfg->readStsRegQeCmd->command == 0) ||                        /* 0 - if configurator generated code */
        (smifMemConfigs[0]->deviceCfg->readStsRegQeCmd->command == CY_SMIF_NO_COMMAND_OR_MODE)) /* 0xFF's if SFDP enumerated          */
    {
        smif_status = Cy_SMIF_MemInitSfdpMode(SMIF0,
                                              smifMemConfigs[0],
                                              CY_SMIF_WIDTH_QUAD,
                                              CY_SMIF_SFDP_QER_1,
                                              &ota_QSPI_context);
        if (smif_status != CY_SMIF_SUCCESS)
        {
            result = smif_status;
            goto _bail;
        }
    }
#else /* NON - CYW20829B0LKML/CYW89829B01MKSBG */
    #if !defined(CY_RUN_CODE_FROM_XIP) && (OTA_USE_EXTERNAL_FLASH)
        {
            /* Choose SMIF slot number (slave select).
             * Acceptable values are:
             * 0 - SMIF disabled (no external memory);
             * 1, 2, 3 or 4 - slave select line memory module is connected to.
             */
    #define SMIF_ID         (1U) /* Assume SlaveSelect_0 is used for External Memory */
    #include "flash_qspi.h"
            cy_en_smif_status_t qspi_status = CY_SMIF_SUCCESS;
            qspi_status = qspi_init_sfdp(SMIF_ID);
            if(CY_SMIF_SUCCESS == qspi_status)
            {
                result = CY_RSLT_SUCCESS;
            }
            else
            {
                result = CY_RSLT_TYPE_ERROR;
            }
        }
    #endif /* ! CY_RUN_CODE_FROM_XIP */
#endif /* CYW20829B0LKML/CYW89829B01MKSBG */

    smif_status = IsQuadEnabled(smifMemConfigs[0], &QE_status);
    if(smif_status != CY_RSLT_SUCCESS)
    {
        result = CY_RSLT_TYPE_ERROR;
    }

    /* If not enabled, enable quad mode */
    if(!QE_status)
    {
        /* Enable Quad mode */
        smif_status = EnableQuadMode(smifMemConfigs[0]);
        if(smif_status != CY_RSLT_SUCCESS)
        {
            result = CY_RSLT_TYPE_ERROR;
        }
    }

    SET_FLAG(FLAG_HAL_INIT_DONE);

  _bail:
    /* post-access to SMIF */
    POST_SMIF_ACCESS_TURN_ON_XIP;
#endif
#endif /* CY_IP_MXSMIF & !XMC7200 */
    return result;
}

/**
 * @brief Read from flash, QSPI flash, or any other external memory type
 *
 * @param[in]   mem_type   Memory type @ref cy_ota_mem_type_t
 * @param[in]   addr       Starting address to read from.
 * @param[out]  data       Pointer to the buffer to store the data read from the memory.
 * @param[in]   len        Number of data bytes to read.
 *
 * @return  CY_RSLT_SUCCESS on success
 *          CY_RSLT_TYPE_ERROR on failure
 */
cy_rslt_t cy_ota_mem_read( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
        /* flash_area_read() uses offsets, we need absolute address here */
        addr += CY_FLASH_BASE;

        /* flash read by simple memory copying */
        memcpy((void *)data, (const void*)addr, (size_t)len);
        return result;
#else
        (void)result;
        printf("%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
#endif
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;
        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            cy_smif_result = Cy_SMIF_MemRead(SMIF0, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &ota_QSPI_context);
            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
#else
        return CY_RSLT_TYPE_ERROR;
#endif /* CY_IP_MXSMIF & !XMC7200 */
    }
    else
    {
        printf("%s() READ not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

static cy_rslt_t cy_ota_mem_write_row_size( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
        int rc = 0;

        /* flash_area_write() uses offsets, we need absolute address here */
        addr += CY_FLASH_BASE;

#if defined (XMC7200)
        rc = xmc_internal_flash_write((uint8_t *)data, addr, len);
        if (rc != 0 )
        {
            printf("xmc_internal_flash_write(0x%08x, 0x%08x, %u) FAILED rc:%u\n", (unsigned int)data, (unsigned int)addr, len, rc);
            result = CY_RSLT_TYPE_ERROR;
        }
        return result;
#else
        rc = psoc6_internal_flash_write((uint8_t *)data, addr, len);
        if (rc != 0 )
        {
            result = CY_RSLT_TYPE_ERROR;
        }
        return result;
#endif

#else
        (void)result;
        printf("%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
#endif
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            cy_smif_result = Cy_SMIF_MemWrite(SMIF0, smifBlockConfig.memConfig[MEM_SLOT],
                    addr, data, len, &ota_QSPI_context);

            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }
        else
        {
            cy_smif_result = (cy_en_smif_status_t)CY_RSLT_SERIAL_FLASH_ERR_NOT_INITED;
        }

    #ifdef READBACK_SMIF_WRITE_TEST
        if (cy_smif_result == CY_SMIF_SUCCESS)
        {
            if (ota_mem_read(CY_OTA_MEM_TYPE_EXTERNAL_FLASH, addr, read_back_test, ((16 < length) ? 16 : len)) == CY_RSLT_SUCCESS)
            {
                int i;
                for (i=0; (i<16 && i<len); i++)
                {
                    if(data[i] != read_back_test[i])
                    {
                        result  = -1;
                    }
                }
            }
        }
    #endif

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
#else
        return CY_RSLT_TYPE_ERROR;
#endif /* CY_IP_MXSMIF & !XMC7200 */
    }
    else
    {
        printf("%s() Write not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

/**
 * @brief Write to flash, QSPI flash, or any other external memory type
 *
 * @param[in]   mem_type   Memory type @ref cy_ota_mem_type_t
 * @param[in]   addr       Starting address to write to.
 * @param[in]   data       Pointer to the buffer containing the data to be written.
 * @param[in]   len        Number of bytes to write.
 *
 * @return  CY_RSLT_SUCCESS on success
 *          CY_RSLT_TYPE_ERROR on failure
 */
cy_rslt_t cy_ota_mem_write( cy_ota_mem_type_t mem_type, uint32_t addr, void *data, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /**
     * This is used if a block is < Block size to satisfy requirements
     * of flash_area_write(). "static" so it is not on the stack.
     */
    static uint8_t block_buffer[CY_FLASH_SIZEOF_ROW];
    uint32_t chunk_size = 0;

    uint32_t bytes_to_write = len;
    uint32_t curr_addr = addr;
    uint8_t *curr_src = data;

    while(bytes_to_write > 0x0U)
    {
        chunk_size = bytes_to_write;
        if(chunk_size > CY_FLASH_SIZEOF_ROW)
        {
            chunk_size = CY_FLASH_SIZEOF_ROW;
        }

        /* Is the chunk_size smaller than a flash row? */
        if((chunk_size % CY_FLASH_SIZEOF_ROW) != 0x0U)
        {
            uint32_t row_offset = 0;
            uint32_t row_base = 0;

            row_base   = (curr_addr / CY_FLASH_SIZEOF_ROW) * CY_FLASH_SIZEOF_ROW;
            row_offset = curr_addr - row_base;

            if((row_offset + chunk_size) > CY_FLASH_SIZEOF_ROW)
            {
                chunk_size = (CY_FLASH_SIZEOF_ROW - row_offset);
            }

            /* we will read a CY_FLASH_SIZEOF_ROW byte block, write the new data into the block, then write the whole block */
            result = cy_ota_mem_read( mem_type, row_base, (void *)(&block_buffer[0]), sizeof(block_buffer));
            if(result != CY_RSLT_SUCCESS)
            {
                 return CY_RSLT_TYPE_ERROR;
            }
            memcpy (&block_buffer[row_offset], curr_src, chunk_size);

            result = cy_ota_mem_write_row_size(mem_type, row_base, (void *)(&block_buffer[0]), sizeof(block_buffer));
            if(result != CY_RSLT_SUCCESS)
            {
                return CY_RSLT_TYPE_ERROR;
            }
        }
        else
        {
            result = cy_ota_mem_write_row_size(mem_type, addr, data, len);
            if(result != CY_RSLT_SUCCESS)
            {
                return CY_RSLT_TYPE_ERROR;
            }
        }

        curr_addr += chunk_size;
        curr_src += chunk_size;
        bytes_to_write -= chunk_size;
    }

    return CY_RSLT_SUCCESS;
}

/**
 * @brief Erase flash, QSPI flash, or any other external memory type
 *
 * @param[in]   mem_type   Memory type @ref cy_ota_mem_type_t
 * @param[in]   addr       Starting address to begin erasing.
 * @param[in]   len        Number of bytes to erase.
 *
 * @return  CY_RSLT_SUCCESS
 *          CY_RSLT_TYPE_ERROR
 */
cy_rslt_t cy_ota_mem_erase( cy_ota_mem_type_t mem_type, uint32_t addr, size_t len )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
        int rc = 0;

#if defined (XMC7200)
        rc = xmc_internal_flash_erase(addr, len);
        if (rc != 0 )
        {
            printf("xmc_internal_flash_erase(0x%08x, %u) FAILED rc:%d\n", (unsigned int)addr, len, rc);
            result = CY_RSLT_TYPE_ERROR;
        }
#else
        rc = psoc6_internal_flash_erase(addr, len);
        if (rc != 0 )
        {
            result = CY_RSLT_TYPE_ERROR;
        }
#endif
        return result;
#else
        (void)result;
        printf("%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
#endif
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
        cy_en_smif_status_t cy_smif_result = CY_SMIF_SUCCESS;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* pre-access to SMIF */
            PRE_SMIF_ACCESS_TURN_OFF_XIP;

            // If the erase is for the entire chip, use chip erase command
            if ((addr == 0u) && (len == ota_smif_get_memory_size()))
            {
                cy_smif_result = Cy_SMIF_MemEraseChip(SMIF0,
                                                    smifBlockConfig.memConfig[MEM_SLOT],
                                                    &ota_QSPI_context);
            }
            else
            {
                // Cy_SMIF_MemEraseSector() returns error if (addr + length) > total flash size or if
                // addr is not aligned to erase sector size or if (addr + length) is not aligned to
                // erase sector size.
                /* Make sure the base offset is correct */
                uint32_t erase_size;
                uint32_t diff;
                erase_size = cy_ota_mem_get_erase_size(CY_OTA_MEM_TYPE_EXTERNAL_FLASH, addr);
                diff = addr & (erase_size - 1);
                addr -= diff;
                len += diff;
                /* Make sure the length is correct */
                len = (len + (erase_size - 1)) & ~(erase_size - 1);
                Cy_SMIF_SetReadyPollingDelay(20000, &ota_QSPI_context);
                cy_smif_result = Cy_SMIF_MemEraseSector(SMIF0,
                                                      smifBlockConfig.memConfig[MEM_SLOT],
                                                      addr, len, &ota_QSPI_context);
                Cy_SMIF_SetReadyPollingDelay(0, &ota_QSPI_context);
            }

            /* post-access to SMIF */
            POST_SMIF_ACCESS_TURN_ON_XIP;
        }
        else
        {
            return CY_RSLT_SERIAL_FLASH_ERR_NOT_INITED;
        }

        return (cy_smif_result == CY_SMIF_SUCCESS) ? CY_RSLT_SUCCESS : CY_RSLT_TYPE_ERROR;
#else
        return CY_RSLT_TYPE_ERROR;
#endif /* CY_IP_MXSMIF & !XMC7200 */
    }
    else
    {
        printf("%s() Erase not supported for memory type %d\n", __func__, (int)mem_type);
        return CY_RSLT_TYPE_ERROR;
    }
}

/**
 * @brief To get page size for programming flash, QSPI flash, or any other external memory type
 *
 * @param[in]   mem_type   Memory type @ref cy_ota_mem_type_t
 * @param[in]   addr       Address that belongs to the sector for which programming page size needs to be returned.
 *
 * @return    Page size in bytes.
 */
size_t cy_ota_mem_get_prog_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
        return CY_FLASH_SIZEOF_ROW;
#else
        return 0;
#endif
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
        uint32_t    program_size = 0;
        (void)addr; /* Hybrid parts not yet supported */
        /* pre-access to SMIF is not needed, as we are just reading data from RAM */
        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            if (SMIF0 != NULL)
            {
                program_size = smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->programSize;
            }
        }
        /* post-access to SMIF  is not needed, as we are just reading data from RAM */
        return program_size;
#else
        return 0;
#endif /* CY_IP_MXSMIF & !XMC7200 */
    }
    else
    {
        return 0;
    }
}

/**
 * @brief To get sector size of flash, QSPI flash, or any other external memory type
 *
 * @param[in]   mem_type   Memory type @ref cy_ota_mem_type_t
 * @param[in]   addr       Address that belongs to the sector for which sector erase size needs to be returned.
 *
 * @return    Sector size in bytes.
 */
size_t cy_ota_mem_get_erase_size ( cy_ota_mem_type_t mem_type, uint32_t addr )
{
    if( mem_type == CY_OTA_MEM_TYPE_INTERNAL_FLASH )
    {
#if !(defined (CYW20829B0LKML) || defined (CYW89829B01MKSBG))
        return CY_FLASH_SIZEOF_ROW;
#else
        return 0;
#endif
    }
    else if( mem_type == CY_OTA_MEM_TYPE_EXTERNAL_FLASH )
    {
#if (defined (CY_IP_MXSMIF) && !defined (XMC7200))
        uint32_t                            erase_sector_size = 0;
        cy_stc_smif_hybrid_region_info_t*   hybrid_info = NULL;
        cy_en_smif_status_t                 smif_status;

        if (addr >= CY_SMIF_BASE_MEM_OFFSET)
        {
            addr -= CY_SMIF_BASE_MEM_OFFSET;
        }

        /* pre-access to SMIF is not needed, as we are just reading data from RAM */
        if (IS_FLAG_SET(FLAG_HAL_INIT_DONE))
        {
            /* Cy_SMIF_MemLocateHybridRegion() does not access the external flash, just data tables from RAM  */
            smif_status = Cy_SMIF_MemLocateHybridRegion(smifBlockConfig.memConfig[MEM_SLOT], &hybrid_info, addr);

            if (CY_SMIF_SUCCESS != smif_status)
            {
                erase_sector_size = (size_t)smifBlockConfig.memConfig[MEM_SLOT]->deviceCfg->eraseSize;
            }
            else
            {
                erase_sector_size = (size_t)hybrid_info->eraseSize;
            }
        }
        /* post-access to SMIF is not needed, as we are just reading data from RAM */

        return erase_sector_size;
#else
        return 0;
#endif /* CY_IP_MXSMIF & !XMC7200 */
    }
    else
    {
        return 0;
    }
}
