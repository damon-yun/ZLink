/*******************************************************************************
 * zlink_device.h - definitions for the zlink device interface			     	
 *
 * The Clear BSD License
 * Copyright (C) 2019 Damon Zhang
 * All rights reserved.
 *
 * Author : Damon Zhang
 * Website: https://damon-yun.github.io/blog.github.io/
 * E-mail : damoncheung@foxmail.com
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Modification History
 * - 1.01 19-05-15  dmaon, modified
 * - 1.00 19-05-11  damon, first implementation
 *
 * note
 *    zlink设备端源码
 *
 *****************************************************************************/

#ifndef _ZLINK_COMMON_H_
#define _ZLINK_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "fsl_common.h"

/*! @addtogroup zlink_common */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Configurations
 ******************************************************************************/
#define zLinkMaxCommandArguments                7
#define zLinkDefaultFramingPacketBufferSize     1024
#define zLinkMaxHostPacketSize                  4096
#define zLinkMinPacketBufferSize                512

#define zLinkMinFramingPacketBufferSize         zLinkDefaultFramingPacketBufferSize

#define zLinkCallbackBufferSize                 64    // (frame 4 + 2 + 2 + data 256)  //!< Size for callback buffer, Must be power of 2 for easy wrap


#ifdef ZLINK_HOST
#define zLinkIncomingPacketBufferSize          zLinkMaxHostPacketSize
#define zLinkOutgoingPacketBufferSize          zLinkMaxHostPacketSize
#else
#define zLinkIncomingPacketBufferSize          zLinkMinFramingPacketBufferSize
#define zLinkOutgoingPacketBufferSize          zLinkMinFramingPacketBufferSize
#endif


//! @brief Timeout and other constants.
enum _timing_constants
{
    zHostMaxStartByteReadCount = 2,
    zDefaultByteReadTimeoutMs = 50, //!< Default value for receiving 1 byte timeout
    zCallbackBufferSize = 64        //!< Size for callback buffer, Must be power of 2 for easy wrap
};

enum
{
    zReadRetries = 10,
    zReadDelayMilliseconds = 10
};

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Construct a status code value from a group and code number. */
#define ZLINK_MAKE_STATUS(group, code) ((((group)*100) + (code)))

/*! @brief Construct the version number for drivers. */
#define ZLINK_MAKE_VERSION(major, minor, bugfix) (((major) << 16) | ((minor) << 8) | (bugfix))


/*! @name Driver version */
/*@{*/

/*! @brief FileName driver version 2.1.2. */
#define FSL_ZLINK_DEVICE_VERSION (ZLINK_MAKE_VERSION(1, 0, 0))  /*!< a simple for define */

/*@}*/


/* Suppress unused parameter warning */
#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(x) /*@-noeffect@*/ ( (void)(x) ) /*@+noeffect@*/
#endif

/* Suppress unused variable warning */
#ifndef UNUSED_VARIABLE
#define UNUSED_VARIABLE(x) /*@-noeffect@*/ ( (void)(x) ) /*@+noeffect@*/
#endif


//! @name Min/max macros
//@{
#if !defined(MIN)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if !defined(MAX)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
//@}

//! @brief Computes the number of elements in an array.
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

//! @name Byte swap macros
//@{
#define BSWAP_16(x) (uint16_t)((((x)&0xFF00) >> 0x8) | (((x)&0xFF) << 0x8))
#define BSWAP_32(val) \
    (uint32_t)((BSWAP_16((uint32_t)(val) & (uint32_t)0xFFFF) << 0x10) | (BSWAP_16((uint32_t)((val) >> 0x10))))
//@}

//! @name Alignment macros
//@{
#ifndef ALIGN_DOWN
#define ALIGN_DOWN(x, a) ((x) & -(a))
#endif
#ifndef ALIGN_UP
#define ALIGN_UP(x, a) (-(-(x) & -(a)))
#endif
//@}

/*******************************************************************************
 * zlink status define
 ******************************************************************************/

#ifndef FSL_COMMON_DRIVER_VERSION

/*! @brief Status group numbers. */
enum _status_groups
{
    kStatusGroup_Generic = 0,                 /*!< Group number for generic status codes. */
    kStatusGroup_FLASH = 1,                   /*!< Group number for FLASH status codes. */
    kStatusGroup_LPSPI = 4,                   /*!< Group number for LPSPI status codes. */
    kStatusGroup_FLEXIO_SPI = 5,              /*!< Group number for FLEXIO SPI status codes. */
    kStatusGroup_DSPI = 6,                    /*!< Group number for DSPI status codes. */
    kStatusGroup_FLEXIO_UART = 7,             /*!< Group number for FLEXIO UART status codes. */
    kStatusGroup_FLEXIO_I2C = 8,              /*!< Group number for FLEXIO I2C status codes. */
    kStatusGroup_LPI2C = 9,                   /*!< Group number for LPI2C status codes. */
    kStatusGroup_UART = 10,                   /*!< Group number for UART status codes. */
    kStatusGroup_I2C = 11,                    /*!< Group number for UART status codes. */
    kStatusGroup_LPSCI = 12,                  /*!< Group number for LPSCI status codes. */
    kStatusGroup_LPUART = 13,                 /*!< Group number for LPUART status codes. */
    kStatusGroup_SPI = 14,                    /*!< Group number for SPI status code.*/
    kStatusGroup_XRDC = 15,                   /*!< Group number for XRDC status code.*/
    kStatusGroup_SEMA42 = 16,                 /*!< Group number for SEMA42 status code.*/
    kStatusGroup_SDHC = 17,                   /*!< Group number for SDHC status code */
    kStatusGroup_SDMMC = 18,                  /*!< Group number for SDMMC status code */
    kStatusGroup_SAI = 19,                    /*!< Group number for SAI status code */
    kStatusGroup_MCG = 20,                    /*!< Group number for MCG status codes. */
    kStatusGroup_SCG = 21,                    /*!< Group number for SCG status codes. */
    kStatusGroup_SDSPI = 22,                  /*!< Group number for SDSPI status codes. */
    kStatusGroup_FLEXIO_I2S = 23,             /*!< Group number for FLEXIO I2S status codes */
    kStatusGroup_FLEXIO_MCULCD = 24,          /*!< Group number for FLEXIO LCD status codes */
    kStatusGroup_FLASHIAP = 25,               /*!< Group number for FLASHIAP status codes */
    kStatusGroup_FLEXCOMM_I2C = 26,           /*!< Group number for FLEXCOMM I2C status codes */
    kStatusGroup_I2S = 27,                    /*!< Group number for I2S status codes */
    kStatusGroup_IUART = 28,                  /*!< Group number for IUART status codes */
    kStatusGroup_CSI = 29,                    /*!< Group number for CSI status codes */
    kStatusGroup_MIPI_DSI = 30,               /*!< Group number for MIPI DSI status codes */
    kStatusGroup_SDRAMC = 35,                 /*!< Group number for SDRAMC status codes. */
    kStatusGroup_POWER = 39,                  /*!< Group number for POWER status codes. */
    kStatusGroup_ENET = 40,                   /*!< Group number for ENET status codes. */
    kStatusGroup_PHY = 41,                    /*!< Group number for PHY status codes. */
    kStatusGroup_TRGMUX = 42,                 /*!< Group number for TRGMUX status codes. */
    kStatusGroup_SMARTCARD = 43,              /*!< Group number for SMARTCARD status codes. */
    kStatusGroup_LMEM = 44,                   /*!< Group number for LMEM status codes. */
    kStatusGroup_QSPI = 45,                   /*!< Group number for QSPI status codes. */
    kStatusGroup_DMA = 50,                    /*!< Group number for DMA status codes. */
    kStatusGroup_EDMA = 51,                   /*!< Group number for EDMA status codes. */
    kStatusGroup_DMAMGR = 52,                 /*!< Group number for DMAMGR status codes. */
    kStatusGroup_FLEXCAN = 53,                /*!< Group number for FlexCAN status codes. */
    kStatusGroup_LTC = 54,                    /*!< Group number for LTC status codes. */
    kStatusGroup_FLEXIO_CAMERA = 55,          /*!< Group number for FLEXIO CAMERA status codes. */
    kStatusGroup_LPC_SPI = 56,                /*!< Group number for LPC_SPI status codes. */
    kStatusGroup_LPC_USART = 57,              /*!< Group number for LPC_USART status codes. */
    kStatusGroup_DMIC = 58,                   /*!< Group number for DMIC status codes. */
    kStatusGroup_SDIF = 59,                   /*!< Group number for SDIF status codes.*/
    kStatusGroup_SPIFI = 60,                  /*!< Group number for SPIFI status codes. */
    kStatusGroup_OTP = 61,                    /*!< Group number for OTP status codes. */
    kStatusGroup_MCAN = 62,                   /*!< Group number for MCAN status codes. */
    kStatusGroup_CAAM = 63,                   /*!< Group number for CAAM status codes. */
    kStatusGroup_ECSPI = 64,                  /*!< Group number for ECSPI status codes. */
    kStatusGroup_USDHC = 65,                  /*!< Group number for USDHC status codes.*/
    kStatusGroup_LPC_I2C = 66,                /*!< Group number for LPC_I2C status codes.*/
    kStatusGroup_DCP = 67,                    /*!< Group number for DCP status codes.*/
    kStatusGroup_MSCAN = 68,                  /*!< Group number for MSCAN status codes.*/
    kStatusGroup_ESAI = 69,                   /*!< Group number for ESAI status codes. */
    kStatusGroup_FLEXSPI = 70,                /*!< Group number for FLEXSPI status codes. */
    kStatusGroup_MMDC = 71,                   /*!< Group number for MMDC status codes. */
    kStatusGroup_PDM = 72,                    /*!< Group number for MIC status codes. */
    kStatusGroup_SDMA = 73,                   /*!< Group number for SDMA status codes. */
    kStatusGroup_ICS = 74,                    /*!< Group number for ICS status codes. */
    kStatusGroup_SPDIF = 75,                  /*!< Group number for SPDIF status codes. */
    kStatusGroup_LPC_MINISPI = 76,            /*!< Group number for LPC_MINISPI status codes. */
    kStatusGroup_HASHCRYPT = 77,              /*!< Group number for Hashcrypt status codes */
    kStatusGroup_LPC_SPI_SSP = 78,            /*!< Group number for LPC_SPI_SSP status codes. */
    kStatusGroup_LPC_I2C_1 = 97,              /*!< Group number for LPC_I2C_1 status codes. */
    kStatusGroup_NOTIFIER = 98,               /*!< Group number for NOTIFIER status codes. */
    kStatusGroup_DebugConsole = 99,           /*!< Group number for debug console status codes. */
    kStatusGroup_SEMC = 100,                  /*!< Group number for SEMC status codes. */    
    kStatusGroup_ApplicationRangeStart = 101, /*!< Starting number for application groups. */
    kStatusGroup_IAP = 102,                   /*!< Group number for IAP status codes */

    kStatusGroup_HAL_GPIO = 121,              /*!< Group number for HAL GPIO status codes. */
    kStatusGroup_HAL_UART = 122,              /*!< Group number for HAL UART status codes. */
    kStatusGroup_HAL_TIMER = 123,             /*!< Group number for HAL TIMER status codes. */
    kStatusGroup_HAL_SPI = 124,               /*!< Group number for HAL SPI status codes. */
    kStatusGroup_HAL_I2C = 125,               /*!< Group number for HAL I2C status codes. */
    kStatusGroup_HAL_FLASH = 126,             /*!< Group number for HAL FLASH status codes. */
    kStatusGroup_HAL_PWM = 127,               /*!< Group number for HAL PWM status codes. */
    kStatusGroup_HAL_RNG = 128,               /*!< Group number for HAL RNG status codes. */
    kStatusGroup_TIMERMANAGER = 135,          /*!< Group number for TiMER MANAGER status codes. */
    kStatusGroup_SERIALMANAGER = 136,         /*!< Group number for SERIAL MANAGER status codes. */
    kStatusGroup_LED = 137,                   /*!< Group number for LED status codes. */
    kStatusGroup_BUTTON = 138,                /*!< Group number for BUTTON status codes. */
    kStatusGroup_EXTERN_EEPROM = 139,         /*!< Group number for EXTERN EEPROM status codes. */
    kStatusGroup_SHELL = 140,                 /*!< Group number for SHELL status codes. */
    kStatusGroup_MEM_MANAGER = 141,           /*!< Group number for MEM MANAGER status codes. */
    kStatusGroup_LIST = 142,                  /*!< Group number for List status codes. */
    kStatusGroup_OSA = 143,                   /*!< Group number for OSA status codes. */
    kStatusGroup_COMMON_TASK = 144,           /*!< Group number for Common task status codes. */
    kStatusGroup_MSG = 145,                   /*!< Group number for messaging status codes. */
};

/*! @brief Generic status return codes. */
enum _generic_status
{
    kStatus_Success = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 0),
    kStatus_Fail = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 1),
    kStatus_ReadOnly = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 2),
    kStatus_OutOfRange = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 3),
    kStatus_InvalidArgument = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 4),
    kStatus_Timeout = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 5),
    kStatus_NoTransferInProgress = ZLINK_MAKE_STATUS(kStatusGroup_Generic, 6),
};

/*! @brief Type used for all status and error return values. */
typedef int32_t status_t;

#endif

//! @brief Bootloader status group numbers.
//!
//! @ingroup bl_core
enum _bl_status_groups
{
    kStatusGroup_Zlink = 100,            //!< Bootloader status group number (100).
    kStatusGroup_SBLoader = 101,              //!< SB loader status group number (101).
    kStatusGroup_MemoryInterface = 102,       //!< Memory interface status group number (102).
    kStatusGroup_PropertyStore = 103,         //!< Property store status group number (103).
    kStatusGroup_AppCrcCheck = 104,           //!< Application crc check status group number (104).
    kStatusGroup_Packetizer = 105,            //!< Packetizer status group number (105).
    kStatusGroup_ReliableUpdate = 106,        //!< Reliable Update status groupt number (106).

    kStatusGroup_SerialNorEeprom = 107,       //!< Serial NOR/EEPROM status group number
    kStatusGroup_FlexSPINAND = 200,           //!< FlexSPINAND status group number.
    kStatusGroup_FLEXSPINOR = 201,            //!< FlexSPINOR status group number.
    kStatusGroup_OCOTP = 202,                 //!< OCOTP status group number.
    kStatusGroup_SemcNOR = 211,               //!< SEMC NOR status group number.
    kStatusGroup_SemcNAND = 212,              //!< SEMC NAND status group number.
};

//! @brief Bootloader status codes.
//! @ingroup bl_core
enum _bootloader_status
{
    kStatus_UnknownCommand = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 0),
    kStatus_SecurityViolation = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 1),
    kStatus_AbortDataPhase = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 2),
    kStatus_Ping = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 3),
    kStatus_NoResponse = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 4),
    kStatus_NoResponseExpected = ZLINK_MAKE_STATUS(kStatusGroup_Zlink, 5)
};

//! @brief Memory interface status codes.
enum _memory_interface_status
{
    kStatusMemoryRangeInvalid = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 0),
    kStatusMemoryReadFailed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 1),
    kStatusMemoryWriteFailed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 2),
    kStatusMemoryCumulativeWrite = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 3),
    kStatusMemoryAppOverlapWithExecuteOnlyRegion = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 4),
    kStatusMemoryNotConfigured = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 5),
    kStatusMemoryAlignmentError = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 6),
    kStatusMemoryVerifyFailed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 7),
    kStatusMemoryWriteProtected = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 8),
    kStatusMemoryAddressError = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 9),
    kStatusMemoryBlankCheckFailed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 10),
    kStatusMemoryBlankPageReadDisallowed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 11),
    kStatusMemoryProtectedPageReadDisallowed = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 12),
    kStatusMemoryUnsupportedCommand = ZLINK_MAKE_STATUS(kStatusGroup_MemoryInterface, 13),

};

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
 
/*******************************************************************************
 * API
 ******************************************************************************/

#if (defined(ZLINK_DEBUG) && !defined(ZLINK_DEBUG_PRINT_DISABLE))
static inline void zlink_printf(const char *format, ...);

//! @brief Debug print utility.
//!
//! This print function will only output text when the @a DEBUG macro is defined.
static inline void zlink_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
// Temporarily disable MISRA rule 14.2
#if defined(__ICCARM__)
#pragma diag_suppress = Pm049
#endif
    va_end(args);
#if defined(__ICCARM__)
#pragma diag_default = Pm049
#endif
}
#else // (DEBUG || _DEBUG) && !DEBUG_PRINT_DISABLE
// Empty macro to cause debug_printf() calls to disappear.
#define zlink_printf(x, ...) \
    do                       \
    {                        \
    } while (false)
#endif // (DEBUG || _DEBUG) && !DEBUG_PRINT_DISABLE

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _ZLINK_COMMON_H_ */
