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

#ifndef _ZLINKD_CONTEXT_H_
#define _ZLINKD_CONTEXT_H_

#include "zlink_common.h"

#include "zlink_command.h"
#include "zlink_peripheral.h"
#include "zlink_serial_packet.h"
#ifdef ZLINK_HOST
    #include "zlinkh_cmd_packet.h"
    #include "zlinkh_data_packet.h"
#endif // #ifdef ZLINK_HOST

/*! @addtogroup rom_flexspi */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Configurations
 ******************************************************************************/


/*******************************************************************************
 * Definitions
 ******************************************************************************/

//! @brief Structure of bootloader global context.
typedef struct _zlinkDeviceContext
{
//     //! @name API tree
//     //@{
//     const memory_interface_t *memoryInterface; //!< Abstract interface to memory operations.
//     const memory_map_entry_t *memoryMap;       //!< Memory map used by abstract memory interface.
// #if BL_FEATURE_EXPAND_MEMORY
//     const external_memory_map_entry_t *externalMemoryMap; //!< Memory map used by external memory devices.
// #endif                                                    // BL_FEATURE_EXPAND_MEMORY
//     const property_interface_t *propertyInterface;        //!< Interface to property store.
    const command_interface_t *commandInterface;          //!< Interface to command processor operations.
// #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
// #if !BL_DEVICE_IS_LPC_SERIES
//     const flash_driver_interface_t *flashDriverInterface;    //!< Kinetis Flash driver interface.
// #if BL_FEATURE_SUPPORT_DFLASH
//     const dflash_driver_interface_t *dflashDriverInterface;    //!< Kinetis DFlash driver interface.
// #endif // BL_FEATURE_SUPPORT_DFLASH
// #else
//     const flashiap_driver_interface_t *flashDriverInterface; //!< LPC Flash driver interface.
// #endif // !BL_DEVICE_IS_LPC_SERIES
// #endif // !BL_FEATURE_HAS_NO_INTERNAL_FLASH
//     const peripheral_descriptor_t *allPeripherals;        //!< Array of all peripherals.
//     const aes_driver_interface_t *aesInterface;           //!< Interface to the AES driver
//     //@}

//     //! @name Runtime state
//     //@{
    const peripheral_descriptor_t *activePeripheral; //!< The currently active peripheral.
// #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
// #if !BL_DEVICE_IS_LPC_SERIES
//     flash_config_t *allFlashState;                   //!< Kinetis Flash driver instance.
//     ftfx_cache_config_t *allFlashCacheState;                   //!< FTFx cache driver state information
// #if BL_FEATURE_SUPPORT_DFLASH
//     flexnvm_config_t *dFlashState;             //!< Kinetis DFlash driver instance.
// #endif     
// #else
//     flashiap_config_t *allFlashState;                //!< LPC Flash driver instance.
// #endif
// #endif
    //@}
#ifdef ZLINK_HOST
    const zlinkh_cmd_packet_t *cmdPacketInterface;

    standard_version_t m_version; //!< Framing protocol version.
    uint16_t m_options;           //!< Framing protocol options bitfield.
#endif // #ifdef ZLINK_HOST
} zlink_device_context_t;


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
 * Variables
 ******************************************************************************/
extern zlink_device_context_t g_zLinkDeviceContext;



/*******************************************************************************
 * Prototypes
 ******************************************************************************/


 
/*******************************************************************************
 * API
 ******************************************************************************/

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _ZLINKD_CONTEXT_H_ */
