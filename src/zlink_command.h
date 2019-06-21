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

#ifndef _ZLINK_COMMAND_H_
#define _ZLINK_COMMAND_H_

#include "zlink_common.h"

/*! @addtogroup rom_flexspi */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Configurations
 ******************************************************************************/


/*******************************************************************************
 * Definitions
 ******************************************************************************/

//! @brief Commands codes.
enum _command_tags
{
    zLinkCommandTag_GenericResponse = 0xa0,
    zLinkCommandTag_FlashEraseAll = 0x01,
    zLinkCommandTag_FlashEraseRegion = 0x02,
    zLinkCommandTag_ReadMemory = 0x03,
    zLinkCommandTag_ReadMemoryResponse = 0xa3,
    zLinkCommandTag_WriteMemory = 0x04,
    zLinkCommandTag_FillMemory = 0x05,
    zLinkCommandTag_FlashSecurityDisable = 0x06,
    zLinkCommandTag_GetProperty = 0x07,
    zLinkCommandTag_GetPropertyResponse = 0xa7,
    zLinkCommandTag_ReceiveSbFile = 0x08,
    zLinkCommandTag_Execute = 0x09,
    zLinkCommandTag_Call = 0x0a,
    zLinkCommandTag_Reset = 0x0b,
    zLinkCommandTag_SetProperty = 0x0c,
    zLinkCommandTag_FlashEraseAllUnsecure = 0x0d,
    zLinkCommandTag_FlashProgramOnce = 0x0e,
    zLinkCommandTag_FlashReadOnce = 0x0f,
    zLinkCommandTag_FlashReadOnceResponse = 0xaf,
    zLinkCommandTag_FlashReadResource = 0x10,
    zLinkCommandTag_FlashReadResourceResponse = 0xb0,
    zLinkCommandTag_ConfigureMemory = 0x11,
    zLinkCommandTag_ReliableUpdate = 0x12,
    zLinkCommandTag_GenerateKeyBlob = 0x13,
    zLinkCommandTag_GenerateKeyBlobResponse = 0xb3,
    zLinkCommandTag_KeyProvisioning = 0x15,
    zLinkCommandTag_KeyProvisioningResponse = 0xb5,

    zLinkCommandTag_ConfigureI2c = 0xc1, //! Reserved command tag for Bus Pal
    zLinkCommandTag_ConfigureSpi = 0xc2, //! Reserved command tag for Bus Pal
    zLinkCommandTag_ConfigureCan = 0xc3, //! Reserved command tag for Bus Pal

    zLinkCommandTag_GetDeviceInfo = 0x21, // GetDeviceInfo
    zLinkCommandTag_GetLockStat = 0x22, // GetLockStat

    zLinkCommandTag_GetDeviceInfoResponse = 0xc0, // DeviceInfoResponse

    zLinkFirstCommandTag = zLinkCommandTag_FlashEraseAll,

    //! Maximum linearly incrementing command tag value, excluding the response commands and bus pal commands.
    zLinkLastCommandTag = zLinkCommandTag_GetLockStat,

    zLinkResponseCommandHighNibbleMask =
        0xa0 //!< Mask for the high nibble of a command tag that identifies it as a response command.
};


//! @brief Property tags.
//! @note Do not change any tag values. Add tags at the end.
enum _property_tag
{
    zLinkPropertyTag_ListProperties = 0x00,
    zLinkPropertyTag_BootloaderVersion = 0x01,
    zLinkPropertyTag_AvailablePeripherals = 0x02,
    zLinkPropertyTag_FlashStartAddress = 0x03,
    zLinkPropertyTag_FlashSizeInBytes = 0x04,
    zLinkPropertyTag_FlashSectorSize = 0x05,
    zLinkPropertyTag_FlashBlockCount = 0x06,
    zLinkPropertyTag_AvailableCommands = 0x07,
    zLinkPropertyTag_CrcCheckStatus = 0x08,
    zLinkPropertyTag_Reserved9 = 0x09,
    zLinkPropertyTag_VerifyWrites = 0x0a,
    zLinkPropertyTag_MaxPacketSize = 0x0b,
    zLinkPropertyTag_ReservedRegions = 0x0c,
    zLinkPropertyTag_Reserved13 = 0x0d,
    zLinkPropertyTag_RAMStartAddress = 0x0e,
    zLinkPropertyTag_RAMSizeInBytes = 0x0f,
    zLinkPropertyTag_SystemDeviceId = 0x10,
    zLinkPropertyTag_FlashSecurityState = 0x11,
    zLinkPropertyTag_UniqueDeviceId = 0x12,
    zLinkPropertyTag_FacSupport = 0x13,
    zLinkPropertyTag_FlashAccessSegmentSize = 0x14,
    zLinkPropertyTag_FlashAccessSegmentCount = 0x15,
    zLinkPropertyTag_FlashReadMargin = 0x16,
    zLinkPropertyTag_QspiInitStatus = 0x17,
    zLinkPropertyTag_TargetVersion = 0x18,
    zLinkPropertyTag_ExternalMemoryAttributes = 0x19,
    zLinkPropertyTag_ReliableUpdateStatus = 0x1a,
    zLinkPropertyTag_FlashPageSize = 0x1b,
    zLinkPropertyTag_IrqNotifierPin = 0x1c,
    zLinkPropertyTag_FfrKeystoreUpdateOpt = 0x1d,
    zLinkPropertyTag_InvalidProperty = 0xFF,
};


////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////
//! @brief Bit mask for device ID.
#define DEVICE_ID_MASK 0xff
//! @brief Bit position of device ID.
#define DEVICE_ID_SHIFT 0
//! @brief Bit mask for group ID.
#define GROUP_ID_MASK 0xf00
//! @brief Bit position of group ID.
#define GROUP_ID_SHIFT 8

/*! @brief Construct a memory ID from a given group ID and device ID. */
#define MAKE_MEMORYID(group, device) \
    ((((group) << GROUP_ID_SHIFT) & GROUP_ID_MASK) | (((device) << DEVICE_ID_SHIFT) & DEVICE_ID_MASK))
/*! @brief Get group ID from a given memory ID. */
#define GROUPID(memoryId) (((memoryId)&GROUP_ID_MASK) >> GROUP_ID_SHIFT)

/*! @brief Get device ID from a given memory ID. */
#define DEVICEID(memoryId) (((memoryId)&DEVICE_ID_MASK) >> DEVICE_ID_SHIFT)
/*@}*/


/*! @brief Memory group definition. */
enum _bl_memory_groups
{
    zLinkGroup_Internal = 0, //!<  Kinetis internal 4G memory region.
    zLinkGroup_External = 1, //!<  Kinetis external memory region.
};

/*! @brief Memory device ID definition. */
enum _bl_memory_id
{
    /*  Memory ID bitfiled definition.
        | 11 | 10 | 9 |    8    |  7   |  6   |  5   |  4  |  3   |  2   |  1  |  0  |
        |  Reserved   | INT/EXT | Type                     | Sub-Type                |
        |             | 0: INT  | INT:                     |                         |
        |             | 1: EXT  | 0: NorFlash0             | 0: Internal Flash(FTFX) |
        |             |         |                          | 1: QSPI                 |
        |             |         |                          | 4: IFR                  |
        |             |         |                          | 8: SEMC                 |
        |             |         |                          | 9: FlexSPI              |
        |             |         |                          | A: SPIFI                |
        |             |         |                          | others: Unused          |
        |             |         |                          |                         |
        |             |         | 1: ExecuteOnlyRegion     | 0: Internal Flash(FTFX) |
        |             |         |                          | others: Unused          |
        |             |         |                          |                         |
        |             |         | others: Unused           |                         |
        |             |         |                          |                         |
        |             |         | EXT:                     |                         |
        |             |         | 0: NandFlash             | 0: SEMC                 |
        |             |         |                          | 1: FlexSPI              |
        |             |         |                          | others: Unused          |
        |             |         |                          |                         |
        |             |         | 1: NorFlash/EEPROM       | 0: LPSPI                |
        |             |         |                          | 1: LPI2C                |
        |             |         |                          | others: Unused          |
        |             |         |                          |                         |
        |             |         | 2: SD/SDHC/SDXC/MMC/eMMC | 0: uSDHC SD             |
        |             |         |                          | 1: uSDHC MMC            |
        |             |         |                          | others: Unused          |
        |             |         | others: Unused           |                         |

        INT : Internal 4G memory, including internal memory modules, and XIP external memory modules.
        EXT : Non-XIP external memory modules.
    */
    zLinkMemoryInternal = MAKE_MEMORYID(zLinkGroup_Internal, 0), // Internal memory (include all on chip memory)
    zLinkMemoryQuadSpi0 = MAKE_MEMORYID(zLinkGroup_Internal, 1), // Qsuad SPI memory 0
    zLinkMemoryIFR0 = MAKE_MEMORYID(zLinkGroup_Internal, 4),     // Nonvolatile information register 0. Only used by SB loader.
    zLinkMemorySemcNor = MAKE_MEMORYID(zLinkGroup_Internal, 8),  // SEMC Nor memory
    zLinkMemoryFlexSpiNor = MAKE_MEMORYID(zLinkGroup_Internal, 9),          // Flex SPI Nor memory
    zLinkMemorySpifiNor = MAKE_MEMORYID(zLinkGroup_Internal, 0xA),          // SPIFI Nor memory
    zLinkMemoryFlashExecuteOnly = MAKE_MEMORYID(zLinkGroup_Internal, 0x10), // Execute-only region on internal Flash

    zLinkMemorySemcNand = MAKE_MEMORYID(zLinkGroup_External, 0),        // SEMC NAND memory
    zLinkMemorySpiNand = MAKE_MEMORYID(zLinkGroup_External, 1),         // SPI NAND memory
    zLinkMemorySpiNorEeprom = MAKE_MEMORYID(zLinkGroup_External, 0x10), // SPI NOR/EEPROM memory
    zLinkMemoryI2cNorEeprom = MAKE_MEMORYID(zLinkGroup_External, 0x11), // I2C NOR/EEPROM memory
    zLinkMemorySDCard = MAKE_MEMORYID(zLinkGroup_External, 0x20),       // eSD, SD, SDHC, SDXC memory Card
    zLinkMemoryMMCCard = MAKE_MEMORYID(zLinkGroup_External, 0x21),      // MMC, eMMC memory Card
    //
};

//! @brief Command state machine states.
enum _command_state
{
    zLinkCommandState_CommandPhase,
    zLinkCommandState_DataPhase
};

//! @brief Command packet flags.
enum _command_packet_flags
{
    zLinkCommandFlag_None = 0,
    zLinkCommandFlag_HasDataPhase = 1
};

//! @brief Command packet format.
typedef struct CommandPacket
{
    uint8_t commandTag;     //!< A command tag.
    uint8_t flags;          //!< Combination of packet flags.
    uint8_t reserved;       //!< Reserved, helpful for alignment, set to zero.
    uint8_t parameterCount; //!< Number of parameters that follow in buffer.
} command_packet_t;


//! @brief Generic response packet format.
typedef struct GenericResponsePacket
{
    command_packet_t commandPacket; //!< header
    uint32_t status;                //!< parameter 0
    uint32_t commandTag;            //!< parameter 1
} generic_response_packet_t;

//! @brief ReadMemory packet format.
typedef struct ReadMemoryPacket
{
    command_packet_t commandPacket; //!< header
    uint32_t startAddress;          //!< Paremeter 0: Start address of memory to read from.
    uint32_t byteCount;             //!< Parameter 1: Number of bytes to read.
    uint32_t memoryId;              //!< Parameter 2: ID of the Memory Device to read from.
} read_memory_packet_t;

//! @brief Read Memory response packet format.
typedef struct ReadMemoryResponsePacket
{
    command_packet_t commandPacket; //!< header
    uint32_t status;                //!< parameter 0
    uint32_t dataByteCount;         //!< parameter 1
} read_memory_response_packet_t;

//! @brief Command packet flags.
enum _device_info_index
{
    zLinkDeviceInfo_ProductKey = 1,
    zLinkDeviceInfo_ProductSecret = 2,
    zLinkDeviceInfo_DeviceName = 3,
    zLinkDeviceInfo_DeviceSecret = 4,
    zLinkDeviceInfo_AppVersion = 5
};

//! @brief ReadMemory packet format.
typedef struct GetDeviceInfoPacket
{
    command_packet_t commandPacket; //!< header
    uint32_t info_index;          //!< Paremeter 0: index of device info to read from.
} get_deviceinfo_packet_t;

//! @brief Format of command handler entry.
typedef struct CommandHandlerEntry
{
    void (*handleCommand)(uint8_t *packet, uint32_t packetLength);
    status_t (*handleData)(bool *hasMoreData);
} command_handler_entry_t;

//! @brief Command processor data format.
typedef struct CommandProcessorData
{
    int32_t state;         //!< Current state machine state
    uint8_t *packet;       //!< Pointer to packet in process
    uint32_t packetLength; //!< Length of packet in process
    struct DataPhase
    {
        uint8_t *data;               //!< Data for data phase
        uint32_t count;              //!< Remaining count to produce/consume
        uint32_t address;            //!< Address for data phase
        uint32_t memoryId;           //!< ID of the target memory
        uint32_t dataBytesAvailable; //!< Number of bytes available at data pointer
        uint8_t commandTag;          //!< Tag of command running data phase
        uint8_t option;              //!< option for special command
        uint32_t argument0;          //!< argument0 for special command
        uint32_t argument1;          //!< arugment1 for special command
        uint32_t argument2;          //!< arugment2 for special command

#if defined(__cplusplus)
        DataPhase()
            : data(NULL)
            , count(0)
            , address(0)
            , dataBytesAvailable(0)
            , commandTag(0)
            , option(0)
            , argument0(0)
            , argument1(1)
        {
        }
#endif
    } dataPhase;
    const command_handler_entry_t *handlerEntry; //! Pointer to handler table entry for packet in process
#if defined(__cplusplus)
    CommandProcessorData()
        : state(kCommandState_CommandPhase)
        , packet(NULL)
        , packetLength(0)
        , dataPhase()
    {
    }
#endif
} command_processor_data_t;

//! @brief Interface to command processor operations.
typedef struct CommandInterface
{
    status_t (*init)(void);
    status_t (*pump)(void);
    const command_handler_entry_t *handlerTable;
    command_processor_data_t *stateData;
} command_interface_t;

#ifdef ZLINK_HOST

typedef struct __zlink_command {
    void (*pfun_constructor_packet) (void *arg);
    //! @brief Initialize.
    //!
    //! Subclasses should implement init() to check for valid arguments.
    void (*pfun_init) (void *arg);
    void (*pfun_init2arg) (void *arg, void *data_cousmer);
    status_t (*pfun_sendTo) (void *packe_tizer);
    void (*pfun_progress_cb)(int percentage, int segmentIndex, int segmentCount); //!< The progress callback function.
} zlinkh_command_t;

#endif

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/*******************************************************************************
 * Variables
 ******************************************************************************/

extern command_interface_t g_commandInterface;



/*******************************************************************************
 * Prototypes
 ******************************************************************************/


 
/*******************************************************************************
 * API
 ******************************************************************************/

//! @brief Initialize the command processor component.
extern status_t zlink_command_init(void);

//! @brief Pump the command state machine.
//!
//! Executes one command or data phase transaction.
extern status_t zlink_command_pump(void);

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _ZLINK_DEVICE_H_ */
