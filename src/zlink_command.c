/*******************************************************************************
 * zlink_device.c - definitions for the zlink device interface			     	
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
 *    如需观察串口打印的调试信息，需要将 PIO0_0 引脚连接 PC 串口的 TXD，
 *    PIO0_4 引脚连接 PC 串口的 RXD。
 *
 *****************************************************************************/

#include "zlink_common.h"

#include "zlink_context.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "zlink.command.inteface"
#endif

/*******************************************************************************
 * Configurations
 ******************************************************************************/



/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//! @name State machine
//@{
static status_t handle_command(uint8_t *packet, uint32_t packetLength);
static status_t handle_data(bool *hasMoreData);
//@}


//! @name Command handlers
//@{
void handle_reset(uint8_t *packet, uint32_t packetLength);
void handle_read_memory(uint8_t *packet, uint32_t packetLength);
void handle_get_deviceinfo (uint8_t *packet, uint32_t packetLength);
//@}

//! @name Command responses
//@{

void send_generic_response(uint32_t commandStatus, uint32_t commandTag);

//@}

//! @name Data phase
//@{
// static void reset_data_phase(void);
// void finalize_data_phase(status_t status);
status_t handle_data_producer(bool *hasMoreData);
// status_t handle_data_consumer(bool *hasMoreData);
// status_t handle_data_bidirection(bool *hasMoreData);
status_t handle_deviceinfo_producer(bool *hasMoreData);
//@}

/*******************************************************************************
 * Variables
 ******************************************************************************/


//! @brief Command handler table.
const command_handler_entry_t g_commandHandlerTable[] = {
    { 0 }, // zLinkCommandTag_FlashEraseAll = 0x01
    { 0 }, // zLinkCommandTag_FlashEraseRegion = 0x02
    { handle_read_memory, handle_data_producer }, // zLinkCommandTag_ReadMemory = 0x03
    { 0 }, // zLinkCommandTag_WriteMemory = 0x04
    { 0 }, // zLinkCommandTag_FillMemory = 0x05
    { 0 }, // zLinkCommandTag_FlashSecurityDisable = 0x06
    { 0 }, // zLinkCommandTag_GetProperty = 0x07
    { 0 }, // zLinkCommandTag_ReceiveSbFile = 0x08
    { 0 }, // zLinkCommandTag_Execute = 0x09
    { 0 }, // zLinkCommandTag_Call = 0x0a
    { handle_reset, NULL },                           // zLinkCommandTag_Reset = 0x0b
    { 0 }, // zLinkCommandTag_SetProperty = 0x0c
    { 0 }, // zLinkCommandTag_FlashEraseAllUnsecure = 0x0d
    { 0 }, // zLinkCommandTag_FlashProgramOnce = 0x0e
    { 0 }, // zLinkCommandTag_FlashReadOnce = 0x0f
    { 0 }, // zLinkCommandTag_ReadResource = 0x10
    { 0 }, // zLinkCommandTag_ConfigureMemory = 0x11
    { 0 }, // zLinkCommandTag_ReliableUpdate = 0x12
    { 0 }, // zLinkCommandTag_GenerateKeyBlob = 0x13
    { 0 }, //! Reserved command tag 0x14
    { 0 }, // zLinkCommandTag_KeyProvisioning = 0x15
    { 0 }, //! Reserved command tag 0x16
    { 0 }, //! Reserved command tag 0x17
    { 0 }, //! Reserved command tag 0x18
    { 0 }, //! Reserved command tag 0x19
    { 0 }, //! Reserved command tag 0x1a
    { 0 }, //! Reserved command tag 0x1b
    { 0 }, //! Reserved command tag 0x1c
    { 0 }, //! Reserved command tag 0x1d
    { 0 }, //! Reserved command tag 0x1e
    { 0 }, //! Reserved command tag 0x1f
    { 0 }, //! Reserved command tag 0x20
    { handle_get_deviceinfo, handle_deviceinfo_producer}, // zLinkCommandTag_GetDeviceInfo = 0x21
    { 0 }, //! Reserved command tag 0x22
    { 0 }, //! Reserved command tag 0x23
    { 0 }, //! Reserved command tag 0x24
    { 0 }, //! Reserved command tag 0x25
    { 0 }, //! Reserved command tag 0x26
    { 0 }, //! Reserved command tag 0x27
};

//! @brief Command processor state data.
command_processor_data_t g_commandData;

// See bl_command.h for documentation on this interface.
command_interface_t g_commandInterface = { zlink_command_init, zlink_command_pump,
                                           (command_handler_entry_t *)&g_commandHandlerTable, &g_commandData };



/*******************************************************************************
 * Private Code
 ******************************************************************************/
//! @brief Find command handler entry.
//!
//! @retval NULL if no entry found.
static const command_handler_entry_t *find_entry(uint8_t tag)
{
    if (tag < zLinkFirstCommandTag || tag > zLinkLastCommandTag)
    {
        return 0; // invalid command
    }
    const command_handler_entry_t *entry =
        &g_zLinkDeviceContext.commandInterface->handlerTable[(tag - zLinkFirstCommandTag)];

    return entry;
}

//! @brief Handle a command transaction.
static status_t handle_command(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    uint8_t commandTag = commandPacket->commandTag;
    status_t status = kStatus_Success;

    // Look up the handler entry and save it for the data phaase.
    g_zLinkDeviceContext.commandInterface->stateData->handlerEntry = find_entry(commandTag);

    if (g_zLinkDeviceContext.commandInterface->stateData->handlerEntry &&
        g_zLinkDeviceContext.commandInterface->stateData->handlerEntry->handleCommand)
    {
        // Process the command normally.
        g_zLinkDeviceContext.commandInterface->stateData->handlerEntry->handleCommand(packet, packetLength);
        return kStatus_Success;
    }
    else
    {
        // We don't recognize this command, so return an error response.
        zlink_printf("unknown command 0x%x\r\n", commandPacket->commandTag);
        status = kStatus_UnknownCommand;
    }

    // Should only get to this point if an error occurred before running the command handler.
    send_generic_response(status, commandTag);
    return status;
}

//! @brief Handle a data transaction.
static status_t handle_data(bool *hasMoreData)
{
    if (g_zLinkDeviceContext.commandInterface->stateData->handlerEntry)
    {
        // Run data phase if present, otherwise just return success.
        *hasMoreData = 0;
        return g_zLinkDeviceContext.commandInterface->stateData->handlerEntry->handleData ?
                   g_zLinkDeviceContext.commandInterface->stateData->handlerEntry->handleData(hasMoreData) :
                   kStatus_Success;
    }

    zlink_printf("Error: no handler entry for data phase\r\n");
    return kStatus_Success;
}


/*******************************************************************************
 * Code
 ******************************************************************************/


//! @brief Initialize the command processor component.
status_t zlink_command_init(void)
{
    command_processor_data_t *data = g_zLinkDeviceContext.commandInterface->stateData;

    data->state = zLinkCommandState_CommandPhase;
    return kStatus_Success;
}


//! @brief Pump the command state machine.
//!
//! Executes one command or data phase transaction.
status_t zlink_command_pump(void)
{
    status_t status = kStatus_Success;
    bool hasMoreData = false;

    if (g_zLinkDeviceContext.activePeripheral->packetInterface)
    {
        switch (g_zLinkDeviceContext.commandInterface->stateData->state)
        {
            default:
            case zLinkCommandState_CommandPhase:
                status = g_zLinkDeviceContext.activePeripheral->packetInterface->readPacket(
                    g_zLinkDeviceContext.activePeripheral, &g_zLinkDeviceContext.commandInterface->stateData->packet,
                    &g_zLinkDeviceContext.commandInterface->stateData->packetLength, zLinkPacketType_Command);
                if ((status != kStatus_Timeout) && (status != kStatus_Success) && (status != kStatus_AbortDataPhase) && (status != kStatus_Ping))
                {
                    zlink_printf("Error: readPacket returned status 0x%x\r\n", status);
                    break;
                }
                if (g_zLinkDeviceContext.commandInterface->stateData->packetLength == 0)
                {
                    // No command packet is available. Return success.
                    break;
                }
                status = handle_command(g_zLinkDeviceContext.commandInterface->stateData->packet,
                                        g_zLinkDeviceContext.commandInterface->stateData->packetLength);
                if (status != kStatus_Success)
                {
                    zlink_printf("Error: handle_command returned status 0x%x\r\n", status);
                    break;
                }
                g_zLinkDeviceContext.commandInterface->stateData->state = zLinkCommandState_DataPhase;
                break;

            case zLinkCommandState_DataPhase:
                status = handle_data(&hasMoreData);
                if (status != kStatus_Success)
                {
                    zlink_printf("Error: handle_data returned status 0x%x\r\n", status);
                    g_zLinkDeviceContext.commandInterface->stateData->state = zLinkCommandState_CommandPhase;
                    break;
                }
                g_zLinkDeviceContext.commandInterface->stateData->state =
                    hasMoreData ? zLinkCommandState_DataPhase : zLinkCommandState_CommandPhase;
                break;
        }
    }

    return status;
}


////////////////////////////////////////////////////////////////////////////////
// Command Handlers
////////////////////////////////////////////////////////////////////////////////

//! @brief Send a generic response packet.
void send_generic_response(uint32_t commandStatus, uint32_t commandTag)
{
    status_t status = kStatus_Success;
    
    generic_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = zLinkCommandTag_GenericResponse;
    responsePacket.commandPacket.flags = 0;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.commandTag = commandTag;

    status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        zlink_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

//! @brief Send a read memory response packet.
void send_read_memory_response(uint32_t commandStatus, uint32_t length)
{
    read_memory_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = zLinkCommandTag_ReadMemoryResponse;
    responsePacket.commandPacket.flags = zLinkCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        zlink_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

//! @brief Reset data phase variables.
static void reset_data_phase (void)
{
    memset(&g_zLinkDeviceContext.commandInterface->stateData->dataPhase, 0,
           sizeof(g_zLinkDeviceContext.commandInterface->stateData->dataPhase));
}


//! @brief Complete the data phase, optionally send a response.
void finalize_data_phase(status_t status)
{
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.address = 0;
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count = 0;

    // Force to write cached data to target memory
    if (g_zLinkDeviceContext.commandInterface->stateData->dataPhase.commandTag == zLinkCommandTag_WriteMemory)
    {
        // assert(g_zLinkDeviceContext.memoryInterface->flush);
        // status_t flushStatus = g_zLinkDeviceContext.memoryInterface->flush();

        // // Update status only if the last operation result is successfull in order to reflect
        // // real result of the write operation.
        // if (status == kStatus_Success)
        // {
        //     status = flushStatus;
        // }
    }
// #if BL_FEATURE_EXPAND_MEMORY
//     // Reset the state machine of memory interface.
//     assert(g_zLinkDeviceContext.memoryInterface->finalize);
//     status_t finalizeStatus = g_zLinkDeviceContext.memoryInterface->finalize();
//     if (status == kStatus_Success)
//     {
//         status = finalizeStatus;
//     }
// #endif // BL_FEATURE_EXPAND_MEMORY

    // Send final response packet.
    send_generic_response(status, g_zLinkDeviceContext.commandInterface->stateData->dataPhase.commandTag);
}

//! @brief Read Memory command handler.
void handle_read_memory(uint8_t *packet, uint32_t packetLength)
{
    read_memory_packet_t *command = (read_memory_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.memoryId = command->memoryId;
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count = command->byteCount;
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.address = command->startAddress;
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.commandTag = zLinkCommandTag_ReadMemory;
    send_read_memory_response(kStatus_Success, command->byteCount);
}

//! @brief Handle data phase with data producer (send to host).
status_t handle_data_producer(bool *hasMoreData)
{
    if (g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count == 0)
    {
        // No data phase.
        *hasMoreData = false;
        finalize_data_phase(kStatus_Success);
        return kStatus_Success;
    }

    *hasMoreData = true;
    uint32_t memoryId = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.memoryId;
    uint32_t remaining = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count;
    uint32_t dataAddress = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.address;
    uint8_t *data = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.data;
    uint8_t commandTag = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.commandTag;
    status_t status = kStatus_Success;

    // Initialize the data packet to send.
    uint32_t packetSize;
#if BL_FEATURE_EXPAND_PACKET_SIZE
    uint8_t *packet = s_dataProducerPacket;
    uint32_t packetBufferSize =
        g_zLinkDeviceContext.activePeripheral->packetInterface->getMaxPacketSize(g_zLinkDeviceContext.activePeripheral);
    packetSize = MIN(packetBufferSize, remaining);
#else
    uint8_t packet[zLinkMinPacketBufferSize];
    packetSize = MIN(zLinkMinPacketBufferSize, remaining);
#endif // BL_FEATURE_EXPAND_PACKET_SIZE

    // Copy the data into the data packet.
    if (data)
    {
        // Copy data using compiler-generated memcpy.
        memcpy(packet, data, packetSize);
        data += packetSize;
        status = kStatus_Success;
    }
    else
    {
//        if (commandTag == zLinkCommandTag_ReadMemory)
//        {
//            // Copy data using memory interface.
//            status = g_zLinkDeviceContext.memoryInterface->read(dataAddress, packetSize, packet, memoryId);
//        }
        
// #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
// #if !BL_DEVICE_IS_LPC_SERIES
//         else if (commandTag == kCommandTag_FlashReadResource)
//         {
// // Read data from special-purpose flash memory
//             ftfx_read_resource_opt_t option =
//                 (ftfx_read_resource_opt_t)g_zLinkDeviceContext.commandInterface->stateData->dataPhase.option;
//             lock_acquire();
//             // Note: Both Main and Secondary flash share the same IFR Memory
//             //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
//             status = g_zLinkDeviceContext.flashDriverInterface->flash_read_resource(
//                 g_zLinkDeviceContext.allFlashState, dataAddress, (uint8_t *)packet, packetSize,
//                 option);
//             lock_release();
//         }
// #endif // !BL_DEVICE_IS_LPC_SERIES
// #endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
//         else if (commandTag == kCommandTag_GenerateKeyBlob)
//         {
//             memcpy(packet, (void *)dataAddress, packetSize);
//         }
        dataAddress += packetSize;
    }

    if (status != kStatus_Success)
    {
        zlink_printf("Error: %s returned status 0x%x, abort data phase\r\n",
                     (commandTag == zLinkCommandTag_ReadMemory) ? "read memory" : "flash read resource", status);
        // Send zero length packet to tell host we are aborting data phase
        g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
            g_zLinkDeviceContext.activePeripheral, (const uint8_t *)packet, 0, zLinkPacketType_Data);
        finalize_data_phase(status);
        *hasMoreData = false;
        return kStatus_Success;
    }
    remaining -= packetSize;

#ifdef TEST_SENDER_ABORT
// Disble IAR "statement is unreachable" error
#pragma diag_suppress = Pe111
    // Send zero length packet to abort data phase.
    g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(g_zLinkDeviceContext.activePeripheral,
                                                                       (const uint8_t *)packet, 0, kPacketType_Data);
    finalize_data_phase(kStatus_AbortDataPhase);
    *hasMoreData = false;
    return kStatus_Success;
#endif // TEST_SENDER_ABORT;

    status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)packet, packetSize, zLinkPacketType_Data);

    if (remaining == 0)
    {
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else if (status != kStatus_Success)
    {
        zlink_printf("writePacket aborted due to status 0x%x\r\n", status);
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else
    {
        g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count = remaining;
        g_zLinkDeviceContext.commandInterface->stateData->dataPhase.address = dataAddress;
    }

    return kStatus_Success;
}

//! @brief Reset command handler.
void handle_reset(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    
    send_generic_response(kStatus_Success, commandPacket->commandTag);
    
    // Wait for the ack from the host to the generic response
    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(g_zLinkDeviceContext.activePeripheral);

    g_zLinkDeviceContext.activePeripheral->controlInterface->shutdown(g_zLinkDeviceContext.activePeripheral);
//     // Prepare for shutdown.
//     shutdown_cleanup(kShutdownType_Reset);
// #if defined(BL_FEATURE_6PINS_PERIPHERAL) && BL_FEATURE_6PINS_PERIPHERAL
//     shutdown_cleanup(kShutdownType_Cleanup);
// #endif // BL_FEATURE_6PINS_PERIPHERAL 
    
    NVIC_SystemReset();

    // Does not get here.
    assert(0);
}

//! @brief Send a read memory response packet.
void send_get_deviceinfo_response(uint32_t commandStatus, uint32_t length)
{
    read_memory_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = zLinkCommandTag_GetDeviceInfoResponse;
    responsePacket.commandPacket.flags = zLinkCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        zlink_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

const uint8_t ProductKey[] = "a1Z7tNBsNMj";
const uint8_t ProductSecret[] = "PhIeMRbRz66irxTH";
const uint8_t DeviceName[] = "ZLG_Elock_Dev";
const uint8_t DeviceSecret[] = "ZEx345u01xFExM7GhmpW4eOPTp6Elt0V";
const uint8_t AppVersion[] = "app-1.0.0-20190417.1515";

void handle_get_deviceinfo (uint8_t *packet, uint32_t packetLength)
{
    get_deviceinfo_packet_t *get_deviceinfo_packet = (get_deviceinfo_packet_t *)packet;
    uint32_t info_byteCount = 0;
    uint8_t *data;

    // Start the data phase.
    reset_data_phase();
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.commandTag = zLinkCommandTag_GetDeviceInfo;
    switch (get_deviceinfo_packet->info_index) {
        case zLinkDeviceInfo_ProductKey:
            info_byteCount = sizeof(ProductKey) - 1;
            data = (uint8_t *)ProductKey;
            break;
        case zLinkDeviceInfo_ProductSecret:
            info_byteCount = sizeof(ProductSecret) - 1;
            data = (uint8_t *)ProductSecret;
            break;
        case zLinkDeviceInfo_DeviceName:
            info_byteCount = sizeof(DeviceName) - 1;
            data = (uint8_t *)DeviceName;
            break;
        case zLinkDeviceInfo_DeviceSecret:
            info_byteCount = sizeof(DeviceSecret) - 1;
            data = (uint8_t *)DeviceSecret;
            break;
        case zLinkDeviceInfo_AppVersion:
            info_byteCount = sizeof(AppVersion) - 1;
            data = (uint8_t *)AppVersion;
            break;
        default:
            zlink_printf("Error: unknow device info index.\r\n");
            break;
    }
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.data = data;
    g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count = info_byteCount;
    send_get_deviceinfo_response(kStatus_Success, info_byteCount);
}

//! @brief Handle data phase with data producer (send to host).
status_t handle_deviceinfo_producer(bool *hasMoreData)
{
    status_t status = kStatus_Success;
    uint8_t *data = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.data;
    uint32_t length = g_zLinkDeviceContext.commandInterface->stateData->dataPhase.count;

    if (data) {
        status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
            g_zLinkDeviceContext.activePeripheral, (const uint8_t *)data, length, zLinkPacketType_Data);      

    }

    finalize_data_phase(status);
    *hasMoreData = false;

    return status;
}







/*
 * endfile
 */
