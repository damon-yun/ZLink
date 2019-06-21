/*******************************************************************************
 * zlinkh_cmd_packet.c - definitions for the zlink host command interface			     	
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
 *    zlink host command packet interface
 *
 *****************************************************************************/
#include "zlink_common.h"
#include "zlinkh_cmd_packet.h"

#include "zlink_context.h"
#include "zlink_serial_packet.h"

//! @brief Host Command processor state data.
zlinkh_cmd_data_t g_zLinkHostCmdData;

static const uint8_t *get_data (void) {
    return (const uint8_t *)(&(g_zLinkHostCmdData.m_header));
}

//! @brief Get size of command packet, including arguments.
static uint32_t get_size (void)
{
    return sizeof(command_packet_t) + ((g_zLinkHostCmdData.m_header.parameterCount) * sizeof(uint32_t));
}

static void CmdPacketInit (uint8_t tag, uint8_t flags, uint32_t *arg,uint8_t numArguments)
{
    g_zLinkHostCmdData.m_header.commandTag = tag;
    g_zLinkHostCmdData.m_header.flags = flags;

    g_zLinkHostCmdData.m_header.reserved = 0;
    g_zLinkHostCmdData.m_header.parameterCount = numArguments;

    if (numArguments) {
        memcpy(g_zLinkHostCmdData.m_arguments, arg, numArguments * sizeof(uint32_t));
    }
}


// See host_command.h for documentation of this method.
static bool root_processResponse(const uint8_t *p_packet, uint8_t commandTag)
{
    const generic_response_packet_t *packet = (const generic_response_packet_t *)p_packet;

    if (!packet)
    {
        zlink_printf("processResponse: null packet\n");
        g_zLinkHostCmdData.m_responseValues = (kStatus_NoResponse);
        return false;
    }

    if (commandTag == zLinkCommandTag_GetProperty) {
        if (packet->commandPacket.commandTag != zLinkCommandTag_GetPropertyResponse)
        {
            zlink_printf("Error: expected kCommandTag_GetPropertyResponse (0x%x), received 0x%x\r\n", zLinkCommandTag_GetPropertyResponse,
                       packet->commandPacket.commandTag);
            g_zLinkHostCmdData.m_responseValues = (kStatus_UnknownCommand);
            return false;
        }
        goto processResponse_exit;
    }
    if (packet->commandPacket.commandTag != zLinkCommandTag_GenericResponse)
    {
        zlink_printf("Error: expected kCommandTag_GenericResponse (0x%x), received 0x%x\r\n", zLinkCommandTag_GenericResponse,
                   packet->commandPacket.commandTag);
        g_zLinkHostCmdData.m_responseValues = (kStatus_UnknownCommand);
        return false;
    }
    if (packet->commandTag != commandTag)
    {
        zlink_printf("Error: expected commandTag 0x%x, received 0x%x\r\n", commandTag, packet->commandTag);
        g_zLinkHostCmdData.m_responseValues = (kStatus_UnknownCommand);
        return false;
    }

    // Set the status in the response vector.
    g_zLinkHostCmdData.m_responseValues = (packet->status);

    if (packet->status != kStatus_Success)
    {
        return false;
    }
processResponse_exit:
    if (commandTag != zLinkCommandTag_GenerateKeyBlob)
    {
        zlink_printf("Successful generic response\r\n");
    }
    return true;
}

static uint8_t *SendCommandGetResponse (peripheral_packet_interface_t *packet_interface)
{
    uint8_t *responsePacket = NULL;
    uint32_t responseLength = 0;
    status_t status = kStatus_Success;

    // status = packet_interface->writePacket(get_data(), get_size(), kPacketType_Command);

    status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)get_data(), get_size(), zLinkPacketType_Command);
        
    if (status != kStatus_Success)
    {
        zlink_printf("sendCommandGetResponse.writePacket error %d.\n", status);
        return NULL;
    }

    // status = packe_tizer->pfun_read_packet(&responsePacket, &responseLength, kPacketType_Command);

    status = g_zLinkDeviceContext.activePeripheral->packetInterface->readPacket(
                    g_zLinkDeviceContext.activePeripheral, &responsePacket, &responseLength, zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        zlink_printf("sendCommandGetResponse.readPacket error %d.\n", status);
        responsePacket = NULL;
    } else {

    }

    return responsePacket;
}


const zlinkh_cmd_packet_t g_zlinkHostCmdPacket = {
    .pfun_packet_init = CmdPacketInit,
    .pfun_send_command_get_response = SendCommandGetResponse,
    .pfun_superclass_process_response = root_processResponse
};

