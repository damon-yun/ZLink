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
#include "zlinkh_command.h"
#include "zlink_context.h"


/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "zlink.host.command"
#endif

/*******************************************************************************
 * Configurations
 ******************************************************************************/



/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile uint32_t g_lock_stat = 0x4f4E4C49;//0;

#define ELOCK2_STAT_ONLINE      0x4f4E4C49
#define ELOCK2_STAT_OFFLINE     0x4F46464C
#define ELOCK2_STAT_UNLOCK      0x554E4C4B
#define ELOCK2_STAT_LOCK        0x4C4F434B
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Private Code
 ******************************************************************************/
static void progressCallback (int percentage, int segmentIndex, int segmentCount)
{
    printf("%d%%\r\n",percentage);
}

/*******************************************************************************
 * Ping Command Code
 ******************************************************************************/

static status_t pingSendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;

    ping_response_t response;

    status = g_zLinkDeviceContext.activePeripheral->packetInterface->sendPingGetResponse(1,500,&response);

    return status;
}

zlinkh_command_t ping_cmd = {
    .pfun_sendTo = pingSendTo
};


/*******************************************************************************
 * Getproperty Command Code
 ******************************************************************************/
static void getPropertyInit (void *arg)
{
    if (arg == NULL) {
        return;
    }
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_GetProperty, zLinkCommandFlag_None, arg, 2);
}

static status_t getPropertySendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;
    uint8_t *responsePacket = NULL;

    responsePacket = g_zLinkDeviceContext.cmdPacketInterface->pfun_send_command_get_response(peripheralSelf);

    if (responsePacket == NULL) {
        status = kStatus_NoResponse;
    }

    g_zLinkDeviceContext.cmdPacketInterface->pfun_superclass_process_response(responsePacket, zLinkCommandTag_GetProperty);

    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(peripheralSelf);

    return status;
}

zlinkh_command_t get_property_cmd = {
    .pfun_init = getPropertyInit,
    .pfun_sendTo = getPropertySendTo
};

/*******************************************************************************
 * flashEraseRegion Command Code
 ******************************************************************************/
static void flashEraseRegionInit (void *arg)
{
    if (arg == NULL) {
        return;
    }
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_FlashEraseRegion, zLinkCommandFlag_None, arg, 3);
}

static status_t flashEraseRegionSendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;
    uint8_t *responsePacket = NULL;

    responsePacket = g_zLinkDeviceContext.cmdPacketInterface->pfun_send_command_get_response(peripheralSelf);

    if (responsePacket == NULL) {
        status = kStatus_NoResponse;
    }

    g_zLinkDeviceContext.cmdPacketInterface->pfun_superclass_process_response(responsePacket, zLinkCommandTag_FlashEraseRegion);

    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(peripheralSelf);

    return status;
}

zlinkh_command_t flash_erase_region_cmd = {
    .pfun_init = flashEraseRegionInit,
    .pfun_sendTo = flashEraseRegionSendTo
};
/*******************************************************************************
 * WriteMemory Command Code
 ******************************************************************************/
static void writeMemoryInit (void *arg)
{
    write_memory_arg_t *pArg = (write_memory_arg_t *)arg;
    if (arg == NULL) {
        return;
    }
    g_zlinkHostDataPacket.pfun_send_init(pArg->pdata, pArg->byte_count);
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_WriteMemory, zLinkCommandFlag_HasDataPhase, arg, 3);
}

static status_t writeMemorySendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;
    uint8_t *responsePacket = NULL;

    responsePacket = g_zLinkDeviceContext.cmdPacketInterface->pfun_send_command_get_response(peripheralSelf);

    if (responsePacket == NULL) {
        status = kStatus_NoResponse;
    }
    const generic_response_packet_t *packet =
            (const generic_response_packet_t *)(responsePacket);
    if (packet->status != kStatus_Success) {
        return packet->status;
    }

    // Send command packet.
    uint32_t bytesWritten = 0;

    // g_zLinkDeviceContext.cmdPacketInterface->pfun_superclass_process_response(responsePacket, zLinkCommandTag_WriteMemory);
    g_zlinkHostDataPacket.pfunSendTo(peripheralSelf, &bytesWritten, progressCallback);

    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(peripheralSelf);

    return status;
}

zlinkh_command_t write_memory_cmd = {
    .pfun_init = writeMemoryInit,
    .pfun_sendTo = writeMemorySendTo
};


/*******************************************************************************
 * ReadMemory Command Code
 ******************************************************************************/

//! @brief Process the next data chunk.
void readMemDefProcessData (const uint8_t *data, uint32_t size)
{

}

//! @brief Finalize processing.
void readMemDefFinalize (void)
{

}

static const data_consumer_t data_consumer = {
    .pfunProcessData = readMemDefProcessData,
    .pfunFinalize = readMemDefFinalize
};

static void readMemoryInit (void *arg)
{
    if (arg == NULL) {
        return;
    }

    g_zlinkHostDataPacket.pfun_recv_init((data_consumer_t *)&data_consumer);
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_ReadMemory, zLinkCommandFlag_None, arg, 3);
}

static void readMemoryInit2arg (void *arg, void *arg2)
{
    if (arg == NULL && arg2 == NULL) {
        return;
    }

    g_zlinkHostDataPacket.pfun_recv_init((data_consumer_t *)arg2);
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_ReadMemory, zLinkCommandFlag_None, arg, 3);
}

static status_t readMemorySendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;
    uint8_t *responsePacket = NULL;

    responsePacket = g_zLinkDeviceContext.cmdPacketInterface->pfun_send_command_get_response(peripheralSelf);

    if (responsePacket == NULL) {
        status = kStatus_NoResponse;
    }
    const read_memory_response_packet_t *packet =
            (const read_memory_response_packet_t *)(responsePacket);
    
    if (packet->status != kStatus_Success) {
        return packet->status;
    }
    // recv data packet.
    uint32_t byteCount = packet->dataByteCount;

    g_zlinkHostDataPacket.pfunReceiveFrom(peripheralSelf, &byteCount, progressCallback);

    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(peripheralSelf);

    return status;
}

zlinkh_command_t read_memory_cmd = {
    .pfun_init = readMemoryInit,
    .pfun_init2arg = readMemoryInit2arg,
    .pfun_sendTo = readMemorySendTo
};

/*******************************************************************************
 * GetLockStat Command Code
 ******************************************************************************/
static void getLockStatInit (void *arg)
{
    g_zLinkDeviceContext.cmdPacketInterface->pfun_packet_init(zLinkCommandTag_GetLockStat, zLinkCommandFlag_None, arg, 0);
}

static status_t getLockStatSendTo (void *peripheralSelf)
{
    status_t status = kStatus_Success;
    generic_response_packet_t *responsePacket = NULL;

    responsePacket = (generic_response_packet_t *)g_zLinkDeviceContext.cmdPacketInterface->pfun_send_command_get_response(peripheralSelf);

    if (responsePacket == NULL) {
        status = kStatus_NoResponse;
    } else {
        if (responsePacket->commandPacket.commandTag != 0xA0 || responsePacket->commandTag != zLinkCommandTag_GetLockStat) {
            status = kStatus_UnknownCommand;
        } else {
            if (responsePacket->status == 0x554E4C4B) {  // UNLOCK
                g_lock_stat = 0x554E4C4B;
            } else if (responsePacket->status == 0x4C4F434B) {  //LOCK
                g_lock_stat = 0x4C4F434B;
            } else {
                status = kStatus_UnknownCommand;
            }            
        }
    }

    g_zLinkDeviceContext.activePeripheral->packetInterface->finalize(peripheralSelf);

    return status;
}

zlinkh_command_t get_lockstat_cmd = {
    .pfun_init = getLockStatInit,
    .pfun_sendTo = getLockStatSendTo
};
/*******************************************************************************
 * Code
 ******************************************************************************/





/*
 * endfile
 */
