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
#include "zlinkh_data_packet.h"

#include "zlink_context.h"
#include "zlink_serial_packet.h"

static uint8_t m_packet[zLinkMaxHostPacketSize]; //!< The data packet pointer.

static uint8_t *m_segment;          //!< DataSource::Segment object.
static uint32_t m_segmentSize;      //!< DataSource::Segment object size.
static uint32_t m_byteIndex;        //!< Current byte index.


static data_consumer_t *__p_data_consumer = NULL;

static void dataInit (uint8_t *data, uint32_t size)
{
    m_segment = data;
    m_segmentSize = size;
}

//! @brief Query if more data is available.
static bool hasMoreData (void)
{
    return (m_byteIndex < m_segmentSize);
}

//! @brief Query the total size of the data.
static uint32_t getDataSize (void)
{
    return m_segmentSize;
}

//! @brief Get the next data chunk.
//!
//! Before calling getData(), call hasMoreData() to determine if
//! data is available.
static uint32_t getData (uint8_t *data, uint32_t size)
{
    if (!hasMoreData())
    {
        return 0;
    }

    memcpy(data,(m_segment + m_byteIndex), size);

    return size;
}

const data_producer_t m_dataProducer = {
    .pfunInit = dataInit,
    .pfunHasMoreData = hasMoreData,
    .pfunGetDataSize = getDataSize,
    .pfunGetData = getData
};

static void dataPacketSendInit (uint8_t *data, uint32_t size)
{
    m_dataProducer.pfunInit(data, size);
}

static void dataPacketRecvInit (data_consumer_t *pDataConsumer)
{
    __p_data_consumer = pDataConsumer;
}

//! See zlinkh_data_packet.h for documentation on this function.
static uint8_t *sendTo(peripheral_packet_interface_t *packet_interface, uint32_t *bytesWritten, progressCallback_t progressCallback)
{
    status_t status = kStatus_Success;
    *bytesWritten = 0;

    while (m_dataProducer.pfunHasMoreData() && *bytesWritten < m_dataProducer.pfunGetDataSize())
    {
        uint32_t count = MIN(zLinkMinPacketBufferSize, (m_dataProducer.pfunGetDataSize() - *bytesWritten));
        count = m_dataProducer.pfunGetData(m_packet, count);
        if (count)
        {
            status = g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
                        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)m_packet, count, zLinkPacketType_Data);
            // status_t status = device.writePacket((const uint8_t *)m_packet, count, kPacketType_Data);
            if (status != kStatus_Success)
            {
                zlink_printf("Err:Data phase write aborted by status 0x%x\r\n", status);
                if ((status == kStatus_AbortDataPhase) /* && device.isAbortEnabled() */ )
                {
                    zlink_printf("Possible JUMP or RESET command received.\r\n");
                }
                break;
            }

            *bytesWritten += count;

            if (progressCallback != NULL)
            {
                // execute process callback function.
                progressCallback(*bytesWritten * 100 / m_dataProducer.pfunGetDataSize(),*bytesWritten, m_dataProducer.pfunGetDataSize());
                if (0)//(progress->abortPhase())
                {
                    g_zLinkDeviceContext.activePeripheral->packetInterface->writePacket(
                        g_zLinkDeviceContext.activePeripheral, (const uint8_t *)&m_packet, 0, zLinkPacketType_Data);
                    break;
                }
            }
        }
    }

    // Read final command status
    uint8_t *responsePacket;
    uint32_t responseLength;
    status = g_zLinkDeviceContext.activePeripheral->packetInterface->readPacket(
                    g_zLinkDeviceContext.activePeripheral, &responsePacket, &responseLength, zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        return NULL;
    }
    return responsePacket;
}


//! See zlinkh_data_packet.h for documentation on this function.
uint8_t *receiveFrom(peripheral_packet_interface_t *packet_interface, uint32_t *byteCount, progressCallback_t progressCallback)
{
    status_t status = kStatus_Success;
    uint32_t totalCount = *byteCount;

    while (*byteCount > 0)
    {
        uint8_t *dataPacket;
        uint32_t length;

        status = g_zLinkDeviceContext.activePeripheral->packetInterface->readPacket(
                    g_zLinkDeviceContext.activePeripheral, &dataPacket, &length, zLinkPacketType_Data);

        // Bail if there was an error reading the packet.
        if (status != kStatus_Success)
        {
            zlink_printf("Read data packet error. Sending ACK.\n");
            if (__p_data_consumer && (__p_data_consumer->pfunFinalize)) {
                __p_data_consumer->pfunFinalize();
            }
            g_zLinkDeviceContext.activePeripheral->packetInterface->sync();
            return NULL;
        }

        // Check for sender abort of data phase.
        if (length == 0)
        {
            zlink_printf("Data phase aborted by sender\n");
            break;
        }

        if (__p_data_consumer && (__p_data_consumer->pfunProcessData)) {
            __p_data_consumer->pfunProcessData(dataPacket, length);
        }
        *byteCount -= length;

        if (*byteCount <= 0)
        {
            if (__p_data_consumer && (__p_data_consumer->pfunFinalize)) {
                __p_data_consumer->pfunFinalize();
            }
        }

        if (progressCallback != NULL)
        {
            progressCallback((totalCount - *byteCount) * 100 / totalCount, (totalCount - *byteCount),totalCount);
            if (0)//(progress->abortPhase())
            {
                g_zLinkDeviceContext.activePeripheral->packetInterface->abortPacket(g_zLinkDeviceContext.activePeripheral);
                break;
            }
        }
    }

    // Read the final generic response packet.
    uint8_t *responsePacket;
    uint32_t responseLength;
    status = g_zLinkDeviceContext.activePeripheral->packetInterface->readPacket(
                    g_zLinkDeviceContext.activePeripheral, &responsePacket, &responseLength, zLinkPacketType_Command);
    if (status != kStatus_Success)
    {
        return NULL;
    }
    return responsePacket;
}


const zlinkh_data_packet_t g_zlinkHostDataPacket = {
    .pfun_send_init = dataPacketSendInit,
    .pfun_recv_init = dataPacketRecvInit,
    .pfunSendTo = sendTo,
    .pfunReceiveFrom = receiveFrom
    // .pfun_superclass_process_response = root_processResponse
};

