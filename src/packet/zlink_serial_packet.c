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

#include "zlink_serial_packet.h"
#include "zlink_context.h"
#include "crc/crc16.h"

#if (defined(USE_OS) && (USE_OS == 1))

#else
    #include "zlink_hal_systick.h"
#endif

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "zlink.peripheral.serial.packet"
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

//! @brief Global context data.
static serial_data_t g_serialContext;

//! @brief Ping response.
const ping_response_t k_PingResponse = {
    { { zLinkSerialProtocol_Version_Bugfix, zLinkSerialProtocol_Version_Minor, zLinkSerialProtocol_Version_Major,
        zLinkSerialProtocol_Version_Name } },
    0,     // options, recalculate crc16 if this value changes
    0xAE29 // crc16 of start byte, packet type, version and options.
           // i.e. [5a a7 00 00 01 50 00 00]
           // Calculated using CRC-16/XMODEM.
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static status_t write_data(const uint8_t *buffer, uint32_t byteCount);
static status_t read_data(uint8_t *buffer, uint32_t byteCount, uint32_t timeoutMs);
static status_t read_data_packet(framing_data_packet_t *packet, uint8_t *data, packet_type_t packetType);
static status_t read_start_byte(framing_header_t *header);
static status_t read_header(framing_header_t *header);
static status_t read_length(framing_data_packet_t *packet);
static status_t read_crc16(framing_data_packet_t *packet);
static uint16_t calculate_framing_crc16(framing_data_packet_t *packet, const uint8_t *data);

static status_t wait_for_ack_packet(void);
static status_t send_deferred_ack(void);

/*******************************************************************************
 * Private Code
 ******************************************************************************/
#if (defined(ZLINK_HOST))
static void host_delay_ms(uint32_t milliseconds)
{
// @todo implement for non-win32
#if defined(WIN32)
    Sleep(milliseconds);
#elif defined(LINUX)
    usleep(milliseconds * 1000);
#elif defined(OSAL_RHINO)
    aos_msleep(milliseconds);
#elif defined(FSL_RTOS_FREE_RTOS)
    vTaskDelay(milliseconds);
#else

#if (defined(USE_OS) && (USE_OS == 1))
#error "you need add ms sleep funcation here."
#else
    microseconds_delay(milliseconds * 1000);
#endif

#endif
}
#endif //ZLINK_HOST

#if (defined(USE_OS) && (USE_OS == 1))
//! @brief Write buffer to peripheral until all bytes sent.
static status_t write_data(const uint8_t *buffer, uint32_t byteCount)
{
    status_t retVal;

    retVal = g_zLinkDeviceContext.activePeripheral->byteInterface->write(g_zLinkDeviceContext.activePeripheral, buffer,
                                                                        byteCount);

    return retVal;
}

//! @brief Read from peripheral until specified number of bytes received.
static status_t read_data(uint8_t *buffer, uint32_t byteCount, uint32_t timeoutMs)
{
    status_t retVal;
    uint32_t currentBytesRead = 0;

    retVal = g_zLinkDeviceContext.activePeripheral->byteInterface->read(g_zLinkDeviceContext.activePeripheral, buffer,
                                                                        byteCount, &currentBytesRead, timeoutMs);

    
    if (retVal != kStatus_Success) {
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

#else

//! @brief Write buffer to peripheral until all bytes sent.
static status_t write_data(const uint8_t *buffer, uint32_t byteCount)
{
    status_t retVal;

    retVal = g_zLinkDeviceContext.activePeripheral->byteInterface->write(g_zLinkDeviceContext.activePeripheral, buffer,
                                                                        byteCount);

    return retVal;
}

//! @brief Read from peripheral until specified number of bytes received.
static status_t read_data(uint8_t *buffer, uint32_t byteCount, uint32_t timeoutMs)
{
    // On the target we read from our interrupt buffer
    uint32_t currentBytesRead = 0;
    volatile uint64_t startTicks = microseconds_get_ticks();
    __ISB();
    uint64_t timeOutTicks = microseconds_convert_to_ticks(timeoutMs * 1000);
    volatile uint64_t endTicks = startTicks;
    uint64_t deltaTicks = 0;

    while (currentBytesRead != byteCount)
    {
        endTicks = microseconds_get_ticks();
        deltaTicks = endTicks - startTicks;

        // Check timer roll over
        if (endTicks < startTicks)
        {
            deltaTicks = endTicks + (~startTicks) + 1;
        }

        if (timeOutTicks && (deltaTicks >= timeOutTicks))
        {
            return kStatus_Timeout;
        }

        if (g_serialContext.readOffset != g_serialContext.writeOffset)
        {
            buffer[currentBytesRead++] = g_serialContext.callbackBuffer[g_serialContext.readOffset++];

            g_serialContext.readOffset &= zLinkCallbackBufferSize - 1;
        }
    }

    return kStatus_Success;
}
#endif

//! @brief Read from peripheral until entire data framing packet read.
static status_t read_data_packet(framing_data_packet_t *packet, uint8_t *data, packet_type_t packetType)
{
    // Read the packet header.
    status_t status = read_header(&packet->header);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (packet->header.packetType == zLinkFramingPacketType_Ping)
    {
        return serial_send_ping_response(g_zLinkDeviceContext.activePeripheral);
    }

    uint8_t expectedPacketType = zLinkFramingPacketType_Command;

    if (packetType != zLinkPacketType_Command)
    {
        expectedPacketType = zLinkFramingPacketType_Data;
    }
    if (packet->header.packetType != expectedPacketType)
    {
        zlink_printf("Error: read_data_packet found unexpected packet type 0x%x\r\n", packet->header.packetType);
        return kStatus_Fail;
    }

    // Read the packet length.
    status = read_length(packet);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Make sure the packet doesn't exceed the allocated buffer size.
    packet->length = MIN(zLinkIncomingPacketBufferSize, packet->length);

    // Read the crc
    status = read_crc16(packet);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Read the data.
    if (packet->length > 0)
    {
        // Clear the data area so unsent parameters default to zero.
        memset(data, 0, packet->length);

        status = read_data(data, packet->length, zDefaultByteReadTimeoutMs * packet->length);
    }

    return status;
}

//! @brief Read from peripheral until start byte found.
static status_t read_start_byte(framing_header_t *header)
{
    // Read until start byte found.
    do
    {
#if ZLINK_HOST
        status_t status = read_data(&header->startByte, 1, zDefaultByteReadTimeoutMs); // zDefaultByteReadTimeoutMs  no timeout for first byte of packet
#else
        status_t status = read_data(&header->startByte, 1, 0); // zDefaultByteReadTimeoutMs  no timeout for first byte of packet
#endif
        if (status != kStatus_Success)
        {
            return status;
        }
    } while (header->startByte != zLinkFramingPacketStartByte);

    return kStatus_Success;
}

//! @brief Read from peripheral until packet header found.
static status_t read_header(framing_header_t *header)
{
    // Wait for start byte.
    status_t status = read_start_byte(header);
    if (status != kStatus_Success)
    {
        return status;
    }

    return read_data(&header->packetType, sizeof(header->packetType),
                     zDefaultByteReadTimeoutMs * sizeof(header->packetType));
}

//! @brief Read from peripheral until packet length found.
static status_t read_length(framing_data_packet_t *packet)
{
    union
    {
        uint8_t bytes[sizeof(uint16_t)];
        uint16_t halfword;
    } buffer;

    status_t status = read_data((uint8_t *)&buffer.bytes, sizeof(buffer), zDefaultByteReadTimeoutMs * sizeof(buffer));

    packet->length = buffer.halfword;
    return status;
}

//! @brief Read from peripheral until crc16 is found.
static status_t read_crc16(framing_data_packet_t *packet)
{
    union
    {
        uint8_t bytes[sizeof(uint16_t)];
        uint16_t halfword;
    } buffer;

    status_t status = read_data((uint8_t *)&buffer.bytes, sizeof(buffer), zDefaultByteReadTimeoutMs * sizeof(buffer));

    packet->crc16 = buffer.halfword;
    return status;
}

//! @brief Calculate crc over framing data packet.
static uint16_t calculate_framing_crc16(framing_data_packet_t *packet, const uint8_t *data)
{
    uint16_t crc16;

    // Initialize the CRC16 information
    crc16_data_t crcInfo;
    crc16_init(&crcInfo);

    // Run CRC on all header bytes besides the CRC field
    crc16_update(&crcInfo, (uint8_t *)&packet->header.startByte, sizeof(framing_data_packet_t) - sizeof(uint16_t));

    // Continue running CRC on any payload bytes
    crc16_update(&crcInfo, data, packet->length);

    // Finalize the CRC calculations
    crc16_finalize(&crcInfo, &crc16);

    return crc16;
}


//! @brief Wait for an ACK, handling NAKs as needed.
static status_t wait_for_ack_packet(void)
{
    framing_sync_packet_t sync;
    do
    {
        // Receive the sync packet.
        status_t status = read_header(&sync.header);
        if (status != kStatus_Success)
        {
            return status;
        }

        if ((sync.header.packetType != zLinkFramingPacketType_Ack) && (sync.header.packetType != zLinkFramingPacketType_Nak) &&
            (sync.header.packetType != zLinkFramingPacketType_AckAbort))
        {
            zlink_printf("Error: Unexpected sync byte 0x%x received, expected Ack, AckAbort or Nak\r\n",
                         sync.header.packetType);
            return kStatus_InvalidArgument;
        }

        if (sync.header.packetType == zLinkFramingPacketType_AckAbort)
        {
            return kStatus_AbortDataPhase;
        }

        if (sync.header.packetType == zLinkFramingPacketType_Nak)
        {
// Re-transmit the last packet.
#if defined(TEST_NAK)
            --g_serialContext.framingPacket.dataPacket.crc16;
#endif // TEST_NAK
            status = write_data((uint8_t *)&g_serialContext.framingPacket,
                                sizeof(framing_data_packet_t) + g_serialContext.framingPacket.dataPacket.length);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    } while (sync.header.packetType == zLinkFramingPacketType_Nak);

    return kStatus_Success;
}

//! @brief Send ACK if needed.
static status_t send_deferred_ack(void)
{
    if (g_serialContext.isAckNeeded)
    {
        // Send Ack for last received packet.
        g_serialContext.isAckNeeded = false;
        return serial_packet_send_sync(zLinkFramingPacketType_Ack);
    }
    else if (g_serialContext.isAckAbortNeeded)
    {
        // Send AckAbort for last received packet.
        g_serialContext.isAckAbortNeeded = false;
        return serial_packet_send_sync(zLinkFramingPacketType_AckAbort);
    }
    else
    {
        return kStatus_Success;
    }
}


/*******************************************************************************
 * Code
 ******************************************************************************/

// See serial_packet.h for documentation on this function.
void serial_packet_queue_byte(uint8_t byte)
{
    g_serialContext.callbackBuffer[g_serialContext.writeOffset++] = byte;
    g_serialContext.writeOffset &= zLinkCallbackBufferSize - 1;
}


// See serial_packet.h for documentation on this function.
uint32_t serial_packet_get_max_packet_size(const peripheral_descriptor_t *self)
{
    return zLinkMinFramingPacketBufferSize;
}

// See serial_packet.h for documentation on this function.
void serial_packet_abort(const peripheral_descriptor_t *self)
{
    assert(g_serialContext.isAckNeeded);
    g_serialContext.isAckAbortNeeded = true;
    g_serialContext.isAckNeeded = false;
}

// See serial_packet.h for documentation on this function.
status_t serial_packet_send_sync(uint8_t framingPacketType)
{
    framing_sync_packet_t sync;
    sync.header.startByte = zLinkFramingPacketStartByte;
    sync.header.packetType = framingPacketType;

    // Indicate last transaction was a write.
    g_serialContext.isBackToBackWrite = true;

    status_t status = write_data((uint8_t *)&sync, sizeof(sync));
    if (status != kStatus_Success)
    {
        zlink_printf("Error: cannot send sync packet 0x%x, status = 0x%x\r\n", framingPacketType, status);
        return status;
    }

    return status;
}


// See serial_packet.h for documentation on this function.
status_t serial_send_ping_response(const peripheral_descriptor_t *peripheral)
{
    assert(peripheral);

    // Only reply if we're in an idle state
    if (!g_serialContext.isAckNeeded || !g_serialContext.isBackToBackWrite || !g_serialContext.isAckAbortNeeded)
    {
        const uint8_t header[] = { zLinkFramingPacketStartByte, zLinkFramingPacketType_PingResponse };
        peripheral->byteInterface->write(peripheral, (const uint8_t *)&header, sizeof(header));
        peripheral->byteInterface->write(peripheral, (uint8_t *)&k_PingResponse, sizeof(k_PingResponse));
    }

    return kStatus_Ping;
}

// See serial_packet.h for documentation on this function.
status_t serial_packet_read(const peripheral_descriptor_t *self,
                            uint8_t **packet,
                            uint32_t *packetLength,
                            packet_type_t packetType)
{
    if (!packet || !packetLength)
    {
        zlink_printf("Error: invalid packet\r\n");
        return kStatus_InvalidArgument;
    }
    *packetLength = 0;
    status_t status;

    g_serialContext.isBackToBackWrite = false;

    // Send ACK if needed.
    status = send_deferred_ack();
    if (status != kStatus_Success)
    {
        return status;
    }

    framing_data_packet_t framingPacket;

    bool isPacketOk;
    do
    {
        // Clear the packet data area so unsent parameters default to zero.
        memset(g_serialContext.data, 0, sizeof(g_serialContext.data));

        // Receive the framing data packet.
        isPacketOk = true;
        status_t status = read_data_packet(&framingPacket, g_serialContext.data, packetType);
        if (status != kStatus_Success)
        {
            // No packet available.
            *packetLength = 0;
            return status;
        }

        // Verify crc.
        uint16_t calculated_crc = calculate_framing_crc16(&framingPacket, g_serialContext.data);
        if (framingPacket.crc16 != calculated_crc)
        {
            zlink_printf("Error: invalid crc 0x%x, expected 0x%x\r\n", framingPacket.crc16, calculated_crc);
            isPacketOk = false;
        }

        // Send Nak if necessary.
        if (!isPacketOk)
        {
            serial_packet_send_sync(zLinkFramingPacketType_Nak);
        }
    } while (!isPacketOk);

    // Indicate an ACK must be sent.
    g_serialContext.isAckNeeded = true;

    // Set caller's data buffer and length
    *packet = g_serialContext.data;
    *packetLength = framingPacket.length;

    return kStatus_Success;
}


// See serial_packet.h for documentation on this function.
status_t serial_packet_write(const peripheral_descriptor_t *self,
                             const uint8_t *packet,
                             uint32_t byteCount,
                             packet_type_t packetType)
{
    if (!packet || (byteCount > zLinkOutgoingPacketBufferSize))
    {
        zlink_printf("Error: invalid packet or packet size %d\r\n", byteCount);
        return kStatus_InvalidArgument;
    }

    // Send ACK if needed.
    status_t status = send_deferred_ack();
    if (status != kStatus_Success)
    {
        return status;
    }

    // Back-to-back writes require delay for receiver to enter peripheral read routine.
    if (g_serialContext.isBackToBackWrite)
    {
        g_serialContext.isBackToBackWrite = false;
    }

    // Initialize the framing data packet.
    serial_framing_packet_t *framingPacket = &g_serialContext.framingPacket;
    framingPacket->dataPacket.header.startByte = zLinkFramingPacketStartByte;
    framingPacket->dataPacket.header.packetType = zLinkFramingPacketType_Command;
    if (packetType != zLinkPacketType_Command)
    {
        framingPacket->dataPacket.header.packetType = zLinkFramingPacketType_Data;
    }
    framingPacket->dataPacket.length = (uint16_t)byteCount;

    // Copy the caller's data buffer into the framing packet.
    if (byteCount)
    {
        memcpy(framingPacket->data, packet, byteCount);
    }

    // Calculate and set the framing packet crc.
    framingPacket->dataPacket.crc16 =
        calculate_framing_crc16(&framingPacket->dataPacket, (uint8_t *)framingPacket->data);

    // Send the framing data packet.
    status = write_data((uint8_t *)framingPacket, sizeof(framing_data_packet_t) + byteCount);
    if (status != kStatus_Success)
    {
        return status;
    }

    return wait_for_ack_packet();
}

#ifdef ZLINK_HOST

// See SerialPacketizer.h for documentation of this method.
status_t serial_packet_ping(int retries, unsigned int delay, void *pingResponse)
{
    status_t status = kStatus_NoPingResponse;
    uint8_t startByte = 0;
    uint32_t bytesRead = 0;
    const int initialRetries = retries;

    framing_header_t pingPacket;
    pingPacket.startByte = zLinkFramingPacketStartByte;
    pingPacket.packetType = zLinkFramingPacketType_Ping;

    // Send ping until we receive a start byte.
    do
    {
        // Send the ping
        if (write_data((uint8_t *)&pingPacket, sizeof(pingPacket)) == kStatus_Success)
        {
            uint32_t timeout_ms = 500;
            uint32_t duration_ms = 0;

            // Try for half a second to get a response from the ping.
            while (duration_ms < timeout_ms)
            {
                if (read_data(&startByte, sizeof(startByte), zDefaultByteReadTimeoutMs) == kStatus_Success)
                {
                    if (startByte == zLinkFramingPacketStartByte)
                    {
                        break;
                    }
                }

                host_delay_ms(zReadDelayMilliseconds);

                duration_ms += zReadDelayMilliseconds;
            }

            // If we got our start byte, move on to read the response packet
            if (startByte == zLinkFramingPacketStartByte)
            {
                break;
            }
        }

        host_delay_ms(delay);

    } while (retries--);

    if (startByte == zLinkFramingPacketStartByte)
    {
       zlink_printf("Ping responded in %d attempt(s)\n", (initialRetries - retries) + 1);

        // Read response packet type.
        uint8_t packetType;
        status = read_data(&packetType, sizeof(packetType), sizeof(packetType) * zDefaultByteReadTimeoutMs);
        if (status == kStatus_Success)
        {
            if (packetType == zLinkFramingPacketType_PingResponse)
            {
                // Read response.
                ping_response_t response;
                status = read_data((uint8_t *)&response, sizeof(response), sizeof(response) * zDefaultByteReadTimeoutMs);
                if (status == kStatus_Success)
                {
                    // Validate reponse CRC.

                    // Initialize the CRC16 information.
                    uint16_t crc16;
                    crc16_data_t crcInfo;
                    crc16_init(&crcInfo);

                    // Include the start byte and packetType in the CRC.
                    crc16_update(&crcInfo, &startByte, sizeof(startByte));
                    crc16_update(&crcInfo, &packetType, sizeof(packetType));

                    // Run CRC on all other bytes except the CRC field.
                    crc16_update(&crcInfo, (uint8_t *)&response, sizeof(response) - sizeof(uint16_t));

                    // Finalize the CRC calculations
                    crc16_finalize(&crcInfo, &crc16);

                    if (response.crc16 == crc16)
                    {
                        zlink_printf("Framing protocol version = 0x%x, options = 0x%x\n",
                                          response.version.version, response.options);
                        g_zLinkDeviceContext.m_version = response.version;
                        g_zLinkDeviceContext.m_options = response.options;

                        if (pingResponse)
                        {
                            *((ping_response_t *)pingResponse) = response;
                        }

                        status = kStatus_Success;
                    }
                    else
                    {
                        zlink_printf("Error: ping crc16 failed, received 0x%x, expected 0x%x\n", response.crc16, crc16);
                        status = kStatus_InvalidCRC;
                    }
                }
            }
            else
            {
                status = kStatus_InvalidPacketType;
            }
        }
    }


    return status;
}

#endif  //ZLINK_HOST

// See serial_packet.h for documentation on this function.
status_t serial_packet_init(const peripheral_descriptor_t *self)
{
    g_serialContext.readOffset = 0;
    g_serialContext.writeOffset = 0;

    return kStatus_Success;
}

// See serial_packet.h for documentation on this function.
status_t serial_packet_finalize(const peripheral_descriptor_t *self)
{
    return send_deferred_ack();
}

// See serial_packet.h for documentation of this method.
void serial_packet_sync(void)
{
    serial_packet_send_sync(zLinkFramingPacketType_Ack);
}

// See peripharal.h for documentation on this interface.
const peripheral_packet_interface_t g_framingPacketInterface = {
    serial_packet_init,
    serial_packet_read,
    serial_packet_write,
    serial_packet_abort,
    serial_packet_finalize,
    serial_packet_get_max_packet_size,
    serial_packet_queue_byte,
    serial_packet_sync,
#ifdef ZLINK_HOST
    serial_packet_ping,
#endif
};

/*
 * endfile
 */
