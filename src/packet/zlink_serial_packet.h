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

#ifndef _ZLINK_SERIAL_PACKET_H_
#define _ZLINK_SERIAL_PACKET_H_

#include "zlink_common.h"

#include "zlink_peripheral.h"

/*! @addtogroup rom_flexspi */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Configurations
 ******************************************************************************/
//! @brief Packetizer status codes.
enum _packetizer_status
{
    kStatus_NoPingResponse = ZLINK_MAKE_STATUS(kStatusGroup_Packetizer, 0),
    kStatus_InvalidPacketType = ZLINK_MAKE_STATUS(kStatusGroup_Packetizer, 1),
    kStatus_InvalidCRC = ZLINK_MAKE_STATUS(kStatusGroup_Packetizer, 2),
    kStatus_NoCommandResponse = ZLINK_MAKE_STATUS(kStatusGroup_Packetizer, 3)
};

//! @breif Constants.
enum _uart_peripheral_constants
{
    // The read() implementation for the UartPeripheral does not use this the timeout parameter.
    kUartPeripheral_UnusedTimeout = 0,
    // Serial timeout is set to this default during init().
    kUartPeripheral_DefaultReadTimeoutMs = 1000,
    kUartPeripheral_DefaultBaudRate = 115200
};

/*******************************************************************************
 * Definitions
 ******************************************************************************/

//! @brief Packet state machine modes.
enum _serial_packet_mode
{
    zLinkSerialModeCmd,
    zLinkSerialModeAck,
    zLinkSerialModeIdle
};

//! @brief Version constants for serial framing protocol.
//! @note Recalculate crc16 in k_PingResponse if these values change.
enum _serial_protocol_version_constants
{
    zLinkSerialProtocol_Version_Name = 'P',
    zLinkSerialProtocol_Version_Major = 1,
    zLinkSerialProtocol_Version_Minor = 0,
    zLinkSerialProtocol_Version_Bugfix = 0
};

//! @brief Serial framing packet constants.
enum _framing_packet_constants
{
    zLinkFramingPacketStartByte = 0x5a,
    zLinkFramingPacketType_Ack = 0xa1,
    zLinkFramingPacketType_Nak = 0xa2,
    zLinkFramingPacketType_AckAbort = 0xa3,
    zLinkFramingPacketType_Command = 0xa4,
    zLinkFramingPacketType_Data = 0xa5,
    zLinkFramingPacketType_Ping = 0xa6,
    zLinkFramingPacketType_PingResponse = 0xa7
};

#pragma pack(1)

//! @brief Serial framing header.
typedef struct FramingHeader
{
    uint8_t startByte;  //!< #kFramingPacketStartByte
    uint8_t packetType; //!< Framing packet type
} framing_header_t;

typedef struct FramingDataPacket
{
    framing_header_t header; //!< Framing packet header
    uint16_t length;         //!< Number of data bytes that follow
    uint16_t crc16;          //!< CRC-16 of data packet header and data
} framing_data_packet_t;

//! @brief Serial framing sync packet.
typedef struct FramingSyncPacket
{
    framing_header_t header; //!< Framing packet header
} framing_sync_packet_t;

//! @brief Framing packet with data area.
typedef struct SerialFramingPacket
{
    framing_data_packet_t dataPacket;        //!< Packet header.
    uint8_t data[zLinkOutgoingPacketBufferSize]; //!< Payload.
} serial_framing_packet_t;

//! @brief Format of global context data.
typedef struct SerialData
{
    uint8_t data[zLinkIncomingPacketBufferSize]; //!< Buffer for incomming packet data payload, must be uint32_t aligned.
    uint8_t callbackBuffer[zLinkCallbackBufferSize]; //!< Buffer for incoming data from the byte callback
    serial_framing_packet_t framingPacket;       //!< Buffer for outgoing packet.
    volatile uint32_t writeOffset;               //!< The offset into the buffer that the ISR will queue data into
    uint32_t readOffset;                         //!< The offset into the buffer that the app has read out
    bool isAckNeeded;                            //!< True if need to send ACK to previously received packet
    bool isBackToBackWrite;                      //!< True if executing back-to-back write
    bool isAckAbortNeeded;                       //!< True if next ACK should be ACK Abort
} serial_data_t;


/**
 * \brief 使用匿名联合体段开始
 * @{
 */
#if (__ARMCC_VERSION < 6010050)

#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
    
  /* 默认使能匿名联合体 */
#elif defined(__TMS470__)

  /* 默认使能匿名联合体 */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler t
#endif

#endif //__ARMCC_VERSION

/** @} */


//! @brief Structure of version property.
//!
//! @ingroup bl_core
typedef union StandardVersion
{
    struct
    {
        uint8_t bugfix; //!< bugfix version [7:0]
        uint8_t minor;  //!< minor version [15:8]
        uint8_t major;  //!< major version [23:16]
        char name;      //!< name [31:24]
    };
    uint32_t version; //!< combined version numbers

#if defined(__cplusplus)
    StandardVersion()
        : version(0)
    {
    }
    StandardVersion(uint32_t version)
        : version(version)
    {
    }
#endif
} standard_version_t;

//! @brief Serial ping response format.
//!
//! This is the format of the response to a Ping packet.
typedef struct PingResponse
{
    standard_version_t version; //!< Serial framing protocol version
    uint16_t options;           //!< Serial framing protocol options bitfield
    uint16_t crc16;             //!< CRC-16 of other fields
} ping_response_t;

/**
 * \brief 使用匿名联合体段结束
 * @{
 */
#if (__ARMCC_VERSION < 6010050)

#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
 
  /* 允许匿名联合体使能 */
#elif defined(__GNUC__)

  /* 默认使用匿名联合体 */
#elif defined(__TMS470__)

  /* 默认使用匿名联合体 */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler t
#endif

#endif  //__ARMCC_VERSION

/** @} */

#pragma pack()


#if defined(__CC_ARM)
#pragma anon_unions
#endif


/*******************************************************************************
 * Variables
 ******************************************************************************/
extern const peripheral_packet_interface_t g_framingPacketInterface;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

 
/*******************************************************************************
 * API
 ******************************************************************************/

//! @brief Initialize component.
status_t serial_packet_init(const peripheral_descriptor_t *self);

//! @brief Read packet using serial framing.
//!
//! On return, caller must call flow control method to send AckContinue or AckWait followed by Continue.
status_t serial_packet_read(const peripheral_descriptor_t *self,
                            uint8_t **packet,
                            uint32_t *packetLength,
                            packet_type_t packetType);

//! @brief Write packet using serial framing.
status_t serial_packet_write(const peripheral_descriptor_t *self,
                             const uint8_t *packet,
                             uint32_t byteCount,
                             packet_type_t packetType);

//! @brief Abort data phase.
//!
//! Respond to next host data packet with AckAbort instead of Ack
//! (i.e. receiver data phase abort).
void serial_packet_abort(const peripheral_descriptor_t *self);

//! @brief Finalize.
status_t serial_packet_finalize(const peripheral_descriptor_t *self);

//! @brief Get max packet size.
uint32_t serial_packet_get_max_packet_size(const peripheral_descriptor_t *self);

//! @brief Send a sync packet of the specified type.
status_t serial_packet_send_sync(uint8_t framingPacketType);

//! @brief Send a ping message back in response to a ping.
status_t serial_send_ping_response(const peripheral_descriptor_t *peripheral);

//! @brief Queues a byte received by the active peripheral
void serial_packet_queue_byte(uint8_t byte);



/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _zlink_SERIAL_PACKET_H_ */
