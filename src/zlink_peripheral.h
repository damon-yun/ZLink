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

#ifndef _ZLINK_PERIPHERAL_H_
#define _ZLINK_PERIPHERAL_H_

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
//! @brief Peripheral type bit mask definitions.
//!
//! These bit mask constants serve multiple purposes. They are each a unique value that identifies
//! a peripheral type. They are also the mask for the bits used in the bootloader configuration
//! flash region to list available peripherals and control which peripherals are enabled.
enum _peripheral_types
{
    zLinkPeripheralType_UART = (1 << 0),
    zLinkPeripheralType_I2CSlave = (1 << 1),
    zLinkPeripheralType_SPISlave = (1 << 2),
    zLinkPeripheralType_CAN = (1 << 3),
    zLinkPeripheralType_USB_HID = (1 << 4),
    zLinkPeripheralType_USB_CDC = (1 << 5),
    zLinkPeripheralType_USB_DFU = (1 << 6),
    zLinkPeripheralType_USB_MSC = (1 << 7)
};

//! @brief Pinmux types.
typedef enum _pinmux_types
{
    zLinkPinmuxType_Default = 0,
    zLinkPinmuxType_PollForActivity = 1,
    zLinkPinmuxType_Peripheral = 2,
    zLinkPinmuxType_RestoreForActivity = 3
} pinmux_type_t;

// Forward declaration.
typedef struct PeripheralDescriptor peripheral_descriptor_t;

typedef void (*serial_byte_receive_func_t)(uint8_t);

//! @brief Peripheral control interface.
typedef struct _peripheral_control_interface
{
    bool (*pollForActivity)(const peripheral_descriptor_t *self);
    status_t (*init)(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
    void (*shutdown)(const peripheral_descriptor_t *self);
    void (*pump)(const peripheral_descriptor_t *self);
} peripheral_control_interface_t;

//! @brief Peripheral abstract byte interface.
typedef struct _peripheral_byte_inteface
{
    status_t (*init)(const peripheral_descriptor_t *self);
#if (defined(USE_OS) || defined(ZLINK_HOST))
    status_t (*read)(const peripheral_descriptor_t *self, uint8_t *buffer, uint32_t requestedBytes, uint32_t *recv_size, uint32_t timeoutMs);
#endif // USE_OS
    status_t (*write)(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount);
} peripheral_byte_inteface_t;

//! @brief Packet types.
typedef enum _packet_type
{
    zLinkPacketType_Command, //!< Send or expect a command packet
    zLinkPacketType_Data     //!< Send or expect a data packet
} packet_type_t;


//! @brief Peripheral Packet Interface.
typedef struct _peripheral_packet_interface
{
    status_t (*init)(const peripheral_descriptor_t *self);
    status_t (*readPacket)(const peripheral_descriptor_t *self,
                           uint8_t **packet,
                           uint32_t *packetLength,
                           packet_type_t packetType);
    status_t (*writePacket)(const peripheral_descriptor_t *self,
                            const uint8_t *packet,
                            uint32_t byteCount,
                            packet_type_t packetType);
    void (*abortPacket)(const peripheral_descriptor_t *self);
    status_t (*finalize)(const peripheral_descriptor_t *self);
    uint32_t (*getMaxPacketSize)(const peripheral_descriptor_t *self);
    void (*byteReceivedCallback)(uint8_t byte);
    //! @brief Send framing packet ack.
    void (*sync)(void);
    //! @brief Send a ping packet and receive an ack.
    //!
    //! This is a method for host only side pinging of the target. The reponse from the
    //! target to a ping packet is a ping response packet. Since the target may or may
    //! not be online there is optionally a series of retries to make the best attempt
    //! at communication possible
    //!
    //! @param retries The number of attempts that should be made.
    //! @param delay The time in milliseconds between each attempt.
    //! @param comSpeed The peripheral baud rate. Used in order to calculate the
    //!     receive delay in the case of low com speeds such as 100 and 300 which need
    //!     nearly a second to complete
    status_t (*sendPingGetResponse) (int retries, unsigned int delay, void *response);
    
} peripheral_packet_interface_t;


//! @brief Peripheral descriptor.
//!
//! Instances of this struct describe a particular instance of a peripheral that is
//! available for bootloading.
struct PeripheralDescriptor
{
    //! @brief Bit mask identifying the peripheral type.
    //!
    //! See #_peripheral_types for a list of valid bits.
    uint32_t typeMask;

    //! @brief The instance number of the peripheral.
    uint32_t instance;

   //! @brief Configure pinmux setting for the peripheral.
   void (*pinmuxConfig)(uint32_t instance, pinmux_type_t pinmux);

    //! @brief Control interface for the peripheral.
    const peripheral_control_interface_t *controlInterface;

    //! @brief Byte-level interface for the peripheral.
    //!
    //! May be NULL since not all periperhals support this interface.
    const peripheral_byte_inteface_t *byteInterface;

    //! @brief Packet level interface for the peripheral.
    const peripheral_packet_interface_t *packetInterface;

//#if (defined(USE_OS) && (USE_OS == 1))
//    zlink_os_mutex_t peripheralTxMutex;
//    zlink_os_mutex_t peripheralRxMutex;
//    zlink_os_sem_t   peripheralTxSem;
//    zlink_os_sem_t   peripheralTxSem;
//#endif

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

/* @} */

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _ZLINK_PERIPHERAL_H_ */
