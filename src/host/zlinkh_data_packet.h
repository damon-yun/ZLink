/*******************************************************************************
 * zlinkh_cmd_packet.h - definitions for the zlink host command interface			     	
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

#ifndef __ZLINK_DATA_PACKET_H_
#define __ZLINK_DATA_PACKET_H_

#include "zlink_common.h"

#include "zlink_serial_packet.h"
#include "zlink_command.h"

typedef struct __data_producer {
    void (*pfunInit) (uint8_t *data, uint32_t size);
    //! @brief Query if more data is available.
    bool (*pfunHasMoreData)(void);

    //! @brief Query the total size of the data.
    uint32_t (*pfunGetDataSize)(void);

    //! @brief Get the next data chunk.
    //!
    //! Before calling getData(), call hasMoreData() to determine if
    //! data is available.
    uint32_t (*pfunGetData)(uint8_t *data, uint32_t size);
} data_producer_t;


typedef struct __data_consumer {
    //! @brief Process the next data chunk.
    void (*pfunProcessData) (const uint8_t *data, uint32_t size);

    //! @brief Finalize processing.
    void (*pfunFinalize) (void);
} data_consumer_t;


typedef void(*progressCallback_t)(int percentage, int segmentIndex, int segmentCount); //!< The progress callback function.

typedef struct __zlinkh_data_packet {
    //funcs
    void     (*pfun_send_init) (uint8_t *data, uint32_t size);

    void     (*pfun_recv_init) (data_consumer_t *pDataConsumer);

    //! @brief Send data packet to device.
    //!
    //! Calls the data provide to get the data to send.
    uint8_t *(*pfunSendTo)(peripheral_packet_interface_t *packet_interface, uint32_t *bytesWritten, progressCallback_t progress);

    //! @brief Receive data packet from device.
    //!
    //! Calls the data consumer to process the receied data.
    uint8_t *(*pfunReceiveFrom)(peripheral_packet_interface_t *packet_interface, uint32_t *byteCount, progressCallback_t progress);

} zlinkh_data_packet_t;

extern const zlinkh_data_packet_t g_zlinkHostDataPacket;
extern const data_producer_t m_dataProducer;

#endif /* __ZLINK_DATA_PACKET_H_ */
