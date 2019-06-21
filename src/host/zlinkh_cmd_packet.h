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

#ifndef ZLINK_CMD_PACKET_H_
#define ZLINK_CMD_PACKET_H_

#include "zlink_common.h"

#include "zlink_serial_packet.h"
#include "zlink_command.h"

typedef struct __zlinkh_cmd_data {
    //arg
    command_packet_t m_header;                                      //!< Packet header.
    uint32_t m_arguments[zLinkMaxCommandArguments];                     //!< Command arguments.  DefaultMaxPacketSize = 7;
    uint32_t m_responseValues;    //!< response values.
} zlinkh_cmd_data_t;

typedef struct __zlinkh_cmd_packet_dev {
    //funcs
    void     (*pfun_packet_init) (uint8_t tag, uint8_t flags, uint32_t *arg, uint8_t numArguments);
    uint32_t (*pfun_get_size) (void);
    void     (*pfun_get_data) (uint8_t *data, uint32_t size);
    uint8_t *(*pfun_send_command_get_response) (peripheral_packet_interface_t *packet_interface);
    //! @brief Check generic response packet.
    bool (*pfun_superclass_process_response) (const uint8_t *packet, uint8_t commandTag);
} zlinkh_cmd_packet_t;


typedef zlinkh_cmd_packet_t *zlinkh_cmd_packet_handle_t;

extern const zlinkh_cmd_packet_t g_zlinkHostCmdPacket;

#endif /* ZLINK_CMD_PACKET_H_ */
