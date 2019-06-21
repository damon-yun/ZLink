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

#include "zlink_command.h"
#include "zlink_context.h"

#include "zlinkh_command.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "zlink.device.main"
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

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Private Code
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/
extern volatile uint32_t g_lock_stat;
//! @brief zlink device outer loop.
//!
//! Infinitely calls the command interface and active peripheral control interface pump routines.
void zlink_host_main (void *arg)
{
    volatile status_t status = 0;
    
    const peripheral_descriptor_t *activePeripheral = g_zLinkDeviceContext.activePeripheral;
    assert(g_zLinkDeviceContext.commandInterface->pump);

    // Init the active peripheral.
    if (g_zLinkDeviceContext.activePeripheral->controlInterface &&
        g_zLinkDeviceContext.activePeripheral->controlInterface->init)
    {
        g_zLinkDeviceContext.activePeripheral->controlInterface->init(g_zLinkDeviceContext.activePeripheral, 
                                    g_zLinkDeviceContext.activePeripheral->packetInterface->byteReceivedCallback);
    }
    if (g_zLinkDeviceContext.activePeripheral->byteInterface &&
        g_zLinkDeviceContext.activePeripheral->byteInterface->init)
    {
        g_zLinkDeviceContext.activePeripheral->byteInterface->init(g_zLinkDeviceContext.activePeripheral);
    }
    if (g_zLinkDeviceContext.activePeripheral->packetInterface &&
        g_zLinkDeviceContext.activePeripheral->packetInterface->init)
    {
        g_zLinkDeviceContext.activePeripheral->packetInterface->init(g_zLinkDeviceContext.activePeripheral);
    }

    // Initialize the command processor component.
    g_zLinkDeviceContext.commandInterface->init();
    
    status = ping_cmd.pfun_sendTo(NULL);
    
//    get_property_arg_t get_property_arg = {
//        .property_tag = zLinkPropertyTag_MaxPacketSize,
//        .memory_id = zLinkMemoryInternal
//    };
//    get_property_cmd.pfun_init(&get_property_arg);
//    status = get_property_cmd.pfun_sendTo(NULL);
    
    uint8_t buffer[] = "1234567890";

    flash_erase_region_arg_t flash_erase_region_arg = {
        .start_address = 0x00000000,
        .byte_count = 10,
        .memory_id = 0
    };
    flash_erase_region_cmd.pfun_init(&flash_erase_region_arg);
    status = flash_erase_region_cmd.pfun_sendTo(NULL);

    write_memory_arg_t write_memory_arg = {
        .start_address = 0x00000000,
        .byte_count = 10,
        .memory_id = 0,
        .pdata = buffer
    };
    write_memory_cmd.pfun_init(&write_memory_arg);
    status = write_memory_cmd.pfun_sendTo(NULL);
    
    read_memory_arg_t read_memory_arg = {
        .start_address = 0x00000000,
        .byte_count = 10,
        .memory_id = 0
    };
    read_memory_cmd.pfun_init(&read_memory_arg);
    status = read_memory_cmd.pfun_sendTo(NULL);
    
    status = status;
    // Read and execute commands.
    while (1)
    {
        __NOP();
//         g_zLinkDeviceContext.commandInterface->pump();
    }
}





/*
 * endfile
 */
