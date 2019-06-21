/*******************************************************************************
 * zlink_hal_uart.c - definitions for the zlink uart hal interface			     	  	
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
 *    zlink硬件层uart接口
 *
 *****************************************************************************/

#include "zlink_common.h"

#include "zlink_hal_uart.h"
#include "zlink_peripheral.h"
#include "zlink_serial_packet.h"

#include "board.h"
#include "fsl_usart.h"

#if (defined(USE_OS) && (USE_OS == 1))
   #include "fsl_rtos_uart.h"
#endif

#include <stdbool.h>

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "zlink.hal.uart"
#endif

/*******************************************************************************
 * Configurations
 ******************************************************************************/



/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_USART USART4
#define DEMO_USART_CLK_SRC kCLOCK_Flexcomm4
#define DEMO_USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm4)
#define DEMO_USART_IRQHandler FLEXCOMM4_IRQHandler
#define DEMO_USART_IRQn FLEXCOMM4_IRQn

/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16
/*******************************************************************************
 * Variables
 ******************************************************************************/

static serial_byte_receive_func_t s_lpuart_byte_receive_callback;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Private Code
 ******************************************************************************/

/*******************************************************************************
 * ISR Code
 ******************************************************************************/
#if (defined(USE_OS) && (USE_OS == 1))

#else

void DEMO_USART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEMO_USART))
    {
        data = USART_ReadByte(DEMO_USART);


        

        /* If ring buffer is not full, add data to ring buffer. */
        if (s_lpuart_byte_receive_callback) {
            s_lpuart_byte_receive_callback(data);
        }


    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
#endif

/*******************************************************************************
 * Byte inteface Code
 ******************************************************************************/

status_t hal_uart_init (const peripheral_descriptor_t *self)
{
    UNUSED_PARAMETER(self);

    return kStatus_Success;
}


status_t hal_uart_write (const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount)
{
    UNUSED_PARAMETER(self);
#if (defined(USE_OS) && (USE_OS == 1))
    /* Send g_tipString out. */
    USART_RTOS_Send(DEMO_USART, buffer, byteCount, 0);
#else
    /* Send g_tipString out. */
    USART_WriteBlocking(DEMO_USART, buffer, byteCount);
#endif
    return kStatus_Success;
}

#if (defined(USE_OS) || defined(ZLINK_HOST))
status_t hal_uart_read (const peripheral_descriptor_t *self, uint8_t *buffer, uint32_t requestedBytes, uint32_t *recv_size, uint32_t timeoutMs)
{
    status_t status = kStatus_Success;
    uint32_t currentBytesRead = 0;

#if (defined(USE_OS) && (USE_OS == 1))
    /* Wait for Lock */
    return USART_RTOS_Recv(DEMO_USART, buffer, requestedBytes, NULL, timeoutMs);
#else
    if (timeoutMs == 0) {        //wait forever
        DisableIRQ(DEMO_USART_IRQn);
        USART_ReadBlocking(DEMO_USART, buffer, requestedBytes);
        EnableIRQ(DEMO_USART_IRQn);
        return kStatus_Success;
    }
    
    microseconds_set_delay(timeoutMs * 1000);   //us timeout
    while (1) {
        if (USART_GetStatusFlags(DEMO_USART) & USART_FIFOSTAT_RXNOTEMPTY_MASK) {
            buffer[currentBytesRead] = USART_ReadByte(DEMO_USART);
            microseconds_set_delay(timeoutMs * 1000);   //us timeout
            currentBytesRead++;
            if (currentBytesRead == requestedBytes) {
                *recv_size = currentBytesRead;
                return kStatus_Success;
            }
        } else {
            if (microseconds_timeout()) {
                *recv_size = currentBytesRead;
                return kStatus_Timeout;
            }
        }
    }

#endif
    return kStatus_Success;
}
#endif // USE_OS || ZLINK_HOST

const peripheral_byte_inteface_t g_usartByteInterface = {
    .init = hal_uart_init, 
#if (defined(USE_OS) || defined(ZLINK_HOST))
    .read =  hal_uart_read,
#endif 
    .write = hal_uart_write 
};

/*******************************************************************************
 * Control Inteface Code
 ******************************************************************************/
status_t hal_uart_control_interface_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    UNUSED_PARAMETER(self);

    usart_config_t config;

    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTxFifo = false;
     * config.enableRxFifo = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx = true;
    config.enableRx = true;
    
#if (defined(USE_OS) && (USE_OS == 1))
    USART_RTOS_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);    
#else
    
    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);

    if (function) {
        s_lpuart_byte_receive_callback = function;
    }
    
    /* Enable RX interrupt. */
    USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(DEMO_USART_IRQn);

#endif

    return kStatus_Success;
}

void hal_uart_control_interface_shutdown (const peripheral_descriptor_t *self)
{
    UNUSED_PARAMETER(self);

    DisableIRQ(DEMO_USART_IRQn);
    USART_Deinit(DEMO_USART);
}

void hal_uart_control_interface_pump (const peripheral_descriptor_t *self)  //unused
{
    UNUSED_PARAMETER(self);

    if (USART_GetStatusFlags(DEMO_USART) & USART_FIFOSTAT_RXNOTEMPTY_MASK) { 
                /* If ring buffer is not full, add data to ring buffer. */
        if (s_lpuart_byte_receive_callback) {
            s_lpuart_byte_receive_callback(USART_ReadByte(DEMO_USART));
        }
    }
}

const peripheral_control_interface_t g_lpuartControlInterface = {
    .pollForActivity = 0, .init = hal_uart_control_interface_init, .shutdown = hal_uart_control_interface_shutdown, .pump = 0
};

/*******************************************************************************
 * Peripheral Struct
 ******************************************************************************/

//! @brief Peripheral array for MIMXRT1051.
const peripheral_descriptor_t g_serailPeripherals = {
    .typeMask = zLinkPeripheralType_UART,
    // .instance = 1,
    // .pinmuxConfig = uart_pinmux_config,
    .controlInterface = &g_lpuartControlInterface,
    .byteInterface = &g_usartByteInterface,
    .packetInterface = &g_framingPacketInterface 
};

/*
 * endfile
 */
