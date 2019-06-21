/*******************************************************************************
 * i2c.h - definitions for the i2c-bus interface			     	
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

#include "board.h"
#include "fsl_usart.h"
#include "fsl_rtos_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers._rtos_uart"
#endif

/*******************************************************************************
 * Configurations
 ******************************************************************************/



/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Then transfer byte larger than the threshold, use non-blocking method. */
#define UART_TX_INT_THRESHOLD 4



/*******************************************************************************
 * Variables
 ******************************************************************************/
static usart_handle_t s_handle[9];

#if (defined(FSL_RTOS_FREE_RTOS))
/*! @brief IRQ name array */
static const IRQn_Type s_usartIRQ[] = USART_IRQS;
#endif

static SemaphoreHandle_t s_uartSendSem[9];
static SemaphoreHandle_t s_uartRecvSem[9];
static SemaphoreHandle_t s_uartRxMutex[9];
static SemaphoreHandle_t s_uartTxMutex[9];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Private Code
 ******************************************************************************/
static void uart_callback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    uint32_t uart_index = USART_GetInstance(base);
    static BaseType_t xHigherPriorityTaskWoken;
    
    xHigherPriorityTaskWoken = pdFALSE;
    if (kStatus_USART_TxIdle == status)
    {
        xSemaphoreGiveFromISR(s_uartSendSem[uart_index], &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    if (kStatus_USART_RxIdle == status)
    {
        xSemaphoreGiveFromISR(s_uartRecvSem[uart_index], &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*******************************************************************************
 * Code
 ******************************************************************************/


/**
 * Initialises a UART interface
 *
 *
 * @param[in]  uart  the interface which should be initialised
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
status_t USART_RTOS_Init (USART_Type *base, const usart_config_t *config, uint32_t srcClock_Hz)
{
    status_t status;
    uint32_t uart_index = USART_GetInstance(base);

    status = USART_Init(base, config, srcClock_Hz);

    if (kStatus_Success != status)
        return kStatus_Fail;

    s_uartSendSem[uart_index] = xSemaphoreCreateCounting(128, 0);
    if (s_uartSendSem[uart_index] == NULL)
    {
        return kStatus_Fail;
    }
    s_uartRecvSem[uart_index] = xSemaphoreCreateCounting(128, 0);
    if (s_uartRecvSem[uart_index] == NULL)
    {
        return kStatus_Fail;
    }
    s_uartRxMutex[uart_index] = xSemaphoreCreateMutex();
    if (s_uartRxMutex[uart_index] == NULL)
    {
        return kStatus_Fail;
    }
    s_uartTxMutex[uart_index] = xSemaphoreCreateMutex();
    if (s_uartTxMutex[uart_index] == NULL)
    {
        return kStatus_Fail;
    }  

    /* Create handle for LPUART */
    USART_TransferCreateHandle(base, &s_handle[uart_index], uart_callback, NULL);
    
#if (defined(FSL_RTOS_FREE_RTOS))
    NVIC_SetPriority(s_usartIRQ[uart_index], 3);
#endif
    
    return kStatus_Success;
}


/**
 * Transmit data on a UART interface
 *
 * @param[in]  uart  the UART interface
 * @param[in]  data  pointer to the start of data
 * @param[in]  size  number of bytes to transmit
 * @param[in]  timeout Systick number for timeout
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
status_t USART_RTOS_Send (USART_Type *base, const void *data, uint32_t size, uint32_t timeout)
{
#if 0
    USART_WriteBlocking((USART_Type *)s_uartBaseAddrs[uart->port], data, size);
#else
    usart_transfer_t xfer;
    usart_handle_t *handle;
    uint32_t uart_index = USART_GetInstance(base);

    handle = &s_handle[uart_index]; 

    if (timeout == 0)
    {
        timeout = portMAX_DELAY;
    }

    /* Wait for Lock */
    if (xSemaphoreTake(s_uartTxMutex[uart_index], timeout) != pdPASS)
    {
       return kStatus_Fail;
    }

    if (size < UART_TX_INT_THRESHOLD)
    {
        USART_WriteBlocking(base, (uint8_t *)data, size);
    }
    else
    {
        xfer.data = (uint8_t *)data;
        xfer.dataSize = size;
        USART_TransferSendNonBlocking(base, handle, &xfer);

        /* Wait for transfer finish */
        if (xSemaphoreTake(s_uartSendSem[uart_index], timeout) != pdPASS)
        {
            USART_TransferAbortSend(base, handle);

            xSemaphoreGive(s_uartTxMutex[uart_index]);
            return kStatus_Fail;
        }
    }

    xSemaphoreGive(s_uartTxMutex[uart_index]);

#endif
    return kStatus_Success;
}


/**
 * Receive data on a UART interface
 *
 * @param[in]   uart         the UART interface
 * @param[out]  data         pointer to the buffer which will store incoming data
 * @param[in]   expect_size  number of bytes to receive
 * @param[out]  recv_size    number of bytes received
 * @param[in]   timeout      timeout in milisecond
 *
 * @return  0 : on success, EIO : if an error occurred with any step
 */
status_t USART_RTOS_Recv (USART_Type *base, void *data, uint32_t expect_size, uint32_t *recv_size, uint32_t timeout)
{
    usart_transfer_t xfer;
    usart_handle_t *handle;
    uint32_t uart_index = USART_GetInstance(base);
    BaseType_t stat = pdTRUE;

    handle = &s_handle[uart_index]; 

    if (timeout == 0)
    {
        timeout = portMAX_DELAY;
    }

    /* Wait for Lock */
    if (xSemaphoreTake(s_uartRxMutex[uart_index], timeout) != pdPASS)
    {
       
        if (recv_size) {
            *recv_size = 0;
        }
        return kStatus_Fail;
    }

    xfer.data = (uint8_t *)data;
    xfer.dataSize = expect_size;
    if (kStatus_Success != USART_TransferReceiveNonBlocking(base, handle, &xfer, NULL))
    {
        xSemaphoreGive(s_uartRxMutex[uart_index]);
        if (recv_size) {
            *recv_size = 0;
        }
        return kStatus_Fail;
    }

    /* Wait for transfer finish */
    if (xSemaphoreTake(s_uartRecvSem[uart_index], timeout) != pdPASS)
    {
        USART_TransferAbortReceive(base, handle);

        xSemaphoreGive(s_uartRxMutex[uart_index]);
        if (recv_size) {
            *recv_size = 0;
        }
        return kStatus_Fail;
    }
    if (recv_size) {
        *recv_size = expect_size;
    }

    xSemaphoreGive(s_uartRxMutex[uart_index]);

    return kStatus_Success;
}




/*
 * endfile
 */
