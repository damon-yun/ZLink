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

#ifndef ___ZLINK_MICROSECONDS_H__
#define ___ZLINK_MICROSECONDS_H__

#include <stdio.h>
#include <stdbool.h>
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

/*******************************************************************************
 * Variables
 ******************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

/********************************************************************/
//! @brief Initialize timer facilities.
void microseconds_init(void);

//! @brief Shutdown the microsecond timer
void microseconds_shutdown(void);

//! @brief Read back the running tick count
uint64_t microseconds_get_ticks(void);

//! @brief Returns the conversion of ticks to actual microseconds
//!        This is used to seperate any calculations from getting a tick
//         value for speed critical scenarios
uint32_t microseconds_convert_to_microseconds(uint32_t ticks);

//! @brief Returns the conversion of microseconds to ticks
uint64_t microseconds_convert_to_ticks(uint32_t microseconds);

//! @brief Delay specified time
void microseconds_delay(uint32_t us);

//! @brief Set delay time
//!
//! @param us Delay time in microseconds unit
void microseconds_set_delay(uint32_t us);

//! @brief Get timeout flag
//!
//! @retval true Timeout is reached.
//! @retval false Timeout isn't reached.
bool microseconds_timeout(void);

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* ___MICROSECONDS_H__ */

/*! @} */

