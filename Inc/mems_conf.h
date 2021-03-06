/**
 ******************************************************************************
 * @file    mems_conf.h
 * @author  MEMS Application Team
 * @version V0.0.1
 * @date    29-January-2018
 * @brief   This file contains definitions for the MEMS components bus interfaces
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_CONF_H__
#define __MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"

#define USE_MOTION_SENSOR_LSM6DSL_0            1U

#define BSP_LSM6DSL_SPI_Init BSP_SPI2_Init
#define BSP_LSM6DSL_SPI_DeInit BSP_SPI2_DeInit
#define BSP_LSM6DSL_SPI_Send BSP_SPI2_Send
#define BSP_LSM6DSL_SPI_Recv BSP_SPI2_Recv

#define BSP_LSM6DSL_CS_PORT GPIOB
#define BSP_LSM6DSL_CS_PIN GPIO_PIN_12

void lsm6dsl_apply_spi_settings(void);

#ifdef __cplusplus
}
#endif

#endif /* __MEMS_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

