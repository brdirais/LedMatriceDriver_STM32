/**
  ******************************************************************************
  * File Name          : matriceLed.h
  * Description        : This file contains all the functions prototypes for
  *                      the LED matrice management
  ******************************************************************************
  * 3-Clause BSD License
  *
  * Copyright 2020 - Bruno DIRAISON
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice, this list
  * of conditions and the following disclaimer.
  *
  * 2. Redistributions in binary form must reproduce the above copyright notice, this
  * list of conditions and the following disclaimer in the documentation and/or other
  * materials provided with the distribution.
  *
  * 3. Neither the name of the copyright holder nor the names of its contributors may
  * be used to endorse or promote products derived from this software without specific
  * prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  * SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATRICE_LED_H
#define __MATRICE_LED_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <string.h>


 /**SPI1 GPIO Configuration
 PA4   ------> SPI1_NSS
 PA5   ------> SPI1_SCK
 PA7   ------> SPI1_MOSI
 */
 #define SPIx                             SPI1
 #define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
 #define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
 #define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
 #define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

 #define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
 #define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

 /* Definition for SPIx Pins */
 #define SPIx_SCK_PIN                     GPIO_PIN_5
 #define SPIx_SCK_GPIO_PORT               GPIOA
 #define SPIx_SCK_AF                      GPIO_AF5_SPI1
 #define SPIx_MOSI_PIN                    GPIO_PIN_7
 #define SPIx_MOSI_GPIO_PORT              GPIOA
 #define SPIx_MOSI_AF                     GPIO_AF5_SPI1

 /* Definition for CS pin */
 #define CS_GPIO_CLK_ENABLE()       	__HAL_RCC_GPIOA_CLK_ENABLE()
 #define CS_PIN							GPIO_PIN_4
 #define CS_GPIO_PORT					GPIOA

#define ML_NB_MAX7219	4



/*
 * MAX7219 type definition
 */

/* Command */
typedef enum {
	NO_OP 			= 0x00,
	DIGIT_0 		= 0x01,
	DIGIT_1,
	DIGIT_2,
	DIGIT_3,
	DIGIT_4,
	DIGIT_5,
	DIGIT_6,
	DIGIT_7,
	DECODE_MODE		= 0x09,
	INTENSITY		= 0x0A,
	SCAN_LIMIT		= 0x0B,
	SHUTDOWN		= 0x0C,
	DISPLAY_TEST 	= 0x0F
} max7219_cmd;


/* Matrice state: on or off */
#define MATRICE_ON  1
#define MATRICE_OFF 0


 /* API Prototypes -----------------------------------------------------------*/
 uint8_t matriceLed_Init(void);
 uint8_t matriceLed_DisplayTestMode( uint8_t OnOff);
 uint8_t matriceLed_SetIntensity( uint8_t intensity);
 uint8_t matriceLed_SwitchOnOff( uint8_t OnOff);
 uint8_t matriceLed_DisplayString( char string[]);


#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
