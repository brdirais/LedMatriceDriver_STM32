/**
  ******************************************************************************
  * @file    SPI/SPI_FullDuplex_ComPolling/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32F3xx SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Polling transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup SPI_FullDuplex_ComPolling
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t errorCode;
uint8_t countOn = 5;
char    ErrorStringToDisplay[30];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
void ConfigureUserLED( void);


/* Private functions ---------------------------------------------------------*/

void HAL_MspInit(void)
{

}



/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  intensity;

  /* STM32F3xx HAL library initialization:
       - Configure the Flash prefetch
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  ErrorStringToDisplay[0] = 0;
  GPIOA->BSRR=0xFF;

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();


  if( matriceLed_Init() )
  {
	  Error_Handler();
  }

  HAL_Delay(1000);

  matriceLed_SwitchOnOff( MATRICE_ON);
  HAL_Delay(100);
  matriceLed_DisplayString( "~~~~~ Degemer Mat ~~~~~");

  HAL_Delay(2000);
  matriceLed_SwitchOnOff( MATRICE_OFF);
  HAL_Delay(2000);
  matriceLed_SwitchOnOff( MATRICE_ON);


  HAL_Delay(100);
  matriceLed_DisplayString( "======> SetIntensity Test");

  for( intensity = 0; intensity<0x10 ; intensity++)
  {
	  matriceLed_SetIntensity( 0x0F - intensity );
	  HAL_Delay(400);
  }

  HAL_Delay(1000);
  matriceLed_SetIntensity( 0x0F );


  HAL_Delay(1000);
  matriceLed_DisplayString( "======> Test Mode");

  HAL_Delay(1000);
  matriceLed_DisplayTestMode( 0x01);


  HAL_Delay(1000);
  matriceLed_DisplayTestMode( 0x00);
  HAL_Delay(100);
  matriceLed_SwitchOnOff( MATRICE_ON);


  /* Infinite loop */
  while (1)
  {
	  matriceLed_DisplayString( "---- Bye ----");
	  HAL_Delay(1000);
  }
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSE / 1) * PLLMUL = (8 / 1) * 2 = 16 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_ON;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL2; // old : RCC_PLL_MUL4  ==> PLL = 64MHz
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}


void Error_Handler( void )
{
	while(1) {}
}



/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
