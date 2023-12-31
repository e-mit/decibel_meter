// Define initialisation functions used by the Hardware Abstraction Layer (HAL)
// MCU Support Package (MSP). These set up the I2S, UART and TMR3 peripherals.
// Also see hardware_profile.h for pin definitions.

#include "hardware_profile.h"
#include "stm32g0xx_hal.h"

/**
  ******************************************************************************
  * @file         stm32g0xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2S_MspInit(I2S_HandleTypeDef* hi2s)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2s->Instance==SPI1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    // setup CLK pin:
    I2S1_CLK_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2S1_CLK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = I2S1_CLK_AF_MAP;
    HAL_GPIO_Init(I2S1_CLK_PORT, &GPIO_InitStruct);

    // setup WS pin:
    I2S1_WS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2S1_WS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = I2S1_WS_AF_MAP;
    HAL_GPIO_Init(I2S1_WS_PORT, &GPIO_InitStruct);

    // setup SD pin:
    I2S1_SD_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2S1_SD_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	#ifdef I2S1_SD_INTERNAL_PD
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	#else
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	#endif
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = I2S1_SD_AF_MAP;
    HAL_GPIO_Init(I2S1_SD_PORT, &GPIO_InitStruct);

    /* I2S1 DMA Init */
    /* SPI1_RX Init */
    dma1.Instance = DMA1_Channel1;
    dma1.Init.Request = DMA_REQUEST_SPI1_RX;
    dma1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    dma1.Init.PeriphInc = DMA_PINC_DISABLE;
    dma1.Init.MemInc = DMA_MINC_ENABLE;
    dma1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    dma1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    dma1.Init.Mode = DMA_CIRCULAR;
    dma1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&dma1) != HAL_OK)
    {
    	errorHandler(__func__, __LINE__, __FILE__);
    }

    __HAL_LINKDMA(hi2s, hdmarx, dma1);
  }

}

void HAL_I2S_MspDeInit(I2S_HandleTypeDef* hi2s)
{
  if(hi2s->Instance==SPI1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    HAL_GPIO_DeInit(I2S1_CLK_PORT, I2S1_CLK_PIN);
    HAL_GPIO_DeInit(I2S1_WS_PORT, I2S1_WS_PIN);
    HAL_GPIO_DeInit(I2S1_SD_PORT, I2S1_SD_PIN);

    /* I2S1 DMA DeInit */
    HAL_DMA_DeInit(hi2s->hdmarx);
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	UNUSED(huart);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_USART4_CLK_ENABLE();

	// TX pin:
	USART4_TX_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = USART4_TX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART4;
	HAL_GPIO_Init(USART4_TX_PORT, &GPIO_InitStruct);

	// RX pin:
	USART4_RX_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = USART4_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART4;
	HAL_GPIO_Init(USART4_RX_PORT, &GPIO_InitStruct);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	UNUSED(huart);
	__HAL_RCC_USART4_CLK_DISABLE();
	HAL_GPIO_DeInit(USART4_RX_PORT,USART4_RX_PIN);
	HAL_GPIO_DeInit(USART4_TX_PORT,USART4_TX_PIN);
}

// NB: interrupt priorities of timers in this function get later overwritten by the
// individual timer init functions.
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
	{
	  __HAL_RCC_TIM3_CLK_ENABLE();
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
}

