// NOTE: was called stm32g0xx_hal_msp.c
#include "project_config.h"
#include "hardware_profile.h"
#include "stm32g0xx_hal.h"

void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

#ifdef HAL_CRC_MODULE_ENABLED

void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  }
}
#endif // HAL_CRC_MODULE_ENABLED

#ifdef HAL_I2S_MODULE_ENABLED

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
    hdma_spi1_rx.Instance = DMA1_Channel1;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_spi1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hi2s,hdmarx,hdma_spi1_rx);
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

#endif //HAL_I2S_MODULE_ENABLED

#ifdef HAL_I2C_MODULE_ENABLED

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1) {

    // setup SCL pin:
	I2C1_SCL_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2C1_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    // setup SDA pin:
    I2C1_SDA_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2C1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(I2C1_SDA_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

#ifdef USE_I2C1_INTERRUPTS
    HAL_NVIC_SetPriority(I2C1_IRQn, I2C1_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
#endif
  }
  else if(hi2c->Instance==I2C2) {

    // setup SCL pin:
    I2C2_SCL_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2C2_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);

    // setup SDA pin:
    I2C2_SDA_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = I2C2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(I2C2_SDA_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

#ifdef USE_I2C2_INTERRUPTS
    HAL_NVIC_SetPriority(I2C2_IRQn, I2C2_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
#endif
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    HAL_GPIO_DeInit(I2C1_SCL_PORT, I2C1_SCL_PIN);
    HAL_GPIO_DeInit(I2C1_SDA_PORT, I2C1_SDA_PIN);

#ifdef USE_I2C1_INTERRUPTS
    HAL_NVIC_DisableIRQ(I2C1_IRQn);
#endif
  }
  else if(hi2c->Instance==I2C2) {

    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    HAL_GPIO_DeInit(I2C2_SCL_PORT, I2C2_SCL_PIN);
    HAL_GPIO_DeInit(I2C2_SDA_PORT, I2C2_SDA_PIN);

#ifdef USE_I2C2_INTERRUPTS
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
#endif
  }
}

#endif //HAL_I2C_MODULE_ENABLED

#ifdef HAL_UART_MODULE_ENABLED

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (huart->Instance==LPUART1) {
    __HAL_RCC_LPUART1_CLK_ENABLE();
  
    // TX pin:
    LPUART1_TX_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = LPUART1_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_LPUART1;
    HAL_GPIO_Init(LPUART1_TX_PORT, &GPIO_InitStruct);

    // RX pin:
    LPUART1_RX_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = LPUART1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF6_LPUART1;
    HAL_GPIO_Init(LPUART1_RX_PORT, &GPIO_InitStruct);
  }
  else if(huart->Instance==USART4) {
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
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==LPUART1)
  {
    __HAL_RCC_LPUART1_CLK_DISABLE();
  
    HAL_GPIO_DeInit(LPUART1_RX_PORT,LPUART1_RX_PIN);
    HAL_GPIO_DeInit(LPUART1_TX_PORT,LPUART1_TX_PIN);
  }
  else if(huart->Instance==USART4)
  {
    __HAL_RCC_USART4_CLK_DISABLE();

    HAL_GPIO_DeInit(USART4_RX_PORT,USART4_RX_PIN);
    HAL_GPIO_DeInit(USART4_TX_PORT,USART4_TX_PIN);
  }
}

#endif // HAL_UART_MODULE_ENABLED

// NB: interrupt priorities of timers in this function get later overwritten by the
// individual timer init functions.
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
  
    // set up the gate pin:
    TIM2_GATE_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = TIM2_GATE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
    HAL_GPIO_Init(TIM2_GATE_PORT, &GPIO_InitStruct);
  }
  else if(htim_base->Instance==TIM3)
	{
	  __HAL_RCC_TIM3_CLK_ENABLE();
	  /* TIM3 interrupt Init */
	  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
  else if(htim_base->Instance==TIM14)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();
    /* TIM14 interrupt Init */
    HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
  }
  else if(htim_base->Instance==TIM15)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
    /* TIM15 interrupt Init */
    HAL_NVIC_SetPriority(TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);
  }
  else if(htim_base->Instance==TIM16)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM16_CLK_ENABLE();
    /* TIM16 interrupt Init */
    HAL_NVIC_SetPriority(TIM16_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM16_IRQn);
  }
  else if(htim_base->Instance==TIM17)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM17_CLK_ENABLE();
    /* TIM17 interrupt Init */
    HAL_NVIC_SetPriority(TIM17_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM17_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_GPIO_DeInit(TIM2_GATE_PORT, TIM2_GATE_PIN);
  }
  else if(htim_base->Instance==TIM3)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
  else if(htim_base->Instance==TIM14)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM14_CLK_DISABLE();

    /* TIM14 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM14_IRQn);
  }
  else if(htim_base->Instance==TIM15)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();

    /* TIM15 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM15_IRQn);
  }
  else if(htim_base->Instance==TIM16)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM16_CLK_DISABLE();

    /* TIM16 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM16_IRQn);
  }
  else if(htim_base->Instance==TIM17)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM17_CLK_DISABLE();

    /* TIM17 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM17_IRQn);
  }
}

