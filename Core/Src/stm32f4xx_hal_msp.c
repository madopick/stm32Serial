/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

extern DMA_HandleTypeDef hdma_usart2_rx;

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* External functions --------------------------------------------------------*/

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin 		= USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull 		= GPIO_NOPULL;
    GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate 	= GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance 				= DMA1_Stream5;
    hdma_usart2_rx.Init.Channel 			= DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc 				= DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment 	= DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode 				= DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority 			= DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode 			= DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }

}

/**
  * @brief I2C MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  *           - DMA configuration for transmission request by peripheral
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if (hi2c->Instance==I2C1)
  {
	  /*##-1- Enable GPIO Clocks #################################################*/
	  /* Enable GPIO TX/RX clock */
	  I2Cx_SCL_GPIO_CLK_ENABLE();
	  I2Cx_SDA_GPIO_CLK_ENABLE();

	  /*##-2- Configure peripheral GPIO ##########################################*/
	  /* I2C TX GPIO pin configuration  */
	  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
	  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull      = GPIO_PULLUP;
	  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

	  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

	  /* I2C RX GPIO pin configuration  */
	  GPIO_InitStruct.Pin 		= I2Cx_SDA_PIN;
	  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;

	  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

	  /*##-3- Enable I2C peripheral Clock ########################################*/
	  /* Enable I2C1 clock */
	  I2Cx_CLK_ENABLE();

	  /*##-4- Configure the NVIC for I2C #########################################*/
	  /* NVIC for I2C1 */
	  HAL_NVIC_SetPriority(I2Cx_ER_IRQn, 3, 0);
	  HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
	  HAL_NVIC_SetPriority(I2Cx_EV_IRQn, 4, 0);
	  HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
  }
  else if (hi2c->Instance==I2C3)
  {

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /**I2C3 GPIO Configuration
	  PA8     ------> I2C3_SCL
	  PB4     ------> I2C3_SDA
	  */
	  GPIO_InitStruct.Pin 		= GPIO_PIN_8;
	  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	  GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin 		= GPIO_PIN_4;
	  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_OD;
	  GPIO_InitStruct.Pull 		= GPIO_PULLUP;
	  GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_VERY_HIGH;
	  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* Peripheral clock enable */
	  __HAL_RCC_I2C3_CLK_ENABLE();

	  /*##-4- Configure the NVIC for I2C #########################################*/
	  /* NVIC for I2C1 */
	  HAL_NVIC_SetPriority(I2C3_ER_IRQn, 1, 0);
	  HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
	  HAL_NVIC_SetPriority(I2C3_EV_IRQn, 2, 0);
	  HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
  }
}

/**
  * @brief I2C MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance==I2C1)
	{
		/*##-1- Reset peripherals ##################################################*/
		I2Cx_FORCE_RESET();
		I2Cx_RELEASE_RESET();

		/*##-2- Disable peripherals and GPIO Clocks ################################*/
		/* Configure I2C Tx as alternate function  */
		HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
		/* Configure I2C Rx as alternate function  */
		HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

		/*##-3- Disable the NVIC for I2C ###########################################*/
		HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
		HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
	}
	else if (hi2c->Instance==I2C3)
	{
		__HAL_RCC_I2C3_CLK_DISABLE();

		/**I2C3 GPIO Configuration
		PA8     ------> I2C3_SCL
		PB4     ------> I2C3_SDA
		*/
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

		HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
		HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
	}
}


