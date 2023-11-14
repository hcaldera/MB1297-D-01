/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#if 0
/* Debug Exception and Monitor Control Register base address */
#define DEMCR                 *((volatile uint32_t *)0xE000EDFCU)
/* ITM register addresses */
#define ITM_STIMULUS_PORT0    *((volatile uint32_t *)0xE0000000)
#define ITM_TRACE_EN          *((volatile uint32_t *)0xE0000E00)
#endif

#define USART_INC_IDX(i, val, max) ((i) + (val)) & ((max) - 1)

#define UART4_TX_BUFFER_SIZE    (1 << 9)  /* 512  */
#define UART4_RX_BUFFER_SIZE    (1 << 9)  /* 512  */
#define UART4_RX_IT_BUFFER_SIZE (1 << 6)  /* 64   */

#if (UART4_RX_BUFFER_SIZE) < (UART4_RX_IT_BUFFER_SIZE)
#error "UART4_RX_BUFFER_SIZE can't be smaller than UART4_RX_IT_BUFFER_SIZE"
#endif

static int8_t  *uart4_tx_buffer = (int8_t *)(SRAM2_BASE + 0x800U);
static int16_t uart4_tx_in = 0;

static int8_t  *uart4_rx_buffer = (int8_t *)(SRAM2_BASE + 0x800U + UART4_TX_BUFFER_SIZE);
static int8_t  uart4_rx_it_buffer[UART4_RX_IT_BUFFER_SIZE];
static int16_t uart4_rx_in = 0;
static int16_t uart4_rx_out = 0;
static bool uart4_rx_received = false;

/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_TXINVERT_INIT|UART_ADVFEATURE_RXINVERT_INIT;
  huart4.AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
  huart4.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t *)uart4_rx_it_buffer, UART4_RX_IT_BUFFER_SIZE);

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = ST_LINK_UART1_TX_Pin|ST_LINK_UART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

    if (HAL_UART_STATE_READY != uartHandle->gState) {
      HAL_UART_AbortReceive_IT(&huart4);
      HAL_UART_AbortTransmit_IT(&huart4);
    }

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, ARD_D1_Pin|ARD_D0_Pin);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, ST_LINK_UART1_TX_Pin|ST_LINK_UART1_RX_Pin);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#if 0
/* Implementation of printf like feature using ARM Cortex M3/M4/ ITM functionality
 * This function will not work for ARM Cortex M0/M0+
 * If you are using Cortex M0, then you can use semihosting feature of openOCD
 */
void ITM_SendChar(uint8_t ch)
{
  /* Enable TRCENA */
  DEMCR |= ( 1 << 24);

  /* Enable stimulus port 0 */
  ITM_TRACE_EN |= ( 1 << 0);

  /* Read FIFO status in bit [0]: */
  while(!(ITM_STIMULUS_PORT0 & 1));

  /* Write to ITM stimulus port0 */
  ITM_STIMULUS_PORT0 = ch;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  int dataIdx;

  for (dataIdx = 0; dataIdx < len; dataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
#endif

int16_t MX_UART4_Read(int8_t * ptrBfr)
{
  int16_t len = 0;

  if (uart4_rx_received) {
    if (uart4_rx_in > uart4_rx_out) {
      len = uart4_rx_in - uart4_rx_out;
      memcpy(ptrBfr, &uart4_rx_buffer[uart4_rx_out], len);
    } else {
      len = UART4_RX_BUFFER_SIZE - (uart4_rx_out - uart4_rx_in);
      memcpy(&ptrBfr[0], &uart4_rx_buffer[uart4_rx_out], UART4_RX_BUFFER_SIZE - uart4_rx_out);
      memcpy(&ptrBfr[UART4_RX_BUFFER_SIZE - uart4_rx_out], &uart4_rx_buffer[0], uart4_rx_in);
    }
    uart4_rx_out = uart4_rx_in;
    uart4_rx_received = false;
  }

  return len;
}

bool MX_UART4_Write(const int8_t *ptrBfr, size_t len)
{
  bool ret = false;
  if ((len <= UART4_TX_BUFFER_SIZE) && (len > 0)) {
    if ((uart4_tx_in + len) > UART4_TX_BUFFER_SIZE) {
      uart4_tx_in = 0;
    }
    memcpy(&uart4_tx_buffer[uart4_tx_in], ptrBfr, len);
    ret = (HAL_OK == HAL_UART_Transmit_IT(&huart4, (uint8_t *)&uart4_tx_buffer[uart4_tx_in], len));
    uart4_tx_in = USART_INC_IDX(uart4_tx_in, len, UART4_TX_BUFFER_SIZE);
  }

  return ret;
}

/**
  * @brief  Reception Event Callback (Rx event notification called after use of advanced reception service).
  * @param  huart UART handle
  * @param  Size  Number of data available in application reception buffer (indicates a position in
  *               reception buffer until which, data are available)
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (UART4 == huart->Instance) {
    if ((Size > 0) && (Size <= UART4_RX_IT_BUFFER_SIZE)) {
      if ((uart4_rx_in + Size) <= UART4_RX_BUFFER_SIZE) {
        memcpy(&uart4_rx_buffer[uart4_rx_in], &uart4_rx_it_buffer[0], Size);
      } else {
        memcpy(&uart4_rx_buffer[uart4_rx_in],
               &uart4_rx_it_buffer[0],
               UART4_RX_BUFFER_SIZE - uart4_rx_in);
        memcpy(&uart4_rx_buffer[0],
               &uart4_rx_it_buffer[UART4_RX_BUFFER_SIZE - uart4_rx_in],
               USART_INC_IDX(uart4_rx_in, Size, UART4_RX_BUFFER_SIZE));
      }
      uart4_rx_in = USART_INC_IDX(uart4_rx_in, Size, UART4_RX_BUFFER_SIZE);
      uart4_rx_received = true;
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, (uint8_t *)uart4_rx_it_buffer, UART4_RX_IT_BUFFER_SIZE);
  }
}

/* USER CODE END 1 */
