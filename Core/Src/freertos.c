/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart4_write_timer */
osTimerId_t uart4_write_timerHandle;
const osTimerAttr_t uart4_write_timer_attributes = {
  .name = "uart4_write_timer"
};
/* Definitions for uart4_read_timer */
osTimerId_t uart4_read_timerHandle;
const osTimerAttr_t uart4_read_timer_attributes = {
  .name = "uart4_read_timer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void uart4_write_callback(void *argument);
void uart4_read_callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of uart4_write_timer */
  uart4_write_timerHandle = osTimerNew(uart4_write_callback, osTimerPeriodic, NULL, &uart4_write_timer_attributes);

  /* creation of uart4_read_timer */
  uart4_read_timerHandle = osTimerNew(uart4_read_callback, osTimerPeriodic, NULL, &uart4_read_timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */

  /* start timers, add new ones, ... */
  osTimerStart(uart4_write_timerHandle, 5000);
  osTimerStart(uart4_read_timerHandle, 100);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  MX_UART4_Write((const int8_t *)"Hello World!\n", strlen("Hello World!\n"));

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* uart4_write_callback function */
void uart4_write_callback(void *argument)
{
  /* USER CODE BEGIN uart4_write_callback */

  MX_UART4_Write((const int8_t *)"\nFive seconds elapsed...\n", strlen("\nFive seconds elapsed...\n"));

  /* USER CODE END uart4_write_callback */
}

/* uart4_read_callback function */
void uart4_read_callback(void *argument)
{
  /* USER CODE BEGIN uart4_read_callback */

  int8_t main_buffer[256];

  (void)MX_UART4_Write(main_buffer, MX_UART4_Read(main_buffer));

  /* USER CODE END uart4_read_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

