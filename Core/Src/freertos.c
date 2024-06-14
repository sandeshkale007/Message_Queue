/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
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

unsigned char rxBuffer[4];



char buffer[18];

uint8_t flag = 0;

uint8_t lower_limit[] = "Below The Lower Limit\r\n";
uint8_t message1[] = "Reading Error\r\n";
uint8_t queue_fail[] = "Failed to Receive\r\n";

/* USER CODE END Variables */

/* Definitions for defaultTask */
TaskHandle_t receive_task, create_task, send_task;
QueueHandle_t distance_queue;
/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */
void receive_task_callback(void *argument);
void send_task_callback(void *argument);
void create_task_callback(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	HAL_UART_Transmit(&huart2, (uint8_t*) "RTOS Function\r\n",
			sizeof("RTOS Function\r\n"), HAL_MAX_DELAY);
	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */

	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	distance_queue = xQueueCreate(10, sizeof(uint16_t));
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	BaseType_t status;
//	status = xTaskCreate(receive_task_callback, "Task receive", 200, NULL, 31,&receive_task);
	status = xTaskCreate(create_task_callback, "Task receive", 200, NULL, 31,
			&create_task);
//	status = xTaskCreate(receive_task_callback, "Task receive", 200, NULL, 31,&send_task);
	if (status != pdPASS) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Fail to Create Task\r\n",
				sizeof("Fail to Create Task\r\n"), HAL_MAX_DELAY);
	}
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void receive_task_callback(void *argument) {
	uint16_t distance;
	for (;;) {
		if (xQueueReceive(distance_queue, &distance, portMAX_DELAY) == pdPASS) {
			snprintf(buffer, sizeof(buffer), "Distance: %u\n", distance);
			HAL_UART_Transmit(&huart2, (uint8_t*) buffer, sizeof(buffer),
			HAL_MAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart2, (uint8_t*) queue_fail,
					sizeof(queue_fail), HAL_MAX_DELAY);
		}
	}
}
void create_task_callback(void *argument) {
	BaseType_t status;
	status = xTaskCreate(receive_task_callback, "Task receive", 200, NULL, 31,
			&receive_task);
	if (status != pdPASS) {
		HAL_UART_Transmit(&huart2, (uint8_t*) "Fail to Create Sub Task\r\n",
				sizeof("Fail to Create Sub Task\r\n"), HAL_MAX_DELAY);
	}
	for (;;) {
		status = xTaskCreate(receive_task_callback, "Task receive", 200, NULL,
				31, &send_task);
		if (status != pdPASS) {
			HAL_UART_Transmit(&huart2, (uint8_t*) "Fail to Create Sub Task\r\n",
					sizeof("Fail to Create Sub Task\r\n"), HAL_MAX_DELAY);
		}
	}
}

void send_task_callback(void *argument) {
	// Infinite loop for the task
	uint16_t distance;
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	for (int i = 0; i < 5; i++) {
		HAL_UART_Receive_DMA(&huart6, rxBuffer, 4);
		vTaskDelay(pdMS_TO_TICKS(10));
		HAL_UART_Receive_DMA(&huart6, rxBuffer, 4);
		if (rxBuffer[0] == 0xff) {
			distance = rxBuffer[1] * 256 + rxBuffer[2];
			if (distance == 250 || distance == 0) {
				flag = 3;
			} else {
				flag = 1;
			}
		} else {
			flag = 2;
		}
		if (flag == 3) {
			HAL_UART_Transmit(&huart2, lower_limit, sizeof(lower_limit), HAL_MAX_DELAY);
		} else if (flag == 1) {
			xQueueSend(distance_queue,distance,portMAX_DELAY);
		} else {
			HAL_UART_Transmit(&huart2, message1, sizeof(message1),
			HAL_MAX_DELAY);
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(10));
	vTaskDelete(receive_task);
}

/* USER CODE END Application */

