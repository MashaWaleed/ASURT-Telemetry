/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

///////////////////////////////////
///   BUILD CHOICE DIRECTIVE  ////
/////////////////////////////////


// CHOOSE IF YOU WANT TO BUILD TRANSMITTER
#define TRANSMITTER

// CHOOSE IF YOU WANT TO BUILD RECEIVER
//#define RECEIVER


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



// SENT AND RECEIVED DATA WRAPPER 
// THESE ARE THE PACKETS SENT BY NRF

struct RX_Message
{
    uint16_t ID; 					 	/**--> CAN MSG ID  **/
		uint8_t Data[8];				/**--> CAN MSG DATA  **/
}RX_Message_S;

struct TX_Message
{
    uint16_t ID;
		uint8_t Data[8];
}TX_Message_S;


// YOUR CHANNEL ADDRESS OF CHOICE
// CHANGE FROM RESET VALUE TO AVOID 

uint8_t comm_address[5] = {0x14,0x14,0x14,0x14,0x14};
uint8_t comm_address_t[5] = {0x14,0x14,0x14,0x14,0x14};

uint8_t pl = 0;

// CAN STUFF, YOU CAN SEE MY FILTER CONFIGS IN CAN.C 

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// CAN STUFF END

uint8_t PL_Size;
uint64_t RxNRF;

uint8_t ACKrx[1];
uint8_t ACKtx1[1] = {'1'};
uint8_t ACKtx2[1] = {'2'};

#ifdef twoWay
#ifdef RECEIVER
uint8_t received = 0;

#define TwoWayBufSize 1
uint8_t TwoWayBuf[TwoWayBufSize];
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size){
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, TwoWayBuf,TwoWayBufSize);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	received = 1;
}
#endif
#endif

// THIS IS ONLY NECESSRAY IF I AM TRANSMITTING IN CAN OR TESTING IN LOOP BACK
/*
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];
*/


#ifdef TRANSMITTER
TaskHandle_t xTX_NRF_TaskHandle = NULL;
TaskHandle_t xTX_CAN_RX_TaskHandle = NULL;
TaskHandle_t xTX_NRF_Init_TaskHandle = NULL;
TaskHandle_t xTX_NRF_State_TaskHandle = NULL;
QueueHandle_t xCAN_QueueHandle;

NRF_Config_t tx_cfg;
#endif
	
#ifdef RECEIVER
TaskHandle_t xRX_NRF_TaskHandle = NULL;
TaskHandle_t xRX_NRF_Init_TaskHandle = NULL;
TaskHandle_t xRX_NRF_State_TaskHandle = NULL;

NRF_Config_t rx_cfg;
#endif
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef TRANSMITTER
void vTX_NRF_TaskHandler( void * pvParameters );
void vTX_CAN_RX_TaskHandler( void * pvParameters );
void vTX_NRF_Init_TaskHandler( void * pvParameters );
void vTX_NRF_State_TaskHandler( void * pvParameters );
#endif

#ifdef RECEIVER
void vRX_NRF_TaskHandler( void * pvParameters );
void vRX_NRF_Init_TaskHandler( void * pvParameters );
void vRX_NRF_State_TaskHandler( void * pvParameters );
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_CAN_Start(&hcan);
	
	// THIS IS ONLY NECESSRAY IF I AM TRANSMITTING IN CAN OR TESTING IN LOOP BACK
	
	/*
	TxHeader.DLC = 1;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x103;
	TxHeader.TransmitGlobalTime = DISABLE;
	*/
		
	/***********< Task Creation **********/
	#ifdef TRANSMITTER

	xTaskCreate(
              vTX_NRF_TaskHandler,
              "NRF Transmit",          
              128,             
              NULL,            
              1,               
              &xTX_NRF_TaskHandle );
							
	xTaskCreate(
              vTX_NRF_Init_TaskHandler,
              "NRF tx init",          
              configMINIMAL_STACK_SIZE,             
              NULL,            
              configMAX_PRIORITIES - 1 ,               
              &xTX_NRF_Init_TaskHandle );
	xTaskCreate(
              vTX_CAN_RX_TaskHandler,
              "CAN Receive",          
              128,             
              NULL,            
              1,               
              &xTX_CAN_RX_TaskHandle );
	xTaskCreate(
              vTX_NRF_State_TaskHandler,
              "CAN Receive",          
              128,             
              NULL,            
              1,               
              &xTX_NRF_State_TaskHandle );
							
	xCAN_QueueHandle = xQueueCreate(15, sizeof(&TX_Message_S));
	#endif
	#ifdef RECEIVER
	xTaskCreate(
              vRX_NRF_TaskHandler,
              "NRF Receive",          
              256,             
              NULL,            
              1,               
              &xRX_NRF_TaskHandle );
	xTaskCreate(
              vRX_NRF_Init_TaskHandler,
              "NRF rx init",          
              configMINIMAL_STACK_SIZE,             
              NULL,            
              configMAX_PRIORITIES - 1,               
              &xRX_NRF_Init_TaskHandle );
//	xTaskCreate(
//              vRX_NRF_State_TaskHandler,
//              "NRF rx state",          
//              128,             
//              NULL,            
//              1,               
//              &xRX_NRF_State_TaskHandle );
	#endif
	
	vTaskStartScheduler();
	/***********<End of Task Creation **********/
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

#ifdef TRANSMITTER
void vTX_NRF_TaskHandler( void * pvParameters )
{
		struct TX_Message * RX_Local;
    for( ;; )
    {
			if( xQueueReceive( xCAN_QueueHandle,&( RX_Local ),( TickType_t ) 0 ) == pdPASS )
			{
				
				if(ECUAL_NRF_TX_Transmit((uint8_t *)(RX_Local),10))
				{
					
					#ifdef twoWay
					// TWO WAY COMMUNICATION TESTING 	
					//ECUAL_NRF_flush_rx_fifo();
					pl = ECUAL_NRF_RX_Receive_Single(ACKrx);
					if(pl)
					{
						if(ACKrx[0] == '1')
						{
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
						}
						else
						{
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
						}
							
					}
					#endif
					
					ECUAL_NRF_tx_irq();
					
				}
				
				vTaskDelay(1);
				
			}
    }
}

void vTX_NRF_Init_TaskHandler( void * pvParameters )
{
		//ONE TIME INIT TASK
		// MY CUBE IDE STYLE NRF CONFIGURATIONS
		
		tx_cfg.NRF_Power = NRF_PWR_0dBm;
		tx_cfg.NRF_Rate = NRF_DATA_RATE_1MBPS;
		tx_cfg.NRF_Channel = 100;
		tx_cfg.NRF_Payload_Length = 10;
		tx_cfg.NRF_Addr_Width = NRF_ADDR_WIDTH_5;
		tx_cfg.NRF_Address = comm_address;
		tx_cfg.NRF_Re_Count = 5;
		tx_cfg.NRF_Re_Delay = 5;
	
		ECUAL_NRF_Tx_Init(&tx_cfg);
		vTaskDelete(NULL);
    for( ;; )
    {
    }
}

void vTX_NRF_State_TaskHandler( void * pvParameters )
{
    for( ;; )
    {
			if((ECUAL_NRF_chech_pwr()) && (ECUAL_NRF_chech_tx()) )
			{
				// do nothing, business as usual
			}
			else
			{
				//There was noise power/signal and now i have to restart the NRF to unlock communication
				ECUAL_NRF_Tx_Init(&tx_cfg);
			}
			
			vTaskDelay(1000);
    }
}

void vTX_CAN_RX_TaskHandler( void * pvParameters )
{
		struct TX_Message *TX_Local;
	
		struct TX_Message t1;
		struct TX_Message t2;
	
		uint64_t a = 0x00000000;
		uint64_t b = 0x00000000;
	
		t1.ID = 0x103;
		t2.ID = 0x100;
	
		memcpy(t1.Data, &a, 8);
		memcpy(t2.Data, &b, 8);
		
		int i =0;
	
    for( ;; )
    {
			/** REAL CAN FOR TESTING **/
		
			if(HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,RxData) == HAL_OK)
			{
				if(((RxHeader.StdId >> 5) > 0x19 )&&((RxHeader.StdId >> 5) < 0x25 )) // Controller messages
				{
					memcpy(TX_Message_S.Data, RxData,8);
					//TX_Message_S.Data = RxData;
					TX_Message_S.ID = RxHeader.StdId >> 5;
					TX_Local = &TX_Message_S;
				}
				else
				{
					memcpy(TX_Message_S.Data, RxData,8);
					//TX_Message_S.Data = RxData;
					TX_Message_S.ID = RxHeader.StdId;
					TX_Local = &TX_Message_S;
				}
				xQueueSendToBack(xCAN_QueueHandle, (void *) &TX_Local, (TickType_t ) 0);
				vTaskDelay(5);
			}


			/** PSEUDO CAN FOR DEBUGGING NRF **/
//			i++;
//			if(i ==1)
//			{
//				a++;
//				if(a > 0xFFFFFFFE)
//				{
//					a = 0x00000000;
//				}
//				memcpy(t1.Data, &a, 8);
//				TX_Message_S.ID = t1.ID;
//				memcpy(TX_Message_S.Data,t1.Data,8);
//				TX_Local = &TX_Message_S;
//				xQueueSendToBack(xCAN_QueueHandle, (void *) &TX_Local, (TickType_t ) 0);
//			}
//			
//			else if(i == 2)
//			{
//				b++;
//				if(b > 0xFFFFFFFE)
//				{
//					b = 0x00000000;
//				}
//				memcpy(t2.Data, &b, 8);
//				i = 0;
//				TX_Message_S.ID = t2.ID;
//				memcpy(TX_Message_S.Data,t2.Data,8);
//				TX_Local = &TX_Message_S;
//				xQueueSendToBack(xCAN_QueueHandle, (void *) &TX_Local, (TickType_t ) 0);
//				
//			}
//			vTaskDelay(1);
    }
}

#endif

#ifdef RECEIVER
void vRX_NRF_TaskHandler( void * pvParameters )
{
		// FORMAT OF SERIAL MESSAGE 
		// $		  										--->  						MSG ID  							--->  MSG DATA
		// 1 letter										--->						1 to 3 digits						--->  double with 3 decimal point precision
		// uinque, avoid noise, start --->						ID linked to DB         --->  max precision expandable data 
uint8_t i = 0;
    for( ;; )
    {
			PL_Size = ECUAL_NRF_RX_Receive_Single((uint8_t *)(&RX_Message_S));
			if(PL_Size)
			{
				
				#ifdef twoWay
				// 2 way test
//				i++;
//				if(i == 1)
//				{
//					ECUAL_NRF_Wrtie_ACK(1,ACKtx1, 1 	);
//				}
//				else if (i == 2)
//				{
//					ECUAL_NRF_Wrtie_ACK(1,ACKtx2, 1);
//					i = 0;
//				}
				if(received)
				{
					received  = 0;
					ECUAL_NRF_Wrtie_ACK(1,TwoWayBuf, 1 );
				}
				#endif
				
			uint8_t str[30] = {0};
			if((RX_Message_S.ID > 0x19 )&&(RX_Message_S.ID < 0x25 )) 	// Controller messages
			{
				memcpy(&RxNRF, RX_Message_S.Data, 8);
				sprintf(str,"$%d,%"PRIu64, RX_Message_S.ID,RxNRF);				
				HAL_UART_Transmit(&huart1, str,30,100);
			}
			else // LV system
			{
				memcpy(&RxNRF, RX_Message_S.Data, 8);
				sprintf(str,"$%d,%"PRIu64, RX_Message_S.ID,RxNRF);		
				HAL_UART_Transmit(&huart1, str,30,100);
				
				
				
			}
			
			HAL_UART_Transmit(&huart1, (uint8_t *)"\n\r", strlen("\n\r"), 100);
			vTaskDelay(1);
			}
    }
}

void vRX_NRF_Init_TaskHandler( void * pvParameters )
{
		// ONE TIME INIT TASK
		// MY CUBE IDE STYLE NRF CONFIGURATIONS
	
		rx_cfg.NRF_Power = NRF_PWR_0dBm;
		rx_cfg.NRF_Rate = NRF_DATA_RATE_1MBPS;
		rx_cfg.NRF_Channel = 100;
		rx_cfg.NRF_Payload_Length = 10;
		rx_cfg.NRF_Addr_Width = NRF_ADDR_WIDTH_5;
		rx_cfg.NRF_Address = comm_address;
		rx_cfg.NRF_Re_Count = 5;
		rx_cfg.NRF_Re_Delay = 5;
		
		ECUAL_NRF_Rx_Init(&rx_cfg);
	#ifdef twoWay
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, TwoWayBuf,TwoWayBufSize);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	#endif
		vTaskDelete(NULL);
    for( ;; )
    {
    }
}

void vRX_NRF_State_TaskHandler( void * pvParameters )
{
    for( ;; )
    {
			if(ECUAL_NRF_chech_pwr())
			{
				// do nothing, business as ususal
				ECUAL_NRF_flush_rx_fifo();
			}
			else
			{
				//reinit nrf, power/signal potentially lost briefly, resetting module and hence locked
				//this is crucial and new from last year, preventing communication lock
				ECUAL_NRF_Rx_Init(&rx_cfg);
				ECUAL_NRF_flush_rx_fifo();
			}
			
			vTaskDelay(1000);
    }
}

// SCREW KEIL IDE I NEVER WANT TO WORK WITH IT ANOTHER DAY IN MY LIFE :DDD
#endif



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
