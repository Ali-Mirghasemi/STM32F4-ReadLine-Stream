/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "InputStream.h"
#include "OutputStream.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	UART_HandleTypeDef* 		HUART;
	DMA_HandleTypeDef*			RxDMA;
	DMA_HandleTypeDef*			TxDMA;
	IStream									Input;
	OStream									Output;
} UARTStream;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const char CRLF[2] = "\r\n";

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static uint8_t	streamRxBuff[10];
static uint8_t	streamTxBuff[10];

static UARTStream stream;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UARTStream_stopReceiveDMA(UARTStream* stream);
void UARTStream_stopTransmitDMA(UARTStream* stream);
Stream_LenType UARTStream_checkReceivedBytes(IStream* stream);
void UARTStream_receive(IStream* stream, uint8_t* buff, Stream_LenType len);
void UARTStream_transmit(OStream* stream, uint8_t* buff, Stream_LenType len);
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
	char str[32];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	// init uart stream
	stream.HUART = &huart1;
	stream.RxDMA = &hdma_usart1_rx;
	stream.TxDMA = &hdma_usart1_tx;
  IStream_init(&stream.Input, UARTStream_receive, streamRxBuff, sizeof(streamRxBuff));
	IStream_setCheckReceive(&stream.Input, UARTStream_checkReceivedBytes);
	IStream_setArgs(&stream.Input, &stream);
	OStream_init(&stream.Output, UARTStream_transmit, streamTxBuff, sizeof(streamTxBuff));
	OStream_setArgs(&stream.Output, &stream);
	// start receive
	IStream_receive(&stream.Input);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (IStream_available(&stream.Input) > 0) {
			// Way 1
			Stream_LenType len = IStream_readBytesUntilPattern(&stream.Input, (uint8_t*) CRLF, sizeof(CRLF), (uint8_t*) str, sizeof(str));
			if (len > 0) {
				OStream_writeBytes(&stream.Output, (uint8_t*) str, len);
				OStream_flush(&stream.Output);
			}
			
			// Way 2
			/*Stream_LenType lineLen = IStream_findPattern(&stream.Input, (uint8_t*) CRLF, sizeof(CRLF));
			if (lineLen > 0) {
				OStream_writeStream(&stream.Output, &stream.Input, lineLen + sizeof(CRLF));
				OStream_flush(&stream.Output);
			}*/
		}
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void UARTStream_stopReceiveDMA(UARTStream* stream) {
	UART_HandleTypeDef* huart = stream->HUART;
	uint32_t dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
  if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx stream */
    if (huart->hdmarx != NULL)
    {
      HAL_DMA_Abort(huart->hdmarx);
    }
    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
		CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
		CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* At end of Rx process, restore huart->RxState to Ready */
		huart->RxState = HAL_UART_STATE_READY;
  }
}
void UARTStream_stopTransmitDMA(UARTStream* stream) {
	UART_HandleTypeDef* huart = stream->HUART;
	uint32_t dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT);
  if ((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    /* Abort the UART DMA Tx stream */
    if (huart->hdmatx != NULL)
    {
      HAL_DMA_Abort(huart->hdmatx);
    }
    /* Disable TXEIE and TCIE interrupts */
		CLEAR_BIT(huart->Instance->CR1, (USART_CR1_TXEIE | USART_CR1_TCIE));

		/* At end of Tx process, restore huart->gState to Ready */
		huart->gState = HAL_UART_STATE_READY;
  }
}
Stream_LenType UARTStream_checkReceivedBytes(IStream* stream) {
	UARTStream* uartStream = (UARTStream*) IStream_getArgs(stream);
	return IStream_incomingBytes(stream) - uartStream->RxDMA->Instance->NDTR;
}
void UARTStream_receive(IStream* stream, uint8_t* buff, Stream_LenType len) {
	UARTStream* uartStream = (UARTStream*) IStream_getArgs(stream);
	UARTStream_stopReceiveDMA(uartStream);
	HAL_UART_Receive_DMA(uartStream->HUART, buff, len);
}
void UARTStream_transmit(OStream* stream, uint8_t* buff, Stream_LenType len) {
	UARTStream* uartStream = (UARTStream*) OStream_getArgs(stream);
	UARTStream_stopTransmitDMA(uartStream);
	HAL_UART_Transmit_DMA(uartStream->HUART, buff, len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	switch ((uint32_t) huart->Instance) {
		case USART1_BASE:
			OStream_handle(&stream.Output, OStream_outgoingBytes(&stream.Output));
			break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	switch ((uint32_t) huart->Instance) {
		case USART1_BASE:
			IStream_handle(&stream.Input, IStream_incomingBytes(&stream.Input));
			break;
	}
}

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
