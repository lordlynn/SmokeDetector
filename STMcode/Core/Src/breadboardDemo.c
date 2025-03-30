/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "bme68x.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_COUNT  			UINT16_C(100)
#define BUS_TIMEOUT             1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
// Buffer
#define RxBuffSize 1
uint8_t RxBuff[RxBuffSize];

int buffIsFull = 0;
int buffIsEmpty = 1;

#define buffLen 1024
int buffHead = 0;
int buffTail = 0;

char buffer[buffLen];

int push(char data);
char pop();
void get(char arr[], int num);


// BT
void checkForBTConnect(void);
uint8_t BTSendReply = 0;


// SENSOR
char chBuffer[256];
uint8_t GTXBuffer[512], GRXBuffer[2048];
static uint8_t dev_addr;

void BME688ForceModeRead();
BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint32_t ReadNumbr, void *intf_ptr);
BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint32_t WriteNumbr, void *intf_ptr);
void DelayUs(uint32_t Delay);
void bme68x_delay_us(uint32_t period, void *intf_ptr);
void UART_Printf(uint8_t* buff, uint16_t size);
void PDEBUG(char *format, ...);


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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if (BTSendReply) {
//		  HAL_Delay(1500);
//		  HAL_UART_Transmit(&huart3, "AT+BINREQACK\r\n", 14, BUS_TIMEOUT);
//		  HAL_Delay(1500);
//		  HAL_UART_Transmit(&huart3, "BLE Connected", 13, BUS_TIMEOUT);
//		  BTSendReply = 0;
////		  BME688ForceModeRead();
//	  }
//	  else {
//		  checkForBTConnect();
//	  }




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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuff, RxBuffSize);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
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
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) RxBuff, RxBuffSize);
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART3) {
		if (RxBuff[0] != '\0')
			push(RxBuff[0]);

		/* start the DMA again */
		RxBuff[0] = '\0';
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) RxBuff, RxBuffSize);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}

	else if (huart->Instance == USART1) {
		if (RxBuff[0] != '\0')
			push(RxBuff[0]);

		/* start the DMA again */
		RxBuff[0] = '\0';
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuff, RxBuffSize);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}


int push(char data) {

    if (buffIsFull) {                                               // If the buffer is full
        return 0;
    }

    buffer[buffHead] = data;

    buffHead++;

    if (buffHead == buffLen) {                                          // Wrap back to 0 if buffer is at last index
        buffHead = 0;
    }


    if ((buffHead + 1) % buffLen == buffTail) {                             // Check if buffHead has reached buffTail
        buffIsFull = 1;
    }

    buffIsEmpty = 0;
    return 1;
}

void get(char arr[], int num) {
	int tempTail = buffTail;
	int tempbuffHead = buffHead;
	int tempBuffIsEmpty = buffIsEmpty;

	memset(arr, '\0', num);

	for (int i = 0; i < num; i++) {
		// 1.) Make sure buffTail is valid
		if (tempBuffIsEmpty) {
			return;
		}

		// 2.) Add byte to str
		arr[i] = buffer[tempTail];
		tempTail++;

		// 3.) check new buffTail position
		if (tempTail == buffLen) {
			tempTail = 0;
		}
		if (tempbuffHead == tempTail) {         							// If buffTail has reached buffHead
			tempBuffIsEmpty = 1;
		}
	}
}


char pop() {
	char data;
    int tempbuffHead = buffHead;                                       		// Save copy and use this in case buffHead changes


	// 1.) Make sure buffTail is valid
	if (buffIsEmpty) {
		return '\0';
	}

	// 2.) Add byte to str
	data = buffer[buffTail];
	buffTail++;

	// 3.) check new buffTail position
	if (buffTail == buffLen) {
		buffTail = 0;
	}
	if (tempbuffHead == buffTail) {         								// If buffTail has reached buffHead
		buffIsEmpty = 1;
	}

    return data;
}


void checkForBTConnect(void) {
	char trash;

	/* Check if a command was entered - read length of longest command*/
	char temp[10];
	get(temp, 10);

	// If the buffer does not have enough chars for a command yet
	if (strlen(temp) < 10) {
		return;
	}

	else if (strncmp(temp, "+CONNECTED", 10) == 0) {
		BTSendReply = 1;
		for (int i = 0; i < 10; i++) pop();
	}
	// If no good command was read, advance the tail by popping once
	else {
		trash = pop();
	}

}



/* SENSOR FUNCTIONS */
void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            PDEBUG("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            PDEBUG("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            PDEBUG("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            PDEBUG("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            PDEBUG("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            PDEBUG("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            PDEBUG("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
	int8_t rslt = BME68X_OK;

	if (bme != NULL)
	{
		//PDEBUG("I2C Interface\r\n");
		dev_addr = BME68X_I2C_ADDR_HIGH;
		bme->read = SensorAPI_I2Cx_Read;
		bme->write = SensorAPI_I2Cx_Write;
		bme->intf = BME68X_I2C_INTF;

		bme->delay_us = bme68x_delay_us;
		bme->intf_ptr = &dev_addr;
		bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
	}
	else
	{
		rslt = BME68X_E_NULL_PTR;
	}

    return rslt;
}

void BME688ForceModeRead()
{
	struct bme68x_dev bme;
	int8_t rslt;
	struct bme68x_conf conf;
	struct bme68x_heatr_conf heatr_conf;
	struct bme68x_data data;
	uint32_t del_period;
	uint32_t time_ms = 0;
	uint8_t n_fields;
	uint16_t sample_count = 1;


	/* Interface preference is updated as a parameter
	 * For I2C : BME68X_I2C_INTF
	 * For SPI : BME68X_SPI_INTF
	 */
	rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
	bme68x_check_rslt("bme68x_interface_init", rslt);

	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);

	rslt = bme68x_init(&bme);
	bme68x_check_rslt("bme68x_init", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	conf.filter = BME68X_FILTER_OFF;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_16X;
	conf.os_pres = BME68X_OS_1X;
	conf.os_temp = BME68X_OS_2X;
	rslt = bme68x_set_conf(&conf, &bme);
	bme68x_check_rslt("bme68x_set_conf", rslt);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = 300;
	heatr_conf.heatr_dur = 100;
	rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
	bme68x_check_rslt("bme68x_set_heatr_conf", rslt);


	while (sample_count <= SAMPLE_COUNT)
	{
		rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
		bme68x_check_rslt("bme68x_set_op_mode", rslt);

		/* Calculate delay period in microseconds */
		del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
		bme.delay_us(del_period, bme.intf_ptr);

		time_ms = HAL_GetTick();

		/* Check if rslt == BME68X_OK, report or handle if otherwise */
		rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
		bme68x_check_rslt("bme68x_get_data", rslt);

		if (n_fields)
		{

			sprintf(chBuffer, "TIME STAMP: %lu ms      TEMPERATURE: %.2f Deg C      PRESSURE: %.2f Pa      HUMIDITY: %.2f %%      GAS RESISTANCE: %.2f Ohm\r\n",
					(long unsigned int)time_ms,
					data.temperature,
					data.pressure,
					data.humidity,
					data.gas_resistance);

			HAL_UART_Transmit(&huart1, chBuffer, strlen(chBuffer), BUS_TIMEOUT);
			HAL_UART_Transmit(&huart3, chBuffer, strlen(chBuffer), BUS_TIMEOUT);


			sample_count++;
		}
	}
}

BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint32_t ReadNumbr, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	// send register address
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &subaddress, 1, BUS_TIMEOUT);
	HAL_I2C_Master_Receive(&hi2c1, DevAddress, pBuffer, ReadNumbr, BUS_TIMEOUT);
	return 0;
}

BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint32_t WriteNumbr, void *intf_ptr)
{
	uint8_t dev_addr = *(uint8_t*)intf_ptr;
	uint16_t DevAddress = dev_addr << 1;

	GTXBuffer[0] = subaddress;
	memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

	// send register address
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, GTXBuffer, WriteNumbr+1, BUS_TIMEOUT);
	return 0;
}

void DelayUs(uint32_t Delay)
{
	uint32_t i;

	while(Delay--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}

void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
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
