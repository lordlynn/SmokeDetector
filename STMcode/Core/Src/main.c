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
#include "stuff.h"
#include "string.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "bme68x.h"


#include <sys/time.h>
#include<time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int SAMPLE_COUNT = UINT16_C(30);
#define BUS_TIMEOUT             1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

// BT Buffer
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



// ADC Buffers
#define ADC_BUF_LEN 1
uint16_t adc_buf1[ADC_BUF_LEN];
uint16_t adc_buf2[ADC_BUF_LEN];
double batteryVoltage = 0.0;
double SPVoltage = 0.0;
void checkADCReading();



// SENSOR
#define HIGH_GAS 175000.0 //~21 C - 145000.0  | ~24 - 185000 |  ~29 C - 3000000.0
double avgTemp = 0;
double avgGas = 0;
char chBuffer[256];
uint8_t GTXBuffer[512], GRXBuffer[2048];
static uint8_t dev_addr;
void checkGasReading();

void BME688ForceModeRead();
BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Read(uint8_t subaddress, uint8_t *pBuffer, uint32_t ReadNumbr, void *intf_ptr);
BME68X_INTF_RET_TYPE SensorAPI_I2Cx_Write(uint8_t subaddress, uint8_t *pBuffer, uint32_t WriteNumbr, void *intf_ptr);
void DelayUs(uint32_t Delay);
void bme68x_delay_us(uint32_t period, void *intf_ptr);

// PWM
void setDC(TIM_HandleTypeDef *handle, uint8_t CH, uint16_t dc);
void setPeriod(TIM_HandleTypeDef *handle, uint8_t CH, uint16_t period);

// Flash Memory
void flashStateMachine();
void parseCommand();
void parseStoreCommand();
void parseINodeNumber();
enum states state = idle;
uint8_t INodeNumber = 0;
char filename[NAME_LIM];
uint8_t fileType = 255;
uint8_t endOfFile = 0;
uint32_t fileLen = 0;
uint8_t fileIn[1024*4];		// Dynamic memory would be better but kept breaking spi stuff???
struct INode node;
uint8_t *fileReadFromMem;
uint8_t didFileWrite = 0;
uint8_t UARTFlag = 1;
uint8_t lastSaveAlertFlag = 1;
uint8_t saveAlertFlag = 1;
uint8_t batAlertFlag = 0;
uint8_t lastBatAlertFlag = 0;
uint8_t lastSPAlertFlag = 0;
uint8_t SPAlertFlag = 0;


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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Set LED period and freq
  setPeriod(&htim2, TIM_CHANNEL_1, 65535);
  setPeriod(&htim2, TIM_CHANNEL_3, 65535);
  setPeriod(&htim2, TIM_CHANNEL_4, 65535);

  setDC(&htim2, TIM_CHANNEL_1, 20000);
  setDC(&htim2, TIM_CHANNEL_3, 20000);
  setDC(&htim2, TIM_CHANNEL_4, 20000);


  // Turn on blue LED during initial readings
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // Take 30 readings from the sensor before starting to allow system to stabilize
  BME688ForceModeRead();
  SAMPLE_COUNT = UINT16_C(20);

  // Set buzzer period and frequency
  setDC(&htim1, TIM_CHANNEL_1, 50000);
  setPeriod(&htim1, TIM_CHANNEL_1, 65535);


  // Setup ADCs
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_buf1,ADC_BUF_LEN);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&adc_buf2,ADC_BUF_LEN);


  // Turn off blue LED after init
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
  HAL_Delay(25);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Read ADC values
	  batteryVoltage = (adc_buf1[0] / 4095.0) * 3.3 * 2.165; // 2.165 found experimentally by measuring the drop across voltage divider
	  SPVoltage = ((adc_buf2[0] - 2048) / 2048.0) * 3.3;
	  checkADCReading();


	  // Read gas sensor. Check gas readings and file system calls within this function to improve response time
	  BME688ForceModeRead();

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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32767;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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
  HAL_GPIO_WritePin(CS_PIN_GPIO_Port, CS_PIN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_PIN_Pin */
  GPIO_InitStruct.Pin = CS_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_PIN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART3) {
		if (RxBuff[0] != '\0') {
			push(RxBuff[0]);
			UARTFlag = 0;
		}
		/* start the DMA again */
		RxBuff[0] = '\0';
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) RxBuff, RxBuffSize);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}

	else if (huart->Instance == USART1) {
		if (RxBuff[0] != '\0') {
			push(RxBuff[0]);
			UARTFlag = 1;
		}
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




/* SENSOR FUNCTIONS */
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

	avgGas = 0;
	avgTemp = 0;

	/* Interface preference is updated as a parameter
	 * For I2C : BME68X_I2C_INTF
	 * For SPI : BME68X_SPI_INTF
	 */
	rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);

	bme68x_soft_reset(&bme);
	bme.delay_us(10, bme.intf_ptr);

	rslt = bme68x_init(&bme);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	conf.filter = BME68X_FILTER_OFF;
	conf.odr = BME68X_ODR_NONE;
	conf.os_hum = BME68X_OS_16X;
	conf.os_pres = BME68X_OS_1X;
	conf.os_temp = BME68X_OS_2X;
	rslt = bme68x_set_conf(&conf, &bme);

	/* Check if rslt == BME68X_OK, report or handle if otherwise */
	heatr_conf.enable = BME68X_ENABLE;
	heatr_conf.heatr_temp = 300;
	heatr_conf.heatr_dur = 100;
	rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

	while (sample_count <= SAMPLE_COUNT)
	{
		rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);

		/* Calculate delay period in microseconds */
		del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
		bme.delay_us(del_period, bme.intf_ptr);

		/* Check if rslt == BME68X_OK, report or handle if otherwise */
		rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);

		if (n_fields)
		{
			avgTemp += data.temperature;
			avgGas += data.gas_resistance;

			sample_count++;
		}

		// If we are past the initial sampling check the gas level
		if (SAMPLE_COUNT != 30 && sample_count % 3 == 0) {
			// First sensor readings are always bad. needs time to heat up so ingore them
			if (sample_count == 3) {
				avgTemp = 0.0;
				avgGas = 0.0;
				continue;
			}


			sprintf(chBuffer, "TEMPERATURE: %.2f Deg C\t\tGAS RESISTANCE: %.2f Ohm\r\n",
							  data.temperature,
							  data.gas_resistance);
			HAL_UART_Transmit(&huart3, chBuffer, strlen(chBuffer), BUS_TIMEOUT);


			avgTemp = avgTemp / 3.0;
			avgGas = avgGas / 3.0;

			checkGasReading();

			avgTemp = 0;
			avgGas = 0;
		}
//		if (sample_count % 2 == 0) {
		flashStateMachine();
//		}
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

void checkGasReading() {
	static uint8_t hysteresis = 0;

	// Check gas reading and ADC values
	if (avgGas <= HIGH_GAS) {
	  lastSaveAlertFlag = saveAlertFlag;
	  saveAlertFlag += 1;

	  // Save a single alert file for each instance of a high gas level
	  if (lastSaveAlertFlag == 1 && saveAlertFlag == 2) {

		  // Turn on Buzzer
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

		  // Turn on red LED
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

		  // Turn off green LED
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

		  HAL_UART_Transmit(&huart1, "High Gas Level Detected!\n", 25, BUS_TIMEOUT);
		  HAL_UART_Transmit(&huart3, "High Gas Level Detected!\n", 25, BUS_TIMEOUT);


		  time_t t = HAL_GetTick() / 1000;

		  struct tm tm = *localtime(&t);

		  sprintf(chBuffer, "%d-%d-%d %d:%d:%d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
												 tm.tm_hour, tm.tm_min, tm.tm_sec);

		  sprintf(fileIn, "High gas level detected at %s\nTemperature: %.2lf\nGas reistance: %.2lf\n", chBuffer, avgTemp, avgGas);
		  fileLen = strlen(fileIn);
		  sprintf(filename, "Alert_%s", chBuffer);

		  didFileWrite = writeFile(fileIn, fileLen, filename, 1);

		  // If the file failed to write try again
		  if (didFileWrite == 0) {
			  writeFile(fileIn, fileLen, filename, fileType);
		  }

		}

	  }
	  else {
		  hysteresis += 1;
		  // Must have two consecutive normal readings to remove warning
		  if (hysteresis >= 2) {
			  // Avoid overflow error
			  hysteresis -= 1;
			  lastSaveAlertFlag = 0;
			  saveAlertFlag = 0;
			  // Stop the buzzer
			  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

			  // Turn off red LED
			  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

			  // Turn on green LED
			  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

		  }
	  }
}

void checkADCReading() {
	if (batteryVoltage < 3.600) {
		if (batAlertFlag == 1 && lastBatAlertFlag == 1) {
			// do nothing
		}
		else {
			lastBatAlertFlag = batAlertFlag;
			batAlertFlag = 1;
			HAL_UART_Transmit(&huart1, "Low battery voltage detected!\n", 30, BUS_TIMEOUT);
			HAL_UART_Transmit(&huart3, "Low battery voltage detected!\n", 30, BUS_TIMEOUT);
		}
	}
	else {
		lastBatAlertFlag = 0;
		batAlertFlag = 0;
	}



	if (SPVoltage < 200) {
		if (SPAlertFlag == 1 && lastSPAlertFlag == 1)
			return;

		lastSPAlertFlag = SPAlertFlag;
		SPAlertFlag = 1;

		HAL_UART_Transmit(&huart1, "Low solar panel voltage!\n", 25, BUS_TIMEOUT);
		HAL_UART_Transmit(&huart3, "Low solar panel voltage!\n", 25, BUS_TIMEOUT);
	}
	else {
		lastSPAlertFlag = 0;
		SPAlertFlag = 0;
	}



}

// PWM functions
void setDC(TIM_HandleTypeDef *handle, uint8_t CH, uint16_t dc) {
	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = dc;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(handle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
	Error_Handler();
	}
}
void setPeriod(TIM_HandleTypeDef *handle, uint8_t CH, uint16_t period) {
	(*handle).Init.Period = period;

	if (HAL_TIM_PWM_Init(handle) != HAL_OK)
	{
	Error_Handler();
	}
}

// FLASH MEMORY FUNCTIONS
void flashStateMachine() {

	for (int i = 0; i < 25; i++) {
		HAL_Delay(20);

		switch (state) {
			 case idle:
				 parseCommand();
				 break;

			 case store:
				 parseStoreCommand();

				 if (fileType != 1) {
					 state = idle;
					 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);
					 break;
				 }

				 // TODO : If in here for too long, go back to idle and free buffer

				 didFileWrite = writeFile(fileIn, fileLen, filename, fileType);

				 if (didFileWrite == 1) {
					 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Wrote file successfully\n", 24, 100);
				 }
				 else {
					 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Failed to Write file\n", 21, 100);
				 }

				 state = idle;
				 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);


				 break;

			 case dir:
				getDir();
				state = idle;
				HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);
				break;

			 case mem:
				 profileMemory();
				 state = idle;
				 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);
				 break;

			 case del:
				 parseINodeNumber();

				 if (INodeNumber != 0) {

					 if (deleteINode(INodeNumber)) {
						 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "File has been deleted\n", 22, 100);
					 }
					 else {
						 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Failed to find INode\n", 21, 100);
					 }
					 state = idle;
					 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);

					 INodeNumber = 0;
				 }
				 else {
					 state = idle;
				 }

				 break;

			 case read:
				 parseINodeNumber();

				 if (INodeNumber != 0) {

					 if (getINode(INodeNumber, &node)) {
						 fileReadFromMem = readFile(node);
						 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, fileReadFromMem, node.fileSize, 10000);
						 free(fileReadFromMem);
						 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n", 1, 100);
					 }
					 else {
						 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Failed to read file\n", 20, 100);
					 }
					 state = idle;
					 HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);

					 INodeNumber = 0;
				 }
				 else {
					 state = idle;
				 }
				 break;

			 case clear:
				if (chipErase()) {
					if (SetupFilesystem()) {
						HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Memory has been cleared\n", 24, 100);
					}
					else {
						HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Failed to initialize file system\n", 33, 100);
					}

				}
				else {
					HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Failed to clear memory\n", 23, 100);
				}
				state = idle;

				HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);

				break;
		 }
	}
}


void parseCommand() {
	char trash;

	/* Check if a command was entered - read length of longest command*/
	char temp[10];
	get(temp, 10);

	// If the buffer does not have enough chars for a command yet
	if (strlen(temp) < 3) {
		return;
	}


	if (strncmp(temp, "store", 5) == 0) {
		state = store;
		memset(filename, 0, NAME_LIM);
		fileType = 255;
		endOfFile = 0;
		fileLen = 0;
		for (int i = 0; i < 6; i++) pop();

	}
	else if (strncmp(temp, "dir", 3) == 0) {
		state = dir;
		for (int i = 0; i < 3; i++) pop();
	}

	else if (strncmp(temp, "mem", 3) == 0) {
		state = mem;
		for (int i = 0; i < 3; i++) pop();
	}


	else if (strncmp(temp, "delete", 6) == 0) {
		state = del;
		for (int i = 0; i < 6; i++) pop();
	}

	else if (strncmp(temp, "read", 4) == 0) {
		INodeNumber = 0;
		state = read;
		for (int i = 0; i < 4; i++) pop();
	}

	else if (strncmp(temp, "clear", 5) == 0) {
		state = clear;
		for (int i = 0; i < 5; i++) pop();
	}

	// If no good command was read, advance the tail by popping once
	else {
		trash = pop();
	}
}

void parseStoreCommand() {
	int waitCount = 0;
	int i = 0;
	int flag = 0;
	char type[4] = {'\0'};
	memset(filename, '\0', NAME_LIM);

	/* Parse filename */
	while (waitCount < 250) {
		filename[i] = pop();

		if (filename[i] == '\0'){
			waitCount++;
			HAL_Delay(1);
			continue;
		}

		if (filename[i] == '.') {
			filename[i] = '\0';
			break;
		}
		if (i >= NAME_LIM) {
			HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Unable to parse file name\n", 26, 100);
			flag = 1;
			break;
		}
		i++;
	}

	if (waitCount == 250) {
		flag = 1;
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Unable to parse file name\n", 26, 100);
	}

	/* Check file type */
	waitCount = 0;
	for (i = 0; i < 3; i++) {
		type[i] = pop();

		if (type[i] == '\0') {
			waitCount++;
			HAL_Delay(1);
			if (waitCount > 100) {
				break;
			}
			i--;
		}


	}

	if (strcmp(type, "txt") != 0 && flag == 0) {
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Unable to parse file type\n", 26, 100);
		flag = 1;
	}

	// If parsing the filename or file type failed
	if (flag) {
		HAL_Delay(250);
		buffTail = buffHead;
		fileType = 0xFF;
		return;
	}


	fileType = 1;
	memset(fileIn, '\0', 1024*4);

	/* Read file contents */
	i = 0;
	char temp;
	waitCount = 0;

	while (i < 1024*4) {
		temp = pop();
		if (temp == '\0') {
			HAL_Delay(1);
			waitCount++;
			if (waitCount > 2500) {
				fileType = 0xFF;
				return;
			}
			continue;
		}

		if (temp == 0x04) {
			fileLen = i;
			endOfFile = 1;
			break;
		}

		fileIn[i] = temp;
		i++;
	}
}

void parseINodeNumber() {
	pop(); // get rid of space
	char num[4] = {'\0'};
	num[0] = pop();
	num[1] = pop();
	num[2] = pop();

	/* Read in the file number to read from mem */
	if (atoi(num) > 0 && atoi(num) < MAX_INODES) {
		INodeNumber = atoi(num);
	}
	else {
		state = idle;
		INodeNumber = 0;
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "Unable to parse file number\n", 28, 100);
		HAL_UART_Transmit((UARTFlag) ? &huart1 : &huart3, "\n>", 2, 1);
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
