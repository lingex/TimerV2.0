/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include "player.h"
#include "spi_flash.h"
#include "lcd.h"
#include "rx8025t.h"
#include "menu.h"
#include <time.h>
#include <stdio.h>
#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
#define LCD_BACKLIGHT(val) (HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, val > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define USB_EN(val) (HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, val > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define BAT_ADC_EN(val) (HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, val != 0 ? GPIO_PIN_RESET : GPIO_PIN_SET))

#define USB_DET ((HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin) == GPIO_PIN_SET) ? 1 : 0)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
__IO uint32_t btnVal = 0;
__IO uint8_t usbDet = 0;
uint32_t batVoltage = 0;
uint8_t musicVolume = 95;
DevState_t devState = DevStateStandby;
__IO uint32_t wkupReason = WKUP_REASON_POWER;
RTC_WKUP_INTERVAL_t wkupInterval = RTC_WKUP_SEC;
uint32_t currTime = 0;
uint32_t lcdBLTimeout = 0;
uint32_t menuTimeout = 0;
uint32_t sleepTimeout = 0;
uint8_t lcdUpdate = 0;

FATFS FatFs;
u8g2_t u8g2;
char tmpstr[] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
char musicUsing[] = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
const char *builtTime = __DATE__ "," __TIME__;

uint8_t musicSize = 0;
char musicList[MUSIC_MAX][MUSIC_FILE_NAME_LEN] = {'\0'};

_RTC rtc = {
	.Year = 22, .Month = 7, .Date = 3, .DaysOfWeek = SUNDAY, .Hour = 13, .Min = 25, .Sec = 22};

_COUNTER runCounter = {
	.Hour = 0, .Min = 5, .Sec = 0};


extern uint8_t playing;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void BatteryVoltage(void);
uint32_t GetBatAdc(void);
void TimerProcess(void);
void Sleep(void);

void SaveConfigs(void);

void RtcToTimestamp(void);
void RtcToTm(_RTC *rtc, struct tm *t);

void TimestampToRTC(void);

void PlayerStartCallback(void);
void PlayerStopCallback(void);
void LoadFileListing(void);

void _putchar(char character)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 5);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SCB->VTOR = APP_TIMER_ADDR;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //__HAL_RCC_DMA1_CLK_ENABLE();
  //__HAL_RCC_DMA2_CLK_ENABLE();
  //spi & i2s dma will not work without this
  MX_DMA_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	printf("Starting, tick: %lu.\n", HAL_GetTick());
	printf("Hardware: %s.\n", HARDWARE_VERSION);
	printf("Firmware: %s.\n", FIRMWARE_VERSION);
	printf("Built: %s.\n\n", builtTime);

	LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
	usbDet = USB_DET;
	if (usbDet == 1)
	{
		USB_EN(1);
	}
	printf("Init LCD, tick: %lu.\n", HAL_GetTick());
	u8g2Init(&u8g2);
	u8g2_DrawLine(&u8g2, 0, 11, 127, 11);
	u8g2_SetFont(&u8g2, u8g2_font_7x14B_tr);
	sprintf(tmpstr, "Timer V2.0");
	u8g2_DrawStr(&u8g2, 28, 10, tmpstr);
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(200);

	printf("Init FATFS, tick: %lu.\n", HAL_GetTick());
	FRESULT rc = f_mount(&FatFs, "", 0);
	if (rc != FR_OK)
	{
		printf("FATFS mount failed! tick: %lu.\n", HAL_GetTick());
		//return rc;
	}
	printf("Init config, tick: %lu.\n", HAL_GetTick());
	LoadConfigs();
	PlayerVolumeAdj(musicVolume);
	//LoadFileListing();

	printf("Init player, tick: %lu.\n", HAL_GetTick());
	PlayerInit(&hi2s2);

	printf("Init RTC, tick: %lu.\n", HAL_GetTick());
	RX8025T_Init(&hi2c1);
	RX8025T_SetINTPerSec();
	// RX8025T_SetINTPerMin();
	//RX8025T_SetTime(&rtc);
	RtcToTimestamp();
	printf("Timestamp: %lu, tick: %lu.\n", currTime, HAL_GetTick());

	lcdBLTimeout = currTime + 10;
	sleepTimeout = currTime + 20;

	BatteryVoltage();
	printf("Battery voltage: %lu mV, tick: %lu.\n\n", batVoltage, HAL_GetTick());

	printf("Loop start, tick: %lu.\n\n", HAL_GetTick());
	//DispUpdate();

	//PlayerStart("test.MP3");
	//PlayerStart("Take my breath away.mp3");
	//W25QXX_Erase_Chip();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (wkupReason != 0)
	{
		if ((wkupReason & WKUP_REASON_BTN) != 0)
		{
			if (devState == DevStateSleep)
			{
				RtcToTimestamp();
			}

			lcdBLTimeout = currTime + 10;
			LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
			if (wkupInterval != RTC_WKUP_SEC)
			{
				wkupInterval = RTC_WKUP_SEC;
				RX8025T_SetINTPerSec();
			}
			OnBtnDown(btnVal);
			if (devState == DevStateTimerRun || devState == DevStateTimerPause || devState == DevStateMenuMusic)
			{
				menuTimeout = 0;
				sleepTimeout = 0;
			}
			else
			{
				menuTimeout = currTime + 10;
			}
			btnVal = 0;
			lcdUpdate = 1;
		}
		if ((wkupReason & WKUP_REASON_USB) != 0)
		{
			RtcToTimestamp();
			lcdBLTimeout = currTime + 10;
			LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
			if (usbDet == 1)
			{
				devState = DevStateStandby;
				if (W25QXX_GetPowerState() == 0)
				{
					W25QXX_WAKEUP();
				}
				USB_EN(1);
			}
			else
			{
				USB_EN(0);
				if (W25QXX_GetPowerState() != 0 && playing == 0)
				{
					W25QXX_PowerDown();
				}
			}
			lcdUpdate = 1;
		}
		if ((wkupReason & WKUP_REASON_RTC) != 0)
		{
#if 0	//counter
			currTime++;
			TimestampToRTC();
#else	//RTC
			RtcToTimestamp();
#endif
			if (devState == DevStateTimerRun)
			{
				TimerProcess();
			}
			//BatteryVoltage();
			lcdUpdate = 1;
		}
		if ((wkupReason & WKUP_REASON_POWER) != 0)
		{
			lcdUpdate = 1;
		}
		wkupReason = 0;
	}

	if (lcdBLTimeout > 0 && currTime >= lcdBLTimeout)
	{
		lcdBLTimeout = 0;
		LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
	}
	if (menuTimeout > 0 && currTime >= menuTimeout)
	{
		menuTimeout = 0;
		sleepTimeout = currTime + 10;
		devState = DevStateStandby;
		lcdUpdate = 1;
	}
	if (sleepTimeout > 0 && currTime >= sleepTimeout)
	{
		sleepTimeout = 0;
		devState = DevStateSleep;
		if (W25QXX_GetPowerState() != 0 && usbDet == 0)
		{
			W25QXX_PowerDown();
		}
		lcdUpdate = 1;
	}
	if (lcdUpdate != 0)
	{
		lcdUpdate = 0;
		DispUpdate();
	}
	if (playing != 0)
	{
		PlayerUpdate();
	}

	if (playing == 0/* && usbDet == 0*/)
	{
		switch (devState)
		{
		case DevStateStandby:
			/* code */
			break;
		case DevStateAlarm:
			//devState = DevStateSleep;
			break;
		case DevStateSleep:
			LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
			if (wkupInterval != RTC_WKUP_MIN)
			{
				wkupInterval = RTC_WKUP_MIN;
				RX8025T_SetINTPerMin();
			}
			if (W25QXX_GetPowerState() != 0)
			{
				//W25QXX_PowerDown();
			}
			break;
		default:
			break;
		}
		if (usbDet == 0)
		{
			//printf("Sleep: %02d:%02d:%02d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, HAL_GetTick());
			Sleep();
			//printf("Wkup: %02d:%02d:%02d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, HAL_GetTick());
		}
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_I2S2
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.I2s2ClockSelection = RCC_I2S2CLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_22K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
	W25QXX_WAKEUP();
  /* USER CODE END SPI3_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_CS_Pin|LCD_BL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPK_EN_GPIO_Port, SPK_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BTN_3_Pin BTN_4_Pin BTN_5_Pin BTN_0_Pin
                           BTN_1_Pin BTN_2_Pin */
  GPIO_InitStruct.Pin = BTN_3_Pin|BTN_4_Pin|BTN_5_Pin|BTN_0_Pin
                          |BTN_1_Pin|BTN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC2 PC3 PC6
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RTC_INT_Pin */
  GPIO_InitStruct.Pin = RTC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RTC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_DET_Pin */
  GPIO_InitStruct.Pin = USB_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA8 PA9 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_BL_Pin USB_EN_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_BL_Pin|USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_DC_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_ADC_EN_Pin SPK_EN_Pin */
  GPIO_InitStruct.Pin = BAT_ADC_EN_Pin|SPK_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != RTC_INT_Pin)
	{
		//printf("EXTI: %lu\r\n", HAL_GetTick());
	}

	switch (GPIO_Pin)
	{
	case RTC_INT_Pin:
		wkupReason |= WKUP_REASON_RTC;
		break;
	case USB_DET_Pin:
		usbDet = USB_DET;
		wkupReason |= WKUP_REASON_USB;
		break;
	case BTN_0_Pin:
		btnVal |= BTN_VAL_GO;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_1_Pin:
		btnVal |= BTN_VAL_UP;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_2_Pin:
		btnVal |= BTN_VAL_RIGHT;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_3_Pin:
		btnVal |= BTN_VAL_DOWN;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_4_Pin:
		btnVal |= BTN_VAL_LEFT;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_5_Pin:
		btnVal |= BTN_VAL_ESC;
		wkupReason |= WKUP_REASON_BTN;
		break;

	default:
		break;
	}
}

void BatteryVoltage(void)
{
	BAT_ADC_EN(1);
	MX_ADC1_Init();
	//HAL_Delay(1);
	batVoltage = GetBatAdc() * 2;
	BAT_ADC_EN(0);
	HAL_ADC_DeInit(&hadc1);
}

uint32_t GetBatAdc(void)
{
	uint32_t temp = 0;
	const uint32_t fixVal = 322;

	for (uint8_t n = 0; n < 5; n++)
	{
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
		{
			temp += HAL_ADC_GetValue(&hadc1) - fixVal;
		}
	}
	return (temp / 5);
}

// 21th century only
// 0=sunday, 1=monday
int CalcDaysOfWeek(int year, int month, int date)
{
	/*
	蔡勒公式�??????????????????????

	W = [C/4] - 2C + y + [y/4] + [13 * (M+1) / 5] + d - 1

	C 是世纪数减一，y 是年份后两位，M 是月份，d 是日数�??1 月和 2 月要按上�??????????????????????年的 13 月和
	14 月来算，这时 C�?????????????????????? y均按上一年取值�??

		两个公式中的[...]均指只取计算结果的整数部分�?�算出来的W 除以 7，余数是几就是星�??????????????????????
	几�?�如果余数是 0，则为星期日�??????????????????????
	*/
	int C = 21 - 1;
	int M = month;
	int y = year;
	int d = date;
	if (M <= 2)
	{
		y = year - 1;
		M = month + 12;
	}
	int W = (C / 4) - 2 * C + y + (y / 4) + (13 * (M + 1) / 5) + d - 1;

	return (W % 7);
}

void TimerProcess(void)
{
	if (devState == DevStateTimerRun)
	{
		if (runCounter.Sec > 0)
		{
			runCounter.Sec--;
		}

		if (runCounter.Sec == 0)
		{
			if (runCounter.Min == 0)
			{
				if (runCounter.Hour == 0)
				{
					lcdBLTimeout = currTime + 5;
					LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
					devState = DevStateAlarm;
					u8g2_ClearBuffer(&u8g2);
					u8g2_SendBuffer(&u8g2);
					PlayerStart(musicUsing);
					//DispAlarm();
				}
				else
				{
					runCounter.Hour--;
					runCounter.Min = 59;
				}
			}
			else
			{
				runCounter.Min--;
				runCounter.Sec = 59;
			}
		}
	}
}

void RunApp(uint32_t addr)
{
	/* Function pointer to the address of the user application. */

	uint32_t appStack;
	//pFunction appEntry;

	USB_EN(0);

	//__disable_irq();		//won't work when this execute
	// HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);

	// get the application stack pointer (1st entry in the app vector table)
	appStack = (uint32_t) * ((__IO uint32_t *)addr);

	// Get the app entry point (2nd entry in the app vector table
	//appEntry = (pFunction) * (__IO uint32_t *)(addr + 4);

	HAL_RCC_DeInit();
	HAL_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	// Reconfigure vector table offset to match the app location
	SCB->VTOR = addr;
	__set_MSP(appStack); // Set app stack pointer
#if 1
	NVIC_SystemReset();
#else
	appEntry();			 // Start the app
#endif
	while (1)
	{
		// never reached
	}
}

void GoDfu()
{
	RunApp(APP_BOOTLOADER_ADDR);
}

void Sleep(void)
{
#if 0
	//if (usbDet != 0)
	{
		sprintf(tmpstr, "Go to sleep:%02d:%02d:%02d, battery:%%d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, batVoltage, HAL_GetTick());
		HAL_UART_Transmit(&huart3, (uint8_t *)tmpstr, strlen(tmpstr), 100);
	}

	HAL_UART_DeInit(&huart3);
	//HAL_DAC_DeInit(&hdac);
#endif
	{
		HAL_DBGMCU_DisableDBGStopMode();
		HAL_DBGMCU_DisableDBGStandbyMode();
	}

	if (usbDet == 0 && batVoltage < 3450) // about 3.45V
	{
		printf("Low battery, adc:%d, tick:%lu.\n", batVoltage, HAL_GetTick());

		// low battery
		__disable_irq();
		// LCD_Off();
		u8g2_ClearBuffer(&u8g2);
		u8g2_SendBuffer(&u8g2);
		u8g2_SetPowerSave(&u8g2, 1);
		HAL_PWR_EnterSTANDBYMode();
		// after power down here, device has to reboot to wakeup
		//TODO usbdet wakup config
	}
	else
	{
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	}
	SystemClock_Config();

	//{
	// disable BTN interrupt

	//	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	//	HAL_NVIC_DisableIRQ(EXTI0_IRQn);

	//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	//	HAL_NVIC_DisableIRQ(EXTI1_IRQn);

	//	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	//	HAL_NVIC_DisableIRQ(EXTI3_IRQn);

	//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	//	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	//}
}

void RtcToTimestamp(void)
{
	RX8025T_GetTime(&rtc);
	struct tm t;
	RtcToTm(&rtc, &t);
	currTime = mktime(&t) - 8 * 3600;
}

void RtcToTm(_RTC *rtc, struct tm *t)
{
	t->tm_year = (int)rtc->Year + 2000 - 1900;
	t->tm_mon = rtc->Month - 1;
	t->tm_mday = rtc->Date;
	t->tm_hour = rtc->Hour;
	t->tm_min = rtc->Min;
	t->tm_sec = rtc->Sec;
}

void TimestampToRTC(void)
{
	time_t rawtime = currTime;
	struct tm * timeinfo;

	time (&rawtime);
	timeinfo = localtime (&rawtime);
	rtc.Year = timeinfo->tm_year - 2000;
	rtc.Month = timeinfo->tm_mon;
	rtc.Date = timeinfo->tm_mday;
	rtc.DaysOfWeek = timeinfo->tm_wday;
	rtc.Hour = timeinfo->tm_hour;
	rtc.Min = timeinfo->tm_min;
	rtc.Sec = timeinfo->tm_sec;
}

void PlayerStartCallback(void)
{
	if (W25QXX_GetPowerState() == 0)
	{
		W25QXX_WAKEUP();
	}

}

void PlayerStopCallback(void)
{
	if (devState == DevStateMenuMusic)
	{
		devState = DevStateMenuMain;
		menuTimeout = currTime + 10;
	}
	else if (devState == DevStateAlarm)
	{
		devState = DevStateStandby;
		sleepTimeout = currTime + 10;
		if (usbDet == 0)
		{
			W25QXX_PowerDown();
		}
	}
}

//load mp3 file list
void LoadFileListing() 
{
	DIR rootdir;
	static FILINFO finfo;
	FRESULT res = FR_OK;

/*
When LFN feature is enabled, lfname and lfsize in the file information structure 
must be initialized with valid value prior to use the f_readdir function.
*/
	char *buff = malloc(_MAX_LFN);
	finfo.lfname = buff;
	finfo.lfsize = _MAX_LFN;

#if 0	// init once
	if (musicSize > 0)
	{
		return;
	}
#else //dynamic refresh
	musicSize = 0;
#endif
	uint8_t oPowerState = W25QXX_GetPowerState();
	if (oPowerState == 0)
	{
		W25QXX_WAKEUP();
	}

	if ((res = f_opendir(&rootdir, "/")) != FR_OK)
	{
		printf("Error opening root directory %d\r\n", res);
		return;
	}

	while (f_readdir(&rootdir, &finfo) == FR_OK)
	{
		if (finfo.fname[0] == '\0') break;
		if (finfo.fname[0] == '.') continue;

		if (finfo.fattrib & AM_DIR)
		{
			//printf("found directory %s\r\n", finfo.fname);
			continue;
		}
		//printf("found file %s\r\n", finfo.fname);

		if ((strstr(finfo.fname, ".mp3") || strstr(finfo.fname, ".MP3")) && !strstr(finfo.fname, "test.mp3"))
		{
			//printf("mp3 file: %s\r\n", finfo.lfname);
			if (finfo.lfsize == 0)
			{
				continue;
			}
			
			strcpy(musicList[musicSize], finfo.lfname);
			musicSize++;
			if (musicSize >= MUSIC_MAX)
			{
				break;
			}
		}
	}
	f_closedir(&rootdir);
	free(buff);
	//printf("done reading rootdir\r\n");

	//printf("Music size: %u\r\n", musicSize);
	if (oPowerState == 0)
	{
		W25QXX_PowerDown();
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
