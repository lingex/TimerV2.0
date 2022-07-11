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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);
#define LCD_BACKLIGHT(val) (HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, val > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET))
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

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t timerFlag = 0;
volatile uint8_t rtcIntFlag = 0;
volatile uint32_t btnVal = 0;
volatile uint8_t usbDet = 0;
volatile uint8_t usbDetFlag = 0;
volatile uint8_t usbTimeout = 6;
uint8_t usbState = 0;
uint16_t batAdc = 0;
uint8_t musicIndex = 0;
uint8_t musicVolume = 95;
DevState_t devState = DevStateStandby;
int8_t blTick = 3;
int8_t sleepTick = 5;
int8_t waitIdleTick = 8;
extern uint8_t playing;
extern USBD_HandleTypeDef hUsbDeviceFS;

FATFS FatFs;
u8g2_t u8g2;
char tmpstr[] = "0123456789abcdef0123456789abcdef0123456789abcdef\0";
const char *music[] = {"Take my breath away.mp3", "Woman In Love.mp3", "ph.mp3", "pp.mp3", "mm.mp3", "cc.mp3"};

_RTC rtc = {
	.Year = 22, .Month = 7, .Date = 3, .DaysOfWeek = SUNDAY, .Hour = 13, .Min = 25, .Sec = 22};

_COUNTER runCounter = {
	.Hour = 0, .Min = 5, .Sec = 0};

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
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t Get_BatAdc(void);
void TimerProcess(void);
void Sleep(void);

uint8_t GetConfigVolume(void);
void SetConfigVolume(uint8_t volume);
uint8_t GetConfigMusicIndex(void);
void SetConfigMusicIndex(uint8_t index);

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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

	musicIndex = GetConfigMusicIndex();
	musicVolume = GetConfigVolume();

	HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
	usbDet = (HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin) == GPIO_PIN_SET) ? 1 : 0;

	FRESULT rc = f_mount(&FatFs, "", 0);
	if (rc != FR_OK)
	{
		return rc;
	}
	PlayerInit(&hi2s2);

	u8g2Init(&u8g2);

	u8g2_DrawLine(&u8g2, 0, 11, 127, 11);

	u8g2_SetFont(&u8g2, u8g2_font_7x14B_tr);

	sprintf(tmpstr, "Timer V2.0");
	u8g2_DrawStr(&u8g2, 28, 10, tmpstr);
	u8g2_SendBuffer(&u8g2);
	HAL_Delay(200);

	RX8025T_Init(&hi2c1);
	RX8025T_SetINTPerSec();
	// RX8025T_SetINTPerMin();
	RX8025T_GetTime(&rtc);
	//RX8025T_SetTime(&rtc);

	//DispUpdate();

	DevState_t bState = DevStateStandby;

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
#if 0
	if (rtcIntFlag != 0)
	{
		rtcIntFlag = 0;
		sprintf(tmpstr, "Timer V2.0");
		u8g2_DrawStr(&u8g2, 28, 10, tmpstr);
		RX8025T_GetTime(&rtc);
		sprintf(tmpstr, "%04d-%02d-%02d", rtc.Year + 2000, rtc.Month, rtc.Date);
		u8g2_DrawStr(&u8g2, 20, 30, tmpstr);
		sprintf(tmpstr, "%02d:%02d:%02d", rtc.Hour, rtc.Min, rtc.Sec);
		u8g2_DrawStr(&u8g2, 20, 45, tmpstr);

		u8g2_SendBuffer(&u8g2);
	}
#endif
#if 0
	if ((btnVal & 0x0008) != 0)
	{
		btnVal &= ~0x0008;
		//PlayerStart("Take my breath away.mp3");
		PlayerStart("test.mp3");
	}
	if ((btnVal & 0x0001) != 0)
	{
		btnVal &= ~0x0001;
		PlayerStart("For Elise.mp3");
	}
	if ((btnVal & 0x0020) != 0)
	{
		btnVal &= ~0x0020;
		PlayerStart("Take my breath away.mp3");
	}
	if ((btnVal & 0x0002) != 0)
	{
		btnVal &= ~0x0002;
		HAL_GPIO_TogglePin(LCD_BL_GPIO_Port, LCD_BL_Pin);
	}
#endif

	if (btnVal != 0)
	{
		sprintf(tmpstr, "BC %lu.\n", HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);

		LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
		blTick = 6;
		sleepTick = 8;
		waitIdleTick = 30;
		if (devState == DevStateSleep)
		{
			devState = DevStateStandby;
		}
		sprintf(tmpstr, "BA %lu.\n", HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
		OnBtnDown(btnVal);
		sprintf(tmpstr, "AA %lu.\n", HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
		if (bState != devState)
		{
			// bState = devState;
			u8g2_ClearBuffer(&u8g2);
			u8g2_SendBuffer(&u8g2);
		}
		btnVal = 0;
		DispUpdate();
		sprintf(tmpstr, "BD %lu.\n", HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
		//RX8025T_SetINTPerSec();
	}
	if (rtcIntFlag != 0)
	{
		rtcIntFlag = 0;
		// MX_I2C2_Init();
		RX8025T_GetTime(&rtc);
		// HAL_I2C_DeInit(&hi2c2);

		// check battery
		HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_RESET);
		MX_ADC1_Init();
		//HAL_Delay(1);
		batAdc = Get_BatAdc();
		HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, GPIO_PIN_SET);
		HAL_ADC_DeInit(&hadc1);

		if (devState == DevStateTimerRun)
		{
			TimerProcess();
		}
		DispUpdate();
		if (blTick > 0)
		{
			blTick--;
			if (blTick == 0)
			{
				LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
			}
		}
		if (sleepTick > 0)
		{
			sleepTick--;
		}
		if (waitIdleTick > 0)
		{
			waitIdleTick--;
		}
		if (usbTimeout > 0)
		{
			usbTimeout--;
		}
	}
	if (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED && usbTimeout == 1)
	{
		HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);
		W25QXX_PowerDown(); // 2-4mA
	}

	if (usbDetFlag != 0)
	{
		usbDetFlag = 0;
		LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
		blTick += 5;
		sleepTick += 8;
		waitIdleTick += 20;
		if (usbDet == 1)
		{
			if (playing)
			{
				PlayerStop();
			}
			W25QXX_WAKEUP();
			HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_SET);
		}
	}
	if (playing != 0)
	{
		PlayerUpdate();
	}
	else if (waitIdleTick == 0)
	{
		devState = DevStateSleep;
	}
	if (playing == 0 && usbDet == 0)
	{
#if 1 // debug
		sprintf(tmpstr, "S: %02d:%02d:%02d, battery:%d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, batAdc, HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
#endif
		Sleep();
#if 1 // debug
		sprintf(tmpstr, "W: %02d:%02d:%02d, battery:%d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, batAdc, HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
#endif
	}

#if 0
	if (bState != devState || usbState != usbDet)
	{
		usbState = usbDet;
		bState = devState;
		DispUpdate();
		if (devState == DevStateSleep)
		{
			if (usbDet == 0)
			{
				blTick = 0;
				sleepTick = 0;
				waitIdleTick = 0;
				//RX8025T_SetINTPerMin();
				LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
				RX8025T_SetINTDisable();
			}
			else if (usbDet == 1)
			{
				// USB plug in
				RX8025T_SetINTPerSec();
			}
		}
	}
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_I2S2|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
	switch (GPIO_Pin)
	{
	case RTC_INT_Pin:
		rtcIntFlag = 1;
		break;
	case USB_DET_Pin:
		usbDet = (HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin) == GPIO_PIN_SET) ? 1 : 0;
		usbDetFlag = 1;
		usbTimeout = 6;
		break;
	case BTN_0_Pin:
		btnVal |= BTN_VAL_GO;
		break;
	case BTN_1_Pin:
		btnVal |= BTN_VAL_UP;
		break;
	case BTN_2_Pin:
		btnVal |= BTN_VAL_RIGHT;
		break;
	case BTN_3_Pin:
		btnVal |= BTN_VAL_DOWN;
		break;
	case BTN_4_Pin:
		btnVal |= BTN_VAL_LEFT;
		break;
	case BTN_5_Pin:
		btnVal |= BTN_VAL_ESC;
		//sprintf(tmpstr, "CB %lu.\n", HAL_GetTick());
		//HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);
		break;
	
	default:
		break;
	}
}

uint16_t Get_BatAdc(void)
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
	蔡勒公式�??????????????

	W = [C/4] - 2C + y + [y/4] + [13 * (M+1) / 5] + d - 1

	C 是世纪数减一，y 是年份后两位，M 是月份，d 是日数�??1 月和 2 月要按上�??????????????年的 13 月和
	14 月来算，这时 C�?????????????? y均按上一年取值�??

		两个公式中的[...]均指只取计算结果的整数部分�?�算出来的W 除以 7，余数是几就是星�??????????????
	几�?�如果余数是 0，则为星期日�??????????????
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
					blTick = 20;
					waitIdleTick = 0;
					LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
					devState = DevStateAlarm;
					u8g2_ClearBuffer(&u8g2);
					u8g2_SendBuffer(&u8g2);
					PlayerStart(music[musicIndex]);
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
	pFunction appEntry;

	HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);

	//__disable_irq();		//won't work when this execute
	// HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);

	// get the application stack pointer (1st entry in the app vector table)
	appStack = (uint32_t) * ((__IO uint32_t *)addr);

	// Get the app entry point (2nd entry in the app vector table
	appEntry = (pFunction) * (__IO uint32_t *)(addr + 4);

	HAL_RCC_DeInit();
	// LL_RCC_DeInit();
	HAL_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	// Reconfigure vector table offset to match the app location
	SCB->VTOR = addr;
	__set_MSP(appStack); // Set app stack pointer
	NVIC_SystemReset();
	appEntry();			 // Start the app

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
	//{
	//	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	//	HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI2_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI3_IRQn);
	//	HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

	//	// disable BTN interrupt
	//	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	//	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	//	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	//	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	//}
	if (devState == DevStateSleep)
	{
		LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
	}


#if 0
	//if (usbDet != 0)
	{
		sprintf(tmpstr, "Go to sleep:%02d:%02d:%02d, battery:%%d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, batAdc, HAL_GetTick());
		HAL_UART_Transmit(&huart3, (uint8_t *)tmpstr, strlen(tmpstr), 100);
	}

	HAL_UART_DeInit(&huart3);
	//HAL_DAC_DeInit(&hdac);
#endif
	{
		HAL_DBGMCU_DisableDBGStopMode();
	}

	if (usbDet == 0 && batAdc < 1725 && 0) // about 3.45V
	{
		sprintf(tmpstr, "Low battery, adc:%d, tick:%lu.\n", batAdc, HAL_GetTick());
		HAL_UART_Transmit(&huart2, (uint8_t *)tmpstr, strlen(tmpstr), 100);

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
#if 1
	SystemClock_Config();
	// MX_DAC_Init();
#else
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
	}
#endif
#if 0
	//if (usbDet != 0)
	{
		HAL_UART_Init(&huart3);
		sprintf(tmpstr, "Wake up, 20%02d-%02d-%02d, tick:%lu.\n", rtc.Year, rtc.Month, rtc.Date, HAL_GetTick());
		HAL_UART_Transmit(&huart3, (uint8_t *)tmpstr, strlen(tmpstr), 100);
	}
#endif

	//{
	//	// disable BTN interrupt

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

void SysTickCb(void)
{
	//button_ticks();
}

uint8_t GetConfigVolume(void)
{
	uint8_t tmp = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	if (tmp > 100)
	{
		tmp = 100;
	}
	
	return tmp;
}
void SetConfigVolume(uint8_t volume)
{
	// Write Back Up Register 1 Data
	HAL_PWR_EnableBkUpAccess();
	// Writes a data in a RTC Backup data Register 1
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, volume);
	HAL_PWR_DisableBkUpAccess();

}
uint8_t GetConfigMusicIndex(void)
{
	uint8_t tmp = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	if (tmp > MUSIC_SIZE)
	{
		tmp = MUSIC_SIZE;
	}
	return tmp;
}
void SetConfigMusicIndex(uint8_t index)
{
	// Write Back Up Register 2 Data
	HAL_PWR_EnableBkUpAccess();
	// Writes a data in a RTC Backup data Register 2
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, index);
	HAL_PWR_DisableBkUpAccess();
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
