#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

#include "printf.h"
#include "player.h"
#include "spi_flash.h"
#include "lcd.h"
#include "rx8025t.h"
#include "menu.h"
#include <time.h>
#include <stdio.h>
#include "config.h"


typedef void (*pFunction)(void);
#define LCD_BACKLIGHT(val) (HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, val > 0 ? GPIO_PIN_RESET : GPIO_PIN_SET))
#define USB_EN(val) (HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, val > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET))
#define BAT_ADC_EN(val) (HAL_GPIO_WritePin(BAT_ADC_EN_GPIO_Port, BAT_ADC_EN_Pin, val != 0 ? GPIO_PIN_RESET : GPIO_PIN_SET))

#define USB_DET ((HAL_GPIO_ReadPin(USB_DET_GPIO_Port, USB_DET_Pin) == GPIO_PIN_SET) ? 1 : 0)

#define MENU_TIMEOUT 30
#define SLEEP_TIMEOUT 10
#define BL_TIMEOUT 8

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern I2S_HandleTypeDef hi2s2;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

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

_RTC rtc = {
	.Year = 22, .Month = 7, .Date = 3, .DaysOfWeek = SUNDAY, .Hour = 13, .Min = 25, .Sec = 22};

_COUNTER runCounter = {
	.Hour = 0, .Min = 5, .Sec = 0};

Player player(&hi2s2);

void BatteryVoltage(void);
uint32_t GetBatAdc(void);
void TimerProcess(void);
void Sleep(void);
void RtcToTimestamp(void);
void RtcToTm(_RTC *rtc, struct tm *t);
void TimestampToRTC(void);
void PlayerStartCallback(void);
void PlayerStopCallback(void);

int main(void)
{
	SCB->VTOR = APP_TIMER_ADDR;

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	InitProc();
	W25QXX_WAKEUP();

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
	u8g2Init(&u8g2, &hspi1);
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

	printf("Init player, tick: %lu.\n", HAL_GetTick());

	player.SetVolume(musicVolume);

	printf("Init RTC, tick: %lu.\n", HAL_GetTick());
	RX8025T_Init(&hi2c1);
	RX8025T_SetINTPerSec();
	// RX8025T_SetINTPerMin();
	//RX8025T_SetTime(&rtc);
	RtcToTimestamp();
	printf("Timestamp: %lu, tick: %lu.\n", currTime, HAL_GetTick());

	lcdBLTimeout = currTime + BL_TIMEOUT;
	sleepTimeout = currTime + SLEEP_TIMEOUT;

	BatteryVoltage();
	printf("Battery voltage: %lu mV, tick: %lu.\n\n", batVoltage, HAL_GetTick());

	printf("Loop start, tick: %lu.\n\n", HAL_GetTick());
	//DispUpdate();

	//PlayerStart("test.MP3");
	//PlayerStart("Take my breath away.mp3");
	//W25QXX_Erase_Chip();

  /* Infinite loop */

  while (1)
  {
	if (wkupReason != 0)
	{
		//printf("wkup:%#06x\n", wkupReason);

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
			BatteryVoltage();
			lcdUpdate = 1;
		}
		if ((wkupReason & WKUP_REASON_BTN) != 0)
		{
			if (wkupInterval != RTC_WKUP_SEC && (wkupReason & WKUP_REASON_RTC) == 0)
			{
				RtcToTimestamp();
			}

			lcdBLTimeout = currTime + BL_TIMEOUT;
			LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
			if (wkupInterval != RTC_WKUP_SEC)
			{
				wkupInterval = RTC_WKUP_SEC;
				RX8025T_SetINTPerSec();
			}
			OnBtnDown(btnVal);
			if (devState == DevStateTimerRun || devState == DevStateMenuMusic)
			{
				menuTimeout = 0;
				sleepTimeout = 0;
			}
			else
			{
				sleepTimeout = 0;
				menuTimeout = currTime + MENU_TIMEOUT;
			}
			btnVal = 0;
			lcdUpdate = 1;
		}
		if ((wkupReason & WKUP_REASON_USB) != 0)
		{
			if (devState == DevStateSleep)
			{
				devState = DevStateStandby;
				menuTimeout = currTime + MENU_TIMEOUT;
			}
			RtcToTimestamp();
			lcdBLTimeout = currTime + BL_TIMEOUT;
			LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
			if (wkupInterval != RTC_WKUP_SEC)
			{
				wkupInterval = RTC_WKUP_SEC;
				RX8025T_SetINTPerSec();
			}
			if (usbDet == 1)
			{
				if (player.IsBusy())
				{
					player.Stop();
				}
				if (W25QXX_GetPowerState() == 0)
				{
					W25QXX_WAKEUP();
				}
				USB_EN(1);
			}
			else
			{
				USB_EN(0);
				if (W25QXX_GetPowerState() != 0 && !player.IsBusy())
				{
					W25QXX_PowerDown();
				}
			}
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
		sleepTimeout = currTime + SLEEP_TIMEOUT;
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
	if (player.IsBusy())
	{
		player.Tick();
	}

	if (!player.IsBusy() && usbDet == 0 && devState == DevStateSleep && wkupReason == 0)
	{
		LCD_BACKLIGHT(BL_BRIGHTNESS_OFF);
		if (wkupInterval != RTC_WKUP_MIN)
		{
			wkupInterval = RTC_WKUP_MIN;
			RX8025T_SetINTPerMin();
		}
		if (W25QXX_GetPowerState() != 0)
		{
			W25QXX_PowerDown();
		}

		//printf("Sleep: %02d:%02d:%02d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, HAL_GetTick());
		Sleep();
		//printf("Wkup: %02d:%02d:%02d, tick:%lu.\n", rtc.Hour, rtc.Min, rtc.Sec, HAL_GetTick());
	}
  }
}

void _putchar(char character)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 5);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//printf("c:%#06x\n", GPIO_Pin);
	switch (GPIO_Pin)
	{
	case RTC_INT_Pin:
		wkupReason |= WKUP_REASON_RTC;
		break;
	case USB_DET_Pin:
		usbDet = USB_DET;
		wkupReason |= WKUP_REASON_USB;
		break;
	case BTN_GO_Pin:
		btnVal |= BTN_VAL_GO;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_UP_Pin:
		btnVal |= BTN_VAL_UP;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_RIGHT_Pin:
		btnVal |= BTN_VAL_RIGHT;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_DOWN_Pin:
		btnVal |= BTN_VAL_DOWN;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_LEFT_Pin:
		btnVal |= BTN_VAL_LEFT;
		wkupReason |= WKUP_REASON_BTN;
		break;
	case BTN_ESC_Pin:
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
	//MX_ADC1_Init();
	ADC_Config();
	//HAL_Delay(1);
	batVoltage = GetBatAdc() * 2;
	BAT_ADC_EN(0);
	HAL_ADC_DeInit(&hadc1);
}

uint32_t GetBatAdc(void)
{
	uint32_t temp = 0;
	const uint32_t fixVal = 325;

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
					lcdBLTimeout = currTime + BL_TIMEOUT;
					LCD_BACKLIGHT(BL_BRIGHTNESS_ON);
					devState = DevStateAlarm;
					u8g2_ClearBuffer(&u8g2);
					u8g2_SendBuffer(&u8g2);
					player.Play(musicUsing);
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
#endif
	{
		HAL_DBGMCU_DisableDBGStopMode();
		HAL_DBGMCU_DisableDBGStandbyMode();
	}

	if (usbDet == 0 && batVoltage < 3450) // about 3.45V
	{
		printf("Low battery, adc:%lu, tick:%lu.\n", batVoltage, HAL_GetTick());

		// low battery
		__disable_irq();

		u8g2_ClearBuffer(&u8g2);
		u8g2_SendBuffer(&u8g2);
		u8g2_SetPowerSave(&u8g2, 1);

		HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN1);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 
		/* Enable WKUP pin */
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		/* Request to enter STANDBY mode (Wake Up flag is cleared in HAL_PWR_EnterSTANDBYMode function) */
		HAL_PWR_EnterSTANDBYMode();
		// after power down here, device has to charge/reset to wakeup
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
		menuTimeout = currTime + MENU_TIMEOUT;
	}
	else if (devState == DevStateAlarm)
	{
		devState = DevStateStandby;
		sleepTimeout = currTime + SLEEP_TIMEOUT;
		if (usbDet == 0)
		{
			W25QXX_PowerDown();
		}
	}
	wkupReason |= WKUP_REASON_POWER;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s)
{
	player.TransmitHalfCpltCallback();
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s)
{
	player.TransmitCpltCallback();
}