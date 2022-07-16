/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int CalcDaysOfWeek(int year, int month, int date);
void GoDfu();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_3_Pin GPIO_PIN_13
#define BTN_3_GPIO_Port GPIOC
#define BTN_3_EXTI_IRQn EXTI15_10_IRQn
#define BTN_4_Pin GPIO_PIN_14
#define BTN_4_GPIO_Port GPIOC
#define BTN_4_EXTI_IRQn EXTI15_10_IRQn
#define BTN_5_Pin GPIO_PIN_15
#define BTN_5_GPIO_Port GPIOC
#define BTN_5_EXTI_IRQn EXTI15_10_IRQn
#define RTC_INT_Pin GPIO_PIN_1
#define RTC_INT_GPIO_Port GPIOC
#define RTC_INT_EXTI_IRQn EXTI1_IRQn
#define USB_DET_Pin GPIO_PIN_0
#define USB_DET_GPIO_Port GPIOA
#define USB_DET_EXTI_IRQn EXTI0_IRQn
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOA
#define LCD_BL_Pin GPIO_PIN_6
#define LCD_BL_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_5
#define LCD_RST_GPIO_Port GPIOC
#define BAT_ADC_EN_Pin GPIO_PIN_0
#define BAT_ADC_EN_GPIO_Port GPIOB
#define BAT_ADC_Pin GPIO_PIN_1
#define BAT_ADC_GPIO_Port GPIOB
#define SPK_EN_Pin GPIO_PIN_14
#define SPK_EN_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_10
#define USB_EN_GPIO_Port GPIOA
#define BTN_0_Pin GPIO_PIN_10
#define BTN_0_GPIO_Port GPIOC
#define BTN_0_EXTI_IRQn EXTI15_10_IRQn
#define BTN_1_Pin GPIO_PIN_11
#define BTN_1_GPIO_Port GPIOC
#define BTN_1_EXTI_IRQn EXTI15_10_IRQn
#define BTN_2_Pin GPIO_PIN_12
#define BTN_2_GPIO_Port GPIOC
#define BTN_2_EXTI_IRQn EXTI15_10_IRQn
#define SPI_CS_Pin GPIO_PIN_2
#define SPI_CS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */
#ifndef MATH_MIN
#define MATH_MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MATH_MAX
#define MATH_MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#define MUSIC_MAX 5
#define MUSIC_FILE_NAME_LEN 32

	typedef enum
	{
		DevStateSleep = 0,
		DevStateStandby,
		DevStateTimerSet,
		DevStateTimerPause,
		DevStateTimerRun,
		DevStateAlarm,
		DevStateMenuMain,
		DevStateMenuClock,
		DevStateMenuMusic,
		DevStateMenuVolume,
		DevStateMenuVersion,
		DevStateUsbMode

	} DevState_t;

	extern DevState_t devState;

	typedef enum
	{
		RTC_WKUP_SEC = 0,
		RTC_WKUP_MIN,
	} RTC_WKUP_INTERVAL_t;

	typedef struct
	{
		uint8_t Hour;
		uint8_t Min;
		uint8_t Sec;
	} _COUNTER;

	extern _COUNTER runCounter;

	extern char musicList[MUSIC_MAX][MUSIC_FILE_NAME_LEN];
	extern uint8_t musicSize;


#define BL_BRIGHTNESS_ON 1
#define BL_BRIGHTNESS_OFF 0

#define BTN_VAL_GO 		0x0001
#define BTN_VAL_ESC 	0x0002
#define BTN_VAL_UP 		0x0004
#define BTN_VAL_DOWN 	0x0008
#define BTN_VAL_LEFT 	0x0010
#define BTN_VAL_RIGHT 	0x0020

#define WKUP_REASON_POWER 0x0001
#define WKUP_REASON_BTN   0x0002
#define WKUP_REASON_RTC   0x0004
#define WKUP_REASON_USB   0x0008


#define HARDWARE_VERSION "V2.0"
#define FIRMWARE_VERSION "V0.8"
extern const char *builtTime;

#define APP_BOOTLOADER_ADDR (FLASH_BASE | 0x0000)
#define APP_TIMER_ADDR (FLASH_BASE | 0x10000)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
