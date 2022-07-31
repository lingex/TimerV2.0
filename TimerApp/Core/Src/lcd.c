#include "lcd.h"

SPI_HandleTypeDef* pSpi = NULL;

uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	switch (msg)
	{
	case U8X8_MSG_BYTE_INIT:
	break;
	case U8X8_MSG_BYTE_START_TRANSFER:
	break;
	case U8X8_MSG_BYTE_SEND:
	{
		HAL_SPI_Transmit(pSpi, (uint8_t *)arg_ptr, arg_int, arg_int * 10);
	}
	break;
	case U8X8_MSG_BYTE_END_TRANSFER:

		break;
	case U8X8_MSG_BYTE_SET_DC:
		if (arg_int)
			LCD_DC_SET;
		else
			LCD_DC_RESET;
		break;
	default:
		return 0;
	}
	return 1;
}

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
	switch (msg)
	{
	case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
		__NOP();
		break;
	case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
		for (uint16_t i = 0; i < arg_int; i++)
		{
			__NOP();
		}
		break;
	case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_MENU_SELECT:
		u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
		break;
	case U8X8_MSG_GPIO_MENU_NEXT:
		u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
		break;
	case U8X8_MSG_GPIO_MENU_PREV:
		u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
		break;
	case U8X8_MSG_GPIO_MENU_HOME:
		u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
		break;
	case U8X8_MSG_GPIO_CS:
		if (arg_int)
			LCD_CS_SET;
		else
			LCD_CS_RESET;
		break;
	case U8X8_MSG_GPIO_DC:
		if (arg_int)
			LCD_DC_SET;
		else
			LCD_DC_RESET;
		break;
	case U8X8_MSG_GPIO_RESET:
		if (arg_int)
			LCD_RST_SET;
		else
			LCD_RST_RESET;
		break;
	default:
		u8x8_SetGPIOResult(u8x8, 1); // default return value
		break;
	}
	return 1;
}
void u8g2Init(u8g2_t *u8g2, SPI_HandleTypeDef* hspi)
{
	pSpi = hspi;
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

	//U8G2_UC1701_MINI12864_F_4W_SW_SPI
	//u8g2_Setup_st7565_64128n_f(u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
	u8g2_Setup_uc1701_mini12864_f(u8g2, U8G2_R0, u8x8_byte_hw_spi, u8x8_gpio_and_delay);
	u8g2_InitDisplay(u8g2);
	u8g2_SetPowerSave(u8g2, 0);
	u8g2_ClearBuffer(u8g2);
}