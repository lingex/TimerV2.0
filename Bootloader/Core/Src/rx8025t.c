#include "rx8025t.h"

I2C_HandleTypeDef *pI2c = NULL;

static uint8_t B2D(uint8_t bcd);
static uint8_t D2B(uint8_t decimal);
static uint8_t GetDoW(uint8_t val);
static uint8_t SetDoW(uint8_t val);

bool RX8025T_Init(I2C_HandleTypeDef *handle)
{
	uint8_t regVal = 0;

	pI2c = handle;
	if (!ReadRegister(REGADDR_FLAG, &regVal))
	{
		return false;
	}
	if ((regVal & FLAG_VLF) != 0)
	{
		return true;
	}
	WriteRegister(REGADDR_FLAG, 0x00);
	WriteRegister(REGADDR_CONTROL, 0x40);

	return true;
}

bool RX8025T_GetTime(_RTC *rtc)
{
	uint8_t startAddr = REGADDR_FLAG;
	uint8_t buffer[7] = {0};

	if (!ReadRegister(REGADDR_FLAG, buffer))
	{
		return false;
	}

	if ((buffer[0] & FLAG_UF) != 0)
	{
		startAddr = REGADDR_SEC;
		if (HAL_I2C_Master_Transmit(pI2c, RX8025T_READ_ADDR, &startAddr, 1, HAL_MAX_DELAY) != HAL_OK)
		{
			return false;
		}
		if (HAL_I2C_Master_Receive(pI2c, RX8025T_READ_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK)
		{
			return false;
		}
		rtc->Sec = B2D(buffer[0] & 0x7F);
		rtc->Min = B2D(buffer[1] & 0x7F);
		rtc->Hour = B2D(buffer[2] & 0x3F);
		rtc->DaysOfWeek = GetDoW(buffer[3] & 0x7f);
		rtc->Date = B2D(buffer[4] & 0x3F);
		rtc->Month = B2D(buffer[5] & 0x1F);
		rtc->Year = B2D(buffer[6]);
	}
	return true;
}

bool RX8025T_SetTime(_RTC *rtc)
{
	uint8_t startAddr = REGADDR_SEC;
	uint8_t buffer[8] = {startAddr, D2B(rtc->Sec), D2B(rtc->Min), D2B(rtc->Hour), SetDoW(rtc->DaysOfWeek), D2B(rtc->Date), D2B(rtc->Month), D2B(rtc->Year)};
	if (HAL_I2C_Master_Transmit(pI2c, RX8025T_WRITE_ADDR, buffer, sizeof(buffer), 100) != HAL_OK)
	{
		return false;
	}
	return true;
}

bool RX8025T_SetINTPerSec(void)
{
	// once per second
	uint8_t regVal = 0;
	ReadRegister(REGADDR_CONTROL, &regVal);
	// regVal |= (CONTR_UIE);
	regVal |= (CONTR_UIE) | (CONTR_TIE);
	WriteRegister(REGADDR_CONTROL, regVal);

	ReadRegister(REGADDR_EXTEN, &regVal);
	regVal &= ~(EXTEN_USEL);
	regVal |= (EXTEN_TSEL1);
	regVal &= ~(EXTEN_TSEL0);
	WriteRegister(REGADDR_EXTEN, regVal);

	return true;
}

bool RX8025T_SetINTPerMin(void)
{
	// once per min
	uint8_t regVal = 0;
	ReadRegister(REGADDR_CONTROL, &regVal);
	// regVal |= (CONTR_UIE);
	regVal |= (CONTR_UIE) | (CONTR_TIE);
	WriteRegister(REGADDR_CONTROL, regVal);

	ReadRegister(REGADDR_EXTEN, &regVal);
	regVal |= (EXTEN_USEL);
	// regVal &= ~(EXTEN_USEL);
	regVal |= (EXTEN_TSEL1) | (EXTEN_TSEL0);
	WriteRegister(REGADDR_EXTEN, regVal);

	return true;
}

bool RX8025T_SetINTDisable(void)
{
	uint8_t regVal = 0;
	ReadRegister(REGADDR_CONTROL, &regVal);
	// regVal |= (CONTR_UIE);
	regVal &= ~(CONTR_UIE);
	WriteRegister(REGADDR_CONTROL, regVal);

	return true;
}

bool ReadRegister(uint8_t regAddr, uint8_t *value)
{
	if (HAL_I2C_Master_Transmit(pI2c, RX8025T_READ_ADDR, &regAddr, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	if (HAL_I2C_Master_Receive(pI2c, RX8025T_READ_ADDR, value, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	return true;
}

bool WriteRegister(uint8_t regAddr, uint8_t value)
{
	uint8_t buffer[2] = {regAddr, value};
	if (HAL_I2C_Master_Transmit(pI2c, RX8025T_WRITE_ADDR, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK)
	{
		return false;
	}
	return true;
}

static uint8_t B2D(uint8_t bcd)
{
	return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t D2B(uint8_t decimal)
{
	return (((decimal / 10) << 4) | (decimal % 10));
}

static uint8_t GetDoW(uint8_t val)
{
	uint8_t ret = 0;
	switch (val)
	{
	case 0x01:
		ret = 0;
		break;
	case 0x02:
		ret = 1;
		break;
	case 0x04:
		ret = 2;
		break;
	case 0x08:
		ret = 3;
		break;
	case 0x10:
		ret = 4;
		break;
	case 0x20:
		ret = 5;
		break;
	case 0x40:
		ret = 6;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static uint8_t SetDoW(uint8_t val)
{
	return ((val > 6) ? 0 : 0x01 << val);
}