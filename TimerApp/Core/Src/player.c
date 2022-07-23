#include <string.h>
#include "player.h"
#include "ff.h"
#include "spiritMP3Dec.h"
#include "printf.h"
#include "lcd.h"

extern FATFS FatFs;

static FIL playingFile;
uint16_t audioBuffer[BUFFER_SIZE];
uint16_t volume = 100;	//default: 80

extern u8g2_t u8g2;
extern char tmpstr[];

I2S_HandleTypeDef* pI2s = NULL;

uint8_t playing = 0;

#ifdef DEBUG
extern UART_HandleTypeDef hlpuart1;
char tmpBuf[32] = {0};
#endif

const uint32_t I2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t I2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t I2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};



static volatile PlayerState_t playerState = PLAYER_STATE_IDLE;
static TSpiritMP3Decoder decoder;

static void PlayerSetSampleRate(uint32_t audioFreq);
static void PlayStream(uint16_t* pBuffer, uint32_t Size);
static unsigned int ReadData(void* data, unsigned int size, void* token);
static void VolumeScale(uint16_t* pBuffer, unsigned int samples);
static int PlayFirstFrame(void);

extern void W25QXX_WAKEUP(void);
extern void PlayerStartCallback(void);
extern void PlayerStopCallback(void);

static void PlayerSetSampleRate(uint32_t audioFreq)
{
#if 1
	pI2s->Instance = SPI2;
	pI2s->Init.Mode = I2S_MODE_MASTER_TX;
	pI2s->Init.Standard = I2S_STANDARD_PHILIPS;
	pI2s->Init.DataFormat = I2S_DATAFORMAT_16B;
	pI2s->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	//pI2s->Init.AudioFreq = I2S_AUDIOFREQ_8K;
	pI2s->Init.AudioFreq = audioFreq;

	pI2s->Init.CPOL = I2S_CPOL_LOW;
	if (HAL_I2S_Init(pI2s) != HAL_OK)
	{
		Error_Handler();
	}
#endif
}

static void PlayStream(uint16_t* pBuffer, uint32_t Size)
{
	//HAL_I2S_Transmit_DMA(pI2s, pBuffer, DMA_MAX(Size / AUDIODATA_SIZE)); 
	HAL_I2S_Transmit_DMA(pI2s, pBuffer, Size);
}

static int PlayFirstFrame(void)
{
	int rc = -1;
	unsigned int samples = 0;

	TSpiritMP3Info mp3Info;
	samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[0], BUFFER_SIZE/2, &mp3Info);

	if (samples < 1)
	{
		// Error, EOF
		return rc;
	}
	playing = 1;
	HAL_GPIO_WritePin(SPK_EN_GPIO_Port, SPK_EN_Pin, GPIO_PIN_SET);

	VolumeScale(audioBuffer, samples);	
	printf("Playing sample rate: %dHz, bitrate:%dkbps \r\n", mp3Info.nSampleRateHz, mp3Info.nBitrateKbps);

	//note: no need to consider about those 'Variable Bit Rate (VBR) files', this demo decoder version not supported
	PlayerSetSampleRate(mp3Info.nSampleRateHz / 2);
	PlayStream(audioBuffer, samples);

	return rc;
}

static unsigned int ReadData(void* data, unsigned int size, void* token)
{
	unsigned int read = 0;
	if (f_read(&playingFile, data, size, &read) != FR_OK)
	{
		return 0;
	}
	return read;
}

int PlayerStart(const char *fileName)
{
	int rc = 0;

	if (playing)
	{
		PlayerStop();
	}
	PlayerStartCallback();

	rc = f_open(&playingFile, fileName, FA_READ);

	if (rc == FR_OK)
	{
		/* Initialize MP3 decoder */
#if 1	//without callback
		SpiritMP3DecoderInit(&decoder, ReadData, NULL, NULL);
#else	//with callback
		SpiritMP3DecoderInit(&decoder, ReadData, DacProcess, NULL);
#endif
		PlayFirstFrame();

		sprintf(tmpstr, "%s", fileName);
		u8g2_DrawStr(&u8g2, 0, 58, tmpstr);

		u8g2_SendBuffer(&u8g2);
	}
	return rc;
}

void PlayerStop(void)
{
	playing = 0;
	HAL_I2S_DMAStop(pI2s);
	HAL_GPIO_WritePin(SPK_EN_GPIO_Port, SPK_EN_Pin, GPIO_PIN_RESET);
#ifdef DEBUG
	sprintf(tmpBuf, "Tick:%lu.\n\r", HAL_GetTick());
	HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif

	f_close(&playingFile);

	u8g2_ClearBuffer(&u8g2);
	u8g2_SendBuffer(&u8g2);
}

void PlayerUpdate(void)
{
	unsigned int samples = 0;

	switch (playerState)
	{
	case PLAYER_STATE_DMA_TX_COMPLETE:
	{
		playerState = PLAYER_STATE_IDLE;
		samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[BUFFER_SIZE/2], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			printf("Stop.\n\r");
			PlayerStop();
			PlayerStopCallback();
			break;
		}
		VolumeScale(&audioBuffer[BUFFER_SIZE/2], samples);
	}
		break;
	case PLAYER_STATE_DMA_TX_HALF:
	{
		playerState = PLAYER_STATE_IDLE;

#ifdef DEBUG
			uint32_t start = HAL_GetTick();
#endif
		samples = SpiritMP3Decode(&decoder, (short*)&audioBuffer[0], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			break;
		}
#ifdef DEBUG
		//printf("Read:%lu.\n\r", HAL_GetTick() - start);
		uint32_t endT = HAL_GetTick();
		sprintf(tmpBuf, "Re:%lu, tick:%lu.\n\r", endT - start, endT);
		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
		//start = HAL_GetTick();
#endif
		VolumeScale(&audioBuffer[0], samples);

#ifdef DEBUG
		//sprintf(tmpBuf, "Ve:%lu.\n\r", HAL_GetTick() - start);
		//HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)tmpBuf, strlen(tmpBuf));
#endif
	}
		break;
	default:
		break;
	}
}

void PlayerPause(void)
{
	HAL_I2S_DMAPause(pI2s);
}

void PlayerResume(void)
{
	HAL_I2S_DMAResume(pI2s);
}

void PlayerInit(I2S_HandleTypeDef* sI2s)
{
	pI2s = sI2s;
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s)
{
	playerState = PLAYER_STATE_DMA_TX_HALF;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s)
{
	playerState = PLAYER_STATE_DMA_TX_COMPLETE;

	//point to the head of the audio buffer
	PlayStream(audioBuffer, BUFFER_SIZE/2);
}

static void VolumeScale(uint16_t* pBuffer, unsigned int samples)
{
	for(int i = 0; i < samples * 2; i++)
	{
		int32_t val = (int16_t)*pBuffer;

		*pBuffer = val * volume / 100;
		pBuffer++;
	}
}

void PlayerVolumeAdj(int32_t vol)
{
	if (vol >= 0)
	{
		volume = vol;
	}
	else
	{
		if (volume > 10)
		{
			volume -= 10;
		}
		else
		{
			volume = 100;
		}
	}
}

void DacProcess(void* buff, int isShort, int ch, void* token)
{
	//nothing to do right this moment
}
