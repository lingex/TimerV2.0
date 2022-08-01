#include "player.h"
#include "ff.h"
#include "printf.h"
#include "spi_flash.h"

FIL m_file;

Player::Player(I2S_HandleTypeDef* i2s)
	:m_pI2s(i2s)
	,m_busy(false)
	,m_volume(50)
	,m_state(PLAYER_STATE_IDLE)
{
}

Player::~Player()
{
}

unsigned int ReadData(void* data, unsigned int size, void* token)
{
	unsigned int read = 0;
	if (f_read(&m_file, data, size, &read) != FR_OK)
	{
		return 0;
	}
	return read;
}

void Player::SetSampleRate(uint32_t audioFreq)
{
	uint32_t freq = audioFreq / 2;

	m_pI2s->Instance = SPI2;
	m_pI2s->Init.Mode = I2S_MODE_MASTER_TX;
	m_pI2s->Init.Standard = I2S_STANDARD_PHILIPS;
	m_pI2s->Init.DataFormat = I2S_DATAFORMAT_16B;
	m_pI2s->Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	//m_pI2s->Init.AudioFreq = I2S_AUDIOFREQ_8K;
	m_pI2s->Init.AudioFreq = freq;

	m_pI2s->Init.CPOL = I2S_CPOL_LOW;
	if (HAL_I2S_Init(m_pI2s) != HAL_OK)
	{
		Error_Handler();
	}
}

int Player::PlayFirstFrame()
{
	int rc = -1;
	unsigned int samples = 0;

	TSpiritMP3Info mp3Info;
	samples = SpiritMP3Decode(&m_decoder, (short*)&m_buff[0], BUFFER_SIZE/2, &mp3Info);

	if (samples < 1)
	{
		// Error, EOF
		return rc;
	}
	m_busy = true;
	SPEAKER_ON;

	VolumeScale(m_buff, samples);
	printf("Playing sample rate: %dHz, bitrate:%dkbps \r\n", mp3Info.nSampleRateHz, mp3Info.nBitrateKbps);

	//note: no need to consider about those 'Variable Bit Rate (VBR) files', this demo decoder version not supported
	SetSampleRate(mp3Info.nSampleRateHz);

	PlayStream(m_buff, samples);

	return rc;
}
void Player::PlayStream(uint16_t* pBuffer, uint32_t Size)
{
	//HAL_I2S_Transmit_DMA(m_pI2s, pBuffer, DMA_MAX(Size / AUDIODATA_SIZE)); 
	HAL_I2S_Transmit_DMA(m_pI2s, pBuffer, Size);
}

void Player::VolumeScale(uint16_t* pBuffer, unsigned int samples)
{
	for(unsigned int i = 0; i < samples * 2; i++)
	{
		int32_t val = (int16_t)*pBuffer;

		*pBuffer = val * m_volume / 100;
		pBuffer++;
	}
}

void Player::Play(const char* fileName)
{
	if (m_busy)
	{
		Stop();
	}
	PlayerStartCallback();
	int rc = f_open(&m_file, fileName, FA_READ);
	if (rc == FR_OK)
	{
		/* Initialize MP3 decoder */
#if 1	//without callback
		SpiritMP3DecoderInit(&m_decoder, ReadData, NULL, NULL);
#else	//with callback
		SpiritMP3DecoderInit(&decoder, ReadData, DacProcess, NULL);
#endif
		PlayFirstFrame();
	}
}

void Player::Stop()
{
	if (!m_busy)
	{
		return;
	}
	
	m_busy = false;
	HAL_I2S_DMAStop(m_pI2s);
	SPEAKER_OFF;
	f_close(&m_file);
}

bool Player::IsBusy()
{
	return m_busy;
}

void Player::Pause()
{
	HAL_I2S_DMAPause(m_pI2s);
}

void Player::Resume()
{
	HAL_I2S_DMAResume(m_pI2s);
}

void Player::Tick()
{
	unsigned int samples = 0;

	switch (m_state)
	{
	case PLAYER_STATE_DMA_TX_COMPLETE:
	{
		m_state = PLAYER_STATE_IDLE;
		samples = SpiritMP3Decode(&m_decoder, (short*)&m_buff[BUFFER_SIZE/2], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			printf("Stop.\n\r");
			Stop();
			PlayerStopCallback();
			break;
		}
		VolumeScale(&m_buff[BUFFER_SIZE/2], samples);
	}
		break;
	case PLAYER_STATE_DMA_TX_HALF:
	{
		m_state = PLAYER_STATE_IDLE;

#ifdef DEBUG
			uint32_t start = HAL_GetTick();
#endif
		samples = SpiritMP3Decode(&m_decoder, (short*)&m_buff[0], BUFFER_SIZE/4, NULL);
		if (samples == 0)
		{
			break;
		}
#ifdef DEBUG
		//printf("Read:%lu.\n\r", HAL_GetTick() - start);
		uint32_t endT = HAL_GetTick();
		printf(tmpBuf, "Re:%lu, tick:%lu.\n\r", endT - start, endT);
		start = HAL_GetTick();
#endif
		VolumeScale(&m_buff[0], samples);

#ifdef DEBUG
		printf(tmpBuf, "Ve:%lu.\n\r", HAL_GetTick() - start);
#endif
	}
		break;
	default:
		break;
	}
}

void Player::SetVolume(uint8_t vol)
{
	m_volume = vol;
}

void Player::TransmitHalfCpltCallback()
{
	m_state = PLAYER_STATE_DMA_TX_HALF;
}
void Player::TransmitCpltCallback()
{
	m_state = PLAYER_STATE_DMA_TX_COMPLETE;
	//point to the head of the audio buffer
	PlayStream(m_buff, BUFFER_SIZE/2);
}


//C interfaces
void* CreatePlayer(I2S_HandleTypeDef* i2s)
{
	//Player *out(new Player(i2s));
	//return (reinterpret_cast< void* >(out));
	static Player out(i2s);
	return &out;
}
void DestroyPlayer(void* pPlayer)
{
	//delete(reinterpret_cast<Player*>(pPlayer));
}
void PlayerPlay(void *pPlayer, const char* fileName)
{
	((Player*)pPlayer)->Play(fileName);
}
void PlayerStop(void *pPlayer)
{
	((Player*)pPlayer)->Stop();
}
bool PlayerIsBusy(void *pPlayer)
{
	return ((Player*)pPlayer)->IsBusy();
}
void PlayerPause(void *pPlayer)
{
	((Player*)pPlayer)->Pause();
}
void PlayerResume(void *pPlayer)
{
	((Player*)pPlayer)->Resume();
}
void PlayerTick(void *pPlayer)
{
	((Player*)pPlayer)->Tick();
}
void PlayerSetVolume(void *pPlayer, uint8_t vol)
{
	((Player*)pPlayer)->SetVolume(vol);
}

void PlayerI2sTxHalfCpltCallback(void *pPlayer)
{
	((Player*)pPlayer)->TransmitHalfCpltCallback();
}
void PlayerI2sTxCpltCallback(void *pPlayer)
{
	((Player*)pPlayer)->TransmitCpltCallback();
}