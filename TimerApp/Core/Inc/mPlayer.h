#ifndef __MPLAYER_H__
#define __MPLAYER_H__


#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus

#include <string>

#include "main.h"
#include "spiritMP3Dec.h"

using namespace std;

#define BUFFER_SIZE (576 * 8)	//in bytes

#define AUDIODATA_SIZE      2

#define DMA_MAX_SZE                     0xFFFF
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

typedef enum
{
	PLAYER_STATE_IDLE,
	PLAYER_STATE_DMA_TX_COMPLETE,
	PLAYER_STATE_DMA_TX_HALF,
}PlayerState_t;

class Player
{
private:
	I2S_HandleTypeDef* m_pI2s;
	bool m_busy;
	uint8_t m_volume;
	PlayerState_t m_state;
	TSpiritMP3Decoder m_decoder;
	uint16_t m_buff[BUFFER_SIZE];

private:
	//unsigned int ReadData(void* data, unsigned int size, void* token);
	int PlayFirstFrame();
	void PlayStream(uint16_t* pBuffer, uint32_t Size);
	void VolumeScale(uint16_t* pBuffer, unsigned int samples);
	void SetSampleRate(uint32_t audioFreq);

public:
	Player(I2S_HandleTypeDef* i2s);
	~Player();

	void Play(const char* fileName);
	void Stop();
	bool IsBusy();
	void Pause();
	void Resume();
	void Tick();
	void SetVolume(uint8_t vol);
	void TransmitHalfCpltCallback();
	void TransmitCpltCallback();
};

#endif

//interfaces for C

#ifdef __cplusplus
extern "C" 
{
#endif

void* CreatePlayer(I2S_HandleTypeDef* i2s);
void DestroyPlayer(void *pPlayer);
void PlayerPlay(void *pPlayer, const char* fileName);
void PlayerStop(void *pPlayer);
bool PlayerIsBusy(void *pPlayer);
void PlayerPause(void *pPlayer);
void PlayerResume(void *pPlayer);
void PlayerTick(void *pPlayer);
void PlayerSetVolume(void *pPlayer, uint8_t vol);
void PlayerI2sTxHalfCpltCallback(void *pPlayer);
void PlayerI2sTxCpltCallback(void *pPlayer);

#ifdef __cplusplus
}
#endif


#endif	//end of __MPLAYER_H__