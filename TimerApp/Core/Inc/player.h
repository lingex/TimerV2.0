
#ifndef __PLAYER_H_
#define __PLAYER_H_

#include "main.h"

#define BUFFER_SIZE		(576 * 8)	//in bytes

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

int  PlayerStart(const char *path);
void PlayerStop(void);
void PlayerPause(void);
void PlayerResume(void);
void PlayerUpdate(void);
void PlayerVolumeAdj(int32_t vol);

void PlayerInit(I2S_HandleTypeDef* sI2s);

void DacProcess(void* buff, int isShort, int ch, void* token);


#endif // __PLAYER_H_
