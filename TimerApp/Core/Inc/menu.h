#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

typedef void(*DISPACTION)(void);

void DispMenuCursor(uint8_t col, uint8_t line);

void DispClockSettingsCursor(uint8_t col, uint8_t line);

uint8_t *GetBatteryIcon(uint32_t voltage);

void DispCommonItems();

void DispSleep();

void DispStandby();

void DispTimerRun();

void DispTimerPause();

void DispAlarm();

void DispTimerSettings();

void DispClockSettings();

void DispMenu();

void DispMusicSettings();

void DispVolumeSettings();

void DispInfo();

void DispUsbSetings();

void BtnActionOnStandby(uint32_t btnVal);
void BtnActionOnMenuMain(uint32_t btnVal);
void BtnActionOnTimerSet(uint32_t btnVal);
void BtnActionOnTimerPause(uint32_t btnVal);
void BtnActionOnTimerRun(uint32_t btnVal);
void BtnActionOnAlarm(uint32_t btnVal);
void BtnActionOnMenuClock(uint32_t btnVal);
void BtnActionOnMenuMusic(uint32_t btnVal);
void BtnActionOnMenuVolume(uint32_t btnVal);
void BtnActionOnMenuVersion(uint32_t btnVal);
void BtnActionOnUsbMode(uint32_t btnVal);

void OnBtnDown(uint32_t btnVal);

void DispUpdate(void);

void LoadMp3File(void);

#ifdef __cplusplus
}
#endif
