#include "main.h"
#include "printf.h"
#include "string.h"

void DispMenuCursor(uint8_t col, uint8_t line);

void DispClockSettingsCursor(uint8_t col, uint8_t line);

uint8_t *GetBatteryIcon(uint16_t adcVal);

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

void OnBtnDown(uint32_t btnVal);

void DispUpdate(void);
