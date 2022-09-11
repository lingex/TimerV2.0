#include "menu.h"
#include "rx8025t.h"
#include "player.h"
#include "lcd.h"
#include <vector>
#include <map>
#include <string>
#include "printf.h"
#include "ff.h"
#include "spi_flash.h"

using namespace std;


extern uint32_t batVoltage;
extern _RTC rtc;
extern char tmpstr[];
extern char musicUsing[];
extern uint8_t musicVolume;
extern __IO uint8_t usbDet;
extern u8g2_t u8g2;
extern Player player;

static uint8_t menuSelCur = 0;
static uint8_t menuVolume = 0;
static uint8_t musicOffset = 0;

static _RTC setRtc = {
	.Year = 22, .Month = 7, .Date = 3, .DaysOfWeek = SUNDAY, .Hour = 20, .Min = 37, .Sec = 22};

static _COUNTER counter = {
	.Hour = 0, .Min = 5, .Sec = 0};

const char *testFile = "test.MP3";
const char *dowStr[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Err"};


// title text in the middle
static const char *menuStr[] =
	{
		"",		  // sleep
		"",		  // standby
		"TIMER",  // timerSet
		"TIMER",  // timerPause
		"TIMER",  // timerRun
		"ALARM",  // alarm
		"MENU",	  // menu
		"TIME",	  // timeMenu
		"MUSIC",  // musicMenu
		"VOLUME", // volumeMenu
		"INFO",	  // version
		"USB",	  // mode select
};

// bottom button explain text
static const char *btnMenuStr[] =
	{
		"",						 // sleep
		"",						 // standby
		"ESC  Reset Next Start", // timer settings
		"ESC  Reset Next Cont.", // timer pause
		"ESC             Pause", // timer run
		"Esc                OK", // alarm
		"Esc                OK", // menu
		"Esc                OK", // time settings
		"Esc                OK", // music settings
		"Esc                OK", // volume settings
		"Esc                OK", // version info
		"Esc                OK", // usb mode
};

#define MAIN_MENU_SIZE 5
// main menu item text (less or equal than 5)
static const char *mainMenuItems[] =
	{
		"1.Time",	  // change the RTC
		"2.Music",	  // change the alarm music
		"3.Volume",	  // change the music volume
		"4.About",	  // firmware version
		"5.USB Mode", // Disk mode
};

#if DISPLAY_OFF_WHILE_SLEEP
static uint8_t lcdPower = 1;
#endif

static uint8_t csPosVec[] = {13, 32, 50, 74, 92, 109};	//clock setting cursor
static uint8_t tsPosVec[] = {20, 60, 100};	//timer setting cursor

const uint8_t musicDispMax = 5;
vector<string> musicVec;

//<devState, function>
const map<uint8_t, DISPACTION> dispProc = {
	{ DevStateSleep,		DispSleep },
	{ DevStateStandby,		DispStandby },
	{ DevStateTimerSet,		DispTimerSettings },
	{ DevStateTimerPause,	DispTimerPause },
	{ DevStateTimerRun,		DispTimerRun },
	{ DevStateAlarm,		DispAlarm },
	{ DevStateMenuMain,		DispMenu },
	{ DevStateMenuClock,	DispClockSettings },
	{ DevStateMenuMusic,	DispMusicSettings },
	{ DevStateMenuVolume,	DispVolumeSettings },
	{ DevStateMenuVersion,	DispInfo },
	{ DevStateUsbMode,		DispUsbSetings },
};
const vector<uint32_t> batVolStep = { 3450, 3700, 3850, 3950, 4050 };


void DispMenuCursor(uint8_t x, uint8_t y)
{
	u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf); //up arrow
	u8g2_SetFontDirection(&u8g2, 1);
	u8g2_DrawGlyph(&u8g2, x, y, 0x0062);
	u8g2_SetFontDirection(&u8g2, 0);
}

void DispClockSettingsCursor(uint8_t x, uint8_t y)
{
	u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf);
	u8g2_DrawGlyph(&u8g2, x, y, 0x0062);
}

uint8_t GetBatteryIndex(uint32_t voltage)
{
	static uint8_t lastRtcSec;
	static uint8_t chargeStep;

	uint8_t index = 0;

	for (auto vol : batVolStep)
	{
		if (voltage < vol)
		{
			break;
		}
		index++;
	}

	if (usbDet == 1)
	{
		if (rtc.Sec != lastRtcSec)
		{
			lastRtcSec = rtc.Sec;
			if (chargeStep < index)
			{
				chargeStep = index;
			}
			else if (chargeStep >= batVolStep.size())
			{
				chargeStep = index;
			}
			else
			{
				chargeStep++;
			}
		}
		index = chargeStep;
	}

	return index;
}

void DispCommonItems()
{
	sprintf(tmpstr, "%s", menuStr[devState]);
	u8g2_SetFont(&u8g2, u8g2_font_7x14B_tr);
	u8g2_DrawStr(&u8g2, 50, 10, tmpstr);

	// small time
	sprintf(tmpstr, "%02d:%02d", rtc.Hour, rtc.Min);
	u8g2_DrawStr(&u8g2, 0, 10, tmpstr);

	if (usbDet == 1)
	{
		u8g2_SetFont(&u8g2, u8g2_font_siji_t_6x10);
		u8g2_DrawGlyph(&u8g2, 96, 8, 0xe00c); // USB
		u8g2_SetFontDirection(&u8g2, 0);
	}

	// line
	u8g2_DrawLine(&u8g2, 0, 11, 127, 11);

	// battery
#if 1
	u8g2_SetFont(&u8g2, u8g2_font_battery19_tn);
	u8g2_SetFontDirection(&u8g2, 1);
	u8g2_DrawGlyph(&u8g2, 108, 1, 0x0030 + GetBatteryIndex(batVoltage));
	u8g2_SetFontDirection(&u8g2, 0);
#else 
	sprintf(tmpstr, "%lu", batVoltage);
	u8g2_DrawStr(&u8g2, 100, 10, tmpstr);
#endif
}

void DispSleep()
{
#if DISPLAY_OFF_WHILE_SLEEP
	if (usbDet == 0)
	{
		u8g2_SetPowerSave(&u8g2, 1);
		lcdPower = 0;
		return;
	}
#endif
/*
  11:30  Zzz 4.17V
  ----------------
	  11:30

  ----------------
  2021-11-14   Sun
  */
	DispCommonItems();

	sprintf(tmpstr, "%02d:%02d", rtc.Hour, rtc.Min);
	u8g2_SetFont(&u8g2, u8g2_font_osb26_tr);
	u8g2_DrawStr(&u8g2, 18, 44, tmpstr);

	sprintf(tmpstr, "20%02d-%02d-%02d", rtc.Year, rtc.Month, rtc.Date);
	// u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_SetFont(&u8g2, u8g2_font_7x14B_tr);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);

#if 1	//week str
	sprintf(tmpstr, "%s", dowStr[rtc.DaysOfWeek]);
	u8g2_DrawStr(&u8g2, 106, 63, tmpstr);
#else	//batval
	sprintf(tmpstr, "%umV", batVoltage);
	u8g2_DrawStr(&u8g2, 86, 63, tmpstr);
#endif
	sprintf(tmpstr, (usbDet == 1) ? "ZZz" : "Zzz");
	u8g2_DrawStr(&u8g2, 58, 10, tmpstr);
}

void DispStandby()
{
	/*
  11:30      4.17V
  ----------------
	  11:30

  ----------------
  2021-11-14   Sun
  */
	DispCommonItems();

	sprintf(tmpstr, "%02d:%02d", rtc.Hour, rtc.Min);
	u8g2_SetFont(&u8g2, u8g2_font_osb26_tr);
	u8g2_DrawStr(&u8g2, 18, 44, tmpstr);

	sprintf(tmpstr, "20%02d-%02d-%02d", rtc.Year, rtc.Month, rtc.Date);
	u8g2_SetFont(&u8g2, u8g2_font_7x14B_tr);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);

	sprintf(tmpstr, "%s", dowStr[rtc.DaysOfWeek]);
	u8g2_DrawStr(&u8g2, 106, 63, tmpstr);
}

void DispTimerRun()
{
	/*
  11:30  TIMER 4.17V
  ----------------
	 00:11:30

  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	sprintf(tmpstr, "%02d:%02d:%02d", runCounter.Hour, runCounter.Min, runCounter.Sec);
	u8g2_SetFont(&u8g2, u8g2_font_osb21_tr);
	u8g2_DrawStr(&u8g2, 8, 44, tmpstr);

	sprintf(tmpstr, "20%02d-%02d-%02d", rtc.Year, rtc.Month, rtc.Date);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 0, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispTimerPause()
{
	/*
  11:30  TIMER 4.17V
  ----------------
	 00:11:30

  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();
	uint8_t pos = tsPosVec[menuSelCur];
	DispClockSettingsCursor(pos, 54);

	sprintf(tmpstr, "%02d:%02d:%02d", runCounter.Hour, runCounter.Min, runCounter.Sec);
	u8g2_SetFont(&u8g2, u8g2_font_osb21_tr);
	u8g2_DrawStr(&u8g2, 8, 44, tmpstr);

	sprintf(tmpstr, "20%02d-%02d-%02d", rtc.Year, rtc.Month, rtc.Date);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 0, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispTimerSettings()
{
	/*
  11:30  TIMER 4.17V
  ----------------
	 00:11:30

  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	uint8_t pos = tsPosVec[menuSelCur];
	DispClockSettingsCursor(pos, 54);

	sprintf(tmpstr, "%02d:%02d:%02d", counter.Hour, counter.Min, counter.Sec);
	u8g2_SetFont(&u8g2, u8g2_font_osb21_tr);
	u8g2_DrawStr(&u8g2, 8, 44, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispAlarm()
{
	/*
  11:30  TIMER 4.17V
  ----------------
	 00:11:30

  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	sprintf(tmpstr, "Playing:");
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 24, tmpstr);
	sprintf(tmpstr, "%s", musicUsing);
	u8g2_DrawStr(&u8g2, 0, 44, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispClockSettings()
{
	/*
  11:30  TIME  4.17V
  ----------------
  2021-11-13 00:11:30

  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	uint8_t pos = csPosVec[menuSelCur];
	DispClockSettingsCursor(pos, 35);

	sprintf(tmpstr, "20%02d-%02d-%02d  %02d:%02d:%02d", setRtc.Year, setRtc.Month, setRtc.Date, setRtc.Hour, setRtc.Min, setRtc.Sec);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 24, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispMenu()
{
	/*
  11:30  MENU 4.17V
  ----------------
	TIME
	MUSIC
	VOLUME
	EXIT
  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	uint8_t posX = 14;
	uint8_t posY = 22;
	DispMenuCursor(posX - 12, 14 + menuSelCur * 8);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	for (int i = 0; i < MAIN_MENU_SIZE; i++)
	{
		sprintf(tmpstr, "%s", mainMenuItems[i]);
		u8g2_DrawStr(&u8g2, posX, posY + i * 8, tmpstr);
	}

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispMusicSettings()
{
	/*
  11:30  MUSIC 4.17V
  ----------------
	Swan Lake
	~
  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();

	uint8_t posX = 14;
	uint8_t posY = 22;
	DispMenuCursor(posX - 12, 14 + menuSelCur * 8);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	uint8_t mSize = MATH_MIN(musicDispMax, (uint8_t)musicVec.size());
	for (size_t i = 0; i < mSize; i++)
	{
		sprintf(tmpstr, "%d.%s", i + musicOffset + 1, musicVec[i + musicOffset].c_str());
		u8g2_DrawStr(&u8g2, posX, posY + i * 8, tmpstr);
	}

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);

	sprintf(tmpstr, "%d / %d", menuSelCur + musicOffset + 1, (int)musicVec.size());
	u8g2_DrawStr(&u8g2, 50, 63, tmpstr);
}

void DispVolumeSettings()
{
	/*
  11:30  VOLUME 4.17V
  ----------------
		  80
  ----------------
  BOTTOM    STRING
  */
	DispCommonItems();
	u8g2_SetFont(&u8g2, u8g2_font_osb26_tr);

	sprintf(tmpstr, "%d", menuVolume);
	u8g2_DrawStr(&u8g2, 44, 44, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispInfo()
{
	/*
  11:30  INFO 4.17V
  ----------------
	Hardware: V1.1
	Fireware: V1.2

  BOTTOM    STRING
  */
	DispCommonItems();
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

	sprintf(tmpstr, "Hw: %s", HARDWARE_VERSION);
	u8g2_DrawStr(&u8g2, 0, 24, tmpstr);
	sprintf(tmpstr, "Fw: %s", FIRMWARE_VERSION);
	u8g2_DrawStr(&u8g2, 68, 24, tmpstr);
	
	sprintf(tmpstr, "Bat: %u mV", batVoltage);
	u8g2_DrawStr(&u8g2, 0, 34, tmpstr);

	sprintf(tmpstr, "Built:");
	u8g2_DrawStr(&u8g2, 0, 44, tmpstr);
	sprintf(tmpstr, "%s", builtTime);
	u8g2_DrawStr(&u8g2, 0, 54, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void DispUsbSetings()
{
	DispCommonItems();
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

	sprintf(tmpstr, "Enter DFU mode");
	u8g2_DrawStr(&u8g2, 20, 40, tmpstr);

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
}

void BtnActionOnStandby(uint32_t btnVal)
{
	player.Stop();
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		menuSelCur = 0;
		devState = DevStateMenuMain;
	}
	else
	{
		devState = DevStateTimerSet;
		menuSelCur = 1;	//timer default point to the min
	}
}
void BtnActionOnMenuMain(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		switch (menuSelCur)
		{
		case 0:
			devState = DevStateMenuClock;
			setRtc = rtc;
			break;
		case 1:
		{
			devState = DevStateMenuMusic;
			LoadMp3File();
			menuSelCur = 0;
			musicOffset = 0;
			string usingStr = string(musicUsing);
			uint8_t index = 0;
			for (auto music : musicVec)
			{
				if (music == usingStr)
				{
					if (index >= musicDispMax)
					{
						menuSelCur = musicDispMax - 1;		//stick at the last pos
						musicOffset = index - musicDispMax + 1;
					}
					else
					{
						menuSelCur = index;
					}
					break;
				}
				index++;
			}
			player.Play(musicVec[menuSelCur + musicOffset].c_str()); // try me
		}
			break;
		case 2:
			devState = DevStateMenuVolume;
			menuVolume = musicVolume;
			break;
		case 3:
			devState = DevStateMenuVersion;
			break;
		case 4:
			devState = DevStateUsbMode;
			break;
		default:
			devState = DevStateStandby;
			break;
		}
		// menuSelCur = 0;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		if (menuSelCur > 0)
		{
			menuSelCur--;
		}
		else
		{
			menuSelCur = MAIN_MENU_SIZE - 1;
		}
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		if (menuSelCur < MAIN_MENU_SIZE - 1)
		{
			menuSelCur++;
		}
		else
		{
			menuSelCur = 0;
		}
	}
	if ((btnVal & BTN_VAL_ESC) != 0) // ESC
	{
		devState = DevStateStandby;
	}
}
void BtnActionOnTimerSet(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		if (counter.Hour == 0 && counter.Min == 0 && counter.Sec == 0)
		{
			// not valid time
			devState = DevStateStandby;
		}
		else
		{
			devState = DevStateTimerRun;
			runCounter = counter;
		}
	}
	if ((btnVal & BTN_VAL_RIGHT) != 0)
	{
		if (menuSelCur < 2)
		{
			menuSelCur++;
		}
		else
		{
			menuSelCur = 0;
		}
	}
	if ((btnVal & BTN_VAL_LEFT) != 0)
	{
		counter.Hour = 0;
		counter.Min = 0;
		counter.Sec = 0;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			counter.Hour = counter.Hour < 99 ? counter.Hour + 1 : 0;
			break;
		case 1:
			counter.Min = counter.Min < 59 ? counter.Min + 1 : 0;
			break;
		case 2:
			counter.Sec = counter.Sec < 59 ? counter.Sec + 1 : 0;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			counter.Hour = counter.Hour > 0 ? counter.Hour - 1 : 99;
			break;
		case 1:
			counter.Min = counter.Min > 0 ? counter.Min - 1 : 59;
			break;
		case 2:
			counter.Sec = counter.Sec > 0 ? counter.Sec - 1 : 59;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_ESC) != 0) // RESET
	{
		devState = DevStateStandby;
	}
}
void BtnActionOnTimerPause(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		devState = DevStateTimerRun;
	}
	if ((btnVal & BTN_VAL_RIGHT) != 0)
	{
		if (menuSelCur < 2)
		{
			menuSelCur++;
		}
		else
		{
			menuSelCur = 0;
		}
	}
	if ((btnVal & BTN_VAL_LEFT) != 0)
	{
		runCounter.Hour = 0;
		runCounter.Min = 0;
		runCounter.Sec = 0;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			runCounter.Hour = runCounter.Hour < 99 ? runCounter.Hour + 1 : 0;
			break;
		case 1:
			runCounter.Min = runCounter.Min < 59 ? runCounter.Min + 1 : 0;
			break;
		case 2:
			runCounter.Sec = runCounter.Sec < 59 ? runCounter.Sec + 1 : 0;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			runCounter.Hour = runCounter.Hour > 0 ? runCounter.Hour - 1 : 99;
			break;
		case 1:
			runCounter.Min = runCounter.Min > 0 ? runCounter.Min - 1 : 59;
			break;
		case 2:
			runCounter.Sec = runCounter.Sec > 0 ? runCounter.Sec - 1 : 59;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_ESC) != 0) // RESET
	{
		devState = DevStateStandby;
	}
}
void BtnActionOnTimerRun(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		menuSelCur = 1;
		devState = DevStateTimerPause;
	}
	if ((btnVal & BTN_VAL_RIGHT) != 0)
	{
		if (menuSelCur < 2)
		{
			menuSelCur++;
		}
		else
		{
			menuSelCur = 0;
		}
	}
	if ((btnVal & BTN_VAL_LEFT) != 0)
	{
		counter.Hour = 0;
		counter.Min = 0;
		counter.Sec = 0;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			counter.Hour = counter.Hour < 99 ? counter.Hour + 1 : 0;
			break;
		case 1:
			counter.Min = counter.Min < 59 ? counter.Min + 1 : 0;
			break;
		case 2:
			counter.Sec = counter.Sec < 59 ? counter.Sec + 1 : 0;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			counter.Hour = counter.Hour > 0 ? counter.Hour - 1 : 99;
			break;
		case 1:
			counter.Min = counter.Min > 0 ? counter.Min - 1 : 59;
			break;
		case 2:
			counter.Sec = counter.Sec > 0 ? counter.Sec - 1 : 59;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		devState = DevStateStandby;
	}
}
void BtnActionOnAlarm(uint32_t btnVal)
{
	// any key will stop it
	player.Stop();
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		devState = DevStateStandby;
	}
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		devState = DevStateTimerSet;
	}
}
void BtnActionOnMenuClock(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		devState = DevStateStandby;
		setRtc.DaysOfWeek = CalcDaysOfWeek(setRtc.Year, setRtc.Month, setRtc.Date);
		RX8025T_SetTime(&setRtc);
	}
	if ((btnVal & BTN_VAL_RIGHT) != 0)
	{
		if (menuSelCur < 5)
		{
			menuSelCur++;
		}
		else
		{
			menuSelCur = 0;
		}
	}
	if ((btnVal & BTN_VAL_LEFT) != 0)
	{
		if (menuSelCur > 0)
		{
			menuSelCur--;
		}
		else
		{
			menuSelCur = 5;
		}
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			setRtc.Year = setRtc.Year < 99 ? setRtc.Year + 1 : 0;
			break;
		case 1:
			setRtc.Month = setRtc.Month < 12 ? setRtc.Month + 1 : 1;
			break;
		case 2:
			setRtc.Date = setRtc.Date < 31 ? setRtc.Date + 1 : 1;
			break;
		case 3:
			setRtc.Hour = setRtc.Hour < 23 ? setRtc.Hour + 1 : 0;
			break;
		case 4:
			setRtc.Min = setRtc.Min < 59 ? setRtc.Min + 1 : 0;
			break;
		case 5:
			setRtc.Sec = setRtc.Sec < 59 ? setRtc.Sec + 1 : 0;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		switch (menuSelCur)
		{
		case 0:
			setRtc.Year = setRtc.Year > 0 ? setRtc.Year - 1 : 99;
			break;
		case 1:
			setRtc.Month = setRtc.Month > 0 ? setRtc.Month - 1 : 12;
			break;
		case 2:
			setRtc.Date = setRtc.Date > 0 ? setRtc.Date - 1 : 31;
			break;
		case 3:
			setRtc.Hour = setRtc.Hour > 0 ? setRtc.Hour - 1 : 23;
			break;
		case 4:
			setRtc.Min = setRtc.Min > 0 ? setRtc.Min - 1 : 59;
			break;
		case 5:
			setRtc.Sec = setRtc.Sec > 0 ? setRtc.Sec - 1 : 59;
			break;
		default:
			break;
		}
	}
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		devState = DevStateMenuMain;
		menuSelCur = 0;
	}
}
void BtnActionOnMenuMusic(uint32_t btnVal)
{
	player.Stop();
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		devState = DevStateMenuMain;
		menuSelCur = 1;
		musicVec.clear();
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		if (menuSelCur > 0)
		{
			menuSelCur--;
		}
		else if (musicOffset > 0)
		{
			musicOffset--;
		}
		player.Play(musicVec[menuSelCur + musicOffset].c_str()); // try me
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		if (menuSelCur < musicDispMax - 1)
		{
			menuSelCur++;
		}
		else if(musicOffset + menuSelCur < musicVec.size() - 1)
		{
			musicOffset++;
		}
		player.Play(musicVec[menuSelCur + musicOffset].c_str()); // try me
	}
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		if (menuSelCur + musicOffset < musicVec.size())
		{
			strcpy(musicUsing, musicVec[menuSelCur + musicOffset].c_str());
			SaveConfigs();
		}
		devState = DevStateMenuMain;
		menuSelCur = 1;
		musicVec.clear();
	}
}
void BtnActionOnMenuVolume(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		// cancel
		devState = DevStateMenuMain;
		menuSelCur = 2;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{
		if (menuVolume < 100)
		{
			menuVolume += 10;
		}
		player.SetVolume(menuVolume);
		player.Play(testFile); // try me
	}
	if ((btnVal & BTN_VAL_DOWN) != 0)
	{
		if (menuVolume > 10)
		{
			if (menuVolume == 100)
			{
				u8g2_ClearBuffer(&u8g2);
				u8g2_SendBuffer(&u8g2);
			}
			menuVolume -= 10;
		}
		player.SetVolume(menuVolume);
		player.Play(testFile); // try me
	}
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		musicVolume = menuVolume;
		devState = DevStateMenuMain;
		menuSelCur = 2;
		player.Stop();
		player.SetVolume(musicVolume);
		SaveConfigs();
	}
}
void BtnActionOnMenuVersion(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_UP) != 0 || (btnVal & BTN_VAL_DOWN) != 0)
	{
		//wake up backlight only
		return;
	}
	devState = DevStateMenuMain;
	menuSelCur = 3;
}
void BtnActionOnUsbMode(uint32_t btnVal)
{
	if ((btnVal & BTN_VAL_GO) != 0) // GO
	{
		GoDfu();
	}
	if ((btnVal & BTN_VAL_ESC) != 0)
	{
		devState = DevStateMenuMain;
		menuSelCur = 4;
	}
	if ((btnVal & BTN_VAL_UP) != 0)
	{

	}
}

void OnBtnDown(uint32_t btnVal)
{
	switch (devState)
	{
	case DevStateSleep:
	case DevStateStandby:
		BtnActionOnStandby(btnVal);
	break;
	case DevStateMenuMain:
		BtnActionOnMenuMain(btnVal);
	break;
	case DevStateTimerSet:
		BtnActionOnTimerSet(btnVal);
	break;
	case DevStateTimerPause:
		BtnActionOnTimerPause(btnVal);
	break;
	case DevStateTimerRun:
		BtnActionOnTimerRun(btnVal);
	break;
	case DevStateAlarm:
		BtnActionOnAlarm(btnVal);
	break;
	case DevStateMenuClock:
		BtnActionOnMenuClock(btnVal);
	break;
	case DevStateMenuMusic:
		BtnActionOnMenuMusic(btnVal);
	break;
	case DevStateMenuVolume:
		BtnActionOnMenuVolume(btnVal);
	break;
	case DevStateMenuVersion:
		BtnActionOnMenuVersion(btnVal);
	break;
	case DevStateUsbMode:
		BtnActionOnUsbMode(btnVal);
	break;
	default:
		devState = DevStateStandby;
		break;
	}
}

void DispUpdate(void)
{
#if DISPLAY_OFF_WHILE_SLEEP
	if (devState != DevStateSleep && lcdPower == 0)
	{
		lcdPower = 1;
		u8g2_SetPowerSave(&u8g2, 0);
		return;
	}
#endif

	u8g2_ClearBuffer(&u8g2);

	auto it = dispProc.find(devState);
	if (it != dispProc.end())
	{
		//return (it->second)(lcdPower);
		(it->second)();
	}
	u8g2_SendBuffer(&u8g2);
}


//load mp3 file list
void LoadMp3File() 
{
	DIR rootdir;
	static FILINFO finfo;
	FRESULT res = FR_OK;

/*
When LFN feature is enabled, lfname and lfsize in the file information structure 
must be initialized with valid value prior to use the f_readdir function.
*/
	static char buff[_MAX_LFN];
	finfo.lfname = buff;
	finfo.lfsize = _MAX_LFN;

	musicVec.clear();

	uint8_t oPowerState = W25QXX_GetPowerState();
	if (oPowerState == 0)
	{
		W25QXX_WAKEUP();
	}

	if ((res = f_opendir(&rootdir, "/")) != FR_OK)
	{
		printf("Error opening root directory %d\r\n", res);
		return;
	}

	while (f_readdir(&rootdir, &finfo) == FR_OK)
	{
		if (finfo.fname[0] == '\0') break;
		if (finfo.fname[0] == '.') continue;

		if (finfo.fattrib & AM_DIR)
		{
			//printf("found directory %s\r\n", finfo.fname);
			continue;
		}
		//printf("found file %s\r\n", finfo.fname);

		if ((strstr(finfo.fname, ".mp3") || strstr(finfo.fname, ".MP3")) && !strstr(finfo.fname, "test.mp3"))
		{
			//printf("mp3 file: %s\r\n", finfo.lfname);
			if (finfo.lfsize == 0)
			{
				continue;
			}
			
			musicVec.push_back(finfo.lfname);
			if (musicVec.size() >= MUSIC_MAX)
			{
				break;
			}
		}
	}
	f_closedir(&rootdir);
	//free(buff);
	//printf("done reading rootdir\r\n");

	if (oPowerState == 0)
	{
		W25QXX_PowerDown();
	}
}