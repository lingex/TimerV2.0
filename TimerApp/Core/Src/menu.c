#include "menu.h"
#include "rx8025t.h"
#include "player.h"
#include "lcd.h"

extern uint32_t batVoltage;
extern _RTC rtc;
extern char tmpstr[];
extern char musicUsing[];
extern uint8_t musicVolume;
extern __IO uint8_t usbDet;
extern u8g2_t u8g2;

static uint8_t menuSelCur = 0;
static uint8_t menuVolume = 0;
static uint8_t lcdPower = 1;

static _RTC setRtc = {
	.Year = 22, .Month = 7, .Date = 3, .DaysOfWeek = SUNDAY, .Hour = 20, .Min = 37, .Sec = 22};

static _COUNTER counter = {
	.Hour = 0, .Min = 5, .Sec = 0};

const char *testWave = "test.MP3";
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
		"ESC  Reset Next Pause", // timer run
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

static uint8_t csPosVec[] = {12, 31, 48, 72, 90, 108};	//clock setting cursor
static uint8_t tsPosVec[] = {20, 60, 100};	//timer setting cursor


extern void SaveConfigs(void);
extern void LoadFileListing(void);

void DispMenuCursor(uint8_t x, uint8_t y)
{
	// u8g2_SetFont(&u8g2, u8g2_font_siji_t_6x10);
	// u8g2_DrawGlyph(&u8g2, x, y, 0xe062);
	u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf); //up arrow
	u8g2_SetFontDirection(&u8g2, 1);
	u8g2_DrawGlyph(&u8g2, x, y, 0x0062);
	u8g2_SetFontDirection(&u8g2, 0);
}

void DispClockSettingsCursor(uint8_t x, uint8_t y)
{
	// u8g2_SetFont(&u8g2, u8g2_font_siji_t_6x10);
	// u8g2_DrawGlyph(&u8g2, x, y, 0xe060);
	u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf);
	u8g2_DrawGlyph(&u8g2, x, y, 0x0062);
}

uint8_t GetBatteryIndex(uint32_t voltage)
{
	static uint8_t lastRtcSec;
	static uint8_t chargeStep;

	uint8_t index = 0;

	if (voltage > 4010)
	{
		index = 5;
	}
	else if (voltage > 3960)
	{
		index = 4;
	}
	else if (voltage > 3850)
	{
		index = 3;
	}
	else if (voltage > 3700)
	{
		index = 2;
	}
	else if (voltage > 3460)
	{
		index = 1;
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
			else if (chargeStep >= 5)
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
		// not enough space for this
		u8g2_SetFont(&u8g2, u8g2_font_siji_t_6x10);
		u8g2_DrawGlyph(&u8g2, 96, 8, 0xe00c); // USB

		//u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf);
		//u8g2_DrawGlyph(&u8g2, 96, 9, 0x0044); // correct symbol
		//u8g2_DrawGlyph(&u8g2, 96, 9, 0x0052); //

		//u8g2_SetFont(&u8g2, u8g2_font_m2icon_9_tf); //up arrow
		//u8g2_SetFontDirection(&u8g2, 1);
		//u8g2_DrawGlyph(&u8g2, 97, 1, 0x0062);
		u8g2_SetFontDirection(&u8g2, 0);
	}

	// line
	u8g2_DrawLine(&u8g2, 0, 11, 127, 11);

	// battery
	u8g2_SetFont(&u8g2, u8g2_font_battery19_tn);
	u8g2_SetFontDirection(&u8g2, 1);
	u8g2_DrawGlyph(&u8g2, 108, 1, 0x0030 + GetBatteryIndex(batVoltage));
	u8g2_SetFontDirection(&u8g2, 0);
}

void DispSleep()
{
	//u8g2_SetPowerSave(&u8g2, 1);
	//lcdPower = 0;
	//return;
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
	for (size_t i = 0; i < musicSize; i++)
	{
		sprintf(tmpstr, "%s", musicList[i]);
		u8g2_DrawStr(&u8g2, posX, posY + i * 8, tmpstr);
	}

	sprintf(tmpstr, "%s", btnMenuStr[devState]);
	u8g2_DrawStr(&u8g2, 0, 63, tmpstr);
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

	sprintf(tmpstr, "Hardware: %s", HARDWARE_VERSION);
	u8g2_DrawStr(&u8g2, 0, 24, tmpstr);
	sprintf(tmpstr, "Firmware: %s", FIRMWARE_VERSION);
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

void OnBtnDown(uint32_t btnVal)
{
	switch (devState)
	{
	case DevStateSleep:
	case DevStateStandby:
	{
		PlayerStop();
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
	break;
	case DevStateMenuMain:
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
				devState = DevStateMenuMusic;
				LoadFileListing();
				menuSelCur = 0;
				PlayerStart(musicList[menuSelCur]); // try me
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
	break;
	case DevStateTimerSet:
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
			//counter.Hour = 0;
			//counter.Min = 0;
			//counter.Sec = 0;
		}
	}
	break;
	case DevStateTimerPause:
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
	break;
	case DevStateTimerRun:
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
	break;
	case DevStateAlarm:
	{
		// any key will stop it
		PlayerStop();
		if ((btnVal & BTN_VAL_GO) != 0) // GO
		{
			devState = DevStateStandby;
		}
		if ((btnVal & BTN_VAL_ESC) != 0)
		{
			devState = DevStateTimerSet;
		}
	}
	break;
	case DevStateMenuClock:
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
			// cancel
			devState = DevStateStandby;
		}
	}
	break;
	case DevStateMenuMusic:
	{
		PlayerStop();
		if ((btnVal & BTN_VAL_ESC) != 0)
		{
			devState = DevStateMenuMain;
			menuSelCur = 1;
		}
		if ((btnVal & BTN_VAL_UP) != 0)
		{
			if (menuSelCur > 0)
			{
				menuSelCur--;
			}
			else
			{
				menuSelCur = musicSize - 1;
			}
			PlayerStart(musicList[menuSelCur]); // try me
		}
		if ((btnVal & BTN_VAL_DOWN) != 0)
		{
			if (menuSelCur < musicSize - 1)
			{
				menuSelCur++;
			}
			else
			{
				menuSelCur = 0;
			}
			PlayerStart(musicList[menuSelCur]); // try me
		}
		if ((btnVal & BTN_VAL_GO) != 0) // GO
		{
			if (musicSize > 0 && menuSelCur < musicSize)
			{
				strcpy(musicUsing, musicList[menuSelCur]);
				SaveConfigs();
			}
			devState = DevStateMenuMain;
			menuSelCur = 1;
		}
	}
	break;
	case DevStateMenuVolume:
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
			PlayerVolumeAdj(menuVolume);
			PlayerStart(testWave); // try me
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
			PlayerVolumeAdj(menuVolume);
			PlayerStart(testWave); // try me
		}
		if ((btnVal & BTN_VAL_GO) != 0) // GO
		{
			musicVolume = menuVolume;
			devState = DevStateMenuMain;
			menuSelCur = 2;
			//WriteEEPROM(SETTING_MUSIC_VOLUME_ADDR, musicVolume);
			PlayerVolumeAdj(musicVolume);
			SaveConfigs();
		}
	}
	break;
	case DevStateMenuVersion:
	{
		devState = DevStateMenuMain;
		menuSelCur = 3;
	}
	break;
	case DevStateUsbMode:
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
	break;
	default:
		devState = DevStateStandby;
		break;
	}
}

void DispUpdate(void)
{
	u8g2_ClearBuffer(&u8g2);

	if (devState != DevStateSleep && lcdPower == 0 && 0)
	{
		lcdPower = 1;
		u8g2_SetPowerSave(&u8g2, 0);
	}

	switch (devState)
	{
	case DevStateSleep:
	{
		DispSleep();
	}
	break;
	case DevStateStandby:
	{
		DispStandby();
	}
	break;
	case DevStateTimerSet:
	{
		DispTimerSettings();
	}
	break;
	case DevStateTimerPause:
	{
		DispTimerPause();
	}
	break;
	case DevStateTimerRun:
	{
		DispTimerRun();
	}
	break;
	case DevStateAlarm:
	{
		DispAlarm();
	}
	break;
	case DevStateMenuMain:
	{
		DispMenu();
	}
	break;
	case DevStateMenuClock:
	{
		DispClockSettings();
	}
	break;
	case DevStateMenuMusic:
	{
		DispMusicSettings();
	}
	break;
	case DevStateMenuVolume:
	{
		DispVolumeSettings();
	}
	break;
	case DevStateMenuVersion:
	{
		DispInfo();
	}
	break;
	case DevStateUsbMode:
	{
		DispUsbSetings();
	}
	break;
	default:
		break;
	}
	u8g2_SendBuffer(&u8g2);
}
