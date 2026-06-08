#include "config.h"
#include <string.h>
#include "ff.h"
#include "printf.h"
#include "cJSON.h"
#include "spi_flash.h"
//#include "cJSON_Utils.h"
#include <stdlib.h>

static char dataBuff[CONFIG_BUFF_SIZE];

extern uint8_t sleepDispEn;
extern uint8_t musicVolume;
extern char musicUsing[];
const char *defaultMusic = "test.mp3";
const char* configName = "config.json";

static void CopyMusicName(const char *name)
{
	snprintf(musicUsing, MUSIC_FILE_NAME_LEN, "%s", name != NULL ? name : "");
}

void LoadConfigs(void)
{
	CopyMusicName(defaultMusic);
	if (IsUsbMscActive())
	{
		return;
	}

	uint8_t oPowerState = W25QXX_GetPowerState();
	if (oPowerState == 0)
	{
		W25QXX_WAKEUP();
	}

	FIL cjfil;
	int rc = f_open(&cjfil, configName, FA_READ);
	UINT read = 0;
	if (rc != FR_OK)
	{
		if (oPowerState == 0)
		{
			W25QXX_PowerDown();
		}
		return;
	}

	if (f_read(&cjfil, dataBuff, CONFIG_BUFF_SIZE - 1, &read) == FR_OK)
	{
		dataBuff[read] = '\0';
		cJSON *json = cJSON_Parse(dataBuff);
		if (json != NULL)
		{
			//debug
			//char *string = cJSON_Print(json);
			//printf("%s", string);
			const cJSON *pVolume = cJSON_GetObjectItem(json, "volume");
			//if (pVolume != NULL && cJSON_IsNumber(pVolume))
			//{
			//	musicVolume = pVolume->valueint;
			//}
			if (pVolume != NULL && cJSON_IsString(pVolume))
			{
				int vol = atoi(pVolume->valuestring);
				if (vol < 0)
				{
					vol = 0;
				}
				else if (vol > 100)
				{
					vol = 100;
				}
				musicVolume = (uint8_t)vol;
			}
			const cJSON *pMusic = cJSON_GetObjectItem(json, "music");
			if (pMusic != NULL && cJSON_IsString(pMusic))
			{
				CopyMusicName(pMusic->valuestring);
				//musicUsing = pMusic->valuestring;
			}
			else
			{
				const cJSON *pDefault = cJSON_GetObjectItem(json, "default");
				if (pDefault != NULL && cJSON_IsString(pDefault))
				{
					CopyMusicName(pDefault->valuestring);
					//musicUsing = pDefault->valuestring;
				}
				else
				{
					CopyMusicName(defaultMusic);
					//musicUsing = defaultMusic;
				}
			}
			const cJSON *pDispEn = cJSON_GetObjectItem(json, "sleepDisp");
			if (pDispEn != NULL && cJSON_IsString(pDispEn))
			{
				sleepDispEn = strcmp(pDispEn->valuestring, "on") == 0 ? 1 : 0;
			}
		}
		cJSON_Delete(json);
	}
	f_close(&cjfil);
	if (oPowerState == 0)
	{
		W25QXX_PowerDown();
	}
}

void SaveConfigs(void)
{
	if (IsUsbMscActive())
	{
		return;
	}

	uint8_t oPowerState = W25QXX_GetPowerState();
	if (oPowerState == 0)
	{
		W25QXX_WAKEUP();
	}

	FIL cjfil;

	int rc = f_open(&cjfil, configName, FA_READ| FA_WRITE | FA_CREATE_ALWAYS);
	//int rc = f_open(&cjfil, "test.json", FA_READ| FA_WRITE | FA_CREATE_ALWAYS);
	if (rc == FR_OK)
	{
		cJSON *json = cJSON_CreateObject();
		if (json != NULL)
		{
			cJSON *pMusic = NULL;
			cJSON *pVol = NULL;
			cJSON *pDef = NULL;
			cJSON *pDispEn = NULL;

			pMusic = cJSON_CreateString(musicUsing);
			if (pMusic != NULL)
			{
				/* after creation was successful, immediately add it to the monitor,
				* thereby transferring ownership of the pointer to it */
				cJSON_AddItemToObject(json, "music", pMusic);
			}
			//pVol = cJSON_CreateNumber(musicVolume);
			//if (pVol != NULL)
			//{
			//	cJSON_AddItemToObject(json, "volume", pVol);
			//}
			char tmpStr[4] = {0};
			snprintf(tmpStr, sizeof(tmpStr), "%d", musicVolume);
			pVol = cJSON_CreateString(tmpStr);
			if (pVol != NULL)
			{
				cJSON_AddItemToObject(json, "volume", pVol);
			}
			pDef = cJSON_CreateString(defaultMusic);
			if (pDef != NULL)
			{
				cJSON_AddItemToObject(json, "default", pDef);
			}
			pDispEn = cJSON_CreateString(sleepDispEn == 1 ? "on" : "off");
			if (pDispEn != NULL)
			{
				cJSON_AddItemToObject(json, "sleepDisp", pDispEn);
			}

			char* out = cJSON_Print(json);
			if (out != NULL)
			{
				f_printf(&cjfil, "%s", out);
				free(out);
			}
			//f_puts(out, &cjfil);
			cJSON_Delete(json);
		}
		f_close(&cjfil);
	}
	if (oPowerState == 0)
	{
		W25QXX_PowerDown();
	}
}
