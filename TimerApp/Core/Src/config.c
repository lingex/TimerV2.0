#include "CONFIG.h"
#include <string.h>
#include "ff.h"
#include "printf.h"
#include "cJSON.h"
//#include "cJSON_Utils.h"
#include <stdlib.h>

extern FATFS FatFs;
static char dataBuff[CONFIG_BUFF_SIZE];

extern uint8_t musicVolume;
extern char musicUsing[];
const char *defaultMusic = "test.mp3";
const char* configName = "config.json";

cJSON *json = NULL;

void InitConfig()
{
	FIL cjfil;

	int rc = f_open(&cjfil, configName, FA_READ);
	UINT read = 0;
	if (rc == FR_OK && f_read(&cjfil, dataBuff, CONFIG_BUFF_SIZE, &read) == FR_OK)
	{
		json = cJSON_Parse(dataBuff);
		if (json != NULL)
		{
			//debug
			//char *string = cJSON_Print(json);
			//printf("%s", string);
			const cJSON *pVolume = cJSON_GetObjectItem(json, "volume");
			if (pVolume != NULL && cJSON_IsNumber(pVolume))
			{
				musicVolume = pVolume->valueint;
			}
			const cJSON *pMusic = cJSON_GetObjectItem(json, "music");
			if (pMusic != NULL && cJSON_IsString(pMusic))
			{
				strcpy(musicUsing, pMusic->valuestring);
			}
			else
			{
				const cJSON *pDefault = cJSON_GetObjectItem(json, "default");
				if (pDefault != NULL && cJSON_IsString(pDefault))
				{
					strcpy(musicUsing, pDefault->valuestring);
				}
				else
				{
					strcpy(musicUsing, defaultMusic);
				}
			}
		}
		cJSON_Delete(json);
	}
	f_close(&cjfil);
}

void SaveVolumeConfig()
{
	FIL cjfil;

	int rc = f_open(&cjfil, configName, FA_READ | FA_WRITE);
	UINT read = 0;
	if (rc == FR_OK && f_read(&cjfil, dataBuff, CONFIG_BUFF_SIZE, &read) == FR_OK)
	{
		json = cJSON_Parse(dataBuff);
		if (json != NULL)
		{
			//debug
			//char *string = cJSON_Print(json);
			//printf("%s", string);
			cJSON *pVolume = cJSON_GetObjectItem(json, "volume");
			if (pVolume != NULL)
			{
				cJSON_SetNumberValue(pVolume, musicVolume);
			}
			else
			{
				cJSON_AddNumberToObject(json, "volume", musicVolume);
			}
		}
		cJSON_Delete(json);
		UINT bytesBeWritten = 0;
		f_write(&cjfil, dataBuff, cjfil.fsize, &bytesBeWritten);
		f_close(&cjfil);
	}
}

void SaveMusicConfig()
{
	FIL cjfil;

	int rc = f_open(&cjfil, configName, FA_READ | FA_WRITE);
	if (rc == FR_OK)
	{
		UINT read = 0;
		rc = f_read(&cjfil, dataBuff, CONFIG_BUFF_SIZE, &read);
		if (rc == FR_OK)
		{
			json = cJSON_Parse(dataBuff);
			if (json != NULL)
			{
				//debug
				//char *string = cJSON_Print(json);
				//printf("%s", string);
				cJSON *pMusic = cJSON_GetObjectItem(json, "music");
				if (pMusic != NULL)
				{
					cJSON_SetValuestring(pMusic, musicUsing);
				}
				else
				{
					cJSON_AddStringToObject(json, "music", musicUsing);
				}
			}
			//strcpy(dataBuff, cJSON_Print(json));
			char* out = cJSON_Print(json);
			cJSON_Delete(json);
			UINT bytesBeWritten = 0;
			//f_printf(&cjfil, "%s", out);
			f_puts(out, &cjfil);
			f_write(&cjfil, out, cjfil.fsize, &bytesBeWritten);
			free(out);
		}
		f_close(&cjfil);
	}
}
