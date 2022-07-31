#ifndef __CONFIG_H__
#define __CONFIG_H__


#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

#define CONFIG_BUFF_SIZE (1024 * 1)


void LoadConfigs(void);

void SaveConfigs(void);


#ifdef __cplusplus
}
#endif


#endif //__CONFIG_H__
