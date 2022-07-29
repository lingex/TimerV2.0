#ifndef __RX8025T_H
#define __RX8025T_H

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdlib.h>
#include <stdbool.h>

#include "main.h"

#define RX8025T_READ_ADDR       0x65
#define RX8025T_WRITE_ADDR      0x64

#define REGADDR_SEC             0x00
#define REGADDR_MIN             0x01
#define REGADDR_HOUR            0x02
#define REGADDR_WEEK            0x03
#define REGADDR_DAY             0x04
#define REGADDR_MONTH           0x05
#define REGADDR_YEAR            0x06

#define REGADDR_RAM             0x07	//read/write accessible for any data in the range from 00h to ffh
#define REGADDR_MIN_ALARM       0x08
#define REGADDR_HOUR_ALARM      0x09
#define REGADDR_WEEK_DAY_ALARM  0x0A
#define REGADDR_TIM_CNT0        0x0B	//timer counter 0
#define REGADDR_TIM_CNT1        0x0C	//timer counter 1
#define REGADDR_EXTEN           0x0D
  #define EXTEN_TEST            (1<<7)	//dont' use it
  #define EXTEN_WADA            (1<<6)	//week alarm/day alarm, specify either week or day as the target of the alarm INT func 1=day 0=week
  #define EXTEN_USEL            (1<<5)	//update interrupt select, specify either second update or minute update as the update generation timing of the time update INT func(default: second)
  #define EXTEN_TE              (1<<4)	//timer enable,controls the start/stop setting for the fixed-cycle timer INT fucn, 0=stop
  #define EXTEN_FSEL1           (1<<3)	//FOUT frequency select bit(s) with FSEL0
  #define EXTEN_FSEL0           (1<<2)
  #define EXTEN_TSEL1           (1<<1)	//timer select 0, 1 bit(s) with TSEL0, used to set the countdown period (source clock) for the fixed-cycle timer INT func
  #define EXTEN_TSEL0           (1<<0)
#define REGADDR_FLAG            0x0E
  #define FLAG_UF               (1<<5)	//update flag, changes from 0 to 1 when a time update INT has occurred
  #define FLAG_TF               (1<<4)	//time flag, changes from 0 to 1 when a fixed-cycle timer INT has occurred
  #define FLAG_AF               (1<<3)	//alarm flag, changes from 0 to 1 when an alarm INT has occuerred
  #define FLAG_VLF              (1<<1)	//voltage low flag, all registers must be initialized
  #define FLAG_VDET             (1<<0)	//voltage detettion flag
#define REGADDR_CONTROL         0x0F
  #define CONTR_CSEL1           (1<<7)	//compensation interval select bit(s) with CSEL 0 (default: 2.0s)
  #define CONTR_CSEL0           (1<<6)
  #define CONTR_UIE             (1<<5)	//update INT enable when a time update INT event is generated (when the UF bit value changes from 0 to 1) (/INT status changes from Hi=Z to low), automatically cleared after 7.8ms
  #define CONTR_TIE             (1<<4)	//timer INT enable when a fixed-cycle timer INT occurs (when the TF bit value changes from 0 to 1) automatically cleared after 7.8ms
  #define CONTR_AIE             (1<<3)	//alarm INT enable, retained until the AF bit value is cleared to zero, no automatic cancellation
  #define CONTR_RESET           (1<<0)	//not use it

typedef enum
{
  SUNDAY = 0,
  MONDAY,
  TUESDAY,
  WEDNESDAY,
  THURSDAY,
  FRIDAY,
  SATURDAY
} DaysOfWeek;

typedef struct
{
  uint8_t Year;
  uint8_t Month;
  uint8_t Date;
  uint8_t DaysOfWeek;
  uint8_t Hour;
  uint8_t Min;
  uint8_t Sec;
} _RTC;


bool RX8025T_Init(I2C_HandleTypeDef *handle);
bool RX8025T_GetTime(_RTC *rtc);
bool RX8025T_SetTime(_RTC *rtc);

bool RX8025T_ClearAlarm(void);
bool RX8025T_SetINTPerSec(void);
bool RX8025T_SetINTPerMin(void);
bool RX8025T_SetINTDisable(void);
bool ReadRegister(uint8_t regAddr, uint8_t *value);
bool WriteRegister(uint8_t regAddr, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
