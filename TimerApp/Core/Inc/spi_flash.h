/*******************************************************************************
* Module Name      : spi_flash.h
* Create Date      : 2016-08-02
* Author           : huanglong
* Copyright        :
* Description      : 外部 SPI FLASH
* Revision History :
* Date          Author        Version        Notes
  2016-08-02    huanglong     V1.0           创建
*******************************************************************************/
#ifndef BSP_SPI_FLASH_H
#define BSP_SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* 包含头文件 ----------------------------------------------------------------*/
#include <string.h>
#include "main.h"

#define SPI_FLASH_WRITE_BUFFER_LEN	4096		//default:4096

#define SPI_FLASH_SECTOR_WRITE_ENABLE 1

#define SPI_FLASH_DMA_ENABLE 1

#define SPI_FLASH_SECTOR_SIZE    512      //
#define SPI_FLASH_SECTOR_COUNT   32768    // 32768*512 = 16MB
#define SPI_FLASH_BLOCK_SIZE     8        // 4KB

#define SF_CS_L   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
#define SF_CS_H   HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

// W25X系列/Q系列芯片列表
// W25Q80  ID  0XEF13
// W25Q16  ID  0XEF14
// W25Q32  ID  0XEF15
// W25Q64  ID  0XEF16
// W25Q128 ID  0XEF17
// W25Q256 ID  0XEF18
#define W25Q80     0XEF13
#define W25Q16     0XEF14
#define W25Q32     0XEF15
#define W25Q64     0XEF16
#define W25Q128    0XEF17
#define W25Q256    0XEF18

extern uint16_t W25QXX_TYPE;                             // 定义W25QXX芯片型号

//////////////////////////////////////////////////////////////////////////////////
// W25X 指令表
#define W25X_WriteEnable           0x06
#define W25X_WriteDisable          0x04
#define W25X_ReadStatusReg1        0x05
#define W25X_ReadStatusReg2        0x35
#define W25X_ReadStatusReg3        0x15
#define W25X_WriteStatusReg1       0x01
#define W25X_WriteStatusReg2       0x31
#define W25X_WriteStatusReg3       0x11
#define W25X_ReadData              0x03
#define W25X_FastReadData          0x0B
#define W25X_FastReadDual          0x3B
#define W25X_PageProgram           0x02
#define W25X_BlockErase            0xD8
#define W25X_SectorErase           0x20
#define W25X_ChipErase             0xC7
#define W25X_PowerDown             0xB9
#define W25X_ReleasePowerDown      0xAB
#define W25X_DeviceID              0xAB
#define W25X_ManufactDeviceID      0x90
#define W25X_JedecDeviceID         0x9F
#define W25X_Enable4ByteAddr       0xB7
#define W25X_Exit4ByteAddr         0xE9

void W25QXX_Init(void);
uint16_t W25QXX_ReadID(void);                        // 读取FLASH ID
uint8_t W25QXX_ReadSR(uint8_t regno);                // 读取状态寄存器
void W25QXX_4ByteAddr_Enable(void);                  // 使能4字节地址模式
void W25QXX_Write_SR(uint8_t regno,uint8_t sr);      // 写状态寄存器
void W25QXX_Write_Enable(void);                      // 写使能
void W25QXX_Write_Disable(void);                     // 写保护
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);            // 读取flash
#if SPI_FLASH_SECTOR_WRITE_ENABLE
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);         // 写入flash
#endif
void W25QXX_Erase_Chip(void);                        // 整片擦除
void W25QXX_Erase_Sector(uint32_t Dst_Addr);         // 扇区擦除
void W25QXX_Wait_Busy(void);                         // 等待空闲
void W25QXX_PowerDown(void);                         // 进入掉电模式
void W25QXX_WAKEUP(void);                            // 唤醒

uint32_t W25QXX_GetSectorCount(void);                //获取 Sector 数量
uint8_t W25QXX_GetPowerState(void);                  //获取电源状态

#ifdef __cplusplus
}
#endif

#endif
