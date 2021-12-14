/*
 * flash.c
 *
 *  Created on: 2021/10/23
 *      Author: Owner
 */
#include "main.h"
#include"flash.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16

#define BACKUP_FLASH_SECTOR_NUM2     FLASH_SECTOR_6
#define BACKUP_FLASH_SECTOR_SIZE2    1024*16

extern uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
extern char _backup_flash_start;

extern float Driving_log[BACKUP_FLASH_SECTOR_SIZE2] __attribute__ ((aligned(4)));
extern char _backup_flash_start2;

bool Flash_clear()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 1;

    // Eraseに失敗したsector番号がerror_sectorに入る
    // 正常にEraseができたときは0xFFFFFFFFが入る
    uint32_t error_sector;
    HAL_StatusTypeDef result = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result == HAL_OK && error_sector == 0xFFFFFFFF;
}

bool Flash_clear2()
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = BACKUP_FLASH_SECTOR_NUM2;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.NbSectors = 6;



    // Eraseに失敗したsector番号がerror_sectorに入る
    // 正常にEraseができたときは0xFFFFFFFFが入る
    uint32_t error_sector;
    HAL_StatusTypeDef result2 = HAL_FLASHEx_Erase(&EraseInitStruct, &error_sector);

    HAL_FLASH_Lock();

    return result2 == HAL_OK && error_sector == 0xFFFFFFFF;
}

uint16_t* Flash_load()
{
    memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
    return work_ram;
}
float* Flash_load2()
{
    memcpy(Driving_log, &_backup_flash_start2, BACKUP_FLASH_SECTOR_SIZE2);
    return Driving_log;
}
bool Flash_store()
{
    // Flashをclear
    if (!Flash_clear()) return false;

    uint32_t *p_work_ram = (uint32_t*)work_ram;

    HAL_FLASH_Unlock();

    // work_ramにあるデータを4バイトごとまとめて書き込む
    HAL_StatusTypeDef result;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE / sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start) + sizeof(uint32_t) * i,
                    p_work_ram[i]
                );
        if (result != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result == HAL_OK;
}
bool Flash_store2()
{
    // Flashをclear
    if (!Flash_clear2()) {
    	printf("error\r\n");
    	return false;
    }
    uint32_t *p_Driving_log = (uint32_t*)Driving_log;

    HAL_FLASH_Unlock();

    // work_ramにあるデータを4バイトごとまとめて書き込む
    HAL_StatusTypeDef result2;
    const size_t write_cnt = BACKUP_FLASH_SECTOR_SIZE2 / sizeof(uint32_t);

    for (size_t i=0; i<write_cnt; i++)
    {
        result2 = HAL_FLASH_Program(
                    FLASH_TYPEPROGRAM_WORD,
                    (uint32_t)(&_backup_flash_start2) + sizeof(uint32_t) * i,
                    p_Driving_log[i]
                );
        if (result2 != HAL_OK) break;
    }

    HAL_FLASH_Lock();

    return result2 == HAL_OK;
}


