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


extern uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
extern char _backup_flash_start;


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


uint16_t* Flash_load()
{
    memcpy(work_ram, &_backup_flash_start, BACKUP_FLASH_SECTOR_SIZE);
    return work_ram;
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



