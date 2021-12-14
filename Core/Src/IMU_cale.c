/*
 * IMU_cale.c
 *
 *  Created on: 2021/11/17
 *      Author: Owner
 */
#include"main.h"
#include <initial.h>
#include<IMU_cale.h>
#include <ICM_20648.h>
#include <flash.h>
#include "stdio.h"
#define BACKUP_FLASH_SECTOR_SIZE    1024*16
#define PI 3.141592


extern  uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
 char _backup_flash_start;

extern float ang_average ;
 float ang;
void off_angle(){
	float average;
	int i;
	Flash_load();
	IMU_init();
	HAL_Delay(1500);

	for(i=0;i<=1000;i++){
		read_gyro_data();
		average = average+zg;
		HAL_Delay(1);
	}
	LED(4);
	if(average<=0) average = -average*10;
	work_ram[31]= average;
	if(average>=50) average=-average/10;
	ang_average = average/1000;

	printf("%f\r\n",ang_average);
	printf("%d\r\n",work_ram[31]);
	Flash_store();
}
float calc_angle(){
	float omega_z=0;
	//float angle;
	read_gyro_data();
	omega_z = (((float)zg-ang_average) / 16.4) * PI / 180;
	//angle = angle+ (omega_z * 0.01);

	return omega_z;
}


