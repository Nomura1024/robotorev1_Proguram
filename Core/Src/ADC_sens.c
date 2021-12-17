/*
 * ADC_sens.c
 *
 *  Created on: 2021/11/11
 *      Author: Owner
 */
#include "main.h"
#include "initial.h"
#include <ADC_sens.h>
#include <AQM0802.h>
#include <flash.h>
#include "stdio.h"
#include "string.h"
#include "initial.h"
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

#define SENSOR_NUMBER 13
#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16


extern uint16_t analog[SENSOR_NUMBER];
extern uint16_t di[SENSOR_NUMBER];                  //Maximum and minimum difference
extern uint16_t sens[SENSOR_NUMBER];
extern uint16_t sensRatio[SENSOR_NUMBER];
extern uint16_t b[SENSOR_NUMBER];
extern float sensL, sensR;
extern  int count;
extern uint8_t cros;

extern uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
char _backup_flash_start;

void start(){
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *) analog, SENSOR_NUMBER) != HAL_OK){
	  Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK){
	      Error_Handler();
	}
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK){
				      Error_Handler();
				}
}
void ADCinit(){
	Flash_load();
	HAL_Delay(100);
	uint16_t a[SENSOR_NUMBER];
	uint16_t i;
	i = 0;


	while(i < SENSOR_NUMBER){
		a[i] = 0;
		b[i] = 10000;
		i++;
	}
	i = 0;
	while (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14))
	{
		if(analog[i] > a[i]){
			a[i] = analog[i];
		}
		if(analog[i] < b[i]){
			b[i] = analog[i];
		}
		i++;
		if(i == SENSOR_NUMBER){
			i=0;
		}
		lcd_clear();lcd_locate(0,0);
		lcd_printf("ADCinit");
		LED(2);
	}
	i = 0;
	 printf("MAX: %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n", a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12]);
	 printf("mini: %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n", b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8],b[9],b[10],b[11],b[12]);
	while(i < SENSOR_NUMBER){
		di[i] = a[i]-b[i];
		work_ram[i+3] = di[i];
		work_ram[i+17] = b[i];
		i++;
	}
	printf("flash: %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n", work_ram[3],work_ram[4],work_ram[5],work_ram[6],work_ram[7],work_ram[8],work_ram[9],work_ram[10],work_ram[11],work_ram[12],work_ram[13],work_ram[14],work_ram[15]);
	Flash_store();
	LED(5);

	HAL_Delay(3000);
}

void sensGet(){
	sensL = 0;
	sensR = 0;
	uint16_t er =0;
	static uint16_t i=0,k=0;

	static uint16_t j;
	for(i=0; i<13; i++){
		sens[i] = analog[i];
	}
	for(i=0; i<13; i++){
		sensRatio[i] = (1000.0f/(float)di[i])*((float)(sens[i]-b[i]));
	}
	for(i=2;i<6;i++){
		sensL += sensRatio[i];
	}
	for(i=7;i<11;i++){
		sensR += sensRatio[i];
	}
	if(sensRatio[7]<800)j =0;
	if(sensRatio[1]<500)count =0;

			er = sensR+sensL;
	if(er > 800) j++;
	if(er > 800 && j>=300 ) error();


	if(er<1200 && k>=0) {
		cros=1;
		k=0;
	}else cros=0;

	if(er > 1200)k++;

}
