/*
 * vari.c
 *
 *  Created on: 2021/11/09
 *      Author: Owner
 */
#include <tuning.h>
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include <flash.h>
#include <AQM0802.h>
#define BACKUP_FLASH_SECTOR_NUM     FLASH_SECTOR_1
#define BACKUP_FLASH_SECTOR_SIZE    1024*16

#define BACKUP_FLASH_SECTOR_NUM2     FLASH_SECTOR_6
#define BACKUP_FLASH_SECTOR_SIZE2    1024*16

extern uint8_t Pgain;
extern uint8_t Igain;
extern uint8_t Dgain;
extern uint16_t Speedbuff;
extern double Kp;//1.2
double  Ki;//Ki= 0.001
double  Kd;//Kd= 0.0005

uint16_t work_ram[BACKUP_FLASH_SECTOR_SIZE] __attribute__ ((aligned(4)));
char _backup_flash_start;

 float Driving_log[BACKUP_FLASH_SECTOR_SIZE2] __attribute__ ((aligned(4)));
char _backup_flash_start2;

void tuning(){
	Flash_load();
	Kp = work_ram[0];
	Ki = work_ram[1];
	Kd = work_ram[2];

while(1){
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_clear();
			lcd_locate(0,0);
			lcd_printf("speed");
			lcd_locate(0,1);
			lcd_printf("%d",Speedbuff);
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				Speedbuff = Speedbuff+100;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				Speedbuff = Speedbuff-100;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				lcd_clear();
				work_ram[0] = Kp;
				work_ram[1] = Ki;
				work_ram[2] = Kd;
				Kp = (double)Kp/100;
				Ki = (double)Ki/1000000;
				Kd = (double)Kd/100;
				Flash_store();
				return ;
			}
		}
		HAL_Delay(100);
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
		lcd_init();
		lcd_clear();
		lcd_locate(0,0);
		lcd_printf("Kp");
		lcd_locate(0,1);
		lcd_printf("%f",Kp);
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
			lcd_clear();
			Kp=Kp+1;
		}
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
			lcd_clear();
			Kp=Kp-1;
		}
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
			lcd_clear();
			work_ram[0] = Kp;
			work_ram[1] = Ki;
			work_ram[2] = Kd;
			Kp = (double)Kp/100;
			Ki = (double)Ki/1000000;
			Kd = (double)Kd/100;
			Flash_store();
			return ;
		}
	}
	HAL_Delay(100);
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_clear();
			lcd_locate(0,0);
			lcd_printf("Ki");
			lcd_locate(0,1);
			lcd_printf("%f",Ki);
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				Ki=Ki+1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				Ki=Ki-1;
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				lcd_clear();
				work_ram[0] = Kp;
				work_ram[1] = Ki;
				work_ram[2] = Kd;
				Kp = (double)Kp/100;
				Ki = (double)Ki/1000000;
				Kd = (double)Kd/100;
				Flash_store();
				return ;
			}
	}
	HAL_Delay(100);
	while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
				lcd_init();
				lcd_clear();
				lcd_locate(0,0);
				lcd_printf("Kd");
				lcd_locate(0,1);
				lcd_printf("%f",Kd);
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
					lcd_clear();
					Kd=Kd+1;
				}
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
					lcd_clear();
					Kd=Kd-1;
				}
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
					lcd_clear();
					work_ram[0] = Kp;
					work_ram[1] = Ki;
					work_ram[2] = Kd;
					Kp = (double)Kp/100;
					Ki = (double)Ki/1000000;
					Kd = (double)Kd/100;
					Flash_store();
					return ;
				}
	}
	HAL_Delay(100);
	}
	lcd_init();
	lcd_clear();
	work_ram[0] = Kp;
	work_ram[1] = Ki;
	work_ram[2] = Kd;
	Kp = (double)Kp/100;
	Ki = (double)Ki/1000000;
	Kd = (double)Kd/100;
	Flash_store();
}
void acc_Speed(){
	while(1){
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("MAX");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[32]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[32]=work_ram[32]+100;
				printf("%d",work_ram[32]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[32]=work_ram[32]-100;
				printf("%d",work_ram[32]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}

		}
		HAL_Delay(100);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("1000");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[33]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[33]=work_ram[33]+100;
				printf("%d",work_ram[33]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[33]=work_ram[33]-100;
				printf("%d",work_ram[33]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}
		}
		HAL_Delay(100);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("800");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[34]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[34]=work_ram[34]+100;
				printf("%d",work_ram[34]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[34]=work_ram[34]-100;
				printf("%d",work_ram[34]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}
		}
		HAL_Delay(100);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("500");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[35]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[35]=work_ram[35]+100;
				printf("%d",work_ram[35]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[35]=work_ram[35]-100;
				printf("%d",work_ram[35]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}
		}
		HAL_Delay(100);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("300");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[36]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[36]=work_ram[36]+100;
				printf("%d",work_ram[32]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[36]=work_ram[36]-100;
				printf("%d",work_ram[36]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}
		}
		HAL_Delay(100);
		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){
			lcd_init();
			lcd_locate(0,0);
			lcd_printf("100");
			lcd_locate(0,1);
			lcd_printf("%d",work_ram[37]);

			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==0){
				lcd_clear();
				work_ram[37]=work_ram[37]+100;
				printf("%d",work_ram[37]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)==0){
				lcd_clear();
				work_ram[37]=work_ram[37]-100;
				printf("%d",work_ram[37]);
			}
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==0){
				Flash_store();
				lcd_clear();
				return;
			}
		}
		HAL_Delay(100);
	}
}
