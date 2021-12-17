/*
 * initial.c
 *
 *  Created on: 2021/11/02
 *      Author: Owner
 */
#include "initial.h"
#include"flash.h"
#include "main.h"
extern uint16_t Speed ;
extern uint16_t Speedbuff ;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim4;
void MX_TIM6_Init(void);
extern uint8_t gh;
void LED(uint8_t x){
	switch(x){
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
			break;

	}

}
void error(){
	while(1){
		Motor(0,0);
		LED(1);
	}
}
void stop(){
	static int i=0;
//	i++;
//	if (i>50000 ){
//		Speed=0;
	HAL_TIM_Base_Stop_IT(&htim6);
		Motor(0,0);
//	}


	LED(6);


}

void Motor(int16_t MotorL,int16_t MotorR)
{
	if(MotorL > 0 ){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

	}else if(MotorL < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		MotorL = -MotorL;
	}
	if(MotorR > 0 ){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	}else if(MotorR < 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		MotorR = -MotorR;

	}
	if (MotorR > 1800) MotorR = 1800;
	if (MotorL > 1800) MotorL = 1800;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MotorL);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MotorR);

}
