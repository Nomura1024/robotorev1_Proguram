/*
 * PID_con.c
 *
 *  Created on: 2021/11/08
 *      Author: Owner
 */
#include"PID_con.h"
#include"main.h"
#include <ADC_sens.h>
#include <tuning.h>
#include "initial.h"
#define T 0.001

extern double Kp;//1.2
extern double  Ki;//Ki= 0.001
extern double  Kd;//Kd= 0.0005
extern  uint16_t Speed ;
extern uint16_t Speedbuff;
extern uint8_t Pgain;
extern uint8_t Igain;
extern uint8_t Dgain;
extern float ahs;
extern uint8_t con;
extern uint8_t secon;
extern int L , R;
extern float sensL, sensR;
extern float speedval_I ;
extern float load_log;
extern float loada[6100];

int MotorCtrl(){
	static float sensvalBuf;
	static float sensval_I = 0,sensval_IBuf ;
	float sensval_D =0;
	int16_t sensval;//偏差


	sensval = sensL - sensR;//hensa

    sensval_I = sensval_I + (float)sensval*T;
   // sensval_IBuf = sensval_IBuf + sensval;

    if(sensval_I >= 100000000) sensval_I = 100000000;
    if(sensval_I <= (-100000000)) sensval_I = (-100000000);

    sensval_D = (sensvalBuf - sensval)/T;
    sensvalBuf = sensval;

	return (((double)sensval* Kp)+((float)sensval_I*Ki)-(sensval_D*Kd));
	//LineMotorR = -(((double)sensval* Kp)+((float)sensval_I*Ki)-(sensval_D*Kd));

}
void SpeedCtrl(){
	int16_t MotorL=0,MotorR=0,sensmotor=0;
	float speedval;
	//static float speedval_I ;
	float sKp = 1.6;//1.8
	float sKi= 10;//10


	speedval = Speed  - speedget();
	speedval_I = speedval_I + speedval*T;


	if(speedval_I >= 100000) speedval_I = 100000;
	if(speedval_I <= (-100000)) speedval_I = (-100000);
	sensmotor  = MotorCtrl();
	MotorL = (speedval* sKp)+(speedval_I*sKi)+sensmotor;
	MotorR = (speedval* sKp)+(speedval_I*sKi)-sensmotor;


	Motor(MotorL,MotorR);

	  TIM1 -> CNT = 32767;
	  TIM3 -> CNT = 32767;

}

float speedget()
{
	static float speedbuffg=0;
	static float speedget ;
	float val ;
	static int i =0;
	L = TIM1 -> CNT;
	R = TIM3 -> CNT;
	R = R -32767;
	L = (65535 - L)-32768;

	val=(L+R)/2;
	speedget = (((32.2/2048)*val)/T);

	speedbuffg += speedget*T;

	if(speedbuffg>=10 && secon==0){
		con=1;
		load_log = speedbuffg;
		speedbuffg=speedbuffg-10;
	}
	if(speedbuffg>= loada[i]&& secon==1){
		con=1;
		i++;

	}
	TIM1 -> CNT = 32767;
	TIM3 -> CNT = 32767;
	return speedget;
}
