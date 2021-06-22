/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "application.h"
#define motorPIDefaults {0,0,0,0,0,0,0,0,0,0,0,0}
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct motorControlStruct{
	float BRperMin;   	/*Input: Breath Rate*/
	float Tbr; 		 	/*Time taken for 1 breath*/
	float Ti;			/*Inspiration Time*/
	float Te;			/*Expiration Time*/	
	float I;				/*Input: Inspiration Factor*/
	float E;				/*Input: Expiration Factor*/
	float Angle;			/*Input: Angle which motor has to traverse*/
	float NpulI;			/*Total Number of Pulse to Generate During Inspiration*/
	float FpulI;			/*Frequency to generate while Inspiration*/
	float FpulE;			/*Frequency to generate while Expiration*/
}motorStruct;

typedef enum motorDirectionEnum{
	mForward,
	mReverse
}motorDirection;

typedef struct motorPIStruct{
    float  Ref;         // Input: REFERENCE variable
    float  Fdb;         // Input: FEEDBACK variable
    float  max;         // maximum limit
    float  min;         // minimum limit
    float  Out;         //out
    float  Kp;          //proportional gain
    float  Ki;          //integral gain
    float  OutPreSat;
    float  Pro_op;
    float  integ_op_past;
    float  integ_op;
    float  error;
} motorPI;

typedef enum motorRunEnum{
	mStop,
	mRun
}motorRun;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define GR 13.0f  //13
#define MICROSTP 128.0f
#define MotorStepAngle 1.8f
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void calcMotorFreq(motorStruct *m);
void uart_irq(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR_M1_Pin GPIO_PIN_1
#define DIR_M1_GPIO_Port GPIOC
#define STEP_M1_Pin GPIO_PIN_2
#define STEP_M1_GPIO_Port GPIOC
#define SLEEP_M1_Pin GPIO_PIN_3
#define SLEEP_M1_GPIO_Port GPIOC
#define IR_M1_Pin GPIO_PIN_0
#define IR_M1_GPIO_Port GPIOA
#define IR1_M1_Pin GPIO_PIN_1
#define IR1_M1_GPIO_Port GPIOA
#define SPICS_M1_Pin GPIO_PIN_4
#define SPICS_M1_GPIO_Port GPIOC
#define SPICS_M2_Pin GPIO_PIN_5
#define SPICS_M2_GPIO_Port GPIOC
#define DIR_M2_Pin GPIO_PIN_6
#define DIR_M2_GPIO_Port GPIOC
#define STEP_M2_Pin GPIO_PIN_7
#define STEP_M2_GPIO_Port GPIOC
#define SLEEP_M2_Pin GPIO_PIN_8
#define SLEEP_M2_GPIO_Port GPIOC
#define IR1_M2_Pin GPIO_PIN_11
#define IR1_M2_GPIO_Port GPIOA
#define IR_M2_Pin GPIO_PIN_12
#define IR_M2_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_10
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_11
#define CS2_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
