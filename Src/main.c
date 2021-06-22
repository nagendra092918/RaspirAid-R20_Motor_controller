/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#include "stepperDriverDRV8711.h"
//#include "communication.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define timerFreq 1000000.0f
#define maxVolumeSamples 5
//#define correctionAngleplus 0.9f /**< Adjustment on angle done if measured volume is less than set*/
//#define correctionAngleminus 0.5 /**< Adjustment on angle done if measured volume is more than set*/

float correctionAngleplus=0.9f; /**< Adjustment on angle done if measured volume is less than set*/
float correctionAngleminus=0.5;/**< Adjustment on angle done if measured volume is more than set*/

//#define correctionAngle 0.55f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern SPI_HandleTypeDef hspi1;

motorStruct m={0};
uint8_t uartRecBuf[8];
uint8_t alarmbuff[8];
/*Setting Variables*/
uint16_t inspiration = 65;  //682   //114
uint16_t expiration = 65;		//682		//114
int16_t TidalVolume = 200;  //swclk swdio gd
uint16_t BreathRate = 12;
uint16_t pressureMax = 40;
float IE=1;
float motorInspirationAngle=21.5;
/*Motor Variables*/
float inputFreq=0;             //frequency of motor Pulses
float prdVal=0;             //ARR value
float forwardCtr=0;         //Inspiration counter
motorDirection mDir=mForward;  //variable to keep track of the motor direction
/*Volume Variables*/
int16_t measuredVolume=0;     //final measured volume for running closed loop
uint16_t updateAngle=0;        //Flag to increase volume and hence Angle
uint16_t updateMotor=0;        //Flag to update motor speed based on new Angle
uint8_t i2cbuf[10]; 
uint16_t slaveAddr = 0x40<<1;  //Flow sensor Addr
float flow,flow1,flow3;             
uint16_t flow2;
float volume,volumeOut=0;      //volume is the intermediate integral ouput
uint16_t counterFlag;          //Final Integration volume
uint16_t rawVolumeArray[10];   //Array to average Volumes
uint16_t volumeSample=0;       //Counter for Taking volume Samples
uint16_t timer15ctr=0;
uint32_t volumeSum; 
motorRun mRun1=mRun;
motorRun mRun2=mRun;
StepperDRVConfig s1=DefaultMotorConfig;
StepperDRVConfig s2=DefaultMotorConfig;
runningParams setparam;
runningParams txParam;
payloadData pd,txPd;
errorState e=noError;
uint8_t tx[8];
HAL_StatusTypeDef sensorError=  HAL_OK;
uint16_t sensorTimeout=0;
uint16_t tidalVolErrCtr=0;//indiviual err increments
uint16_t tverrCheck=0; //check or not
uint16_t tvCheckCtr=0; //when to check TV err. ctr for counting no of Breaths
uint16_t tverrLimit=0;
uint16_t brPMCtr=0;
float minVolsum=0.0;
float minVol=0.0;
float minVolLimit=0.0;
float volLimit=0.0;
float minVolErrCtr;
uint16_t sensorErrorCtr=0;
uint16_t malfunctionCtr=0;
uint16_t txPrescalar=0;
uint32_t reverseMaxSteps=0;
uint32_t forwardMinSteps=0;
uint32_t M1_revSteps=0;
uint32_t M2_revSteps=0;
uint16_t IR_error=0;
uint16_t IE_UpdateCtr=0;
uint8_t RR_IE = 14;
float PEEP,PIP;
uint8_t motor1_driv_slp;
uint8_t motor2_driv_slp;
uint8_t alarm_mal,alarm_val;
uint8_t cnt_tim1=0;
uint8_t uart_flag=0,tim_flag=0;
uint8_t timer_start_flag=0;
uint8_t uart_err_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void motorResetArms(void);
static void motorResetArms_Start(void);
static void calculateVolume(void);
static void stopMotorPins(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
	i2cbuf[0]=0x10;
	i2cbuf[1]=0x00;
	HAL_Delay(1000); //old 100
	while(sensorTimeout < 20)
		{
			sensorError=HAL_I2C_Master_Transmit(&hi2c1,slaveAddr,i2cbuf,2,10);
			HAL_Delay(1);
			if(sensorError!=HAL_OK )
				{
					sensorErrorCtr++;
				}
		sensorTimeout++;
		}
	sensorTimeout=0;
	if(sensorErrorCtr>15)
		{
			e=errSensor;
		}
	else
		{
			e=noError;
		}

	sensorErrorCtr=0;
	//HAL_Delay(500);
	pd.mCMD=mCMDACK;
	if(huart1.RxState==HAL_UART_STATE_READY && huart1.RxState != HAL_UART_STATE_BUSY_RX && huart1.RxState != HAL_UART_STATE_BUSY_TX_RX)
		{
			
			HAL_UART_Receive_IT(&huart1,uartRecBuf,8u);
			//HAL_UART_IRQHandler(&huart1);
		}
	/*Set Direction Pins to Expiration*/
	HAL_GPIO_WritePin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin,GPIO_PIN_SET); //SET=Reverse
	HAL_GPIO_WritePin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin,GPIO_PIN_SET); //SET=Reverse

	/*Initialize the IR Sensor Pins in a defined state*/
	HAL_GPIO_WritePin(IR1_M1_GPIO_Port,IR1_M1_Pin,GPIO_PIN_SET); 
	HAL_GPIO_WritePin(IR_M1_GPIO_Port,IR_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IR1_M2_GPIO_Port,IR1_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IR_M2_GPIO_Port,IR_M2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET); //SET=Reverse
	HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET); //SET=Reverse	  

		/***********************************************************/
	HAL_Delay(1000u);
	s1.M=1u;
	resetSettings(s1);
	clearStatus(s1);
	setDecayMode(&s1,(HPSDDecayMode)AutoMixed);
	setCurrentMilliamps36v4(&s1,3800);   //old 4500
	setStepMode(&s1,(HPSDStepMode)MicroStep128);
	enableDriver(&s1);
	HAL_Delay(1u);
	
	s2.M=2u;
	resetSettings(s2);
	clearStatus(s2);
	setDecayMode(&s2,(HPSDDecayMode)AutoMixed);
	setCurrentMilliamps36v4(&s2,3800);    //old 4500
	setStepMode(&s2,(HPSDStepMode)MicroStep128);
	enableDriver(&s2);
  /*Update the motor varibles and Calculate Ins and Exp Frequencies*/
	m.BRperMin=BreathRate;
	m.Angle=motorInspirationAngle;
	m.I=1;   
	m.E=IE;
	calcMotorFreq(&m);
  /*Set Motor State to forward*/
	mDir=mForward;
  /*Check the Input frequency First and Update ARR register*/
	inputFreq=m.FpulI;
	if(inputFreq!=0)
		{
			prdVal=timerFreq/inputFreq;
		}	
	TIM2->ARR=(uint16_t)(prdVal);    		
	minVolLimit=m.BRperMin*TidalVolume*0.0009;//1000;     //how many liters of volume we are pushing per minit here,ex:20*500*0.0009=9lit we are pushing 
	volLimit=(TidalVolume*0.9)-5;													 // this is for error  acceptance calculation 10%+- 5 ex:(500*0.9)-5=445,if this much volume achive acceptable
	reverseMaxSteps=m.NpulI*1.5; //old 1.5  // need explation
	forwardMinSteps=m.NpulI*0.95;   //old 0.5 // need explation
  /*Change Motor to Forward Direction*/
	HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_RESET);  /*RESET=FORWARD*/
	HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_RESET);  
	

  /*Start Flow Sensor*/
	i2cbuf[0]=0x10;
	i2cbuf[1]=0x00;
	
	HAL_TIM_Base_Start_IT(&htim4);
		
	if(huart1.RxState==HAL_UART_STATE_READY && huart1.RxState != HAL_UART_STATE_BUSY_RX && huart1.RxState != HAL_UART_STATE_BUSY_TX_RX)
	{
			HAL_UART_Receive_IT(&huart1,uartRecBuf,8u);
	}
		
	//HAL_UART_Receive_IT(&huart1,uartRecBuf,8u);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(mDir==mForward && pd.mCMD==mCMDStart)         /*Checking End of Inspiration*/
			{	
				mRun1=mRun;															//run motor1
				mRun2=mRun;															//run motor2
				uint16_t forwardNpul=0;		
			}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		if(huart->Instance == USART1)
		{
			__HAL_UART_FLUSH_DRREGISTER (& huart1); // Clear the buffer to prevent overrun
	if(uartRecBuf[0]==0xAA && uartRecBuf[1]==0x55 && uartRecBuf[6]==0x55 
							&& uartRecBuf[7]==0xAA) 
	{
		uart_flag=1;
//		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_TIM_Base_Stop_IT(&htim3);
		unpackDataRX(uartRecBuf,&setparam,&pd);
		
		if(pd.mCMD == mCMDReset){
//			HAL_TIM_Base_Stop_IT(&htim3);
				HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim4);
			stopMotorPins();
			motorResetArms();
			stopMotorPins();
			IE_UpdateCtr=0;
			forwardCtr=0;
			reverseMaxSteps=0;
			forwardMinSteps=0;
			uart_err_flag=0;
			
		}
		else if(pd.mCMD == mCMDStop){
//			HAL_TIM_Base_Stop_IT(&htim3);
				HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim4);
			stopMotorPins();
			motorResetArms();
			stopMotorPins();
			//HAL_TIM_Base_Stop_IT(&htim3);
			IE_UpdateCtr=0;
			forwardCtr=0;
			reverseMaxSteps=0;
			forwardMinSteps=0;
			uart_err_flag=0;
		}
		else if(pd.mCMD == mCMDStart){
//			HAL_TIM_Base_Stop_IT(&htim3);
				HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim4);
			forwardCtr=0;
			reverseMaxSteps=0;
			forwardMinSteps=0;
			IE_UpdateCtr=0;
			e=noError;
			stopMotorPins();
			motorResetArms_Start();
			stopMotorPins();
//			mRun1=mStop;
//			mRun2=mStop;
	    HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_RESET);  /*RESET=FORWARD*/
	    HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_RESET);			
			m.BRperMin=setparam.RR;
			
			TidalVolume = setparam.TV;
			if(TidalVolume<200)TidalVolume=200;
			if(TidalVolume>750)TidalVolume=600;
			if(BreathRate<=12)
			{
			m.Angle=((14.5/300.0)*((float)TidalVolume-200.0))+ 21.5;    //23.5; //+21.5 //.5/300 //calculating the angle by line eqtion
			}
			if(BreathRate>12 && BreathRate<=20)
			{
			m.Angle=((14.5/300.0)*((float)TidalVolume-200.0))+ 33;    //23.5; //+21.5 //.5/300
			}
			if(BreathRate>20 && BreathRate<=30)
			{
			m.Angle=((14.5/300.0)*((float)TidalVolume-200.0))+ 21.5;  //21.5  //23.5; //+21.5 //.5/300
			}
			if(m.Angle<21.5)m.Angle=21.5;
			if(m.Angle>43.0)m.Angle=43.0;
			
			mDir=mForward;
			m.I=1;
			m.E=setparam.IE;
			calcMotorFreq(&m);
			minVolLimit=m.BRperMin*TidalVolume*0.0009;//1000;   //how many liters of volume we are pushing per minit here,ex:20*500*0.0009=9lit we are pushing 
			volLimit=(TidalVolume*0.9)-5;       // this is for error  acceptance calculation 10%+- 5 ex:(500*0.9)-5=445,if this much volume achive acceptable
			minVol=0;				
			tidalVolErrCtr=0;//indiviual err increments
			tvCheckCtr=0; //when to check TV err. ctr for counting no of Breaths
			brPMCtr=0;
			minVolsum=0.0;
			minVol=0.0;
			mRun1=mRun;
			mRun2=mRun;
			timer_start_flag=1;
			inputFreq=m.FpulI;
			if(inputFreq!=0){
				prdVal=timerFreq/inputFreq;       // 
			}	
			TIM2->ARR=(uint16_t)(prdVal);		 //removed -1
			reverseMaxSteps=m.NpulI*1.5;
			forwardMinSteps=m.NpulI*0.95;
		//	HAL_TIM_Base_Start_IT(&htim3);
			uart_err_flag=0;
		}		
	}
	else if(uartRecBuf[0]==0xBB && uartRecBuf[1]==0x88 && uartRecBuf[6]==0x88 
							&& uartRecBuf[7]==0xBB)
	{
		peep_pip_unpackDataRX(uartRecBuf,alarm_mal);
		e=noError;	 
		uart_err_flag=0;
		
	}
	
	else
		{		
			e=errComm;
			uart_err_flag=1;
			memset(uartRecBuf,0,sizeof(uartRecBuf));
//			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim3);
			
		}

		if(huart1.gState==HAL_UART_STATE_READY && huart1.gState != HAL_UART_STATE_BUSY_TX && huart1.gState != HAL_UART_STATE_ERROR)
		{
			if(tim_flag != 1 && uart_err_flag!=1)
			{
				txPd.fault=e;
				txPd.mCMD=mCMDACK;
				packDataTX(tx,setparam,txPd);
				HAL_UART_Transmit_IT(&huart1,tx,8);
			}
		}
	//	void uart_irq(void);
//		HAL_TIM_Base_Start_IT(&htim2);
		HAL_TIM_Base_Start_IT(&htim3);
	
		if(pd.mCMD == mCMDStart && timer_start_flag==1)
		{
//			HAL_TIM_Base_Start_IT(&htim3);
			HAL_TIM_Base_Start_IT(&htim2);
			HAL_TIM_Base_Start_IT(&htim4);
			timer_start_flag = 0;
		}
	}
	memset(uartRecBuf,0,sizeof(uartRecBuf));
}

void calcMotorFreq(motorStruct *m){
	m->Tbr=60.0/m->BRperMin;
	IE_UpdateCtr++;
	if(IE_UpdateCtr<5 && setparam.IE==1 && (setparam.TV >= 190 && setparam.TV <=249))
	{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.18;//0.075;    //0.05;
				m->E-= 0.18;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if((m->BRperMin > 20) && (m->BRperMin <= 25))
				{
					m->I+= 0.06;  //old 0.12//0.14;//0.075;    //0.05;
					m->E-=  0.06;//old 0.12//0.14;//0.075;      //0.05;
				}			
		}
		if(IE_UpdateCtr<5 && setparam.IE==1 && (setparam.TV >= 250 && setparam.TV <= 299))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.18;//0.075;    //0.05;
				m->E-= 0.18;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
		if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.09;//0.075;    //0.05;
				m->E-= 0.09;//075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
		}
		
		
		
		if(IE_UpdateCtr<4 && setparam.IE==1 && (setparam.TV >= 300  && setparam.TV <= 349))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.06;//0.075;    //0.05;
				m->E-= 0.06;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.09;//0.075;    //0.05;
				m->E-= 0.09;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<4 && setparam.IE==1 && (setparam.TV >= 350 && setparam.TV <= 399))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.06;//0.075;    //0.05;
				m->E-= 0.06;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.09;//0.075;    //0.05;
				m->E-= 0.09;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
		}
		
		
		if(IE_UpdateCtr<3 && setparam.IE==1 && (setparam.TV >= 400 && setparam.TV <= 449) )
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.05;//old 0.2//0.075;    //0.05;
				m->E-= 0.05;//old 0.2//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.15;//0.075;    //0.05;
				m->E-= 0.15;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.08;//0.075;    //0.05;
				m->E-= 0.08;//0.075;      //0.05;
			}
			
		}
		
		
		if(IE_UpdateCtr<3 && setparam.IE==1 && (setparam.TV >= 450 && setparam.TV <= 499))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.05;//old 0.2//0.075;    //0.05;
				m->E-= 0.05;//old 0.2//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.15;//0.075;    //0.05;
				m->E-= 0.15;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==1 && (setparam.TV >= 500 && setparam.TV <= 549))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.05;//old 0.2//0.075;    //0.05;
				m->E-= 0.05;//old 0.2//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.06;//0.075;    //0.05;
				m->E-= 0.06;//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==1 && (setparam.TV >= 550 && setparam.TV <= 599))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.05;//old 0.2//0.075;    //0.05;
				m->E-= 0.05;//old 0.2//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.06;//0.075;    //0.05;
				m->E-= 0.06;//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<2 && setparam.IE==1 && setparam.TV >= 600)
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.2;//0.075;    //0.05;
				m->E-= 0.2;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.22;//0.075;    //0.05;
				m->E-= 0.22;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.07;//0.075;    //0.05;
				m->E-= 0.07;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.06;//0.075;    //0.05;
				m->E-= 0.06;//0.075;      //0.05;
			}
			
		}
		
		//1:2 updated values
		
		if(IE_UpdateCtr<5 && setparam.IE==2 && (setparam.TV >= 190 && setparam.TV <= 249))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
			if((m->BRperMin > 20) && (m->BRperMin <=25))
			{
				m->I+= 0.1;//0.12;//0.075;    //0.05;
				m->E-= 0.1;//0.12;//0.075;      //0.05;
			}
			if((m->BRperMin > 25) && (m->BRperMin <=30))
			{
				m->I+= 0.1;//0.075;    //0.05;
				m->E-= 0.1;//0.075;      //0.05;
			}
			
		}
		
		
		if(IE_UpdateCtr<5 && setparam.IE==2 && (setparam.TV >= 250 && setparam.TV <= 299))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
		}
		
		if(IE_UpdateCtr<4 && setparam.IE==2 && (setparam.TV >= 300 && setparam.TV <= 349))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<4 && setparam.IE==2 && (setparam.TV >= 350 && setparam.TV <= 399))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==2 && (setparam.TV >= 400 && setparam.TV <= 449))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.12;//0.12;    //0.05;
				m->E-= 0.12;//0.12;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==2 && (setparam.TV >= 450 && setparam.TV <= 499))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05; 
			}
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.14;//0.12;    //0.05;
				m->E-= 0.14;//0.12;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==2 && (setparam.TV >= 500 && setparam.TV <= 549))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.08;//0.075;    //0.05;
				m->E-= 0.08;//0.075;      //0.05;
			}
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.1; //old 0.08
				m->E-= 0.1; //old 0.08
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
		}
		
			if(IE_UpdateCtr<3 && setparam.IE==2 && (setparam.TV >= 550 && setparam.TV <= 599))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.12;//0.075;    //0.05;
				m->E-= 0.12;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.27;//old 0.23//0.075;    //0.05;
				m->E-= 0.27;//old 0.23//0.075;      //0.05;
			}
			
		}
		
			if(IE_UpdateCtr<3 && setparam.IE==2 && setparam.TV >= 600)
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.16;//0.075;    //0.05;
				m->E-= 0.16;//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.14;//0.075;    //0.05;
				m->E-= 0.14;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 20 && m->BRperMin <=25)
			{
				m->I+= 0.23;//0.075;    //0.05;
				m->E-= 0.23;//0.075;      //0.05;
			}
			
			if(m->BRperMin > 25 && m->BRperMin <=30)
			{
				m->I+= 0.03;//0.075;    //0.05;
				m->E-= 0.03;//0.075;      //0.05;
			}
		}

	if(IE_UpdateCtr<5 && setparam.IE==3 && (setparam.TV >= 190 && setparam.TV <= 249))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.1;//0.14//0.075;    //0.05;
				m->E-= 0.1;//0.14//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.1;//0.14//0.075;    //0.05;
				m->E-= 0.1;//0.14//0.075;      //0.05;
			}		
			
		}
		
		
		if(IE_UpdateCtr<5 && setparam.IE==3 && (setparam.TV >= 250 && setparam.TV <= 299))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.08;//0.14//0.075;    //0.05;
				m->E-= 0.08;//0.14//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.1;//0.12//0.075;    //0.05;
				m->E-= 0.1;//0.12//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<4 && setparam.IE==3 && (setparam.TV >= 300 && setparam.TV <= 349))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.1;//0.14//0.075;    //0.05;
				m->E-= 0.1;//0.14//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.1;//0.14//0.075;    //0.05;
				m->E-= 0.1;//0.14//0.075;      //0.05;
			}
		}
		
		
		if(IE_UpdateCtr<4 && setparam.IE==3 && (setparam.TV >= 350 && setparam.TV <= 399))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.06;//0.14//0.075;    //0.05;
				m->E-= 0.06;//0.14//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.06;//0.12//0.075;    //0.05;
				m->E-= 0.06;//0.12//0.075;      //0.05;
			}
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==3 && (setparam.TV >= 400 && setparam.TV <= 449))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.06;//0.16//0.075;    //0.05;
				m->E-= 0.06;//0.16//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.06;//0.14//0.075;    //0.05;
				m->E-= 0.06;//0.14//0.075;      //0.05;
			}
			
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==3 && (setparam.TV >= 450 && setparam.TV <= 499))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.08;//0.16//0.075;    //0.05;
				m->E-= 0.08;//0.16//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.08;//0.12//0.075;    //0.05;
				m->E-= 0.08;//0.12//0.075;      //0.05;
			}
		}
		
		if(IE_UpdateCtr<3 && setparam.IE==3 && (setparam.TV >= 500 && setparam.TV <= 549))
		{
			if(m->BRperMin <= RR_IE)
			{
				m->I+= 0.08;//0.14//0.075;    //0.05;
				m->E-= 0.08;//0.14//0.075;      //0.05;
			}
			
			if(m->BRperMin > RR_IE && m->BRperMin <=20)
			{
				m->I+= 0.08;//0.12 //old 0.14 //0.075;    //0.05;
				m->E-= 0.08;//0.12 //old 0.14 //0.075;      //0.05;
			}
	}
		
//		if(IE_UpdateCtr<3 && setparam.IE==3 && (setparam.TV >= 550 && setparam.TV <= 599))
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.1;//0.14//0.075;    //0.05;
//				m->E-= 0.1;//0.14//0.075;      //0.05;
//			}
//			
//			if(m->BRperMin > RR_IE && m->BRperMin <=20)
//			{
//				m->I+= 0.1;//old 0.12  //0.075;    //0.05;
//				m->E-= 0.1;// old 0.12 //0.075;      //0.05;
//			}
//		}
		
//			if(IE_UpdateCtr<2 && setparam.IE==3 && setparam.TV >= 600)
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.1;//0.14//0.075;    //0.05;
//				m->E-= 0.1;//0.14//0.075;      //0.05;
//			}
//			
//			if(m->BRperMin > RR_IE && m->BRperMin <=20)
//			{
//				m->I+= 0.1;//old 0.12//0.075;    //0.05;
//				m->E-= 0.1;//old 0.12//0.075;      //0.05;
//			}
//			
//		}
				
		//I:E = 1:4 update  values
//		if(IE_UpdateCtr<3 && setparam.IE==4 && (setparam.TV >= 190 && setparam.TV <= 249))
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I-= 0.08;//0.09;//0.075;    //0.05;
//				m->E+= 0.08;//0.09//0.075;      //0.05;
//			}
//			
//			
//		}
//		
//		if(IE_UpdateCtr<5 && setparam.IE==4 && (setparam.TV >= 250 && setparam.TV <= 299))
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.08;//0,09//0.075;    //0.05;
//				m->E-= 0.08;//0.09//0.075;      //0.05;
//			}
//		
//			
//		}
//		
//		if(IE_UpdateCtr<4 && setparam.IE==4  && (setparam.TV >= 300 && setparam.TV <= 349) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.1;//0.09//0.075;    //0.05;    /////
//				m->E-= 0.1;//0.09//0.075;      //0.05;
//			}		
//		}
//		
//		if(IE_UpdateCtr<4 && setparam.IE==4  && (setparam.TV >= 350 && setparam.TV <= 399) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.08;//0.09//0.075;    //0.05;
//				m->E-= 0.08;//0.09//0.075;      //0.05;
//			}
//			
//			
//		}
//		
//		if(IE_UpdateCtr<3 && setparam.IE==4  && (setparam.TV >= 400 && setparam.TV <= 449) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.09;//0.075;    //0.05;
//				m->E-= 0.09;//0.075;      //0.05;
//			}
//	
//			
//		}
//		
//		if(IE_UpdateCtr<3 && setparam.IE==4  && (setparam.TV >= 450 && setparam.TV <= 499) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.09;//0.075;    //0.05;
//				m->E-= 0.09;//0.075;      //0.05;
//			}
//			
//			
//		}
//		
//		
//		if(IE_UpdateCtr<3 && setparam.IE==4  && (setparam.TV >= 500 && setparam.TV <= 549) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.09;//0.075;    //0.05;
//				m->E-= 0.09;//0.075;      //0.05;
//			}
//			
//			
//		}
//		
//		
//			if(IE_UpdateCtr<3 && setparam.IE==4  && (setparam.TV >= 550 && setparam.TV <= 599) )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.09;//0.075;    //0.05;
//				m->E-= 0.09;//0.075;      //0.05;
//			}
//			
//			
//		}
//		
//			if(IE_UpdateCtr<2 && setparam.IE==4  && setparam.TV >= 600 )
//		{
//			if(m->BRperMin <= RR_IE)
//			{
//				m->I+= 0.09;//0.075;    //0.05;
//				m->E-= 0.09;//0.075;      //0.05;
//			}
//		
//			
//		}
	
		
	m->Ti=(m->Tbr/(m->I+m->E))*m->I;
	m->Te=(m->Tbr/(m->I+m->E))*m->E;	 

  /*Multiply By 2 Beacuse of Timer.*/
	 m->NpulI=2.0*(m->Angle * GR * MICROSTP / MotorStepAngle); 
	m->FpulI=m->NpulI/m->Ti;
	m->FpulE=m->NpulI/m->Te;
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
/*********************************************************************************************************************************************
		Timer 2 is used to receive the uart data every 100ms 
***********************************************************************************************************************************************/	
		
		if(htim->Instance == TIM2)
		{
			tim_flag=0;
				forwardCtr++;
				if(pd.mCMD==mCMDStart)
					{
						
						if(mRun1==mRun)
							{
								HAL_GPIO_TogglePin(STEP_M1_GPIO_Port,STEP_M1_Pin);		
							}
						if(mRun2==mRun)
							{
								HAL_GPIO_TogglePin(STEP_M2_GPIO_Port,STEP_M2_Pin);
							}						
							if(forwardCtr>=(m.NpulI) )		//before  if condition applied for mForware if condition and mRevers if condition 
							{     												//now if condition applied for mForware if condition only	
								//Check if Inspiration Angle is Achieved	  
										
											if(mDir==mForward)
											{
												stopMotorPins();
												forwardCtr=0;												
												//HAL_TIM_Base_Stop_IT(&htim3);  //Inspiration Complete Stop ISR
												HAL_TIM_Base_Stop_IT(&htim2); 
												/*Changing the Direction to Reverse*/
												HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET);
												HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET);
											
												/*Changing The Motor State to Reverse*/
												
												
												
												/*Sample The Volume as Inspiration is Over and Store in Array*/
												volumeOut=volume;
												brPMCtr++;
												minVolsum+=volume;
												if(brPMCtr>=m.BRperMin)
													{
														minVol=minVolsum/1000;
														if(minVol<minVolLimit)
															{
																minVolErrCtr++;
															}
														minVolsum=0;
														//minVol=0;
														brPMCtr=0;
													}
												tvCheckCtr++;
												if(tvCheckCtr>30 && setparam.RR <= 18)
													{
														if(tidalVolErrCtr >=6)
															{
																e=errTV;  //errRR_MINvol
															}
														else if(minVolErrCtr > 10)
															{
																e=errRR_MINvol;
															}
															
															minVolErrCtr=0;
														tidalVolErrCtr=0;
														tvCheckCtr=0;
														}
															
													if(tvCheckCtr>50 && (setparam.RR >=19 && setparam.RR <= 30))
													{
														if(tidalVolErrCtr >=10)
															{
																e=errTV;  //errRR_MINvol
															}
														else if(minVolErrCtr > 10)
															{
																e=errRR_MINvol;
															}
														
														minVolErrCtr=0;
														tidalVolErrCtr=0;
														tvCheckCtr=0;
												}
												volume=0;
												rawVolumeArray[volumeSample]=volumeOut; /*Taking 10 Samples*/
												volumeSample++;
												
												/*Check for 10 samples and Start the Averaging Process After This We 
												Will get correct Tidal Volume*/
												if(volumeSample>=maxVolumeSamples)
													{
														volumeSample=0;
														volumeSum=0;
														for(int i=0;i<maxVolumeSamples;i++)
															{
															volumeSum+=rawVolumeArray[i];
															}
														measuredVolume=volumeSum/maxVolumeSamples;
														measuredVolume*=1.09;
														if(measuredVolume<=220)
															{
																measuredVolume-=20;
																if(measuredVolume<0)
																	measuredVolume=0;
															}
														updateAngle=1;  /*Average Volume Measured Hence start Closed loop*/
														
														txParam.TV=measuredVolume;
														txPd.mCMD=mCMDStart;
													}
												
												if(huart1.gState==HAL_UART_STATE_READY && huart1.gState != HAL_UART_STATE_BUSY_TX && huart1.gState != HAL_UART_STATE_ERROR)
												{
													tim_flag =1;
													if(uart_flag !=1)
													{
													txParam.TV=measuredVolume;
													txPd.mCMD=mCMDStart;
													txPd.fault=e;
													packDataTX(tx,txParam,txPd);
												 HAL_UART_AbortTransmit_IT(&huart1);
												 HAL_UART_Transmit_IT(&huart1,tx,8);
												 for(int i =0;i<500;i++);	
													}
												}
												mDir=mReverse;
												/*Start Expiration Process*/
												inputFreq=m.FpulE;
												if(inputFreq!=0)
													{
													prdVal=timerFreq/inputFreq;
													}						
												TIM2->ARR=(uint16_t)(prdVal);  //-1	
												//HAL_TIM_Base_Start_IT(&htim3);
													HAL_TIM_Base_Start_IT(&htim2);
											}
										}
											if(mDir==mReverse)
											{						   //Check if Inspiration Angle is Achieved	
																		
												if((((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 0)    //checking motor1 ir sensor high
														|| ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 0)))
												{
													mRun1=mStop;																								// stop motor a while
												}
												if((((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 0) 
												|| ((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin) == 0))))   // checking motor2 ir sensor high
												{
													mRun2=mStop;																								// stop motor a while
												}
												if((((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 0) 
														|| ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 0))  //checking motor1 and motor2 ir sensor if both motors ir sensors high enter into if condition
														&&(((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 0) 
														|| ((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin) == 0))))
														{
															stopMotorPins();
															forwardCtr =0;	
															reverseMaxSteps=0;
															forwardMinSteps=0;
														//	HAL_TIM_Base_Stop_IT(&htim3);
															HAL_TIM_Base_Stop_IT(&htim2);
																/*After Inspiration We check whether we need to Update motor based on
																Closed loop output.*/
																if(updateMotor==1)
																	{
																		calcMotorFreq(&m);
																		tverrLimit=24000/m.BRperMin; //required for TV error
																		minVolLimit=m.BRperMin*TidalVolume*0.0009;//1000;
																		volLimit=(TidalVolume*0.9)-5;
																		updateMotor=0;
																		reverseMaxSteps=m.NpulI*1.5;
																		forwardMinSteps=m.NpulI*0.95;
																 }
																
																/*As Arms have Comeback Stop Timer*/
																//HAL_TIM_Base_Stop_IT(&htim3);
																
																/*Changing Direction Pins to Forward*/
																HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_RESET);
																HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_RESET);
																/*Change motor state to forward*/
																
																forwardCtr=0;
																mDir=mForward;
																/*Update Motor Frequency based on Inspiration Value*/
																inputFreq=m.FpulI;
																if(inputFreq!=0){
																	prdVal=timerFreq/inputFreq;
																}						
																TIM2->ARR=(uint16_t)(prdVal);	
																mRun1=mRun;
																mRun2=mRun;
															//	HAL_TIM_Base_Start_IT(&htim3);	
																HAL_TIM_Base_Start_IT(&htim2);	
														}
										
											}				
					}
			
				else
					{
						stopMotorPins();
					}
			
		}

/*********************************************************************************************************************************************
		Timer 3 is used to generate the pulses required to drive the Motors. Timer3 has be given a clock frequency of 1MHz. 
			This means each count of the timer will correspond to 10-6 seconds. 
			Thus, by counting the timer 3 ticks we can generate multiple frequencies by only changing the ARR (Auto Reload Register). 
			This differs from PWM because in PWM mode the frequency of the Pulse is fixed, and we are only changing the Duty Cycle. 
			To change the actual frequency in PWM mode we must reinitialize the whole peripheral. Thus, timer is selected instead of the PWM mode.
***********************************************************************************************************************************************/		

		if(htim->Instance==TIM3)
			{
				cnt_tim1++;
				if(huart1.RxState==HAL_UART_STATE_READY && huart1.RxState != HAL_UART_STATE_BUSY_RX && huart1.RxState != HAL_UART_STATE_BUSY_TX_RX)
				{
						
						HAL_UART_Receive_IT(&huart1,uartRecBuf,8u);
					uart_flag=0;
				}
			}
/**************************************************************************************************************************************
Timer 4 is used to generate an interrupt of 100ms. This timer is required to sample the gas flow from the flow sensor. 
		Every 100ms an I2C read is triggered, this reads the current flow value. To get the volume from the flow we must integrate it. 
		A fixed sampling time helps us to integrate the volume reading easily. Volume=flow*(tn – tn-1).
****************************************************************************************************************************************/			
		if(htim->Instance==TIM4)
			{
				timer15ctr++;
				if(timer15ctr>100)timer15ctr =0;
				//if(huart1.RxState==HAL_UART_STATE_READY && huart1.RxState != HAL_UART_STATE_BUSY_RX && huart1.RxState != HAL_UART_STATE_BUSY_TX_RX)
					//{
					
					//}
				
				
				
				//HAL_UART_Receive_IT(&huart1,uartRecBuf,8u); /*Initialize new Receive*/	
				int16_t errLimitV=0;
				if(TidalVolume>300)
					errLimitV=20;
				if(TidalVolume<=300)
					errLimitV=10;
				
					if(pd.mCMD==mCMDStart)
						{
							if(mDir==mForward)
								{
									calculateVolume();
								}
							if(mDir==mReverse)
								{
									/*working code start*/
									if(updateAngle==1)
										{	
											if(TidalVolume - measuredVolume>=errLimitV)
												{
													if(m.BRperMin<=12)
													{
														correctionAngleplus = 0.9; //old 0.9
													}
													if(m.BRperMin>12 && m.BRperMin <=20)
													{
														correctionAngleplus = 0.9;
													}
													if(m.BRperMin>20 && m.BRperMin <=30)
													{
														correctionAngleplus = 0.3;
													}
													m.Angle+=correctionAngleplus;
													if(m.Angle>43.0) m.Angle=43.0;
													updateMotor=1;
													if(measuredVolume<volLimit)
														{
															tidalVolErrCtr++;
														}
													else
														{
															if(tidalVolErrCtr>0)
																tidalVolErrCtr--;
														}
												}
											else if(TidalVolume - measuredVolume<=-errLimitV)
												{
													if(m.BRperMin<= 20)
													{
														correctionAngleminus = 0.5;
													}
													else if(m.BRperMin>=21)
													{
														correctionAngleminus = 0.5;
													}
													m.Angle-=correctionAngleminus;
													if(m.Angle<10)
														m.Angle=10;
													updateMotor=1;
												}
											else
												{
													updateMotor=0;
												}
											updateAngle=0;
										}
								} 
						}
				
			}
	}

void motorResetArms(void)
	{
		mDir=mReverse;
		//forwardCtr = 0;
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET);
		
		while((((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 1) 
			&& ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 1))
			||(((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 1) 
			&&	((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin) == 1))))
		{		
				//motor1 settings
			if (((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 1) 
				&& ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 1))
			{
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STEP_M1_GPIO_Port,STEP_M1_Pin,GPIO_PIN_SET);
				for(int i=0;i<125;i++);              //old 500ms delay pluse
				HAL_GPIO_WritePin(STEP_M1_GPIO_Port,STEP_M1_Pin,GPIO_PIN_RESET);
				for(int i=0;i<125;i++);
				//M1_revSteps++;			
			}
			
			//motor2 settings
			if (((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 1) 
					&& ((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin)) == 1))
			{
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(STEP_M2_GPIO_Port,STEP_M2_Pin,GPIO_PIN_SET);
				for(int i=0;i<125;i++);       //old 500ms delay pluse
				HAL_GPIO_WritePin(STEP_M2_GPIO_Port,STEP_M2_Pin,GPIO_PIN_RESET);
				for(int i=0;i<125;i++);
			}				
		}
		M1_revSteps=0;
		M2_revSteps=0;
		
		if((((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 0) 
		|| ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 0))
		&&(((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 0) 
		|| ((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin) == 0))))
		{
		//This is two line used for motor driver go to sleep mode because current flow will stop. 
		HAL_GPIO_WritePin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin,GPIO_PIN_RESET); //SET=Reverse
		HAL_GPIO_WritePin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin,GPIO_PIN_RESET); //SET=Reverse
		}
		motor1_driv_slp = HAL_GPIO_ReadPin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin);
		motor2_driv_slp = HAL_GPIO_ReadPin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin);
		stopMotorPins();
	}
	
	void motorResetArms_Start(void)
	{
		
		
		mDir=mReverse;
			
		//This is two line used for motor driver go to sleep mode because current flow will stop. 
		HAL_GPIO_WritePin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin,GPIO_PIN_SET); //SET=Reverse
		HAL_GPIO_WritePin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin,GPIO_PIN_SET); //SET=Reverse
		for(int i = 0;i<2000;i++);
		motor1_driv_slp = HAL_GPIO_ReadPin(SLEEP_M1_GPIO_Port,SLEEP_M1_Pin);
		motor2_driv_slp = HAL_GPIO_ReadPin(SLEEP_M2_GPIO_Port,SLEEP_M2_Pin);
		//forwardCtr = 0;
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET);
		
		while((((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 1) 
			&& ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 1))
			||(((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 1) 
			&&	((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin) == 1))))
		{		
				//motor1 settings
			if (((HAL_GPIO_ReadPin(IR_M1_GPIO_Port,IR_M1_Pin)) == 1) 
				&& ((HAL_GPIO_ReadPin(IR1_M1_GPIO_Port,IR1_M1_Pin)) == 1))
			{
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(STEP_M1_GPIO_Port,STEP_M1_Pin,GPIO_PIN_SET);
				for(int i=0;i<125;i++);              //old 500ms delay pluse
				HAL_GPIO_WritePin(STEP_M1_GPIO_Port,STEP_M1_Pin,GPIO_PIN_RESET);
				for(int i=0;i<125;i++);
				//M1_revSteps++;			
			}
			
			//motor2 settings
			if (((HAL_GPIO_ReadPin(IR_M2_GPIO_Port,IR_M2_Pin)) == 1) 
					&& ((HAL_GPIO_ReadPin(IR1_M2_GPIO_Port,IR1_M2_Pin)) == 1))
			{
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(STEP_M2_GPIO_Port,STEP_M2_Pin,GPIO_PIN_SET);
				for(int i=0;i<125;i++);       //old 500ms delay pluse
				HAL_GPIO_WritePin(STEP_M2_GPIO_Port,STEP_M2_Pin,GPIO_PIN_RESET);
				for(int i=0;i<125;i++);
			}				
		}
		M1_revSteps=0;
		M2_revSteps=0;
		
		
		stopMotorPins();
	}

static void calculateVolume(void)
		{
			if(measuredVolume == 0)
			{
				i2cbuf[0]=0x10;
				i2cbuf[1]=0x00;
				HAL_I2C_Master_Transmit(&hi2c1,slaveAddr,i2cbuf,2,10);
			}
		
			i2cbuf[2]=0;
			sensorError=HAL_I2C_Master_Receive(&hi2c1,slaveAddr,&i2cbuf[2],3,10);
			if(sensorError!=HAL_OK )
				{
					sensorErrorCtr++;
				}
			if(sensorErrorCtr>100)
				{
					if(uart_flag == 0 && tim_flag == 0)
					{
						if(huart1.gState==HAL_UART_STATE_READY && huart1.gState != HAL_UART_STATE_BUSY_TX && huart1.gState != HAL_UART_STATE_ERROR)
						{
							e=errMALFn;
							sensorErrorCtr=0;
							txPd.fault=e;
							txPd.mCMD=mCMDStart;
							packDataTX(tx,txParam,txPd);
							HAL_UART_Transmit_IT(&huart1,tx,8);
						}
					}
				}
			i2cbuf[7]=i2cbuf[2];
			i2cbuf[8]=i2cbuf[3];
			flow2=(i2cbuf[7]<<8)|i2cbuf[8];
			flow1=(float)flow2;
			flow3=(flow1-32000.0)/140.0;    //flow = (mesured flow - offsetflow)/scale flow -------->OFFSET_FLOW 32000.0F from datasheet ------->SCALE_FLOW 140.0F 	
			if(m.BRperMin >= 10 && m.BRperMin <=20)
			{
			flow=((flow3*1.1)/60)*1000;      //*1.3;  //1.125 is the calibration code
			}
			else if(m.BRperMin >= 21 && m.BRperMin <=30)
			{
			flow=((flow3*1.2)/60)*1000;      //*1.3;  //1.125 is the calibration code
			}
			if(flow>0)
				{
					volume+=(flow*0.1);
				//volume-=60.0;
				}
		}

static void stopMotorPins(void)
	{
	
		HAL_GPIO_WritePin(STEP_M1_GPIO_Port,STEP_M1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP_M2_GPIO_Port,STEP_M2_Pin,GPIO_PIN_RESET);
	}
	
	/************************************clear flags*********************************/
	void uart_irq(void)
{
        
	if 
	(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
		 // irq_handler(serial_irq_ids[id], RxIrq);
			__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
	}
	if 
	(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE) != RESET) {
		 volatile uint32_t *c = &huart1.Instance->DR;
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
		HAL_UART_Receive_IT(&huart1,uartRecBuf,8u);
	}
    
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
