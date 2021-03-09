/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//=======================================
//=======================================
#define EndReceiv_UART2_DMA2_FromPC            (DMA1->LISR & DMA_HISR_TCIF5)
#define EndReceiv_UART3_DMA1_FromfMicrometer   (DMA1->LISR & DMA_LISR_TCIF1)
#define sizeBufDMA                              4174
#define size_ADC2                               10
#define SetPupe                                 1
#define ResetPupe                               2
#define SetSolenoid                             1
#define ResetSolenoid                           2





//=======================================
//=======================================
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

//=======================================
//=======================================
 // Parameters of the optical spot 
typedef struct {
 	  uint16_t coordinate_x1;
	  uint16_t coordinate_x2;
	  uint16_t centerOfTheOpticalSpot_x;
	  uint16_t localMinimum;
	  uint16_t startОfSearch;
	  double centroid;
	  uint16_t amplitude; 
	  uint16_t reportPixelsToTheLeft;
	  uint16_t reportPixelsToTheRigh;
	  uint8_t resetPointOfTheReportToMeasure;
  	double pointOfTheReportToMeasure;
	  double measurementMillimeters;
}parametersOpticalSpot;

//=======================================
//=======================================
// Parameters Of The Pneumatic System
typedef struct {
		double PressureFromPiezoelectricSensor;
	  double setPressure;
		uint8_t activationPump;
		uint8_t activatingSolenoidValve;
}parametersOfThePneumaticSystem;

//=======================================
//=======================================
// pointer to structures for the parser
typedef struct{
	char ID;
	char input_mas[4];
}dataParser_UART;

//=======================================
//=======================================
typedef struct {
   parametersOpticalSpot *FirstOpticalSpotStructures; 
   parametersOpticalSpot *SecondOpticalSpotStructures; 
   parametersOpticalSpot *ThirdOpticalSpotStructures; 
   parametersOpticalSpot *FourhtOpticalSpotStructures;
   parametersOfThePneumaticSystem *PneumaticSystemStructures;
	 dataParser_UART *parser_UARTStructures;
	 uint8_t resetOllPointOfTheReportToMeasure;
   	
}pointerToStructuresForParser;
//=======================================
//=======================================
// Oll fiags
		volatile uint8_t flagsEndOfTheCCDLineSurvey_ADC1_DMA2;
		volatile uint8_t flagEndTransfer_UART2_DMA1_ForPC;
		volatile uint8_t flagEndReceiv_UART2_DMA1_FromPC;
		volatile uint8_t flagEndReceiv_UART3_DMA1_FromfMicrometer;
		volatile uint8_t dataRequestForPC;
//=======================================
//=======================================
// data arrays
   uint16_t mas_ADC1_DMA[sizeBufDMA];         
   short mas_DATA[sizeBufDMA];
   short mas_DATA_Pressure[size_ADC2];
   
//=======================================
//=======================================
 parametersOpticalSpot parametersFirstOpticalSpot; 
 parametersOpticalSpot parametersSecondOpticalSpot; 
 parametersOpticalSpot parametersThirdOpticalSpot; 
 parametersOpticalSpot parametersFourhtOpticalSpot;
 parametersOfThePneumaticSystem PneumaticSystem;
 pointerToStructuresForParser ToStructuresForParser;
 dataParser_UART parser_UART;
//=======================================
//=======================================

char P1x [18];

//=======================================
//=======================================


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
  PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

return ch;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void opticalSpotSearch(parametersOpticalSpot* nameStructure);
void сalculationОfTheOpticalSpotCentroid(parametersOpticalSpot* nameStructure);
void pointReportToMeasure(parametersOpticalSpot* nameStructure);
void calculationOfDeflectionMillimeters (parametersOpticalSpot* nameStructure);
void pressureSensorProcessing(parametersOfThePneumaticSystem *nameStructure);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void parserOfDataFromPC(pointerToStructuresForParser *nemeStructure);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesOpticalSpot(parametersOpticalSpot* nemeStract);
void initVariablesPneumaticSystem(parametersOfThePneumaticSystem* nemeStract);
void initFlags();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
//---------------------------------------
//---------------------------------------
initVariablesOpticalSpot(&parametersFirstOpticalSpot);
initVariablesOpticalSpot(&parametersSecondOpticalSpot);
initVariablesOpticalSpot(&parametersThirdOpticalSpot);
initVariablesOpticalSpot(&parametersFourhtOpticalSpot);
initVariablesPneumaticSystem(&PneumaticSystem);
//---------------------------------------
//---------------------------------------
ToStructuresForParser.FirstOpticalSpotStructures = (&parametersFirstOpticalSpot);
ToStructuresForParser.FourhtOpticalSpotStructures= (&parametersFourhtOpticalSpot);
ToStructuresForParser.parser_UARTStructures = (&parser_UART);
ToStructuresForParser.PneumaticSystemStructures = (&PneumaticSystem);
ToStructuresForParser.resetOllPointOfTheReportToMeasure = 0;
ToStructuresForParser.SecondOpticalSpotStructures = (&parametersSecondOpticalSpot);
ToStructuresForParser.ThirdOpticalSpotStructures = (&parametersThirdOpticalSpot);


//****************************************
		HAL_ADC_Start(&hadc1);
//---------------------------------------
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&mas_ADC1_DMA, sizeBufDMA);
//*************************************** 
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)&parser_UART.ID, 5); 
//***************************************
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//*************************************** 
		HAL_TIM_Base_Start(&htim8);
//***************************************	
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//***************************************
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (flagsEndOfTheCCDLineSurvey_ADC1_DMA2==1){
		    parametersFirstOpticalSpot.startОfSearch = 100;
				opticalSpotSearch(&parametersFirstOpticalSpot); 
				opticalSpotSearch(&parametersSecondOpticalSpot);
				opticalSpotSearch(&parametersThirdOpticalSpot);
			  opticalSpotSearch(&parametersFourhtOpticalSpot);
			
			
			if(flagEndTransfer_UART2_DMA1_ForPC ==0){
				flagEndTransfer_UART2_DMA1_ForPC =1;
				sprintf(P1x, "A%d%d%d%d\n", (parametersFirstOpticalSpot.coordinate_x1+1000),
																		(parametersFirstOpticalSpot.coordinate_x2+1000), 
																		(parametersFirstOpticalSpot.centerOfTheOpticalSpot_x+1000),
																		(parametersFirstOpticalSpot.localMinimum+1000));		 
    		
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)P1x, 18); 				
			}
	
				
		  flagsEndOfTheCCDLineSurvey_ADC1_DMA2 = 0;	
		}

		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 140;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++

void opticalSpotSearch(parametersOpticalSpot* nameStructure){
	 for(uint16_t i = 200; i<sizeBufDMA; i++){
			if(mas_DATA[i] <= nameStructure->amplitude){
			nameStructure->coordinate_x1 =i;
			break;
			}
	 }
	 for(uint16_t i = nameStructure->coordinate_x1+10; i<sizeBufDMA; i++){
			if(mas_DATA[i] >= nameStructure->amplitude){
			 nameStructure->coordinate_x2 = i;
				break;
			}
	 }
	 short min = 3500;
	 for(uint16_t i = nameStructure->coordinate_x1-10; i< nameStructure->coordinate_x2+10; i++){
			
				if(mas_DATA[i] < min){
					min = mas_DATA[i];
					nameStructure->localMinimum =i;
				}
	 }
			nameStructure->centerOfTheOpticalSpot_x = (nameStructure->coordinate_x1 + nameStructure->coordinate_x2)/2;
			nameStructure->startОfSearch =  nameStructure->coordinate_x1 + 100;
}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++		
void сalculationОfTheOpticalSpotCentroid(parametersOpticalSpot* nameStructure){
		double summaAmplitud_x = 0;
		double summaAplituda_Pixse_x = 0;
    for( uint16_t i = (nameStructure->centerOfTheOpticalSpot_x - nameStructure->reportPixelsToTheLeft); 
	                   i<(nameStructure->centerOfTheOpticalSpot_x + nameStructure->reportPixelsToTheRigh); i++ ){
			 summaAmplitud_x =  summaAmplitud_x +  (double)mas_DATA[i];
			 summaAplituda_Pixse_x =  summaAplituda_Pixse_x + (double)mas_DATA[i]*i;
		}	 
	 nameStructure->centroid = summaAplituda_Pixse_x/summaAmplitud_x;
}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++
void pointReportToMeasure(parametersOpticalSpot* nameStructure){
		nameStructure->pointOfTheReportToMeasure = nameStructure->centroid;
}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++
void calculationOfDeflectionMillimeters (parametersOpticalSpot* nameStructure){
	 nameStructure->measurementMillimeters = (nameStructure->centroid  - nameStructure->pointOfTheReportToMeasure)*0.007;
}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++
void pressureSensorProcessing(parametersOfThePneumaticSystem *nameStructure){
	uint16_t sum_ADC2_DMA = 0;
	for(int i = 0; i< size_ADC2; i++){                
	HAL_ADC_Start(&hadc2);// 
  while(!(ADC2->SR & ADC_SR_EOC)); 
  mas_DATA_Pressure[i] = ADC2->DR;
	sum_ADC2_DMA = sum_ADC2_DMA + mas_DATA_Pressure[i];
	}
	nameStructure->PressureFromPiezoelectricSensor = ((float)sum_ADC2_DMA/((float)(size_ADC2)) - 578)*75842/4095;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
			 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			 GPIOE->BSRR |=  GPIO_BSRR_BR10;
			 HAL_TIM_Base_Stop(&htim8);
			 GPIOD->MODER &=~ GPIO_MODER_MODER12_Msk;
			 GPIOD->MODER |= GPIO_MODER_MODER12_0;
				
			 if((!(GPIOD->ODR& GPIO_ODR_OD12))==SET)
					{
						 GPIOD->BSRR |=  GPIO_BSRR_BS12;
					}
				
				if (flagsEndOfTheCCDLineSurvey_ADC1_DMA2==0){
						for(int i=0; i<sizeBufDMA; i++){
						mas_DATA[i]=(short)mas_ADC1_DMA[i];
						}
						flagsEndOfTheCCDLineSurvey_ADC1_DMA2 = 1;
				}
				else {
						for(int i=0; i<50000; i++){}
				}
			
//				GPIOD->MODER &=~ GPIO_MODER_MODER12_Msk;
//				GPIOD->MODER |= GPIO_MODER_MODER12_1;
				
        MX_TIM4_Init();
				for(uint8_t i=0; i<5; i++){}
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				for(uint8_t i=0; i<5; i++){}
				HAL_TIM_Base_Start(&htim8);	
        GPIOE->BSRR |=  GPIO_BSRR_BS10;	
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					
	}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
				if(EndReceiv_UART2_DMA2_FromPC==RESET){
				flagEndReceiv_UART2_DMA1_FromPC=1;
			  parserOfDataFromPC(&ToStructuresForParser);
				} 
				if(EndReceiv_UART3_DMA1_FromfMicrometer==RESET){ 
				flagEndReceiv_UART3_DMA1_FromfMicrometer =1;
				} 
				
	}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	  
			flagEndTransfer_UART2_DMA1_ForPC = 0;
				
	}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void initVariablesOpticalSpot(parametersOpticalSpot* nemeStract){
	
	nemeStract->amplitude = 2900;
	nemeStract->centroid= 0;
	nemeStract->measurementMillimeters = 0;
	nemeStract->localMinimum = 0;
	nemeStract->pointOfTheReportToMeasure = 0;
	nemeStract->startОfSearch = 0; 
	nemeStract->resetPointOfTheReportToMeasure = 0;
	nemeStract->reportPixelsToTheLeft = 50;
	nemeStract->reportPixelsToTheRigh= 50;
	nemeStract->coordinate_x1 = 0;
	nemeStract->coordinate_x2 = 0;
	nemeStract->centerOfTheOpticalSpot_x = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesPneumaticSystem(parametersOfThePneumaticSystem* nemeStract){
	nemeStract->PressureFromPiezoelectricSensor =0;
	nemeStract->activatingSolenoidValve=0;
	nemeStract->activationPump=0;
	nemeStract->setPressure=0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initFlags(){
		flagEndTransfer_UART2_DMA1_ForPC = 0;
		flagEndReceiv_UART2_DMA1_FromPC = 0;
	  flagEndReceiv_UART3_DMA1_FromfMicrometer = 0;
		flagsEndOfTheCCDLineSurvey_ADC1_DMA2 = 0;
		dataRequestForPC = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void parserOfDataFromPC(pointerToStructuresForParser *nemeStructure){
	GPIOD->BSRR |=  GPIO_BSRR_BR5;
	uint16_t rx_input = (uint16_t)(atoi((*nemeStructure).parser_UARTStructures->input_mas));
   
	switch((*nemeStructure).parser_UARTStructures->ID)
	{ 
		case 'A':
		nemeStructure->FirstOpticalSpotStructures->reportPixelsToTheRigh = (rx_input-1000);
	  break;
		case 'B':
		nemeStructure->FirstOpticalSpotStructures->reportPixelsToTheLeft = (rx_input-1000);
	  break;
		case 'C':
		nemeStructure->SecondOpticalSpotStructures->reportPixelsToTheRigh = (rx_input-1000);
	  break;
		case 'D':
		nemeStructure->SecondOpticalSpotStructures->reportPixelsToTheLeft = (rx_input-1000);
	  break;
		case 'E':
	  nemeStructure->ThirdOpticalSpotStructures->reportPixelsToTheRigh = (rx_input-1000);
	  break;
		case 'F':
		nemeStructure->ThirdOpticalSpotStructures->reportPixelsToTheLeft = (rx_input-1000);
	  break;
		case 'H':
		nemeStructure->FourhtOpticalSpotStructures->reportPixelsToTheRigh = (rx_input-1000);
	  break;
		case 'G':
		nemeStructure->FourhtOpticalSpotStructures->reportPixelsToTheLeft = (rx_input-1000);	
	  break;
		case 'I':
		(*nemeStructure).FirstOpticalSpotStructures->amplitude = (rx_input-1000);
	  break;
		case 'J':
		nemeStructure->SecondOpticalSpotStructures->amplitude = (rx_input-1000);
	  break;
		case 'K':
		nemeStructure->ThirdOpticalSpotStructures->amplitude =(rx_input-1000);
	  break;
		case 'L':
		nemeStructure->FourhtOpticalSpotStructures->amplitude = (rx_input-1000);		
	  break;
		case 'M':
	  nemeStructure->resetOllPointOfTheReportToMeasure = (uint8_t)(rx_input-1000);
	  break;
		case 'N':
		nemeStructure->FirstOpticalSpotStructures->resetPointOfTheReportToMeasure = (rx_input-1000); 
	  break;
		case 'O':
		nemeStructure->SecondOpticalSpotStructures->resetPointOfTheReportToMeasure = (rx_input-1000); 
	  break;
		case 'P':
		nemeStructure->ThirdOpticalSpotStructures->resetPointOfTheReportToMeasure = (rx_input-1000); 
	  break;
		case 'Q':
		nemeStructure->FourhtOpticalSpotStructures->resetPointOfTheReportToMeasure = (rx_input-1000); 
	  break;
		case 'R':
		nemeStructure->PneumaticSystemStructures->setPressure = (rx_input-1000); 
	  break;	
		case 'W':
			if(rx_input==1001){dataRequestForPC = 1;}// Request for coordinates of the first optical spot
			if(rx_input==1002){dataRequestForPC = 2;}// Request for the coordinates of the second optical spot
		  if(rx_input==1003){dataRequestForPC = 3;}// Request for coordinates of the third optical spot
			if(rx_input==1004){dataRequestForPC = 4;}// Request for coordinates of the fourth optical spot
			if(rx_input==1005){dataRequestForPC = 5;}// Request for coordinates of the first cetroyd of the optical spot
			if(rx_input==1006){dataRequestForPC = 6;}// Request for coordinates of the second cetroyd of the optical spot
  		if(rx_input==1007){dataRequestForPC = 7;}// Request for coordinates of the third cetroyd of the optical spot
			if(rx_input==1008){dataRequestForPC = 8;}// Request for coordinates of the fourth cetroyd of the optical spot
	  	if(rx_input==1009){dataRequestForPC = 9;}// Р?Р·Р�?РµСЂРµРЅРёРµ в„– 1
			if(rx_input==1010){dataRequestForPC = 10;}// Р?Р·Р�?РµСЂРµРЅРёРµ в„– 2
			if(rx_input==1011){dataRequestForPC = 11;}//Р?Р·Р�?РµСЂРµРЅРёРµ в„– 3
			if(rx_input==1012){dataRequestForPC = 12;}//Р?Р·Р�?РµСЂРµРЅРёРµ в„– 4
		  if(rx_input==1013){dataRequestForPC = 13;}// Р?Р·Р�?РµСЂРµРЅРёРµ в„– 1-4
   		if(rx_input==1014){dataRequestForPC = 14;} // РЈРїСЂР°РІР»РµРЅРёРµ РЅР°СЃРѕСЃРѕР�?
	  break;
			case 'X':
			if(rx_input==1001){nemeStructure->PneumaticSystemStructures->activationPump = SetPupe;} 
			if(rx_input==1002){nemeStructure->PneumaticSystemStructures->activationPump = ResetPupe;}
		  if(rx_input==1003){nemeStructure->PneumaticSystemStructures->activatingSolenoidValve = SetSolenoid;}
		  if(rx_input==1004){nemeStructure->PneumaticSystemStructures->activatingSolenoidValve = ResetSolenoid;}
	  break;
			defulte:
			
    break;			
	}
	

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


		

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
