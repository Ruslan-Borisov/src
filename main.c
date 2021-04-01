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
#define EndReceiv_UART2_DMA1_FromPC               (DMA1->LISR & DMA_HISR_TCIF5)
#define EndReceiv_UART3_DMA1_FromfMicrometer      (DMA1->LISR & DMA_LISR_TCIF1)
#define EndReceiv_UART4_DMA1_FromPresuareSensor   (DMA1->LISR & DMA_LISR_TCIF2)


#define flagPump                               (GPIOC->IDR |=  GPIO_BSRR_BS10;)


#define sizeBufDMA                              4174
#define SetPupe                                 1
#define ResetPupe                               2
#define SetSolenoid                             1
#define ResetSolenoid                           2
//================================================
#define request_X1_X2_X_Xmin_FirstOpticalSpot   1
#define request_X1_X2_X_Xmin_SecondOpticalSpot  2
#define request_X1_X2_X_Xmin_ThirdOpticalSpot   3
#define request_X1_X2_X_Xmin_FourhtOpticalSpot  4

#define request_centroid_FirstOpticalSpot       5
#define request_centroid_SecondOpticalSpot      6
#define request_centroid_ThirdOpticalSpot       7
#define request_centroid_FourhtOpticalSpot      8

#define request_measurementMillimeters_FirstOpticalSpot       9
#define request_measurementMillimeters_SecondOpticalSpot      10
#define request_measurementMillimeters_ThirdOpticalSpot       11
#define request_measurementMillimeters_FourhtOpticalSpot      12
#define request_measurementMillimeters_OllOpticalSpot         13

#define request_pressure                                      14
//================================================
#define sizeCharCoord                                         18
#define sizeCharCentroid                                      15
#define sizeCharPresuare                                       4
#define sizeCharPresuareForUART                               23
//================================================
#define endFirstSegment                                       810
#define endSecondSegment                                      1230
//================================================
#define coefficient_k1                                        2588110
#define coefficient_b1                                        0
//================================================
#define coefficient_k2                                        73560690
#define coefficient_b2                                        40637400
//================================================
#define coefficient_k3                                        12562400
#define coefficient_b3                                        104682000
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
	  char  id_OpticalSpot;
 	  uint16_t coordinate_x1;
	  uint16_t coordinate_x2;
	  uint16_t centerOfTheOpticalSpot_x;
	  uint16_t localMinimum;
	  uint16_t startOfSearch;
	  double centroid;
	  uint16_t amplitude; 
	  uint16_t reportPixelsToTheLeft;
	  uint16_t reportPixelsToTheRigh;
	  uint8_t resetPointOfTheReportToMeasure:1;
	  uint8_t errSerchCoordinate;
	  uint8_t rangeReport_Right_Left;
	  uint16_t saveCenterOfTheOpticalSpot_x;
  	double pointOfTheReportToMeasure;
	  double measurementMillimeters;
	  int measurementPresuare;
}parametersOpticalSpot;

//=======================================
//=======================================
// Parameters Of The Pneumatic System

 typedef union {
	  char charPresuare[sizeCharPresuare];
	  float  floatPresuare;
	  int16_t int16_DatadataMicrometrs;
	 } unioncharPresuareStructures ;

//=======================================
//=======================================
typedef struct {
		float PressureFromPiezoelectricSensor;
	  double setPressure;  
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

typedef union {
 char transferPackageForLabVIEW_coordinate[sizeCharCoord]; 
 char transferPackageForLabVIEW_centoide[sizeCharCentroid];
 char transferPackageForLabVIEW_Presuatr[sizeCharPresuareForUART];
}unionCharForUART;   

typedef union {
 uint8_t byteMass[2]; 
 int16_t dataMicrometrs;
}unionbyteMass; 

typedef struct {
   parametersOpticalSpot *FirstOpticalSpotStructures; 
   parametersOpticalSpot *SecondOpticalSpotStructures; 
   parametersOpticalSpot *ThirdOpticalSpotStructures; 
   parametersOpticalSpot *FourhtOpticalSpotStructures;
	 unionbyteMass *unionbyteMassStructures;  
   parametersOfThePneumaticSystem *PneumaticSystemStructures;
	 dataParser_UART *parser_UARTStructures;
	 uint8_t resetOllPointOfTheReportToMeasure;
   	
}pointerToStructuresForParser;
//=======================================
//=======================================
 //=======================================
//=======================================

// Oll fiags
		volatile  uint8_t flagsEndOfTheCCDLineSurvey_ADC1_DMA2;
		extern volatile uint8_t flagEndTransfer_UART2_DMA1_ForPC;
		volatile uint8_t flagEndReceiv_UART2_DMA1_FromPC;
		volatile uint8_t flagEndReceiv_UART3_DMA1_FromfMicrometer;
		volatile uint8_t dataRequestForPC;
	           uint8_t activationPump;
		volatile uint8_t activatingSolenoidValve;
//=======================================
//=======================================
// data arrays
   uint16_t mas_ADC1_DMA[sizeBufDMA];         
   short mas_DATA[sizeBufDMA];
   int16_t int16_DatadataMicrometrs;
   int int_DatadataMicrometrs;
  
   
//=======================================
//=======================================
 parametersOpticalSpot parametersFirstOpticalSpot; 
 parametersOpticalSpot parametersSecondOpticalSpot; 
 parametersOpticalSpot parametersThirdOpticalSpot; 
 parametersOpticalSpot parametersFourhtOpticalSpot;
 parametersOfThePneumaticSystem PneumaticSystem;
 pointerToStructuresForParser ToStructuresForParser;
 dataParser_UART parser_UART;
 unionCharForUART CharForUART;
 unioncharPresuareStructures charPresuare;
 unionbyteMass byteMass;



//=======================================
//=======================================



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
void calculationOpticalSpotCentroid(parametersOpticalSpot* nameStructure);
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
void initVariablesFirstOpticalSpot();
void initVariablesSecondOpticalSpot();
void initVariablesThirdOpticalSpot();
void initVariablesFourhtOpticalSpot();
void initVariablesPneumaticSystem(parametersOfThePneumaticSystem* nemeStructure);
void initFlags();
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_coordinate(parametersOpticalSpot *nemeStructe);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_centroid(parametersOpticalSpot *nemeStructe);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 void atoiOfDataPresuareSensor(parametersOfThePneumaticSystem *structure1, unioncharPresuareStructures *structure2);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void filterByteMassMicromrtrs(unionbyteMass *structure);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_Presuare(pointerToStructuresForParser *nemeStructe,  parametersOpticalSpot *nemeStructe1);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void pressureCalculation(parametersOpticalSpot* nemeStructure);
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
  MX_USART3_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	
//----------------------------------------
//---------------------------------------
initVariablesFirstOpticalSpot();
initVariablesSecondOpticalSpot();
initVariablesThirdOpticalSpot();
initVariablesFourhtOpticalSpot();
//----------------------------------------
//---------------------------------------
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
ToStructuresForParser.unionbyteMassStructures= (&byteMass);
//----------------------------------------
    HAL_UART_Receive_DMA(&huart3, (uint8_t *)&byteMass, 2);  
//----------------------------------------
    HAL_UART_Receive_DMA(&huart2, (uint8_t *)&parser_UART.ID, 5);    
//****************************************
    HAL_UART_Receive_DMA(&huart4, (uint8_t *)&charPresuare , 4);    
//****************************************
		HAL_ADC_Start(&hadc1);
//---------------------------------------
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&mas_ADC1_DMA, sizeBufDMA);
//*************************************** 


			 GPIOE->BSRR |=  GPIO_BSRR_BS10;
			 GPIOD->MODER &=~ GPIO_MODER_MODER12_Msk;
			 GPIOD->MODER |= GPIO_MODER_MODER12_0;	
			 if((!(GPIOD->ODR& GPIO_ODR_OD12))==SET)
					{
						 GPIOD->BSRR |=  GPIO_BSRR_BS12;
					}
			for(int i= 0; i< 5000; i++){}
			
			GPIOE->BSRR |=  GPIO_BSRR_BR10;	
			for(int i= 0; i< 5000; i++){}
			MX_TIM4_Init();
			GPIOE->BSRR |=  GPIO_BSRR_BS10;
				
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//**************************************** 
		HAL_TIM_Base_Start(&htim8);
//***************************************	
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//***************************************
 GPIOC->BSRR |=  GPIO_BSRR_BR7; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(charPresuare.floatPresuare >107500){
		GPIOC->BSRR |=  GPIO_BSRR_BR7; 
		activationPump = 0;
		}
		else if(charPresuare.floatPresuare <107500){
		// pump
			if(activationPump == SetPupe){ GPIOC->BSRR |=  GPIO_BSRR_BS7;  activationPump = 0;}
			if(activationPump == ResetPupe){GPIOC->BSRR |=  GPIO_BSRR_BR7; activationPump = 0;}
	}
		// SetSolenoid 
		if(activatingSolenoidValve == SetSolenoid){GPIOC->BSRR |=  GPIO_BSRR_BS8; activatingSolenoidValve =0;}
		if(activatingSolenoidValve == ResetSolenoid){GPIOC->BSRR |=  GPIO_BSRR_BR8; activatingSolenoidValve =0;}
		
		if (flagsEndOfTheCCDLineSurvey_ADC1_DMA2==1){
			//  GPIOC->BSRR |=  GPIO_BSRR_BR12;	
		  
			// Поиск координат
			if(dataRequestForPC == request_X1_X2_X_Xmin_FirstOpticalSpot){
				opticalSpotSearch(&parametersFirstOpticalSpot); 
				convertToCharAndPassUart_coordinate(&parametersFirstOpticalSpot);		
			}
			if(dataRequestForPC == request_X1_X2_X_Xmin_SecondOpticalSpot){
				opticalSpotSearch(&parametersSecondOpticalSpot);
				convertToCharAndPassUart_coordinate(&parametersSecondOpticalSpot);		
			}
			if(dataRequestForPC == request_X1_X2_X_Xmin_ThirdOpticalSpot){
				opticalSpotSearch(&parametersThirdOpticalSpot);
				convertToCharAndPassUart_coordinate(&parametersThirdOpticalSpot);		
			}
			if(dataRequestForPC == request_X1_X2_X_Xmin_FourhtOpticalSpot){
				 opticalSpotSearch(&parametersFourhtOpticalSpot);
				convertToCharAndPassUart_coordinate(&parametersFourhtOpticalSpot);					
			}
			// поиск центройда
			
			if(dataRequestForPC == request_centroid_FirstOpticalSpot ){ 
				opticalSpotSearch(&parametersFirstOpticalSpot); 
				calculationOpticalSpotCentroid(&parametersFirstOpticalSpot);	
        convertToCharAndPassUart_centroid(&parametersFirstOpticalSpot);				
			}
				if(dataRequestForPC == request_centroid_SecondOpticalSpot ){ 
				opticalSpotSearch(&parametersSecondOpticalSpot); 
				calculationOpticalSpotCentroid(&parametersSecondOpticalSpot);	
        convertToCharAndPassUart_centroid(&parametersSecondOpticalSpot);				
			}
					if(dataRequestForPC == request_centroid_ThirdOpticalSpot ){ 
				opticalSpotSearch(&parametersThirdOpticalSpot); 
				calculationOpticalSpotCentroid(&parametersThirdOpticalSpot);	
        convertToCharAndPassUart_centroid(&parametersThirdOpticalSpot);				
			}
						if(dataRequestForPC == request_centroid_FourhtOpticalSpot ){ 
				opticalSpotSearch(&parametersFourhtOpticalSpot); 
				calculationOpticalSpotCentroid(&parametersFourhtOpticalSpot);	
        convertToCharAndPassUart_centroid(&parametersFourhtOpticalSpot);				
			}
						if(dataRequestForPC==request_pressure){
					  pressureCalculation(&parametersFirstOpticalSpot);
						convertToCharAndPassUart_Presuare(&ToStructuresForParser, &parametersFirstOpticalSpot);
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
	   nameStructure->errSerchCoordinate = 0;
	 for(uint16_t i = nameStructure->saveCenterOfTheOpticalSpot_x - nameStructure->rangeReport_Right_Left; i<nameStructure->saveCenterOfTheOpticalSpot_x + nameStructure->rangeReport_Right_Left; i++){
			if(mas_DATA[i] <= nameStructure->amplitude){
			nameStructure->coordinate_x1 =i;
			nameStructure->errSerchCoordinate = 1;
			break;
			}
	 }
	 for(uint16_t i = nameStructure->coordinate_x1+10; i<nameStructure->saveCenterOfTheOpticalSpot_x + nameStructure->rangeReport_Right_Left; i++){
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
	    nameStructure->saveCenterOfTheOpticalSpot_x = nameStructure->centerOfTheOpticalSpot_x; 
	 
}
// ++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++		
void calculationOpticalSpotCentroid(parametersOpticalSpot* nameStructure){
	
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



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
			 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	     GPIOD->BSRR |=  GPIO_BSRR_BR6;
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
				GPIOD->BSRR |=  GPIO_BSRR_BS6;
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
	
				if(EndReceiv_UART2_DMA1_FromPC==RESET){
				flagEndReceiv_UART2_DMA1_FromPC=1;
			  parserOfDataFromPC(&ToStructuresForParser);
				} 
				if(EndReceiv_UART3_DMA1_FromfMicrometer==RESET){ 
				filterByteMassMicromrtrs(&byteMass);
				flagEndReceiv_UART3_DMA1_FromfMicrometer =1;
				} 
				if(EndReceiv_UART4_DMA1_FromPresuareSensor==RESET){ 
				atoiOfDataPresuareSensor(&PneumaticSystem, &charPresuare);
				}
	}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesFirstOpticalSpot(){
	
	parametersFirstOpticalSpot.id_OpticalSpot = 'A';
	parametersFirstOpticalSpot.saveCenterOfTheOpticalSpot_x = 394; 
	parametersFirstOpticalSpot.amplitude = 2900;
	parametersFirstOpticalSpot.centroid= 0;
	parametersFirstOpticalSpot.measurementMillimeters = 0;
	parametersFirstOpticalSpot.localMinimum = 0;
	parametersFirstOpticalSpot.rangeReport_Right_Left = 40; 
	parametersFirstOpticalSpot.pointOfTheReportToMeasure = 0;
	parametersFirstOpticalSpot.resetPointOfTheReportToMeasure = 0;
	parametersFirstOpticalSpot.reportPixelsToTheLeft = 50;
	parametersFirstOpticalSpot.reportPixelsToTheRigh= 50;
	parametersFirstOpticalSpot.coordinate_x1 = 0;
	parametersFirstOpticalSpot.coordinate_x2 = 0;
	parametersFirstOpticalSpot.centerOfTheOpticalSpot_x = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesSecondOpticalSpot(){
	
	parametersSecondOpticalSpot.id_OpticalSpot = 'B'; 
	parametersSecondOpticalSpot.saveCenterOfTheOpticalSpot_x = 1246;
	parametersSecondOpticalSpot.amplitude = 2900;
	parametersSecondOpticalSpot.centroid= 0;
	parametersSecondOpticalSpot.measurementMillimeters = 0;
	parametersSecondOpticalSpot.localMinimum = 0;
	parametersSecondOpticalSpot.rangeReport_Right_Left = 85;
	parametersSecondOpticalSpot.pointOfTheReportToMeasure = 0;
	parametersSecondOpticalSpot.resetPointOfTheReportToMeasure = 0;
	parametersSecondOpticalSpot.reportPixelsToTheLeft = 70;
	parametersSecondOpticalSpot.reportPixelsToTheRigh= 70;
	parametersSecondOpticalSpot.coordinate_x1 = 0;
	parametersSecondOpticalSpot.coordinate_x2 = 0;
	parametersSecondOpticalSpot.centerOfTheOpticalSpot_x = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesThirdOpticalSpot(){
	
	parametersThirdOpticalSpot.id_OpticalSpot = 'C'; 
	parametersThirdOpticalSpot.saveCenterOfTheOpticalSpot_x = 2097;
	parametersThirdOpticalSpot.amplitude = 2900;
	parametersThirdOpticalSpot.centroid= 0;
	parametersThirdOpticalSpot.measurementMillimeters = 0;
	parametersThirdOpticalSpot.localMinimum = 0;
	parametersThirdOpticalSpot.rangeReport_Right_Left = 95;
	parametersThirdOpticalSpot.pointOfTheReportToMeasure = 0;
	parametersThirdOpticalSpot.resetPointOfTheReportToMeasure = 0;
	parametersThirdOpticalSpot.reportPixelsToTheLeft = 80;
	parametersThirdOpticalSpot.reportPixelsToTheRigh= 80;
	parametersThirdOpticalSpot.coordinate_x1 = 0;
	parametersThirdOpticalSpot.coordinate_x2 = 0;
	parametersThirdOpticalSpot.centerOfTheOpticalSpot_x = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesFourhtOpticalSpot(){
	
	parametersFourhtOpticalSpot.id_OpticalSpot = 'D'; 
	parametersFourhtOpticalSpot.saveCenterOfTheOpticalSpot_x = 2951;
	parametersFourhtOpticalSpot.amplitude = 2900;
	parametersFourhtOpticalSpot.centroid= 0;
	parametersFourhtOpticalSpot.measurementMillimeters = 0;
	parametersFourhtOpticalSpot.localMinimum = 0;
  parametersFourhtOpticalSpot.rangeReport_Right_Left = 105;
	parametersFourhtOpticalSpot.pointOfTheReportToMeasure = 0;
	parametersFourhtOpticalSpot.resetPointOfTheReportToMeasure = 0;
	parametersFourhtOpticalSpot.reportPixelsToTheLeft = 85;
	parametersFourhtOpticalSpot.reportPixelsToTheRigh= 85;
	parametersFourhtOpticalSpot.coordinate_x1 = 0;
	parametersFourhtOpticalSpot.coordinate_x2 = 0;
	parametersFourhtOpticalSpot.centerOfTheOpticalSpot_x = 0;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void initVariablesPneumaticSystem(parametersOfThePneumaticSystem* nemeStract){
	nemeStract->PressureFromPiezoelectricSensor =0;
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
		activationPump =0;
	 activatingSolenoidValve =0;
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
			if(rx_input==1001){dataRequestForPC = request_X1_X2_X_Xmin_FirstOpticalSpot;}
			if(rx_input==1002){dataRequestForPC = request_X1_X2_X_Xmin_SecondOpticalSpot;}
		  if(rx_input==1003){dataRequestForPC = request_X1_X2_X_Xmin_ThirdOpticalSpot;}
			if(rx_input==1004){dataRequestForPC = request_X1_X2_X_Xmin_FourhtOpticalSpot;}
			if(rx_input==1005){dataRequestForPC = request_centroid_FirstOpticalSpot;}
			if(rx_input==1006){dataRequestForPC = request_centroid_SecondOpticalSpot;}
  		if(rx_input==1007){dataRequestForPC = request_centroid_ThirdOpticalSpot;}
			if(rx_input==1008){dataRequestForPC = request_centroid_FourhtOpticalSpot;}
	  	if(rx_input==1009){dataRequestForPC = request_measurementMillimeters_FirstOpticalSpot;}
			if(rx_input==1010){dataRequestForPC = request_measurementMillimeters_SecondOpticalSpot;}
			if(rx_input==1011){dataRequestForPC = request_measurementMillimeters_ThirdOpticalSpot;}
			if(rx_input==1012){dataRequestForPC = request_measurementMillimeters_FourhtOpticalSpot;}
		  if(rx_input==1013){dataRequestForPC = request_measurementMillimeters_OllOpticalSpot;}
   		if(rx_input==1014){dataRequestForPC = request_pressure;} 
	  break;
			case 'X':
			if(rx_input==1001){activationPump = SetPupe;} 
			if(rx_input==1003){activationPump = ResetPupe;}
		  if(rx_input==1002){activatingSolenoidValve = SetSolenoid;}
		  if(rx_input==1004){activatingSolenoidValve = ResetSolenoid;}
	  break;
			defulte:
			
    break;			
	}
	

}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_coordinate(parametersOpticalSpot *nemeStructe){
	
			sprintf(CharForUART.transferPackageForLabVIEW_coordinate, "%c%d%d%d%d%d\n",nemeStructe->id_OpticalSpot, (nemeStructe->coordinate_x1+1000),
																		(nemeStructe->coordinate_x2+1000), 
																		(nemeStructe->centerOfTheOpticalSpot_x+1000),
																		(nemeStructe->localMinimum+1000),
	                                  (nemeStructe->errSerchCoordinate));
	      if(flagEndTransfer_UART2_DMA1_ForPC==1){
        while(flagEndTransfer_UART2_DMA1_ForPC >0){}
				}
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&CharForUART.transferPackageForLabVIEW_coordinate, sizeCharCoord+1);// 	
			  flagEndTransfer_UART2_DMA1_ForPC =1;		
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_centroid(parametersOpticalSpot *nemeStructe){
			sprintf(CharForUART.transferPackageForLabVIEW_centoide, "I%c%d%d\n",nemeStructe->id_OpticalSpot,((int)((nemeStructe->centroid+1000)*100000)),(nemeStructe->centerOfTheOpticalSpot_x+1000));
			if(flagEndTransfer_UART2_DMA1_ForPC==1){      
				while(flagEndTransfer_UART2_DMA1_ForPC >0){}
				}
			   HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CharForUART.transferPackageForLabVIEW_centoide, sizeCharCentroid+1);	
				 flagEndTransfer_UART2_DMA1_ForPC =1;			
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void convertToCharAndPassUart_Presuare(pointerToStructuresForParser *nemeStructe,  parametersOpticalSpot *nemeStructe1){
			sprintf(CharForUART.transferPackageForLabVIEW_Presuatr, "O%d%d\%d\n",(int)(nemeStructe->PneumaticSystemStructures->PressureFromPiezoelectricSensor*100)+10000000,
	           int_DatadataMicrometrs+50000, nemeStructe1->measurementPresuare+100000000);
			if(flagEndTransfer_UART2_DMA1_ForPC==1){      
				while(flagEndTransfer_UART2_DMA1_ForPC >0){}
				}
			   HAL_UART_Transmit_DMA(&huart2, (uint8_t*)CharForUART.transferPackageForLabVIEW_Presuatr, sizeCharPresuareForUART+1);	
				 flagEndTransfer_UART2_DMA1_ForPC =1;			
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void atoiOfDataPresuareSensor(parametersOfThePneumaticSystem *structure1, unioncharPresuareStructures *structure2 ){
  structure1->PressureFromPiezoelectricSensor = structure2->floatPresuare;
	}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void filterByteMassMicromrtrs(unionbyteMass *structure){
  int16_DatadataMicrometrs =structure->byteMass[1]|(structure->byteMass[0]<<8);
	int_DatadataMicrometrs = (int)int16_DatadataMicrometrs;
	}
	
	
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void pressureCalculation(parametersOpticalSpot* nemeStructure){
		 int millimeters = (int)(nemeStructure->measurementMillimeters*1000);
		if(millimeters <= endFirstSegment){
			nemeStructure->measurementPresuare = coefficient_k1*millimeters-coefficient_b1;
		}	
		if(millimeters > endFirstSegment && millimeters <= endSecondSegment ){
		  nemeStructure->measurementPresuare = coefficient_k2*millimeters-coefficient_b2;
		}	
		if(millimeters > endSecondSegment){
		  nemeStructure->measurementPresuare = coefficient_k3*millimeters-coefficient_b3;
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
