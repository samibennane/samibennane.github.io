/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;


typedef enum {
	Idle,
	Avancer_Jusqua_Obstacle,
	Tourner,
	Avancer_1_Metre
} State;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/*
uint32_t valeur_batterie = 0;*/


// États
volatile int etat_robot =0;
volatile State etat_actuel;
volatile int tache_terminee;

// PID
volatile float vitesse_actuelle;
volatile float vitesse_precedente;
volatile float erreur;
volatile float erreur_precedente;
volatile float cumul_erreurs;

// Consignes et distances
volatile float consigne_pwm;
volatile float cible;
volatile float signal_commande;
volatile float debut_distance;
volatile float fin_distance;
volatile float distance_totale;
volatile float distance_initiale;
volatile float dist_obstacle;
volatile int changement_trajectoire;

// Encoders
volatile int ticks_precedents;

// Divers
volatile int arret_force;

// Constantes physiques (tu peux les laisser dans USER CODE BEGIN PD)
#define NB_ECHANTILLONS 10
#define DISTANCE_ARRET 8
#define RAYON_ROUE 3.0
#define VITESSE_MAX 15
#define VITESSE_MIN 2.2
#define SEUIL_DECELERATION 0.8
#define MARGE_DISTANCE 2
#define VITESSE_MAX 15
#define DISTANCE_ARRET 8


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Redirection de printf vers l'UART2 */



#define SEUIL_3V 3723
#define NB_CHAR 30
#define DIRG_Pin GPIO_PIN_2
char msg [NB_CHAR];
volatile uint16_t ADC1_Val;
uint32_t raw;
uint32_t IR3, IR4;
uint32_t tension = 0;
char uart_msg[50];
volatile uint8_t T =1;
uint16_t compteur_distance = 0;
uint16_t compteur = 0;
uint16_t compteur_ = 0;
int16_t pos_d_actuelle = 0, pos_d_prev = 0;
int16_t pos_g_actuelle = 0, pos_g_prev = 0;
float vitesse_d = 0, vitesse_g = 0;
int32_t erreur_d = 0, erreur_g = 0;
uint32_t pwm_consigne_d = 6000, pwm_consigne_g = 6000;
int32_t consigne_d = 0;
int32_t consigne_g = 0;
int32_t prec_consigne_d = 0;
int32_t prec_consigne_g = 0;
// Constantes physiques du robot
#define RAYON_CM 3.0f                          // Rayon de la roue
#define TICKS_PAR_TOUR 333.33f                 // Nombre de ticks par tour complet
#define CIRCONF_CM (2.0f * 3.1416f * RAYON_CM) // Circonférence de la roue
#define CONVERSION_CM_PAR_TICK ((100.0f * CIRCONF_CM) / TICKS_PAR_TOUR) // ≈ 5.655
#define KP 100.0f
volatile float Vd = 0;
volatile float Vg = 0;
volatile uint8_t T_B1=0;
//uint32_t tension_mV;*/



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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim7);


  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Moteur gauche

 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Moteur droit




 etat_robot=0;
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

 while (1)
 {
	 if (etat_robot)
	 {

  if (tension < 3)
  	  	  {
  	      	  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
  	      }
  	  else  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);



  // Si le robot est désactivé (ex: bouton non appuyé), on le force à rester en mode 'Idle'
	    if (!etat_robot) {
	        etat_actuel = Idle;
	    }
	     if (etat_robot){
	    	etat_actuel = Avancer_Jusqua_Obstacle;
	    }

	    switch (etat_actuel) {

	    case Idle:
	        // Arrêt complet des moteurs


	        // Si le robot est activé (ex : bouton appuyé), on passe à l’étape suivante
	        if (etat_robot == 1) {
	           // distance_totale = 0;
	            etat_actuel = Avancer_Jusqua_Obstacle;
	        }
	        break;

	    case Avancer_Jusqua_Obstacle:
	        // Si le déplacement jusqu’à l’obstacle est terminé

	    	 if (T==2)
	    	 	{
	    		 etat_actuel = Tourner;
	    	 	}

	        break;

	    case Tourner:
	    				   if (T==3){
	    					             etat_actuel = Avancer_1_Metre;
	    				   }
	    				   if (T==1)  {
	    					   etat_actuel = Avancer_Jusqua_Obstacle;
	    				   }
	        break;

	    case Avancer_1_Metre:

	            if (T==2) {
	                // Si un obstacle a été détecté pendant l’avance → on recommence la rotation
	                etat_actuel = Tourner;
	            }
	            if (T==1){
	            	etat_actuel = Avancer_Jusqua_Obstacle;}
	            else {
	                // Sinon, on coupe le robot (mission terminée)
	                etat_robot = 0;
	                etat_actuel = Idle;
	            }
	        }
	        break;
	    }else{
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	    }
 }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */



  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */



  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */



  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */



  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */



  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 26666-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */



  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */



  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */



  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */



  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 799;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */



  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */



  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */



  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */



  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED2_Pin|DIR_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIRG_Pin|GPIO_PIN_12|LED4_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIRD_GPIO_Port, DIRD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED2_Pin DIR_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED2_Pin|DIR_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIRG_Pin PB12 LED4_Pin PB15 */
  GPIO_InitStruct.Pin = DIRG_Pin|GPIO_PIN_12|LED4_Pin|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIRD_Pin */
  GPIO_InitStruct.Pin = DIRD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIRD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */



  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback declenche lorsqu'un bouton est presse (interruption externe)
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin ) {
    	// Bascule l'etat d'activation du robot
     	if(T_B1 > 10)//filtre anti rebond
     	{
        etat_robot = !etat_robot;

        if (etat_robot==0)
        {
    		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Moteur 1
    		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // Moteur 2
    		  }
    }}
 }




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) {
		  T_B1++;

/*		  	// Récupération des positions actuelles
		 	  pos_d_actuelle = (int16_t) __HAL_TIM_GET_COUNTER(&htim4);
		  	  pos_g_actuelle = (int16_t) __HAL_TIM_GET_COUNTER(&htim3);

		  	  // --- 2. Calcul des ticks parcourus en 10 ms ---
		  	  int16_t delta_ticks_d = pos_d_actuelle - pos_d_prev;
		  	  int16_t delta_ticks_g = pos_g_actuelle - pos_g_prev;

		  	  // --- 3. Calcul des vitesses réelles en cm/s ---
		  	  float vitesse_d_cm_s = delta_ticks_d * CONVERSION_CM_PAR_TICK;
		  	  float vitesse_g_cm_s = delta_ticks_g * CONVERSION_CM_PAR_TICK;

		  	  Vd = vitesse_d_cm_s;
		  	  Vg = vitesse_g_cm_s;
		  	  // Sauvegarder pour la prochaine boucle
		  	  pos_d_prev = pos_d_actuelle;
		  	  pos_g_prev = pos_g_actuelle;

		  	  // --- 5. Régulation pour atteindre 10 cm/s ---
		  	  float consigne_cm_s = 10.0f;
		  	  float erreur_d = consigne_cm_s - vitesse_d_cm_s;
		  	  float erreur_g = consigne_cm_s - vitesse_g_cm_s;
		  /*
		  	  pwm_consigne_d += erreur_d * KP;
		  	  pwm_consigne_g += erreur_g * KP;*/

		  	  // --- 5. PID sur chaque moteur ---

/*		     static float somme_erreur_d = 0, somme_erreur_g = 0;
		  	  static float erreur_prec_d = 0, erreur_prec_g = 0;

		  	  // Coefficients PID
		  	  float Kp = 0.68f;
		  	  float Ki = 0.01f;
		  	  float Kd = 0.0f;

		  	  // Erreurs
		  	  float delta_d = erreur_d - erreur_prec_d;
		  	  float delta_g = erreur_g - erreur_prec_g;
		  	  somme_erreur_d += erreur_d;
		  	  somme_erreur_g += erreur_g;

		  	  // Anti-windup
		  	  if (somme_erreur_d > 1000) somme_erreur_d = 1000;
		  	  if (somme_erreur_d < -1000) somme_erreur_d = -1000;
		  	  if (somme_erreur_g > 1000) somme_erreur_g = 1000;
		  	  if (somme_erreur_g < -1000) somme_erreur_g = -1000;

		  	  // PID update
		  	  pwm_consigne_d += Kp * erreur_d+ Ki * somme_erreur_d + Kd * delta_d;
		  	  pwm_consigne_g += Kp * erreur_g + Ki * somme_erreur_g + Kd * delta_g;

		  	  // Sauvegarde des erreurs
		  	  erreur_prec_d = erreur_d;
		  	  erreur_prec_g = erreur_g;


		  	  // --- 6. Saturation (pour éviter débordements PWM) ---
		  	  if (pwm_consigne_d > 80000) pwm_consigne_d = 80000;
		  	  if (pwm_consigne_g > 80000) pwm_consigne_g = 80000;
		  	  if (pwm_consigne_d < 0) pwm_consigne_d = 0;
		  	  if (pwm_consigne_g < 0) pwm_consigne_g = 0;
*/

		  	  //char msg[100];
		  	//  sprintf(msgl, "Vd=%.2f cm/s, Vg=%.2f cm/s\r\n", vitesse_d/CONVERSION_CM_PAR_TICK , vitesse_g/CONVERSION_CM_PAR_TICK );
		  	 // HAL_UART_Transmit(&huart2, (uint8_t*)msgl, strlen(msgl), HAL_MAX_DELAY);
		  	  if(etat_robot == 1){


		  HAL_ADC_Start_IT(&hadc1);
		  if (T==1)
		  {
	            HAL_GPIO_WritePin(DIRG_GPIO_Port, DIRG_Pin, GPIO_PIN_SET); // gauche avant
	            HAL_GPIO_WritePin(DIRD_GPIO_Port, DIRD_Pin, GPIO_PIN_SET);   // droit arrière
		    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 5000);
		    	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 5000);
		  sprintf(uart_msg, "ADC1: [%ld, %ld, %ld, %u,%d] \r\n", tension, IR3, IR4, etat_robot,erreur_d);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, sizeof(uart_msg), HAL_MAX_DELAY);
		  compteur_distance=compteur_distance+1;
		  		    		  sprintf(uart_msg, "ADC1: [%ld, %ld, %ld,%d] \r\n", tension, IR3, IR4,erreur_d);
		  		    		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, sizeof(uart_msg), HAL_MAX_DELAY);

		  	                  if (compteur_distance >= 1000) {
		  	                	  compteur_distance=0;
		  	                  T=4;
		  	                  }
		  }
	          if (T == 2) {
	  	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 17000);
	  	            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 17000);
	  	            HAL_GPIO_WritePin(DIRG_GPIO_Port, DIRG_Pin, GPIO_PIN_RESET); // gauche avant
	  	            HAL_GPIO_WritePin(DIRD_GPIO_Port, DIRD_Pin, GPIO_PIN_SET);   // droit arrière
	              compteur=compteur+1;
	    		  sprintf(uart_msg, "ADC1: [%ld, %ld, %ld,%d] \r\n", tension, IR3, IR4,erreur_d);
	    		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, sizeof(uart_msg), HAL_MAX_DELAY);
	              if (compteur >= 158) {
	                  compteur = 0;
	                  compteur_distance=0;
	              	  T = 3;
	              }
	          }
	          if (T == 3 ) {
	  	            HAL_GPIO_WritePin(DIRG_GPIO_Port, DIRG_Pin, GPIO_PIN_SET); // gauche avant
	  	            HAL_GPIO_WritePin(DIRD_GPIO_Port, DIRD_Pin, GPIO_PIN_SET);   // droit arrière
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_consigne_d);
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_consigne_g);
	                  compteur_distance=compteur_distance+1;
		    		  sprintf(uart_msg, "ADC1: [%ld, %ld, %ld,%d] \r\n", tension, IR3, IR4,erreur_d);
		    		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, sizeof(uart_msg), HAL_MAX_DELAY);

	                  if (compteur_distance >= 1000) {
	                	  compteur_distance=0;
	                  T=4;

	                  }
	          }
	          if (T==4){

          		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Moteur 1
          		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // Moteur 2
	          }



		  	  }

	  else {
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Moteur 1
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // Moteur 2
	  }


	  if (htim==&htim7){



	  }
}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	  static uint8_t cpt = 0;
	  cpt++;
	  if (cpt == 1) tension = HAL_ADC_GetValue(&hadc1) * 3300UL / 4095UL;
	  if (cpt == 2) IR3 = HAL_ADC_GetValue(&hadc1);
	  if (cpt == 3)
	  {
	    IR4 = HAL_ADC_GetValue(&hadc1);
	    if ((IR3 > 4000) || (IR4 > 3700)) // Obstacle détecté
	    {
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_consigne_d);
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_consigne_g);
		      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	      T = 2;
	    }
	    cpt = 0;
	    HAL_ADC_Stop(&hadc1);
	  }

}



  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */

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
#endif /* USE_FULL_ASSERT */