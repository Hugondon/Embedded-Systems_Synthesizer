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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "math.h"
#include "fonts.h"
#include "tft.h"
#include "functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 1
#define NOTE_BUFFER_SIZE 1
#define PLAY_BUFFER_SIZE 1
#define C4 60
#define B5 83
#define NEXT_NOTE 1.05946
#define NS 100
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

// LCD
uint16_t ID = 0;
uint8_t cont_am = 0;
uint8_t cont_fm = 0;
uint8_t cont_lp = 0;
uint8_t wave_selection = 1;
uint8_t cont_signal = 0;

// Debug
char message[] = "Ya casi lo logramos, brow\r\n";
char okay_message[] = "OK\r\n";

// Serial
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
uint8_t note_buffer[NOTE_BUFFER_SIZE] = {0};
uint8_t current_note_buffer = 0;
uint8_t received_note_buffer = 0;
volatile bool note_received_flag = false;

// Interrupción
volatile bool EXT_BTN_1_state = true;
volatile bool EXT_BTN_2_state = true;
volatile bool EXT_BTN_3_state = true;
volatile bool EXT_BTN_4_state = true;

// Audio
uint8_t play_note_buffer[PLAY_BUFFER_SIZE] = {0};
float value = 0.2;
uint32_t var;

uint16_t sine_LUT [] = {
		2048, 2177, 2307, 2435, 2562, 2686, 2808, 2928, 3043, 3154, 3261, 3364, 3460, 3552, 3637,
		3715, 3787, 3852, 3910, 3960, 4003, 4037, 4064, 4082, 4093, 4095, 4089, 4074, 4052, 4021,
		3982, 3936, 3882, 3821, 3752, 3677, 3595, 3507, 3413, 3313, 3209, 3099, 2986, 2868, 2748,
		2624, 2499, 2371, 2242, 2112, 1983, 1853, 1724, 1596, 1471, 1347, 1227, 1109, 996, 886,
		782, 682, 588, 500, 418, 343, 274, 213, 159, 113, 74, 43, 21, 6, 0,
		2, 13, 31, 58, 92, 135, 185, 243, 308, 380, 458, 543, 635, 731, 834,
		941, 1052, 1167, 1287, 1409, 1533, 1660, 1788, 1918, 2047
};

uint16_t sawtooth_LUT [] = {
		0, 41, 83, 124, 165, 207, 248, 290, 331, 372, 414, 455, 496, 538, 579,
		620, 662, 703, 745, 786, 827, 869, 910, 951, 993, 1034, 1075, 1117, 1158, 1200,
		1241, 1282, 1324, 1365, 1406, 1448, 1489, 1530, 1572, 1613, 1655, 1696, 1737, 1779, 1820,
		1861, 1903, 1944, 1985, 2027, 2068, 2110, 2151, 2192, 2234, 2275, 2316, 2358, 2399, 2440,
		2482, 2523, 2565, 2606, 2647, 2689, 2730, 2771, 2813, 2854, 2895, 2937, 2978, 3020, 3061,
		3102, 3144, 3185, 3226, 3268, 3309, 3350, 3392, 3433, 3475, 3516, 3557, 3599, 3640, 3681,
		3723, 3764, 3805, 3847, 3888, 3930, 3971, 4012, 4054, 0
};

uint16_t triangle_LUT [] = {
		0, 83, 165, 248, 331, 414, 496, 579, 662, 745, 827, 910, 993, 1075, 1158,
		1241, 1324, 1406, 1489, 1572, 1655, 1737, 1820, 1903, 1985, 2068, 2151, 2234, 2316, 2399,
		2482, 2565, 2647, 2730, 2813, 2895, 2978, 3061, 3144, 3226, 3309, 3392, 3475, 3557, 3640,
		3723, 3805, 3888, 3971, 4054, 4054, 3971, 3888, 3805, 3723, 3640, 3557, 3475, 3392, 3309,
		3226, 3144, 3061, 2978, 2895, 2813, 2730, 2647, 2565, 2482, 2399, 2316, 2234, 2151, 2068,
		1985, 1903, 1820, 1737, 1655, 1572, 1489, 1406, 1324, 1241, 1158, 1075, 993, 910, 827,
		745, 662, 579, 496, 414, 331, 248, 165, 83, 0
};

uint8_t PSC_LUT [] = {55, 52, 49, 43, 42, 38, 54, 54, 54, 48, 55, 56, 49, 54, 51, 48, 42, 47, 54, 51, 48, 51, 35, 52, 49, 54, 51, 29, 34, 43, 27, 23, 217, 51, 193, 91};
uint8_t ARR_LUT [] = {25, 25, 25, 27, 26, 27, 18, 17, 16, 17, 14, 13, 14, 12, 12, 12, 13, 11, 9, 9, 9, 8, 11, 7, 7, 6, 6, 10, 8, 6, 9, 10, 1, 4, 1, 2};
// Utils
uint32_t i = 0;

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
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_UART_Receive_DMA(&huart4, rx_buffer, sizeof(rx_buffer)/sizeof(char));

  ID = readID();
  HAL_Delay(100);
  tft_init(ID);
  setRotation(3);

  // INTERFAZ GLOBAL
  fillRect(0 , 0, 320, 240,(11)&BLUE | (((16)*2)<<5)&GREEN |  ((26)<<(5+6))&RED );//Color base
  fillRect(20 , 80, 280, 120,BLUE_LEV(10) | GREEN_LEV(11) | RED_LEV(17));//Color de tablero
  fillRect(20 , 20, 280, 50,BLUE_LEV(2) | GREEN_LEV(2) | RED_LEV(2));//Color de señales

  // AM
  fillTriangle(62, 105, 72, 90, 82, 105, RED);

  // FM
  fillTriangle(45, 140, 55, 125, 65, 140, RED);
  fillTriangle(76, 140, 86, 125, 96, 140, RED);

  // LP
  fillTriangle(76, 175, 76, 160, 90, 175, RED);
  fillRect(40, 161, 36, 14,RED);

  // PIANO
  fillRect(65 , 205, 190, 35,WHITE);
  fillRect(85 , 205, 10 ,22,BLACK);
  fillRect(115 , 205, 10 ,22,BLACK);
  fillRect(165 , 205, 10 ,22,BLACK);
  fillRect(195 , 205, 10 ,22,BLACK);
  fillRect(225 , 205, 10 ,22,BLACK);

  //CAJAS DE VOLUMEN
  	//HORIZONTALES
  fillRect(130 , 95, 152, 3,BLACK);
  fillRect(130 , 112, 152, 3,BLACK);
  fillRect(130 , 130, 152, 3,BLACK);
  fillRect(130 , 147, 152, 3,BLACK);
  fillRect(130 , 165, 152, 3,BLACK);
  fillRect(130 , 182, 152, 3,BLACK);

  	  //VERTICALES
  for(i = 0; i < 5; i++){
  	fillRect(130+(i*38) , 95, 3, 20,BLACK);
  	fillRect(130+(i*38) , 130, 3, 20,BLACK);
  	fillRect(130+(i*38) , 165, 3, 20,BLACK);
  }

  // Imagenes que identifican
  for(i=0; i < 3; i++){
  	fillRect(40 , 90+(i*35), 2, 20,RED);
  	fillRect(30 , 105+(i*35), 70, 2,RED);
  }
  HAL_TIM_Base_Start(&htim6);

  HAL_UART_Transmit(&huart3, (uint8_t *)message, sizeof(message)/sizeof(char) - 1, 1000);
  HAL_UART_Transmit(&huart3, (uint8_t *)okay_message, sizeof(okay_message)/sizeof(char) - 1, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		  if(!EXT_BTN_1_state){
			  cont_lp ++;
			  if(cont_lp < 5){
				  for (i = 0; i < cont_lp; i++) fillRect(133+(i*38), 168, 35,14,BLUE);
			  }else{
				  for (i = 0; i < 4; i++) fillRect(133+(i*38), 168, 35,14,BLUE_LEV(10) | GREEN_LEV(11) | RED_LEV(17));
				  cont_lp = 0;
			  }
			  EXT_BTN_1_state = true;
		  }
		  if(!EXT_BTN_2_state){
			  cont_fm++;
			  if(cont_fm < 5){
				  for (i = 0; i < cont_fm; i++) fillRect(133+(i*38), 133, 35,14,BLUE);
			  }else{
				  for (i = 0; i < 4; i++) fillRect(133+(i*38), 133, 35,14,BLUE_LEV(10) | GREEN_LEV(11) | RED_LEV(17));
				  cont_fm = 0;
			  }
			  EXT_BTN_2_state = true;
		  }
		  if(!EXT_BTN_3_state){
			  cont_am++;
			  if(cont_am < 5){
				  for (i = 0; i < cont_am; i++) fillRect(133+(i*38), 98, 35,14,BLUE);
			  }else{
			  		for (i = 0; i < 4; i++) fillRect(133+(i*38), 98, 35,14,BLUE_LEV(10) | GREEN_LEV(11) | RED_LEV(17));
			  		cont_am = 0;
			  }
			  EXT_BTN_3_state = true;
		  }
		  if(!EXT_BTN_4_state){
			  cont_signal++;
			  if(cont_signal<5){
				  for (i=1; i<=cont_signal; i++){
					  fillRect(24+((i-2)*65), 22, 3,46,BLACK);
					  fillRect(24+((i-1)*65), 22, 3,46,YELLOW);
					  fillRect(89+((i-1)*65), 22, 3,46,YELLOW);
					  fillRect(24+((i-2)*65), 22, 65,3,BLACK);
					  fillRect(24+((i-2)*65), 65, 65,3,BLACK);
					  fillRect(24+((i-1)*65), 22, 65,3,YELLOW);
					  fillRect(24+((i-1)*65), 65, 65,3,YELLOW);
				  }
			  }else{
				  fillRect(219, 22, 3,46,BLACK);
				  fillRect(284, 22, 3,46,BLACK);
				  fillRect(24, 22, 3,46,YELLOW);
				  fillRect(89, 22, 3,46,YELLOW);
				  fillRect(219, 22, 65,3,BLACK);
				  fillRect(219, 65, 65,3,BLACK);
				  fillRect(24, 22, 65,3,YELLOW);
				  fillRect(24, 65, 65,3,YELLOW);
				  cont_signal = 1;
			  }
			  if(wave_selection < 4){
				  wave_selection++;
			  }else{
				  wave_selection = 1;
			  }
			  EXT_BTN_4_state = true;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim6.Init.Prescaler = 55-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 15-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 31250;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_11|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6
                           PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB11 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_11|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_BTN_1_Pin EXT_BTN_4_Pin */
  GPIO_InitStruct.Pin = EXT_BTN_1_Pin|EXT_BTN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_BTN_2_Pin EXT_BTN_3_Pin */
  GPIO_InitStruct.Pin = EXT_BTN_2_Pin|EXT_BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// Callback para GPIOs
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == EXT_BTN_1_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == EXT_BTN_2_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == EXT_BTN_3_Pin) HAL_TIM_Base_Start_IT(&htim2);
	if(GPIO_Pin == EXT_BTN_4_Pin) HAL_TIM_Base_Start_IT(&htim2);
}
// Callback para Timers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if(htim == &htim2){
	  // Ya pasaron 40ms y sigue en 0, es real esto.
	  if(!HAL_GPIO_ReadPin(EXT_BTN_1_GPIO_Port, EXT_BTN_1_Pin)){
		  EXT_BTN_1_state = false;
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(EXT_BTN_2_GPIO_Port, EXT_BTN_2_Pin)){
		  EXT_BTN_2_state = false;
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(EXT_BTN_3_GPIO_Port, EXT_BTN_3_Pin)){
		  EXT_BTN_3_state = false;
		  HAL_TIM_Base_Stop(&htim2);
	  }
	  if(!HAL_GPIO_ReadPin(EXT_BTN_4_GPIO_Port, EXT_BTN_4_Pin)){
		  EXT_BTN_4_state = false;
		  HAL_TIM_Base_Stop(&htim2);
	  }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	note_received_flag = false;
	for(int i = 0; i < RX_BUFFER_SIZE; i++){
		if(rx_buffer[i] >= C4 && rx_buffer[i] <= B5){
			note_buffer[current_note_buffer] = rx_buffer[i];
			current_note_buffer++;
			received_note_buffer = rx_buffer[i];
			note_received_flag = true;
			if(current_note_buffer >= NOTE_BUFFER_SIZE){
				current_note_buffer = 0;
			}
			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
			htim6.Init.Prescaler = PSC_LUT[received_note_buffer - 60] - 1;
			htim6.Init.Period = ARR_LUT[received_note_buffer - 60] - 1;
			HAL_TIM_Base_Init(&htim6);
			  switch(wave_selection){
				  case 1:
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sine_LUT, NS, DAC_ALIGN_12B_R);
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, sine_LUT, NS, DAC_ALIGN_12B_R);
					  break;
				  case 2:
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, triangle_LUT, NS, DAC_ALIGN_12B_R);
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, triangle_LUT, NS, DAC_ALIGN_12B_R);
					  break;
				  case 3:
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sawtooth_LUT, NS, DAC_ALIGN_12B_R);
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, sawtooth_LUT, NS, DAC_ALIGN_12B_R);
					  break;
				  case 4:
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, sawtooth_LUT, NS, DAC_ALIGN_12B_R);
					  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, sawtooth_LUT, NS, DAC_ALIGN_12B_R);
					  break;
				  default:
					  wave_selection = 1;
					  break;
			  }
		}
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
