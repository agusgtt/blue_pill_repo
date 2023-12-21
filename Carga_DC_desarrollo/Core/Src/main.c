/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "LCD_I2C.h"
#include "keypad_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE_input 4
#define TC74_ADDRESS 0x90//direccion og.48 moviendo un bit 90
#define N_TRANSISTORES 1
#define I_MAX_x_TRANSISTOR 500 // 500 -> 5000mA
#define FACTOR_ADC_VOLTAGE_mult 122
#define FACTOR_ADC_VOLTAGE_div 100
#define FACTOR_ADC_CURRENT_mult 7326//8437
#define FACTOR_ADC_CURRENT_div 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */



uint8_t flag_on_off = 0;		//bandera de conectar el dispositivo
uint8_t flag_config = 0;
uint8_t flag_update_display_1_seg = 0;
uint8_t flag_update_loop_control = 0;
uint8_t cont_timer_update = 0;
uint32_t input_adc[2];
char input_keypad = 0;

//char char_as_str[];// = {input_keypad, '\0'};//char mensaje[];
char buffer[20]="input key ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void agregar_digito(char *buffer, char digito);
void borrar_ultimo_digito(char *buffer);
void display_update_conf(char modo_op,char *dato);
void display_update_stat(char modo_op, char *dato, uint32_t volt);
void display_update_running(char modo_op,uint32_t volt, uint32_t corriente);
void enviar_spi_dac(uint16_t dato);
uint16_t leer_temperatura(void);
uint16_t control_carga(char modo, uint16_t voltage, uint16_t current, uint16_t set_point);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  enviar_spi_dac(0);
  keypad_init();
  HAL_Delay(30);
  LCD_I2C_init();

	//esto podria estar encapsulado
  	  LCD_I2C_cmd(LCD_LINEA1);
	  LCD_I2C_write_text("   UTN FRP   ");
	  LCD_I2C_cmd(LCD_LINEA2);
	  LCD_I2C_write_text("   Carga DC   ");
	  LCD_I2C_cmd(LCD_LINEA4);
	  LCD_I2C_write_text("   A.Gotte/A.Jose   ");
	  HAL_Delay(4000);
	  LCD_I2C_cmd(LCD_CLEAR);


  HAL_TIM_Base_Start_IT(&htim2);

  //HAL_ADC_Start_DMA(&hadc1, input_adc, 2);//cuelga el programa
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);



	//se deberia hacer un POST

  char modo_carga = 'C';
  char input_valor[5]="";
  //char buffer_val[20]="val:  NNNN [utm]";

  uint8_t cont_digitos_input_val = 0;
  uint8_t flag_update_display = 0;
  uint16_t control_spi = 0;
  uint16_t set_point = 0;
  //uint16_t temperatura_sensor = 0;
  enviar_spi_dac(control_spi);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(flag_update_display_1_seg){//update display modo stand by

		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 5);
		  input_adc[0]=HAL_ADC_GetValue(&hadc1);
		  //display_update_stat(modo_carga,input_valor,input_adc[0]);
		  display_update_stat(modo_carga,input_valor,input_adc[0]);
		  flag_update_display_1_seg=0;
	  }

	  if(tipo_dato(input_keypad)==2){//tipo_dato()=2 si input es C,V,P,R
		  //ingresa a la configuracion de modo
		  flag_config=1;

		  while(flag_config){
			  if(tipo_dato(input_keypad)==1 && cont_digitos_input_val<4){//tipo_dato()=1 si input es >=0 y <=9
				  //input_valor[cont_digitos_input_val]=input_keypad;
				  agregar_digito(input_valor, input_keypad);
				  input_keypad=0;
				  cont_digitos_input_val++;
				  flag_update_display=1;
			  }
			  else if(input_keypad=='D'&&cont_digitos_input_val>0){
				  borrar_ultimo_digito(input_valor);
				  input_keypad=0;
				  cont_digitos_input_val--;
				  flag_update_display=1;
			  }
			  else if(input_keypad=='K'){
				  input_keypad=0;
				  //validar parametro ingresado
				  set_point=atoi(input_valor);
				  flag_config=0;//sale del while y vuelve al super loop
				  flag_update_display_1_seg=1;
				  //validar parametro ingresado
			  }
			  else if(tipo_dato(input_keypad)==2){
				  modo_carga=input_keypad;//guardar el modo que se selecciono
				  input_keypad=0;
				  //borrar buffer
				  flag_update_display=1;
			  }
		  if(flag_update_display){
			  display_update_conf(modo_carga,input_valor);
			  flag_update_display=0;
		  }
		}//fin while config
	  }//fin if config
	  if(flag_on_off){
		  //setting del modo de trabajo
		  while(flag_on_off){
			  if(flag_update_loop_control){

					 HAL_ADC_Start(&hadc1);
					 HAL_ADC_PollForConversion(&hadc1, 5);
					 input_adc[0]=HAL_ADC_GetValue(&hadc1);
					 HAL_ADC_Start(&hadc2);
					 HAL_ADC_PollForConversion(&hadc2, 5);
					 input_adc[1]=HAL_ADC_GetValue(&hadc2);

					 control_spi=control_carga(modo_carga,input_adc[0],input_adc[1],set_point);
					 enviar_spi_dac(control_spi);
					 flag_update_loop_control=0;

			  }
			  if(flag_update_display_1_seg){
				  display_update_running(modo_carga,input_adc[0],input_adc[1]);
				  flag_update_display_1_seg=0;
				  }
		  }//fin while flag_on_off
		  //configuracion post carga activa
		  input_keypad=0;
		  control_spi=0;
		  enviar_spi_dac(control_spi);
	  }//fin if flag_on_off




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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 80000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//definiciones de los callback y funciones
// External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_9) // INT Source is pin A9
    {
    //if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)==GPIO_PIN_SET&&!flag_config){
    if(!flag_on_off && !flag_config){
    		flag_on_off=1;// conecta la carga
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else{
    	flag_on_off=0;// desconecta la carga
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    }

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM2){
	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	 input_keypad=keypad_scan();//condicionar la lectura a que no este en modo activo la carga
	 flag_update_loop_control=1; //movemos al timer3

	 if(cont_timer_update>=5){//cada segundo y medio actualiza el display

		 flag_update_display_1_seg=1;//update nombre
		 cont_timer_update=0;
		 //temperatura_sensor=leer_temperatura();
	 }
	 else cont_timer_update++;
 }
 if(htim->Instance == TIM3){
 	 //disparar control de la carga
	 //flag_update_loop_control=1; //flag ciclo de control
  }
}

void agregar_digito(char *buffer, char digito) {
    size_t longitud = strlen(buffer);

    if (longitud < BUFFER_SIZE_input) {
        buffer[longitud] = digito;
        buffer[longitud + 1] = '\0';
    }
}

void borrar_ultimo_digito(char *buffer) {
    size_t longitud = strlen(buffer);

    if (longitud > 0) {
        buffer[longitud - 1] = '\0';
    }
}

uint16_t leer_temperatura(){
	uint8_t dato[1];
	 uint16_t temperature_milicelsius=0;
	 HAL_I2C_Master_Transmit(&hi2c1, TC74_ADDRESS, (uint8_t *)dato, 1, 100);//<< 1
	 while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	 //HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	 HAL_I2C_Master_Receive(&hi2c1, TC74_ADDRESS, (uint8_t *)dato, 1, 100);
	 while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	 temperature_milicelsius = dato[0];  //
	if(temperature_milicelsius >= 0x80){
		temperature_milicelsius=0;
	}
	 return temperature_milicelsius;
}

void enviar_spi_dac(uint16_t dato){
	if(dato<=0x0FFF){
		uint16_t send = 0x3000;
		send = send|dato;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &send, 2, 10);
		while(HAL_SPI_GetState(&hspi1)!=HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

		return;
	}
}
void display_update_conf(char modo_op, char *dato){

	char char_as_str[] = {modo_op, '\0'};//encapsular, funcion de uso recurrente
	char buffer_fun[20]="";//{"Modo C",char_as_str};
	char buffer_dato[20]="";
	snprintf(buffer_fun, sizeof(buffer_fun), "Config Modo C%s:", char_as_str);

	LCD_I2C_cmd(LCD_CLEAR);
	LCD_I2C_cmd(LCD_LINEA1);
	LCD_I2C_write_text(buffer_fun);
	LCD_I2C_cmd(LCD_LINEA3);
	switch(modo_op){//print del modo

	case 'C':
		snprintf(buffer_dato, sizeof(buffer_dato), "Current: %s0 [mA]", dato);
		break;
	case 'V':
		snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %s0 [mV]", dato);
		break;
	case 'R':
		snprintf(buffer_dato, sizeof(buffer_dato), "Resist.: %s [mR]", dato);
		break;
	case 'P':
		snprintf(buffer_dato, sizeof(buffer_dato), "Power.: %s00 [W]", dato);
		break;
	default:
		snprintf(buffer_dato, sizeof(buffer_dato), "Error: case def");
	}
	LCD_I2C_write_text(buffer_dato);

}

void display_update_stat(char modo_op, char *dato,uint32_t volt){
	char char_as_str[] = {modo_op, '\0'};//encapsular, funcion de uso recurrente
	char buffer_fun[20]="";//{"Modo C",char_as_str};
	char buffer_dato[20]="";
	uint32_t volt_convertido = 0;
	volt_convertido=volt*FACTOR_ADC_VOLTAGE_mult;//122;
	volt_convertido=volt_convertido/FACTOR_ADC_VOLTAGE_div;//100;

	snprintf(buffer_fun, sizeof(buffer_fun), "Modo C%s:", char_as_str);

	LCD_I2C_cmd(LCD_CLEAR);
	LCD_I2C_cmd(LCD_LINEA1);
	LCD_I2C_write_text(buffer_fun);

	switch(modo_op){//print del modo

	case 'C':
		LCD_I2C_cmd(LCD_LINEA2);
		snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %d.%02d [V]", (int)volt_convertido/100,(int)volt_convertido%100);//
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA3);
		snprintf(buffer_dato, sizeof(buffer_dato), "Current: %s0 [mA]", dato);
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA4);
		LCD_I2C_write_text("Power: 0[W]");

		break;
	case 'V':
		LCD_I2C_cmd(LCD_LINEA2);
		snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %s0 [mV]", dato);
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA3);
		LCD_I2C_write_text("Current: 0[mA]");
		LCD_I2C_cmd(LCD_LINEA4);
		LCD_I2C_write_text("Power: 0[mW]");
		break;
	case 'R':
		LCD_I2C_cmd(LCD_CLEAR);
		LCD_I2C_cmd(LCD_LINEA1);
		snprintf(buffer_dato, sizeof(buffer_dato), "Res: %s[mohm]", dato);
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA2);
		snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %d.%02d [V]",  (int)volt_convertido/100,(int)volt_convertido%100);//
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA3);
		LCD_I2C_write_text("Current: 0[mA]");
		LCD_I2C_cmd(LCD_LINEA4);
		LCD_I2C_write_text("Power: 0[mW]");
		break;
	case 'P':
		LCD_I2C_cmd(LCD_LINEA2);
		snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %d.%02d [V]",  (int)volt_convertido/100,(int)volt_convertido%100);//
		LCD_I2C_write_text(buffer_dato);
		LCD_I2C_cmd(LCD_LINEA3);
		LCD_I2C_write_text("Current: 0[mA]");
		LCD_I2C_cmd(LCD_LINEA4);
		snprintf(buffer_dato, sizeof(buffer_dato), "Power: %s00 [mW]", dato);
		LCD_I2C_write_text(buffer_dato);

		break;
	default:
		LCD_I2C_cmd(LCD_LINEA1);
		snprintf(buffer_dato, sizeof(buffer_dato), "Error: monit");
	}

}

void display_update_running(char modo_op,uint32_t volt, uint32_t corriente){
	char char_as_str[] = {modo_op, '\0'};//encapsular, funcion de uso recurrente
	char buffer_fun[20]="";//{"Modo C",char_as_str};
	char buffer_dato[20]="";
	uint32_t volt_convertido = 0;
	uint32_t corriente_convertido = 0;
	uint32_t potencia =0;
	uint16_t resistencia =0;
	volt_convertido=volt*FACTOR_ADC_VOLTAGE_mult;//122;
	volt_convertido=volt_convertido/FACTOR_ADC_VOLTAGE_div;//100;
	corriente_convertido=corriente*FACTOR_ADC_CURRENT_mult;//8437;
	corriente_convertido=corriente_convertido/FACTOR_ADC_CURRENT_div;//10000;
	potencia = volt_convertido*corriente_convertido;
	potencia=potencia/1000;
	resistencia=volt_convertido/corriente_convertido;


	LCD_I2C_cmd(LCD_CLEAR);
	if(modo_op == 'R'){
		LCD_I2C_cmd(LCD_LINEA1);
		if(resistencia>4999)snprintf(buffer_dato, sizeof(buffer_dato), "Res: O.Lim");
		else snprintf(buffer_dato, sizeof(buffer_dato), "Res: %d[ohm]", resistencia);
		LCD_I2C_write_text(buffer_dato);
	}
	else{
		snprintf(buffer_fun, sizeof(buffer_fun), "Running Modo C%s:", char_as_str);
		LCD_I2C_cmd(LCD_LINEA1);
		LCD_I2C_write_text(buffer_fun);
	}
	LCD_I2C_cmd(LCD_LINEA2);
	snprintf(buffer_dato, sizeof(buffer_dato), "Voltage: %d.%02d [V]",  (int)volt_convertido/100,(int)volt_convertido%100);//
	LCD_I2C_write_text(buffer_dato);
	LCD_I2C_cmd(LCD_LINEA3);
	snprintf(buffer_dato, sizeof(buffer_dato), "Current: %d.%02d [A]", (int)corriente_convertido/100,(int)corriente_convertido%100);//
	LCD_I2C_write_text(buffer_dato);
	LCD_I2C_cmd(LCD_LINEA4);
	snprintf(buffer_dato, sizeof(buffer_dato), "Pot: %d.%d [W]", (int)potencia/10,(int)potencia%10);//
	LCD_I2C_write_text(buffer_dato);
}

uint16_t control_carga(char modo, uint16_t voltage, uint16_t current, uint16_t set_point){
	uint32_t calculo = 0;
	uint16_t DAC_nuevo_valor = 0;
	uint32_t volt_convertido = 0;
	uint32_t corriente_convertido = 0;

	volt_convertido=voltage*FACTOR_ADC_VOLTAGE_mult;//122;
	volt_convertido=volt_convertido/FACTOR_ADC_VOLTAGE_div;//100;
	corriente_convertido=current*FACTOR_ADC_CURRENT_mult;//8437;
	corriente_convertido=corriente_convertido/FACTOR_ADC_CURRENT_div;//10000;

	switch(modo){
	case 'C'://seteo directo
		calculo = set_point * 4095;//dac resol
		calculo = calculo /N_TRANSISTORES; //div num MOSFET
		calculo = calculo /I_MAX_x_TRANSISTOR;//div corriente max por cada mosfet 250->2500 mA
		break;
	case 'V':
		//rev
		calculo = 0;
		break;
	case 'P':
		calculo = set_point * 1000;// agregamos ceros para que se alinee la coma y el resultado sea con las cifras correspondientes
		calculo=calculo/volt_convertido;//P/V=I
		calculo = calculo * 4095;//dac resol
		calculo = calculo /N_TRANSISTORES; //div num MOSFET
		calculo = calculo /I_MAX_x_TRANSISTOR;//div corriente max por cada mosfet 250->2500 mA
		break;
	case 'R':
		calculo=volt_convertido*1000;//ceros para acomodar la coma
		calculo=volt_convertido/set_point;//por ley de ohm
		calculo = calculo * 4095;//dac resol
		calculo = calculo /N_TRANSISTORES; //div num MOSFET
		calculo = calculo /I_MAX_x_TRANSISTOR;//div corriente max por cada mosfet 250->2500 mA
		break;
	default:
		calculo=0;

	}
	DAC_nuevo_valor=(uint16_t)calculo;
	return DAC_nuevo_valor;
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
