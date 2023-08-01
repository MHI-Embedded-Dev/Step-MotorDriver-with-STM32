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
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define PI 3.14159265
#define a1 60	// Kol UzunluÄŸu 1. Kol (cm)
#define a2 50	// Kol UzunluÄŸu 2. Kol (cm)
#define a3 40	// Kol UzunluÄŸu 3. Kol (cm)

#define motor_tam_tur_mesafe 32 //360 derecede 32mm ilerliyor

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Motor Kontrol Degiskenleri
bool yon = false; //false: sag true:sol
bool yon_degisim_chc;
bool if_giris_kontrol;
uint8_t false_sayaci;
uint16_t sayac_x,sayac_y, limit_x=3200, limit_y=3200;
bool hareket_izin_x=false, hareket_izin_y=false;

bool adım_takip = false;

//Pot Variable
uint32_t ADC_Buffer[3];
float Pot1, Pot2, Pot3;//3 ADC değişkeni

float eksen_X,eksen_Y;//Kartezyen sistem X ve Y Sınırları


float T01[4][4], T12[4][4], T23[4][4], T02[4][4], T03[4][4]; //ASIL Matrisleri

float P[3][1]; //Pozisyon Matrisi (Çıktı Matrisi)

uint16_t x_ekseni_mesafe_aci_cevrimi;
uint16_t y_ekseni_mesafe_aci_cevrimi;

uint16_t x_ekseni_mesafe_aci_cevrimi_son=0;
uint16_t y_ekseni_mesafe_aci_cevrimi_son=0;

uint16_t x_yeni_dönme_acı_cevrimi;
uint16_t y_yeni_dönme_acı_cevrimi;


//UART DMA Degiskenler
#define RxBuf_SIZE   1
uint8_t len;
uint8_t	RxBuffer[RxBuf_SIZE];
extern DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
//HC05 Veri Okuma
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart->Instance == USART1)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, RxBuf_SIZE);
		 __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    for(int i=Size;i<RxBuf_SIZE;i++)
      RxBuffer[i]=0;
  }
}
*/


//Matris Çarpım Fonk
void carpim(float m[4][4], float n[4][4], float result[4][4])
{
  for(int i=0; i < 4; i++)
  {
    for(int j=0; j < 4; j++)
    {
      result[i][j] = 0;
      for (int k = 0; k < 4; k++)
      result[i][j] += m[i][k] * n[k][j];
    }
  }
}

//Matris Live Expresion'da Gösterme Fonksiyonu
void matris_donusturme(float ham_matris[4][4], float islenmis_matris[4][4])
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			islenmis_matris[i][j]=ham_matris[i][j];
		}
	}
}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin (LED1_GPIO_Port, LED1_Pin);
}

//Acı Hesaplama
uint16_t aci_hesaplama(float aci)
{
	float motor_aci = 3200*(aci/360);
	return motor_aci;
}

//Map Fonksiyonu
float map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//ADC CallBack Fonksiyonu
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc -> Instance == ADC1)
	{
		//d1 d2 ve d3 parametreleri 3 adet POT'tan okunmaktadır.
		Pot1 = map(ADC_Buffer[0],0,4096,0,360);
		Pot2 = map(ADC_Buffer[1],0,4096,0,360);
		Pot3 = map(ADC_Buffer[2],0,4096,0,360);
	}
}

//Yon Toggle Yapma Fonk X
void yon_degistir_x(bool buton_input){

	if(buton_input==true){
		false_sayaci=0;

		if(yon_degisim_chc==true && if_giris_kontrol==true){
			HAL_GPIO_WritePin(DIR_X_GPIO_Port,DIR_X_Pin, 0);
			if_giris_kontrol=false;
			yon_degisim_chc=false;
		}
		if(yon_degisim_chc==false && if_giris_kontrol==true){
			HAL_GPIO_WritePin(DIR_X_GPIO_Port,DIR_X_Pin, 1);
			if_giris_kontrol=false;
			yon_degisim_chc=true;
		}
	}
	if(buton_input==false){
		false_sayaci++;
		if(false_sayaci >= 3){
			if_giris_kontrol=true;
			false_sayaci=0;
		}
	}
}

//Yon Toggle Yapma Fonk Y
void yon_degistir_y(bool buton_input){

	if(buton_input==true){
		false_sayaci=0;

		if(yon_degisim_chc==true && if_giris_kontrol==true){
			HAL_GPIO_WritePin(DIR_Y_GPIO_Port,DIR_Y_Pin, 0);
			if_giris_kontrol=false;
			yon_degisim_chc=false;
		}
		if(yon_degisim_chc==false && if_giris_kontrol==true){
			HAL_GPIO_WritePin(DIR_Y_GPIO_Port,DIR_Y_Pin, 1);
			if_giris_kontrol=false;
			yon_degisim_chc=true;
		}
	}
	if(buton_input==false){
		false_sayaci++;
		if(false_sayaci >= 3){
			if_giris_kontrol=true;
			false_sayaci=0;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim1)//10Hz	Kinematik Matris İşlemleri
	{
		float d1 = Pot1;	// Degisken UzunluÄŸu 1. Kol
		float d2 = Pot2;	// Degisken UzunluÄŸu 2. Kol
		float d3 = Pot3;	// Degisken UzunluÄŸu 3. Kol

		float val = PI/180; //Radyan Çevirme Çarpanı
		float teta_1=90, alfa_1=90, r_1=0, d_1=a1+d1; //1 DH Satır
		float teta_2=90, alfa_2=-90, r_2=0, d_2=a2+d2; //2 DH Satır
		float teta_3=0, alfa_3=0, r_3=0, d_3=a3+d3; //3 DH Satır

		//T01 matrisi Degiskenleri
		float a_00 = (cos(teta_1*val));
		float a_01 = -sin(teta_1*val)*cos(alfa_1*val);
		float a_11 = cos(teta_1*val)*cos(alfa_1*val);
		float a_12 = -cos(teta_1*val)*sin(alfa_1*val);
		float a_22 = cos(alfa_1*val);

		//T12 matrisi Degiskenleri
		float b_00 = cos(teta_2*val);
		float b_01 = -sin(teta_2*val)*cos(alfa_2*val);
		float b_11 = cos(teta_2*val)*cos(alfa_2*val);
		float b_12 = -cos(teta_2*val)*sin(alfa_2*val);
		float b_22 = cos(alfa_2*val);

		//T23 matrisi Degiskenleri
		float c_01 = (-sin(teta_3*val)*cos(alfa_3*val));
		float c_12 = -cos(teta_3*val)*sin(alfa_3*val);

		//ASIL MATRÄ°SLER
		float a[4][4]={
			{ a_00, a_01, sin(teta_1*val)*sin(alfa_1*val), r_1*cos(teta_1*val)},
			{ sin(teta_1*val), a_11, a_12, r_1*sin(teta_1*val)},
			{0, sin(alfa_1*val), a_22, d_1},
			{0, 0, 0, 1}};

		float b[4][4]={
			{ b_00, b_01, sin(teta_2*val)*sin(alfa_2*val), r_2*cos(teta_2*val)},
			{ sin(teta_2*val), b_11, b_12, r_2*sin(teta_2*val)},
			{0, sin(alfa_2*val), b_22, d_2},
			{0, 0, 0, 1}};

		float c[4][4]={
			{cos(teta_3*val), c_01, sin(teta_3*val)*sin(alfa_3*val), r_3*cos(teta_3*val)},
			{ sin(teta_3*val), cos(teta_3*val)*cos(alfa_3*val), c_12, r_3*sin(teta_3*val)},
			{0, sin(alfa_3*val), cos(alfa_3*val), d_3},
			{0, 0, 0, 1}};

		  matris_donusturme(a, T01);
		  matris_donusturme(b, T12);
		  matris_donusturme(c, T23);

		  carpim(T01, T12, T02);
		  carpim(T02, T23, T03);

		  P[0][0] = T03[0][3];
		  P[1][0] = T03[1][3];
		  P[2][0] = T03[2][3];

		  //X ve Y ekseni Ölçekleme
		  eksen_X = map(P[0][0],50,405,0,59);
		  eksen_Y = map(P[1][0],-40,-395,0,130);


		  if(adım_takip == true)//
		  {
			  x_ekseni_mesafe_aci_cevrimi = ((eksen_X*10) / motor_tam_tur_mesafe)*360;
			  y_ekseni_mesafe_aci_cevrimi = ((eksen_Y*10) / motor_tam_tur_mesafe)*360;
		  }

		  if(adım_takip == false)
		  {
			  x_ekseni_mesafe_aci_cevrimi_son = ((eksen_X*10) / motor_tam_tur_mesafe)*360;
			  y_ekseni_mesafe_aci_cevrimi_son = ((eksen_Y*10) / motor_tam_tur_mesafe)*360;
		  }


		  //Acı Değerleri Yazdırma Koşulları
		  if(x_ekseni_mesafe_aci_cevrimi > x_ekseni_mesafe_aci_cevrimi_son)
		  {
			  yon_degistir_x(1);
			  x_yeni_dönme_acı_cevrimi = x_ekseni_mesafe_aci_cevrimi- x_ekseni_mesafe_aci_cevrimi_son;
		  }

		  if(x_ekseni_mesafe_aci_cevrimi < x_ekseni_mesafe_aci_cevrimi_son)
		  {
			  x_yeni_dönme_acı_cevrimi = x_ekseni_mesafe_aci_cevrimi- x_ekseni_mesafe_aci_cevrimi_son;
		  }

		  if(y_ekseni_mesafe_aci_cevrimi > y_ekseni_mesafe_aci_cevrimi_son)
		  {
			  yon_degistir_y(1);
			  y_yeni_dönme_acı_cevrimi = y_ekseni_mesafe_aci_cevrimi- y_ekseni_mesafe_aci_cevrimi_son;
		  }

		  if(y_ekseni_mesafe_aci_cevrimi < y_ekseni_mesafe_aci_cevrimi_son)
		  {
			  y_yeni_dönme_acı_cevrimi = y_ekseni_mesafe_aci_cevrimi- y_ekseni_mesafe_aci_cevrimi_son;
		  }



	}


	if (htim == &htim2)//2KHz	Genel Step Motor Ayarları
	{

		HAL_GPIO_WritePin(EN_X_GPIO_Port, EN_X_Pin, 0);
		HAL_GPIO_WritePin(EN_Y_GPIO_Port, EN_Y_Pin, 0);

		yon_degistir_x(HAL_GPIO_ReadPin(X_YON_GPIO_Port, X_YON_Pin));
		yon_degistir_y(HAL_GPIO_ReadPin(Y_YON_GPIO_Port, Y_YON_Pin));

		if(HAL_GPIO_ReadPin(Y_ON_OFF_GPIO_Port, Y_ON_OFF_Pin)){
			hareket_izin_y=true;

			adım_takip = true;

			HAL_TIM_Base_Start_IT(&htim4);
		}

		if(HAL_GPIO_ReadPin(X_ON_OFF_GPIO_Port, X_ON_OFF_Pin)){
			hareket_izin_x=true;

			adım_takip = true;

			HAL_TIM_Base_Start_IT(&htim3);
		}
	}

	if (htim == &htim3)//2KHz	X Ekseni Step Motor Hareket
	{
		if(hareket_izin_x==true){
			limit_x=aci_hesaplama(x_ekseni_mesafe_aci_cevrimi);
		}

		HAL_GPIO_TogglePin(STEP_X_GPIO_Port, STEP_X_Pin);
		sayac_x++;

		if(HAL_GPIO_ReadPin(LimitSwitch_X1_GPIO_Port, LimitSwitch_X1_Pin)){
			yon_degistir_x(1);
			sayac_x=0;
			limit_x=aci_hesaplama(75);
			hareket_izin_x=false;
		}

		if(HAL_GPIO_ReadPin(LimitSwitch_X2_GPIO_Port, LimitSwitch_X2_Pin)){
			yon_degistir_x(1);
			sayac_x=0;
			limit_x=aci_hesaplama(75);
			hareket_izin_x=false;
		}

		if(sayac_x >= limit_x){
			sayac_x=0;
			adım_takip = false;

			HAL_TIM_Base_Stop_IT(&htim3);

			//HAL_TIM_Base_Stop_IT(&htim1);
		}
	}

	if (htim == &htim4)//2KHz	Y Ekseni Step Motor Hareket
	{
		if(hareket_izin_y==true){
			limit_y=aci_hesaplama(y_ekseni_mesafe_aci_cevrimi);
		}
		HAL_GPIO_TogglePin(STEP_Y_GPIO_Port, STEP_Y_Pin);
		sayac_y++;

		if(HAL_GPIO_ReadPin(LimitSwitch_Y1_GPIO_Port, LimitSwitch_Y1_Pin)){
			yon_degistir_y(1);
			sayac_y=0;
			limit_y=aci_hesaplama(75);
			hareket_izin_y=false;
		}

		if(HAL_GPIO_ReadPin(LimitSwitch_Y2_GPIO_Port, LimitSwitch_Y2_Pin)){
			yon_degistir_y(1);
			sayac_y=0;
			limit_y=aci_hesaplama(75);
			hareket_izin_y=false;
		}

		if(sayac_y >= limit_y){
			sayac_y=0;
			adım_takip = false;

			HAL_TIM_Base_Stop_IT(&htim4);

			//HAL_TIM_Base_Stop_IT(&htim1);
		}
	}
}

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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //Timer Start
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);

  //ADC DMA Start
  HAL_ADC_Start_DMA(&hadc1, ADC_Buffer, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LED4_Pin|LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|STEP_X_Pin|EN_X_Pin|DIR_X_Pin
                          |STEP_Y_Pin|EN_Y_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : X_ON_OFF_Pin */
  GPIO_InitStruct.Pin = X_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(X_ON_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitch_X2_Pin */
  GPIO_InitStruct.Pin = LimitSwitch_X2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LimitSwitch_X2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LimitSwitch_Y1_Pin LimitSwitch_Y2_Pin Y_YON_Pin Y_ON_OFF_Pin */
  GPIO_InitStruct.Pin = LimitSwitch_Y1_Pin|LimitSwitch_Y2_Pin|Y_YON_Pin|Y_ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin STEP_X_Pin EN_X_Pin DIR_X_Pin
                           STEP_Y_Pin EN_Y_Pin DIR_Y_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|STEP_X_Pin|EN_X_Pin|DIR_X_Pin
                          |STEP_Y_Pin|EN_Y_Pin|DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : X_YON_Pin */
  GPIO_InitStruct.Pin = X_YON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(X_YON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitch_X1_Pin */
  GPIO_InitStruct.Pin = LimitSwitch_X1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LimitSwitch_X1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
