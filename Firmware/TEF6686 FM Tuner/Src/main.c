/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  uint32_t address;
  uint8_t data[3];
  uint32_t bytes_received;
} ccb_t;

typedef enum
{
  MONO,
  STEREO
} tuner_mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim1_ch3_up;
DMA_HandleTypeDef hdma_tim16_ch1_up;

/* USER CODE BEGIN PV */

//const uint8_t ccb_if_counter[] = {0x58, 0xCE, 0x80}; // bit-reversed from 0x1A7300+1 (IF=10,7MHz)
const uint8_t ccb_if_counter[] = {0x58, 0x66, 0x81}; // bit-reversed from 0x1A6680+1 (IF=10,65MHz, real value read from my Technics SA-EH570)

bool radio_tuned = true;
volatile uint32_t tuner_frequency_new = 0;
uint32_t tuner_frequency_prev = 0;
volatile tuner_mode tuner_mode_new = STEREO;
tuner_mode tuner_mode_prev = STEREO;
uint32_t stereo_detection_delay_counter = 0;
bool stereo_detection_enabled = false;

volatile ccb_t ccb = {0};
//volatile uint32_t ccb_bytes_received = 0;

extern HAL_StatusTypeDef status;
extern uint8_t radio_response[16];

#ifdef __CCB_DEBUG__
uint8_t ccb_captured_data[36];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void quality_indicator( bool enabled )
{
  // just green LED indicator used to control signal quality
  HAL_GPIO_WritePin( ST_GPIO_Port, LED_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

void tuned_indicator( bool enabled )
{
  // set SD=0 to indicate that the carrier is detected 
  HAL_GPIO_WritePin( SD_GPIO_Port, SD_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET );
  quality_indicator( enabled );
}

void stereo_indicator( bool enabled )
{
  // DO/ST line has two main purposes: 
  // 1) stereo indicator when radio is tuned (ST=0 when pilot tone is detected)
  // 2) IF counter stop flag when radio tune is in progress (DO=0 when counter is stopped)
  HAL_GPIO_WritePin( ST_GPIO_Port, ST_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

//void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)

void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 )
    create_waveform( 1 ); // rewrite first section
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM1 )
    create_waveform( 2 ); // rewrite second section
}

void SPI1_IRQHandler() // this interrupt must have the highest priority level to correctly detect CE line state
{
  if( HAL_GPIO_ReadPin( CE_GPIO_Port, CE_Pin ) == GPIO_PIN_RESET ) // address
  {
    ccb.address = LL_SPI_ReceiveData8( SPI1 );
    ccb.bytes_received = 0;
    
    if( ccb.address == 0x2A ) // IF counter read requested
    {
      ccb.address = 0;
      
      LL_SPI_TransmitData8( SPI1, ccb_if_counter[0] ); 
      LL_SPI_TransmitData8( SPI1, ccb_if_counter[1] );
      LL_SPI_TransmitData8( SPI1, ccb_if_counter[2] );
      
      stereo_indicator( false );
    }
  }
  else // data
  {
    if( ccb.bytes_received < 3 )
    {
      ccb.data[ccb.bytes_received] = LL_SPI_ReceiveData8( SPI1 );
      ccb.bytes_received++;
    }
    
    if( ccb.bytes_received == 3 )
    {
      if( ccb.address == 0x28 && ccb.data[2] == 0x3B ) // tuner frequency set
      {
        tuner_frequency_new = *(uint16_t*)&ccb.data[0];
      }
      else if(ccb.address == 0x28 && ccb.data[2] == 0x3F ) // IF count started
      {
        stereo_indicator( true ); // used here as the IF counter stop flag, triggers IF counter read operation
      }
      else if( ccb.address == 0x29 && ccb.data[1] == 0x00 && ccb.data[2] == 0x13 ) // mono/stereo selection performed
      {
        if( ccb.data[0] == 0xAF )
          tuner_mode_new = MONO;
        else if( ccb.data[0] == 0xA7 )
          tuner_mode_new = STEREO;
      }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay( 50 ); // delay for correct powerup
  
  // CCB is not fully compatible with SPI
  // When the main module has been powered on, CCB bus clock line has an incorrect idle value = 0
  // so all next readings will be erroneously shifted by 1 bit
  // Lets skip the first 7 pulses to keep SPI syncing in a right way
  // It is really convenient that we can read the AF pin as usual I/O
  if( HAL_GPIO_ReadPin(CCB_CL_PORT, CCB_CL) == GPIO_PIN_RESET )
  {
    while( (CCB_CL_PORT->IDR & CCB_CL) == 0 );

    for( int i = 0; i < 7; i++ )
    {
      while( (CCB_CL_PORT->IDR & CCB_CL) != 0 );
      while( (CCB_CL_PORT->IDR & CCB_CL) == 0 );
    }
  }
  
  LL_SPI_Enable( SPI1 ); // Enable SPI as soon as possible to keep the bus in sync with the main CPU
  LL_SPI_EnableIT_RXNE( SPI1 );
  LL_SPI_TransmitData8( SPI1, 0xFF ); // switch MISO/DO to high level as a default
  stereo_indicator( false );

  if( radio_get_status() == 0x0000 ) // if radio is not initialized yet
    radio_init();
 
  radio_set_freq( 10620 ); 
  rds_init( &htim1, &htim16 );
 
#ifdef __CCB_DEBUG__  
  uint32_t i = 0;
  LL_SPI_DisableIT_RXNE( SPI1 );
  
  while(1)
  {
    // use ST-LINK debugger to stop the loop and view captured data
    if( LL_SPI_GetRxFIFOLevel(SPI1) != LL_SPI_RX_FIFO_EMPTY )
    {
      ccb_captured_data[i++] = LL_SPI_ReceiveData8( SPI1 );
  
      if( i == sizeof(ccb_captured_data) ) 
        i = 0;
    }
  }
#endif
  
#ifdef __RADIO_DEBUG__
  bool led = false;
  uint16_t level, usn, wam;
  
  while(1)
  {
    radio_set_freq( 10500 );
    HAL_Delay( 50 );
    uint16_t status = radio_get_quality_status();
    radio_set_freq( 10570 );
    HAL_Delay( 50 );
    status = radio_get_quality_status();
    // use ST-LINK debugger to stop the loop and show signal level
    led = radio_station_detected();
    quality_indicator( led );
    level = swap( &radio_response[4] ); // RADIO_LEVEL_THRESHOLD > 300
    usn = swap( &radio_response[6] ); // RADIO_USN_THRESHOLD < 270
    wam = swap( &radio_response[8] ); // RADIO_WAM_THRESHOLD < 230
    
    HAL_Delay( 100 );
  }
#endif
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t tick = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if( tuner_mode_new != tuner_mode_prev )
    {
      tuner_mode_prev = tuner_mode_new;
      // It is not needed to force mono mode manually because radio will aggressively force it in case of poor signal detection
      // So MONO button will switch off the digital signal enhancement features. It is useful for antenna setup
      radio_set_improvement( tuner_mode_new == STEREO );
      //radio_set_stereo( tuner_mode_new == STEREO );
    }

    if( tuner_frequency_new != tuner_frequency_prev )
    {
      tuner_frequency_prev = tuner_frequency_new;
      radio_set_freq( (tuner_frequency_new * 25 * 2 - VIRTUAL_IF_FREQUENCY) / 10 );

      tuned_indicator( false );
      stereo_indicator( false );
      radio_tuned = false;
      stereo_detection_enabled = false;
      stereo_detection_delay_counter = tick;
      rds_clear();
    }
    
    if( !radio_tuned && (tick % RADIO_TUNE_POLL_INTERVAL == 0) )
    {
      if( radio_get_quality_status() == 1000 )
      {
        radio_tuned = true;
        tuned_indicator( radio_station_detected() );
      }
    }
   
    if( radio_tuned && !stereo_detection_enabled && (tick - stereo_detection_delay_counter > STEREO_DETECTION_DELAY) )
      stereo_detection_enabled = true;
    
    if( stereo_detection_enabled && (tick % RADIO_STEREO_POLL_INTERVAL == 0) )
      stereo_indicator( radio_get_signal_status() == 0x8000 );
    
    if( stereo_detection_enabled && (tick % RADIO_RDS_POLL_INTERVAL == 0) )
    {
      radio_get_rds_data();
      
      // if data available, full group data received and decoder synchronized
      if( (radio_response[2] & 0xA2) == 0x82) // 10100010b 
        // if no errors in any group or there were small errors that were corrected 
        if( (radio_response[12] & 0xAA) == 0x00 ) // 10101010b
          rds_add( radio_response );
    }
    
    if( stereo_detection_enabled && (tuner_mode_new == MONO) && (tick % RADIO_QUALITY_POLL_INTERVAL == 0) )
       quality_indicator( radio_station_detected() );
    
    while( HAL_GetTick() == tick )
      HAL_PWR_EnterSLEEPMode( PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI );
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00100411;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, 1);
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_SLAVE;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BitOrder = LL_SPI_LSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */
  LL_SPI_SetRxFIFOThreshold( SPI1, LL_SPI_RX_FIFO_TH_QUARTER );
  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 97;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4655;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2715;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|ST_Pin|SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin ST_Pin SD_Pin */
  GPIO_InitStruct.Pin = LED_Pin|ST_Pin|SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

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
  
  while(1)
  {
    HAL_GPIO_TogglePin( LED_GPIO_Port, LED_Pin );
    for( volatile int i = 0; i < 50000; i++ );
  }
  
  NVIC_SystemReset();
  
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
  
  Error_Handler();
  
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
