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
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct 
{
  uint8_t address;
  uint8_t data[3];
} ccb_t;

typedef enum
{
  MONO,
  STEREO
} tuner_mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define __CCB_DEBUG__
//#define __RADIO_DEBUG__

#define radio_request_address 0x22
#define radio_response_address 0x23
#define radio_io_timeout 10
#define radio_force_mono_threshold 0x7F
#define radio_default_stereo_threshold 0x0E
#define virtual_if_frequency 10650
#define stereo_detection_delay 1000
#define radio_stc_poll_interval 10
#define radio_rsq_poll_interval 300
#define radio_rds_poll_interval 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
const uint8_t radio_power_up[]                        =   {0x01, 0xD0, 0x05}; // startup, GPO2/CTS enabled, XOSC enabled, fm reeceive, analog output
const uint8_t radio_fm_deemphasis[]                   =   {0x12, 0x00, 0x11, 0x00, 0x00, 0x01}; // de-emphasis = 50us
const uint8_t radio_fm_rds_int_source[]               =   {0x12, 0x00, 0x15, 0x00, 0x00, 0x01}; // generate RDSINT when RDS FIFO has at least FM_RDS_INT_FIFO_COUNT entries
const uint8_t radio_fm_rds_int_fifo_count[]           =   {0x12, 0x00, 0x15, 0x01, 0x00, 0x04}; // FM_RDS_INT_FIFO_COUNT = 4
const uint8_t radio_fm_rds_config[]                   =   {0x12, 0x00, 0x15, 0x02, 0x00, 0x01}; // RDS processing enabled
const uint8_t radio_get_int_status[]                  =   {0x14}; // wait for tune completion
const uint8_t radio_fm_rsq_status[]                   =   {0x23, 0x00};
const uint8_t radio_fm_rds_status[]                   =   {0x24, 0x01};
const uint8_t radio_fm_tune_status[]                  =   {0x22, 0x01}; // clear STCINT bit
const uint8_t ccb_if_counter[]                        =   {0x58, 0x66, 0x81}; // bit-reversed from 0x1A6680+1 (IF=10,65MHz, real value readed from my Technics SA-EH570)
//const uint8_t ccb_if_counter[]                        =   {0x58, 0xCE, 0x80}; // bit-reversed from 0x1A7300+1 (IF=10,7MHz)

uint8_t radio_fm_tune_freq[]                          =   {0x20, 0x00, 0x29, 0x7C, 0x00};
uint8_t radio_fm_blend_snr_mono_threshold[]           =   {0x12, 0x00, 0x18, 0x05, 0x00, 0x0E}; 
uint8_t radio_response[16];

volatile uint8_t* radio_response_data = 0;
volatile uint8_t radio_response_size = 0;
volatile bool radio_cts = false;
bool radio_tuned = true;

uint32_t tuner_frequency_new = 0;
uint32_t tuner_frequency_prev = 0;
tuner_mode tuner_mode_new = STEREO;
tuner_mode tuner_mode_prev = STEREO;
uint32_t stereo_detection_delay_counter = 0;
bool stereo_detection_enabled = false;

volatile ccb_t ccb;
volatile uint32_t ccb_bytes_received = 0;
ccb_t ccb_snapshot;
uint32_t ccb_bytes_snapshot;


HAL_StatusTypeDef status;

#ifdef __CCB_DEBUG__
uint8_t ccb_captured_data[36];
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void radio_reset( bool enabled )
{
  HAL_GPIO_WritePin( RST_GPIO_Port, RST_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

void carrier_detected( bool detected )
{
  // set SD=0 to indicate that the carrier is detected 
  HAL_GPIO_WritePin( SD_GPIO_Port, SD_Pin, detected ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

void stereo_indicator( bool enabled )
{
  // DO/ST line has two main purposes: 
  // 1) stereo indicator when radio is tuned (ST=0 when stereo subcarrier is detected)
  // 2) IF counter stop flag when radio tune is in progress (DO=0 when counter is stopped)
  HAL_GPIO_WritePin( ST_GPIO_Port, ST_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET );
}

HAL_StatusTypeDef radio_request( const uint8_t *request_data, uint8_t request_size, uint8_t *response_data, uint8_t response_size )
{
  HAL_StatusTypeDef status = HAL_ERROR;
  radio_cts = false;
  
  if( request_data && request_size )
    status = HAL_I2C_Master_Transmit( &hi2c1, radio_request_address, (uint8_t*) request_data, request_size, radio_io_timeout );
  
  while( !radio_cts );

  if( response_data && response_size && status == HAL_OK )
  {
    status = HAL_I2C_Master_Receive( &hi2c1, radio_response_address, response_data, response_size, radio_io_timeout );

    // There is a bug in the HAL_I2C_Master_Receive(). This function can return fake HAL_TIMEOUT error if there was a small 
    // delay (e.g. systick interrupt) between RXNE and STOPF polling in the I2C_WaitOnRXNEFlagUntilTimeout().
    // To work it around we need to read RX byte, place it to the last byte of the response_data, and safely ignore the error
    if( status == HAL_TIMEOUT &&  HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_NONE )
    {
      response_data[response_size-1] = hi2c1.Instance->RXDR;
      status = HAL_OK;
    }
  }

  return( status );
}

void HAL_ADC_LevelOutOfWindowCallback( ADC_HandleTypeDef* hadc )
{
  // input voltage is out of range, going to the radio reset 
  radio_reset( true );
  while(1);
}

void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  if( GPIO_Pin == CTS_Pin ) 
    radio_cts = true;
}

void SPI1_IRQHandler()
{
  if( HAL_GPIO_ReadPin( CE_GPIO_Port, CE_Pin ) == GPIO_PIN_RESET ) // address
  {
    ccb.address = LL_SPI_ReceiveData8( SPI1 );
    ccb_bytes_received = 0;
  }
  else // data
  {
    if( ccb_bytes_received < 3 )
    {
      ccb.data[ccb_bytes_received] = LL_SPI_ReceiveData8( SPI1 );
      ccb_bytes_received++;
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

#if !defined __CCB_DEBUG__ && !defined __RADIO_DEBUG__
  // CCB is not fully compatible with SPI
  // so we need to adjust SPI polarity and phase manually to work correctly 
  // in cases of 1) normal system startup and 2) MCU debug/reset
  LL_SPI_SetClockPolarity( SPI1, LL_SPI_POLARITY_LOW ); 
  LL_SPI_SetClockPhase( SPI1, LL_SPI_PHASE_1EDGE );
#endif
  
  LL_SPI_Enable( SPI1 ); // Enable SPI as soon as possible to keep the bus in sync with the main CPU
  LL_SPI_EnableIT_RXNE( SPI1 );
  LL_SPI_TransmitData8( SPI1, 0xFF ); // switch MISO/DO to high level as a default
  stereo_indicator( false );

  HAL_Delay( 100 ); // delay for correct radio powerup
  HAL_ADCEx_Calibration_Start( &hadc );
  HAL_ADC_Start( &hadc ); // enable power monitor
  
  //HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3 );

  radio_reset( false ); // enable radio
  HAL_Delay( 1 );
  
  status = radio_request( radio_power_up, sizeof(radio_power_up), radio_response, 1 );
  HAL_Delay( 500 ); // awaiting for crystal startup
  status = radio_request( radio_fm_tune_freq, sizeof(radio_fm_tune_freq), radio_response, 1 );

  do
    status = radio_request( radio_get_int_status, sizeof(radio_get_int_status), radio_response, 1 );
  while( radio_response[0] != 0x81 );
  
  status = radio_request( radio_fm_tune_status, sizeof(radio_fm_tune_status), radio_response, 8 ); 
  status =   radio_request( radio_fm_deemphasis, sizeof(radio_fm_deemphasis), radio_response, 1 );
  HAL_Delay( 10 ); // delay needed after each set_property command
  
//  status = radio_request( radio_fm_rds_config, sizeof(radio_fm_rds_config), radio_response, 1 );
//  HAL_Delay( 10 ); // delay needed after each set_property command
//  
//  status = radio_request( radio_fm_rds_int_source, sizeof(radio_fm_rds_int_source), radio_response, 1 );
//  HAL_Delay( 10 ); // delay needed after each set_property command
//
//  status = radio_request( radio_fm_rds_int_fifo_count, sizeof(radio_fm_rds_int_fifo_count), radio_response, 1 );
//  HAL_Delay( 10 ); // delay needed after each set_property command
  
#ifdef __CCB_DEBUG__  
  uint32_t i = 0;
  LL_SPI_DisableIT_RXNE( SPI1 );
  
  while(1)
  {
    // use ST-LINK debugger to stop the loop and show captured data
    if( LL_SPI_GetRxFIFOLevel(SPI1) != LL_SPI_RX_FIFO_EMPTY )
    {
      ccb_captured_data[i++] = LL_SPI_ReceiveData8( SPI1 );
  
      if( i == sizeof(ccb_captured_data) ) 
        i = 0;
    }
  };
#endif
  
#ifdef __RADIO_DEBUG__
  while(1)
  {
    // use ST-LINK debugger to stop the loop and show requested RSQ data 
    status = radio_request( radio_fm_rsq_status, sizeof(radio_fm_rsq_status), radio_response, 8 );
    HAL_Delay( 500 );
//    status = radio_request( radio_get_int_status, sizeof(radio_get_int_status), radio_response, 1 );
//    status = radio_request( radio_fm_rds_status, sizeof(radio_fm_rds_status), radio_response, 13 );
  }
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    __disable_irq();
      *(uint32_t*) &ccb_snapshot = *(uint32_t*) &ccb; // fast atomic 32-bit copy
      ccb_bytes_snapshot = ccb_bytes_received;
    __enable_irq();
    
    if( ccb_snapshot.address == 0x2A ) // IF counter read requested
    {
      ccb.address = 0;
      stereo_indicator( false );

      LL_SPI_TransmitData8( SPI1, ccb_if_counter[0] ); 
      LL_SPI_TransmitData8( SPI1, ccb_if_counter[1] );
      LL_SPI_TransmitData8( SPI1, ccb_if_counter[2] );
    }
    
    if( ccb_bytes_snapshot == 3 )
    {
      ccb.address = 0;
      
      if( ccb_snapshot.address == 0x29 && ccb_snapshot.data[1] == 0x00 && ccb_snapshot.data[2] == 0x13 ) // mono/stereo selection performed
      {
        switch( ccb_snapshot.data[0] )
        {
        case 0xAF: // force mono
          tuner_mode_new = MONO;
          
          if( tuner_mode_new != tuner_mode_prev )
          {
            tuner_mode_prev = tuner_mode_new;
            radio_fm_blend_snr_mono_threshold[5] = radio_force_mono_threshold;
            status = radio_request( radio_fm_blend_snr_mono_threshold, sizeof(radio_fm_blend_snr_mono_threshold), radio_response, 1 );
          }
          break;
          
        case 0xA7: // auto
          tuner_mode_new = STEREO;
          
          if( tuner_mode_new != tuner_mode_prev )
          {
            tuner_mode_prev = tuner_mode_new;
            radio_fm_blend_snr_mono_threshold[5] = radio_default_stereo_threshold;
            status = radio_request( radio_fm_blend_snr_mono_threshold, sizeof(radio_fm_blend_snr_mono_threshold), radio_response, 1 );
          }
          break;
        }
      }
      
      if( ccb_snapshot.address == 0x28 && ccb_snapshot.data[2] == 0x3B ) // tuner frequency set
      {
        tuner_frequency_new = ccb_snapshot.data[0] + (ccb_snapshot.data[1] << 8);
        
        if( tuner_frequency_new != tuner_frequency_prev )
        {
          tuner_frequency_prev = tuner_frequency_new;
          
          tuner_frequency_new = (tuner_frequency_new * 25 * 2 - virtual_if_frequency) / 10;
          radio_fm_tune_freq[2] = (tuner_frequency_new >> 8) & 0xFF;
          radio_fm_tune_freq[3] = tuner_frequency_new & 0xFF;
          status = radio_request( radio_fm_tune_freq, sizeof(radio_fm_tune_freq), radio_response, 1 );

          carrier_detected( false );
          stereo_indicator( false );
          radio_tuned = false;
          stereo_detection_enabled = false;
          stereo_detection_delay_counter = HAL_GetTick();
        }
      }
      
      if( ccb_snapshot.address == 0x28 && ccb_snapshot.data[2] == 0x3F ) // IF count started
      {
        stereo_indicator( true ); // here used as IF counter stop flag, triggers IF counter read operation
      }
    }
    
    if( !radio_tuned && (HAL_GetTick() % radio_stc_poll_interval == 0) )
    {
      status = radio_request( radio_get_int_status, sizeof(radio_get_int_status), radio_response, 1 );

      if( radio_response[0] & 0x01 ) // check STCINT
      {
        radio_tuned = true;
        status = radio_request( radio_fm_tune_status, sizeof(radio_fm_tune_status), radio_response, 8 ); 
        
        if( (radio_response[1] & 0x03) == 0x01 ) // AFCRL=0, VALID=1
          carrier_detected( true );
      }
    }
   
    if( radio_tuned && !stereo_detection_enabled && (HAL_GetTick() - stereo_detection_delay_counter > stereo_detection_delay) )
      stereo_detection_enabled = true;
    
    if( stereo_detection_enabled && (HAL_GetTick() % radio_rsq_poll_interval == 0) )
    {
      status = radio_request( radio_fm_rsq_status, sizeof(radio_fm_rsq_status), radio_response, 8 );
      stereo_indicator( radio_response[3] > 0x80 );
    }
    
    if( stereo_detection_enabled && (HAL_GetTick() % radio_rds_poll_interval == 0) )
    {
      // status = radio_request( radio_fm_rds_status, sizeof(radio_fm_rds_status), radio_response, 13 );
    }
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the analog watchdog 
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = ENABLE;
  AnalogWDGConfig.HighThreshold = 3100;
  AnalogWDGConfig.LowThreshold = 1200;
  if (HAL_ADC_AnalogWDGConfig(&hadc, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
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
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
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
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST_Pin|SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CTS_Pin */
  GPIO_InitStruct.Pin = CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ST_Pin SD_Pin */
  GPIO_InitStruct.Pin = ST_Pin|SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RDS_Pin */
  GPIO_InitStruct.Pin = RDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(RDS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
  while(1);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
