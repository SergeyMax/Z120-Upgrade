
#include "rds.h"

uint32_t rds_phase = (RDS_PHASE_1 << 16) + RDS_PHASE_0; // random correct value
uint32_t rds_bit_buffer = 0;
uint32_t rds_coded_buffer = 0;
uint32_t rds_prev_bit = 0;
uint8_t rds_waveform_buffer[RDS_SAMPLE_COUNT*4] = {0};
volatile uint32_t rds_data_buffer[RDS_BUFFER_SIZE] = {0};

uint8_t rds_10[RDS_SAMPLE_COUNT*2];
uint8_t rds_11[RDS_SAMPLE_COUNT*2];
uint32_t rds_input_position, rds_output_position, rds_bit_position = 0;

void rds_init( TIM_HandleTypeDef* carrier_timer, TIM_HandleTypeDef* phase_timer )
{
  uint8_t max_duty = carrier_timer->Init.Period;
    
  for( int i = 0; i < RDS_SAMPLE_COUNT; i++ )
  {
    uint8_t duty10 = (uint8_t)roundf( fabsf(RDS_VOLTAGE_RANGE * sinf(PI*i/RDS_SAMPLE_COUNT-PI/2)) ); 
    uint8_t duty11 = (uint8_t)roundf( RDS_VOLTAGE_RANGE * (1 - 0.25 * (1+sinf(2*PI*i/RDS_SAMPLE_COUNT-PI/2)) ) );
    
    rds_10[i*2] = duty10;
    rds_11[i*2] = duty11;
    
    rds_10[i*2+1] = max_duty - duty10;
    rds_11[i*2+1] = max_duty - duty11;
    
    //((uint32_t*)rds_waveform_buffer)[i] = (RDS_VOLTAGE_RANGE << 24) + (0 << 16) + (RDS_VOLTAGE_RANGE << 8) + (0 << 0);
  }
  
  // rewrite carrier_timer polarity on every phase_timer CC event 
  HAL_StatusTypeDef status = HAL_DMA_Start( phase_timer->hdma[TIM_DMA_ID_CC1], (uint32_t)&rds_phase, (uint32_t)&carrier_timer->Instance->CCER, 2 );
  __HAL_TIM_ENABLE_DMA( phase_timer, TIM_DMA_CC1 );
  
  // timer working in the center aligned mode leads us to two update event generations per cycle, so we have to set RCR=1 in the timer configuration to avoid this behavior
  // reload new values for the CCR1 and CCR2 registers on each cycle
  status = HAL_TIM_DMABurst_MultiWriteStart( carrier_timer, TIM_DMABASE_CCR1, TIM_DMA_UPDATE, (uint32_t*)rds_waveform_buffer, TIM_DMABURSTLENGTH_2TRANSFERS, RDS_SAMPLE_COUNT*4 );
  
  status = HAL_TIM_PWM_Start( phase_timer, TIM_CHANNEL_1 );
  status = HAL_TIM_PWM_Start( carrier_timer, TIM_CHANNEL_1 );
  status = HAL_TIM_PWM_Start( carrier_timer, TIM_CHANNEL_2 );
  
  // TIM16 doesn't support master/slave connection, so we are using the software resynchronization
  __disable_irq();
  phase_timer->Instance->CNT = 0;
  carrier_timer->Instance->CNT = 0;
  __enable_irq();
}

void rds_add( uint8_t* data )
{
  rds_push( rds_crc(swap(&data[4])) ^ offset_a );

  rds_push( rds_crc(swap(&data[6])) ^ offset_b );

  if( data[2] & 0x10 ) // type B; A-B-C’-D 
    rds_push( rds_crc(swap(&data[8])) ^ offset_cc );
  else // type A; A-B-C-D 
    rds_push( rds_crc(swap(&data[8])) ^ offset_c );

  rds_push( rds_crc(swap(&data[10])) ^ offset_d );
}

void rds_push( uint32_t data )
{
  rds_data_buffer[rds_input_position] = data;
  
  if( ++rds_input_position >= RDS_BUFFER_SIZE )
    rds_input_position = 0;
}

void rds_clear()
{
  for( uint32_t i = 0; i < RDS_BUFFER_SIZE; i++ )
    rds_data_buffer[i] = 0;
}

void create_waveform( uint32_t section )
{
  uint32_t* buffer;
  uint16_t* phase;
  
  switch( section )
  {
  case 1:
    buffer = (uint32_t*)&rds_waveform_buffer[0];
    phase = (uint16_t*)&rds_phase;
    break;
  case 2:
    buffer = (uint32_t*)&rds_waveform_buffer[RDS_SAMPLE_COUNT*2];
    phase = (uint16_t*)&rds_phase+1;
    break;
  }
  
  switch( rds_coded_buffer >> 30 ) // first two bits (MSB first)
  {
  case 0: // 00b
    copy_waveform( (uint32_t*)rds_11, buffer);
    *phase = RDS_PHASE_0;
    break;
  case 1: // 01b
    copy_waveform( (uint32_t*)rds_10, buffer);
    *phase = RDS_PHASE_1;
    break;
  case 2: // 10b
    copy_waveform( (uint32_t*)rds_10, buffer);
    *phase = RDS_PHASE_0;
    break;
  case 3: // 11b
    copy_waveform( (uint32_t*)rds_11, buffer);
    *phase = RDS_PHASE_1;
    break;
  }
  
  rds_coded_buffer <<= 1;
  
  if( section == 2 ) // each second call
  {
    // differential biphase encoding
    rds_prev_bit ^= (rds_bit_buffer >> 31);
    rds_coded_buffer |= rds_prev_bit + 1; // insert 0 as 01b and 1 as 10b
    
    rds_bit_buffer <<= 1;
    
    if( ++rds_bit_position >= 26 )
    {
      rds_bit_position = 0;
      rds_bit_buffer = rds_data_buffer[rds_output_position];

      if( ++rds_output_position >= RDS_BUFFER_SIZE )
        rds_output_position = 0;
    }
  }
}