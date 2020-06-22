
#ifndef __RDS_H
#define __RDS_H

#include "stm32f0xx_hal.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "util.h"

#define RDS_BUFFER_SIZE 52
#define RDS_SAMPLE_COUNT 24
#define RDS_VOLTAGE_RANGE 30
#define RDS_PHASE_0 0x0011
#define RDS_PHASE_1 0x0033
#define PI 3.14159265358979323846
#define offset_a (0xFC << 6)
#define offset_b (0x198 << 6)
#define offset_c (0x168 << 6)
#define offset_cc (0x350 << 6)
#define offset_d (0x1B4 << 6)

extern uint32_t rds_coder( uint16_t input );
extern void copy_waveform( uint32_t* source, uint32_t* distination );
extern uint32_t rds_crc( uint16_t input );

void rds_init( TIM_HandleTypeDef* carrier_timer, TIM_HandleTypeDef* phase_timer );
void rds_add( uint8_t* data );
void rds_push( uint32_t data );
void rds_clear();
void create_waveform( uint32_t section );

#endif // __RDS_H
