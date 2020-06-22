
#ifndef __TEF6686_H
#define __TEF6686_H

#include "stm32f0xx_hal.h"
#include "stdlib.h"
#include "stdbool.h"
#include "util.h"

#define RADIO_REQUEST_ADDRESS 0xC8
#define RADIO_RESPONSE_ADDRESS 0xC9
#define RADIO_SLICE_SIZE 24
#define RADIO_LEVEL_THRESHOLD 300
#define RADIO_USN_THRESHOLD 270
#define RADIO_WAM_THRESHOLD 230
#define RADIO_OFFSET_THRESHOLD 100
#define RADIO_IO_TIMEOUT 10

HAL_StatusTypeDef radio_request( const uint8_t *request_data, uint8_t request_size, uint8_t *response_data, uint8_t response_size );
void radio_send_patch( const uint8_t* patch, uint32_t patch_size );
void radio_init();
void radio_set_freq( uint16_t frequency );
void radio_set_stereo( bool stereo );
void radio_set_improvement( bool improve );
void radio_get_rds_data();
bool radio_station_detected();

uint16_t radio_get_status();
uint16_t radio_get_quality_status();
uint16_t radio_get_signal_status();
uint16_t radio_get_processing_status();


#endif // __TEF6686_H