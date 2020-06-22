  MODULE rds_coder
  PUBLIC copy_waveform
  PUBLIC rds_crc
  SECTION RDS_CODE:CODE

;===============================================================================
; input: R0 - (uint16_t) data
; output: R0 - (uint32_t) data+crc
; adding 10-bit CRC to 16-bit data
; R0-R3, R12 will not be used by C compiler and may be changed freely
;===============================================================================
rds_crc:
  lsls R0, R0, #16; move initial data to the upper halfword
  mov R12, R0; input value will be added to the CRC later
  ldr R1, =crc_poly
  ldr R1, [R1]; load polynomial value shifted by 16+(16-10) bit left
  movs R3, #16; sixteen input bits will be processed

crc_loop1:
  lsls R0, R0, #1; R0 = R0 << 1
  bcc crc_noxor; if C bit is not set - skip XOR operation
  eors R0, R0, R1; R0 = R0 ^ crc_poly
  
crc_noxor:
  subs R3, R3, #1; decrement main loop counter
  bne crc_loop1

  lsrs R0, R0, #16; move CRC to the lower halfword
  add R0, R0, R12; add initial data

  bx LR; return result in R0 as left-aligned data(16 bit)+crc(10 bit)+0(6 bit)

;===============================================================================
; input: R0 - (uint32_t*) source, R1 - (uint32_t*) destination
; output: none
; copying of the 48 byte array from source to destination
; R0-R3, R12 will not be used by C compiler and may be changed freely
;===============================================================================
copy_waveform:
  rept 6
  ldm R0!, {R2, R3}
  stm R1!, {R2, R3}
  endr
  
  bx LR; return

;===============================================================================
  SECTION RDS_DATA:DATA(4)

crc_poly      dc32  0x05B9 << 16+(16-10); msb 1 will be skipped
;===============================================================================

  END