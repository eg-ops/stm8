#ifndef __DS1621_H
#define __DS1621_H



#include "i2c_lib.h"


#define ADDRESS 0x90 // 0b10010000


void ds1621_start_convert();
void ds1621_stop_convert();
uint8_t ds1621_read_counter();
uint8_t ds1621_read_slope();
uint8_t ds1621_read_config();
void ds1621_write_config(uint8_t config);
uint16_t ds1621_read_temperature();
void ds1621_init();


#endif