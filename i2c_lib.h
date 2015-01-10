#ifndef __I2C_LIB_H
#define __I2C_LIB_H



#include "stm8l15x.h"
#include "stm8l15x_i2c.h"


void i2c_exec();

typedef struct i2c_cmd{
  uint8_t status;
  uint8_t dev_addr;
  uint8_t * tx_data;
  uint8_t tx_size;
  uint8_t * rx_data;
  uint8_t rx_size;

} I2C_CMD;


extern I2C_CMD cmd;


#endif