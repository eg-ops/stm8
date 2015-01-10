#include "ds1621.h"



uint8_t buff[4];
 
 
void ds1621_start_convert(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  cmd.rx_size = 0;
  cmd.rx_data = 0;  
  buff[0] = 0xEE;
  i2c_exec();
}

void ds1621_stop_convert(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  cmd.rx_size = 0;
  cmd.rx_data = 0;  
  buff[0] = 0x22;
  i2c_exec();
}


uint8_t ds1621_read_counter(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  cmd.rx_size = 1;
  cmd.rx_data = (uint8_t*)&buff;  
  *buff = 0xA8;
  i2c_exec();
  return *buff;
}

uint8_t ds1621_read_slope(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  cmd.rx_size = 1;
  cmd.rx_data = (uint8_t*)&buff;  
  *buff = 0xA9;
  i2c_exec();
  return *buff;
}

uint8_t ds1621_read_config(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  cmd.rx_size = 1;
  cmd.rx_data = (uint8_t*)&buff;  
  *buff = 0xAC;
  i2c_exec();
  return *buff;
}

void ds1621_write_config(uint8_t config){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 2;
  cmd.status = 0;
  cmd.rx_size = 0;
  cmd.rx_data = 0;  
  buff[0] = 0xAC;
  buff[1] = config;
  i2c_exec();
}

uint16_t ds1621_read_temperature(){
  cmd.dev_addr = ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 0;
  cmd.rx_size = 2;
  cmd.rx_data = (uint8_t*)&buff;  
  buff[0] = 0xAA;
  i2c_exec();
  return buff[0] << 8 | buff[1];
}


void ds1621_init(){

    /* I2C  clock Enable*/
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  
  I2C_SoftwareResetCmd(I2C1, ENABLE);
  I2C_DeInit(I2C1);
  I2C_Init(I2C1, 100000 /*30000*/, 0, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable,I2C_AcknowledgedAddress_7bit);
  I2C_Cmd(I2C1, ENABLE);

  
}
