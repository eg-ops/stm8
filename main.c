#include "stm8l15x.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_spi.h"
#include "stm8l15x_syscfg.h"
#include "stm8l15x_exti.h"

#include "i2c_lib.h"
#include "ds1621.h"

#include "nrf24l01.h"
#include "nrf24l01_config.h"


#define DS1621_VCC_PIN GPIO_Pin_3 // Port C

uint8_t byte1 = 0, byte2 = 0;

int counter = 0;

uint16_t temp;
uint16_t temp_intr;
uint16_t vcc;

void init_adc();
void init_gpio();
void spi(uint8_t full);
void spi_send(uint16_t temp, uint16_t vcc);
void init_irq();
uint16_t GetVcc(void);

void delay(){
  unsigned int volatile i, y;
  for (y = 0; y < 0x3; y++){
    i = 0xFFF;
    while(i--);
  }
}

int main( void )
{

 
   /*
   CLK_HSICmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  

  
  init_gpio();
  init_adc();
  
  spi(1);
  
  ds1621_init();

 */
  
  
   /* Switch to LSI as system clock source */
   /* system clock prescaler: 1*/
   CLK_LSICmd(ENABLE);
   CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
   CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_LSI);
   CLK_SYSCLKSourceSwitchCmd(ENABLE);
   while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == 0);
   CLK_HSICmd(DISABLE);
   CLK_HSEConfig(CLK_HSE_OFF);

   /*
   GPIO_Init(GPIOC, VCC_PIN, GPIO_Mode_Out_PP_Low_Slow);
   GPIO_SetBits(GPIOC, VCC_PIN);
   delay();
   GPIO_ResetBits(GPIOC, VCC_PIN);
*/
   
   
    //CLK_PeripheralClockConfig(CLK_Peripheral_RTC, DISABLE);
    //RTC_DeInit();
   CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);//Такт для RTC
   //LCD_GLASS_Init(); //Инициализируем стекляшку
   
   //Переключаем все неиспользуемые пины на выход:
    
   //GPIOA->DDR = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3  ;
   //GPIOC->DDR = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_7;
   //GPIOE->DDR = GPIO_Pin_7 | GPIO_Pin_6;
   
   //Настраиваем RTC
    //RTC_WakeUpClock_CK_SPRE_16bits
   RTC_WakeUpClockConfig( RTC_WakeUpClock_RTCCLK_Div16 /*RTC_WakeUpClock_RTCCLK_Div16*/); //time_step = 488uS
   //RTC_WakeUpClock_CK_SPRE_16bits
   RTC_ITConfig(RTC_IT_WUT, ENABLE);
   enableInterrupts();

   
   RTC_WakeUpCmd(DISABLE);
   CFG->GCR |= CFG_GCR_AL; //Поднимаем флаг AL
   RTC_SetWakeUpCounter(2049*20);//Прерывание через каждую секунду
   RTC_WakeUpCmd(ENABLE);
 
   halt();
 
 
  return 0;
  /**/

  CLK_HSICmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);


 
  
}


void init_adc(){
  
    CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);
    ADC_Init(ADC1, ADC_ConversionMode_Single, ADC_Resolution_12Bit,ADC_Prescaler_2);
    ADC_VrefintCmd(ENABLE);
    ADC_TempSensorCmd(ENABLE); //TSON
    ADC_Cmd(ADC1, ENABLE);
  
}


void init_gpio(){

    GPIO_Init(GPIOA, VCC_PIN, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, CSN, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, DS1621_VCC_PIN, GPIO_Mode_Out_PP_Low_Slow);
    

    
    GPIO_Init(GPIOA, CE, GPIO_Mode_Out_PP_Low_Slow);
    
//
    
    //GPIO_SetBits(GPIOC, VCC_PIN);
    GPIO_SetBits(GPIOC, CSN);

}

void init_irq(){

    EXTI_DeInit();
    GPIO_Init(GPIOA, IRQ, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(EXTI_Pin_5, EXTI_Trigger_Falling_Low);
    
}




void spi(uint8_t full){
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
  //SPI1 MISO- MOSI- SCK- NSS(PB7- PB6- PB5- PB4) remapping to PA2- PA3- PC6- PC5
  SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full,ENABLE);
  if (full){
    GPIO_ExternalPullUpConfig(GPIOA, MISO | MOSI, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOC, SCK, ENABLE);
    //GPIO_ExternalPullUpConfig(GPIOC, CSN, ENABLE);
    SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_32, SPI_Mode_Master, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex, SPI_NSS_Soft, 0);
    //SPI_NSSInternalSoftwareCmd(SPI1, ENABLE);
  }
    SPI_Cmd(SPI1, ENABLE);
  
  
  uint8_t status = 0, config = 0, /*fifo = 0,*/ feature = 0, rf_ch = 0, rf_setup = 0;
  uint8_t addr[5] = {0};
  uint8_t addr2[5] = {0};  
  unsigned int volatile i, y;

  
  /*
  
  nrf24l01p_read_reg(STATUS, &status, 1);
  nrf24l01p_read_reg(CONFIG, &config, 1);
  nrf24l01p_read_reg(FEATURE, &feature, 1);
  nrf24l01p_read_reg(RF_CH, &rf_ch, 1); //2
  nrf24l01p_read(TX_ADDR, addr, sizeof(addr));//0xE7 0xE7 0xE7 0xE7 0xE7
  nrf24l01p_read(RX_ADDR_P0, addr2, sizeof(addr2)); //0xE7 0xE7 0xE7 0xE7 0xE7
  
  //rf_setup = 0;
  nrf24l01p_read_reg(RF_SETUP, &rf_setup, 1);
  //(RF_SETUP_RF_PWR_0 | RF_SETUP_RF_DR_250)	
  */
  
  return;
  
  
}


__near __no_init const unsigned char Factory_VREFINT @ 0x4910;
__near __no_init const unsigned char Factory_TS @ 0x4911;

uint16_t GetVcc(void)
{
 uint32_t tmp_value;
 uint32_t res;
 uint32_t factory_ref_voltage; 
 uint8_t count;

 ADC_ChannelCmd(ADC1, ADC_Channel_Vrefint, ENABLE);
  
  tmp_value = 0;
  for (count=0; count<16; count++)
  {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
    tmp_value += ADC_GetConversionValue(ADC1);
    if (count != 0) tmp_value = tmp_value >> 1;
  };  

  if (Factory_VREFINT != 0)
   factory_ref_voltage = 0x600+Factory_VREFINT;
  else
   factory_ref_voltage = 1671;
  
  res = (factory_ref_voltage*100*3)/tmp_value;
    
  ADC_ChannelCmd(ADC1, ADC_Channel_Vrefint, DISABLE);
  
  return (uint16_t)res;  
};  

uint16_t GetTemp(void)
{
 uint32_t tmp_value;
 uint32_t res;
 uint32_t factory_ref_temp; 
 uint8_t count;

 ADC_ChannelCmd(ADC1, ADC_Channel_TempSensor, ENABLE);
  
  tmp_value = 0;
  for (count=0; count<16; count++)
  {
    ADC_SoftwareStartConv(ADC1);
    while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
    tmp_value += ADC_GetConversionValue(ADC1);
    if (count != 0) tmp_value = tmp_value >> 1;
  };  

  if (Factory_TS != 0)
   factory_ref_temp = 0x300+Factory_TS;
  else
   factory_ref_temp = 1671;
  
  res = (factory_ref_temp*100*3)/tmp_value;
  
  ADC_ChannelCmd(ADC1, ADC_Channel_TempSensor, DISABLE);

  return (uint16_t)res;  
}; 

INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
  EXTI_ClearITPendingBit(EXTI_IT_Pin5);
  if (EXTI_GetITStatus(EXTI_IT_Pin5) == SET){
      
  }
  uint8_t status = 0x1E;
  nrf24l01p_write_reg(STATUS, &status, 1);
  nrf24l01p_read_reg(STATUS, &status, 1);
  
}


void spi_send(uint16_t temp, uint16_t vcc){

  uint8_t status = 0, /*config = 0,*/ fifo = 0;
  unsigned int volatile i, y;
  uint16_t data[2] = {temp,vcc};
  nrf24l01p_write(FLUSH_TX, 0, 0);
  nrf24l01p_write(W_TX_PAYLOAD, (uint8_t*)&data, sizeof(data));
  power_up();
  
  nrf24l01p_ce_high(); 
  
  
  nrf24l01p_read_reg(FIFO_STATUS, &fifo, 1);
  nrf24l01p_read_reg(STATUS, &status, 1);
  nrf24l01p_write_reg(STATUS, &status, 1);
  nrf24l01p_read_reg(STATUS, &status, 1);
  nrf24l01p_read_reg(FIFO_STATUS, &fifo, 1);
/**/
  uint8_t fido_st = 0, st = 0;
  
  
  do{
    fido_st = nrf24l01p_read_byte(R_REGISTER | FIFO_STATUS);
    st = nrf24l01p_read_byte(R_REGISTER | STATUS);
  } while( ((fido_st & (1 << TX_FIFO_EMPTY)) == 0) && ( (st & (1 << MAX_RT)) == 0 ) );
  
  if ((st & (1 << MAX_RT)) != 0){
    nrf24l01p_write_byte(W_REGISTER | STATUS, st);
  }

  nrf24l01p_ce_low();
  
}



/**
  * @brief  RTC Interrupt routine.
  * @param  None
  * @retval None
  */


INTERRUPT_HANDLER(RTC_IRQHandler, 4)
{

  /*
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_8);
   while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
  */
   init_adc();
   vcc = GetVcc();
   temp_intr = GetTemp();
   ADC_VrefintCmd(DISABLE);
   ADC_Cmd(ADC1, DISABLE);
   CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);

  
   init_gpio();

   
   GPIO_SetBits(GPIOC, DS1621_VCC_PIN);
   
   ds1621_init();
   ds1621_write_config(1);
   ds1621_start_convert();
   
   while((ds1621_read_config() & 0x80) == 0);
   
   temp = ds1621_read_temperature();

   //ds1621_stop_convert();
   
   I2C_Cmd(I2C1, DISABLE);
   CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, DISABLE);
   
   GPIO_ResetBits(GPIOC, DS1621_VCC_PIN);

   
   
   GPIO_SetBits(GPIOA, VCC_PIN);
   
   spi(1);

   spi_send(temp, vcc);

   GPIO_ResetBits(GPIOA, VCC_PIN);
   
   SPI_Cmd(SPI1, DISABLE);
   GPIO_ExternalPullUpConfig(GPIOA, MISO | MOSI, DISABLE);
   GPIO_ExternalPullUpConfig(GPIOC, SCK, DISABLE);
   SYSCFG_REMAPPinConfig(REMAP_Pin_SPI1Full, DISABLE);
   CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
 
   
   GPIO_ResetBits(GPIOC, GPIO_Pin_All /*VCC_PIN | CSN | SCK*/);
   GPIO_ResetBits(GPIOA, GPIO_Pin_All /*MISO | MOSI| CE | IRQ*/);

  /*
   
   CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
   CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_LSI);
   CLK_HSICmd(DISABLE); 
   while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == 0);
  */ 
 
   RTC_ClearITPendingBit(RTC_IT_WUT); 

}
