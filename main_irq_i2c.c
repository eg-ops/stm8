#include "stm8l15x.h"
#include "stm8l15x_clk.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_i2c.h"

 int  volatile i = 0xFFFF;
uint8_t byte1 = 0, byte2 = 0;
#define EEPROM_ADDRESS 0x90 // 0b10010000

typedef struct i2c_cmd{
  uint8_t status;
  uint8_t dev_addr;
  uint8_t * tx_data;
  uint8_t tx_size;
  uint8_t * rx_data;
  uint8_t rx_size;

} I2C_CMD;

I2C_CMD cmd;

int volatile sevent = 0;

uint8_t buff[2];

int counter = 0;



/**
  * @brief I2C1 / SPI2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler,29)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  I2C_Event_TypeDef event = I2C_GetLastEvent(I2C1);
   switch (event)
   {
        /* EV5 */
    case I2C_EVENT_MASTER_MODE_SELECT :
     /* Send slave Address for write */
      I2C_AcknowledgeConfig(I2C1, ENABLE);
      I2C_AckPositionConfig(I2C1, I2C_AckPosition_Current);
      if (cmd.tx_data != 0 && cmd.tx_size != 0){
          I2C_Send7bitAddress(I2C1, cmd.dev_addr, I2C_Direction_Transmitter);
      } else if (cmd.rx_size != 0 && cmd.rx_data != 0){
          I2C_Send7bitAddress(I2C1, cmd.dev_addr, I2C_Direction_Receiver);
      } else {
          I2C_GenerateSTOP(I2C1, ENABLE);
          cmd.status = 0;
      }
      break;
      
         /* EV6 */
    case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
      if (cmd.tx_size!=0){
        I2C_SendData(I2C1, *cmd.tx_data++);
        cmd.tx_size--;
      }
      //if (NumOfBytes == 0)
      //{
      //  I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
      //}
      break;
      
    
      //break;

      
   case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
     
     if (cmd.rx_size == 1){
        I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_GenerateSTOP(I2C1, ENABLE);
        
        while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
        *cmd.rx_data++ = I2C_ReceiveData(I2C1);
        cmd.rx_size--;
        cmd.status = 0;
        counter = 0;
        
     } else if (cmd.rx_size == 2){
       I2C_AckPositionConfig(I2C1, I2C_AckPosition_Next);
       I2C_AcknowledgeConfig(I2C1, DISABLE);
       I2C_GenerateSTOP(I2C1, ENABLE);
       while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
        *cmd.rx_data++ = I2C_ReceiveData(I2C1);
        cmd.rx_size--;
       while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
        *cmd.rx_data++ = I2C_ReceiveData(I2C1);
        cmd.rx_size--;
      cmd.status = 0;
        counter = 0;
     
     }
     
     
     break;
     
   case 0x0344:
   case 0x0341:
   case I2C_EVENT_MASTER_BYTE_RECEIVED:
 
      
    
     //I2C_GenerateSTOP(I2C1, ENABLE);
 	
      

      if (cmd.rx_size == 1){
         
      }
     

      
      if (cmd.rx_size > 0){
        *cmd.rx_data++ = I2C_ReceiveData(I2C1);
        cmd.rx_size--;
      }

      
      if (cmd.rx_size == 0){
        //I2C_AcknowledgeConfig(I2C1, DISABLE);
        I2C_GenerateSTOP(I2C1, ENABLE);
        cmd.status = 0;
      }
      
      
      

      
     break;
      /* EV8 */
     case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
      /* Transmit Data */
     // I2C_SendData(I2C1, TxBuffer[Tx_Idx++]);
      /* Decrement number of bytes */
     // NumOfBytes--;

    //  if (NumOfBytes == 0)
    //  {
    //    I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);
    //  }
      break;

      /* EV8_2 */
      case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
        
        if (cmd.rx_size == 0 || cmd.rx_data == 0){
           /* Send STOP condition */
          I2C_GenerateSTOP(I2C1, ENABLE);
          cmd.status = 0;
        }  else {
          //I2C_AckPositionConfig(I2C1, I2C_AckPosition_Current);
          I2C_GenerateSTART(I2C1, ENABLE);
    
        }
 
      //I2C_ITConfig(I2C1, I2C_IT_EVT, DISABLE);
      
      
      break;

    default:
      
     
      break;
      
   }
   sevent = event;
    counter++;
}


void exec(){
  
  /* While the bus is busy */
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

        I2C_AcknowledgeConfig(I2C1, ENABLE);
        I2C_AckPositionConfig(I2C1, I2C_AckPosition_Current);

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))	;

        if (cmd.tx_size != 0 && cmd.tx_data != 0 ){

                  
            /* Send EEPROM address for write */
            I2C_Send7bitAddress(I2C1, cmd.dev_addr, I2C_Direction_Transmitter);

            /* Test on EV6 and clear it */
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))	;

            while(cmd.tx_size) {

                /* Send the EEPROM's internal address to write to : MSB of the address first */
                I2C_SendData(I2C1, (uint8_t)*cmd.tx_data++);
                cmd.tx_size--;
                
                while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
                
            }

            if (cmd.rx_size != 0 && cmd.rx_data != 0){

                /* Send START condition */
                I2C_GenerateSTART(I2C1, ENABLE);


                /* Test on EV5 and clear it */
                while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))	;
            
            }
          
        }

        if (cmd.rx_size != 0 && cmd.rx_data != 0){

            /* Send EEPROM address for write */
            I2C_Send7bitAddress(I2C1, cmd.dev_addr, I2C_Direction_Receiver);

            /* Test on EV6 and clear it */
            while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))	;

            if (cmd.rx_size == 1){
                I2C_AcknowledgeConfig(I2C1, DISABLE);
                I2C_GenerateSTOP(I2C1, ENABLE);
                
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                cmd.rx_size--;
                return;
            } else if (cmd.rx_size == 2){
                I2C_AckPositionConfig(I2C1, I2C_AckPosition_Next);
                I2C_AcknowledgeConfig(I2C1, DISABLE);
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                I2C_GenerateSTOP(I2C1, ENABLE);
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                cmd.rx_size--;
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                cmd.rx_size--;
                return;
            }

            while(cmd.rx_size){
            
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                
                cmd.rx_size--;
            }
            
      //	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));



            //byte1 = I2C_ReceiveData(I2C1);


            //while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

            //I2C_AcknowledgeConfig(I2C1, DISABLE);
            //byte2 = I2C_ReceiveData(I2C1);

        
        }
        
	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
}

int main( void )
{
  
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  
  /* I2C  clock Enable*/
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
  
  /* I2C  clock Enable*/
  CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);
  
  I2C_DeInit(I2C1);
  I2C_Init(I2C1, 1000, 0, I2C_Mode_I2C, I2C_DutyCycle_2, I2C_Ack_Enable,I2C_AcknowledgedAddress_7bit);
  I2C_Cmd(I2C1, ENABLE);
  //I2C_ITConfig(I2C1,  (I2C_IT_TypeDef)(I2C_IT_EVT|I2C_IT_EVT), ENABLE );

  enableInterrupts();
  
  GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow);
  
  GPIO_SetBits(GPIOC, GPIO_Pin_2);
  
  //DMA_Init(DMA1_Channel3, &buff, &I2C1->DR, 1, DMA_DIR_MemoryToPeripheral, DMA_Mode_Normal, DMA_MemoryIncMode_Inc, DMA_Priority_Medium, DMA_MemoryDataSize_Byte);
 
  
  
  cmd.dev_addr = EEPROM_ADDRESS;
  cmd.tx_data = (uint8_t*)&buff;
  cmd.tx_size = 1;
  cmd.status = 1;
  buff[0] = 0xEE;


  exec();
  
  
        /* While the bus is busy */
//	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send START condition */
//	I2C_GenerateSTART(I2C1, ENABLE);

//        while(cmd.status);
 
        /*******************/
        
        
        cmd.dev_addr = EEPROM_ADDRESS;
        cmd.tx_data = (uint8_t*)&buff;
        cmd.tx_size = 1;
        cmd.rx_data = (uint8_t*)&buff;
        cmd.rx_size = 1;
        cmd.status = 1;
        buff[0] = 0xAC;
        counter = 0;
        
        exec();
  
 	/* Send START condition */
//	I2C_GenerateSTART(I2C1, ENABLE);

//        while(cmd.status);
 
        
        /*******************/
        
        
        cmd.dev_addr = EEPROM_ADDRESS;
        cmd.tx_data = (uint8_t*)&buff;
        cmd.tx_size = 1;
        cmd.rx_data = (uint8_t*)&buff;
        cmd.rx_size = 2;
        cmd.status = 1;
        buff[0] = 0xAA;
        counter = 0; 
  
 	/* Send START condition */
	//I2C_GenerateSTART(I2C1, ENABLE);

        //while(cmd.status);
       
        exec();
        
        while(1);
        
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))	;

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))	;

	/* Send the EEPROM's internal address to write to : MSB of the address first */
	I2C_SendData(I2C1, (uint8_t)0xEE);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	  /* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);

  
        
        
        
        i = (int)0xFFFFF;
	while(i--);

	/**************************************/

        while(1){

	/* While the bus is busy */
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))	;

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))	;

	/* Send the EEPROM's internal address to write to : MSB of the address first */
	I2C_SendData(I2C1, (uint8_t)0xAA);

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);


	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))	;

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, EEPROM_ADDRESS, I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))	;


//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));



	byte1 = I2C_ReceiveData(I2C1);


	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

	I2C_AcknowledgeConfig(I2C1, DISABLE);
	byte2 = I2C_ReceiveData(I2C1);


	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);

        
  
  
   //GPIO_SetBits(GPIOC, GPIO_Pin_2);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_2);
    //GPIO_SetBits(GPIOC, GPIO_Pin_4);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_4);

    //    GPIO_ToggleBits(GPIOC, GPIO_Pin_2);
  }
  
  //while(1);
    
  return 0;
}
