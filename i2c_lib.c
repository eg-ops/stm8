#include "i2c_lib.h"


I2C_CMD cmd;


void i2c_exec(){
  
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
                //I2C_GenerateSTOP(I2C1, ENABLE);
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                cmd.rx_size--;
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                I2C_GenerateSTOP(I2C1, ENABLE);
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                cmd.rx_size--;
                return;
            }

            while(cmd.rx_size){
            
                while( !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) );
                *cmd.rx_data++ = I2C_ReceiveData(I2C1);
                
                cmd.rx_size--;
                
                if (cmd.rx_size == 2){
                    I2C_AckPositionConfig(I2C1, I2C_AckPosition_Next);
                    I2C_AcknowledgeConfig(I2C1, DISABLE);
                }
                if (cmd.rx_size == 1){
                  I2C_GenerateSTOP(I2C1, ENABLE);
                }
            }
            
        
	
        }

      /* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
}
