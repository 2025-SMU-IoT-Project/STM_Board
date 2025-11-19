#ifndef __VL53L0X_IIC_H
#define __VL53L0X_IIC_H

#include "stdint.h"
#include "sys.h"
#include "delay.h"

#define VL_SDA_RCC  RCC_APB2Periph_GPIOB
#define VL_SDA_PIN  GPIO_Pin_11
#define VL_SDA_IOx  GPIOB  
#define VL_SCL_RCC  RCC_APB2Periph_GPIOB
#define VL_SCL_PIN  GPIO_Pin_10
#define VL_SCL_IOx  GPIOB  

#define VL_SDA_IN()  {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=8<<12;}
#define VL_SDA_OUT() {GPIOB->CRH&=0xFFFF0FFF;GPIOB->CRH|=3<<12;}
#define VL_IIC_SCL    PBout(10)      //SCL
#define VL_IIC_SDA    PBout(11)      //SDA	 
#define VL_READ_SDA   PBin(11) 		 //ÊäÈëSDA 

//Status
#define STATUS_OK       0x00
#define STATUS_FAIL     0x01

void VL53L0X_IIC_Init(void);
uint8_t VL_I2C_Read_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len);
uint8_t VL_I2C_Write_nByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint16_t len);

#endif 

