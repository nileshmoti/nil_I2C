/*
 * main.c
 *
 *  Created on: 14 Sep 2017
 *      Author: nmoti
 */


/* I2C Includes */
 #include "stm32f30x.h"
 //#include "stm32f30x_i2c.h"


/* I2C Defines */
//#define I2C_FLAG_TIMEOUT             ((uint32_t)0x1000)
//#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * I2C_FLAG_TIMEOUT))
//#define I2C_FLAG_TIMEOUT             ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * ((uint32_t)0x1000)))

#define I2C                       I2C1
#define I2C_OK                       ((uint32_t) 0)
/*
#define I2C_SCK_PIN               GPIO_Pin_6
#define I2C_CLK                   RCC_APB1Periph_I2C1
#define I2C_SCK_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define I2C_SCK_GPIO_PORT         GPIOB
#define I2C_SCK_SOURCE            GPIO_PinSource6
#define I2C_SCK_AF                GPIO_AF_4

#define I2C_SDA_PIN               GPIO_Pin_7
#define I2C_SDA_GPIO_CLK          RCC_AHBPeriph_GPIOB
#define I2C_SDA_GPIO_PORT         GPIOB
#define I2C_SDA_SOURCE            GPIO_PinSource7
#define I2C_SDA_AF                GPIO_AF_4
*/


/* I2C Private Variables */
__IO uint32_t  I2C_Timeout = I2C_LONG_TIMEOUT;

/* I2C Functions */
uint32_t I2C_TIMEOUT_UserCallback(void);
static void I2C1_LowLevel_Init(void);

/* read write funtions */
uint16_t I2C_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);
uint16_t I2C_Read(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t* pBuffer, uint16_t NumByteToRead);

/* Delay Function */
void Delay_ms(int Times);

/* Includes */
#include "nil_AIO.h"


#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Variables */
uint8_t receive_data[7];

int main(void)
{

	/* I2C */
	I2C1_LowLevel_Init();


	while(1){

		I2C_Read(0x68, 0x00, receive_data, 7);

		Delay_ms(100);



	}

	return 0;

}









/**
  * @brief  Writes one byte to the LSM303DLHC.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the LSM303DLHC register to be written.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the LSM303DLH.
  * @retval LSM303DLHC Status
  */
uint16_t I2C_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
  /* Test on BUSY Flag */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_BUSY) != RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Send Register address */
  I2C_SendData(I2C, (uint8_t) RegAddr);

  /* Wait until TCR flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_TCR) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Write data to TXDR */
  I2C_SendData(I2C, *pBuffer);

  /* Wait until STOPF flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_STOPF) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2C, I2C_ICR_STOPCF);

  return I2C_OK;
}

/**
  * @brief  Reads a block of data from the LSM303DLHC.
  * @param  DeviceAddr : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
  * @param  RegAddr : specifies the LSM303DLHC internal address register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LSM303DLH.
  * @param  NumByteToRead : number of bytes to read from the LSM303DLH ( NumByteToRead >1  only for the Mgnetometer readinf).
  * @retval LSM303DLHC register value
  */
uint16_t I2C_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_BUSY) != RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  if(NumByteToRead>1)
      RegAddr |= 0x80;


  /* Send Register address */
  I2C_SendData(I2C, (uint8_t)RegAddr);

  /* Wait until TC flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_TC) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2C, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    I2C_Timeout = I2C_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C, I2C_ISR_RXNE) == RESET)
    {
      if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
    }

    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(I2C);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;

    /* Decrement the read bytes counter */
    NumByteToRead--;
  }

  /* Wait until STOPF flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2C, I2C_ISR_STOPF) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2C, I2C_ICR_STOPCF);

  /* If all operations OK */
  return I2C_OK;
}
/**
* @brief  Initializes the low level interface used to drive the I2C
* @param  None
* @retval None
*/
static void I2C1_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* Enable the I2C periph */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

  /* Enable SCK and SDA GPIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOB , ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* I2C SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C SDA pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* I2C configuration -------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x00902025;

  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C1, &I2C_InitStructure);

  /* I2C Peripheral Enable */
  I2C_Cmd(I2C1, ENABLE);
}

uint32_t I2C_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {
  }
}






void Delay_ms(int Times){

	int a, b;

	Times = Times * 2.267;

	for(a = 0; a < Times ; a++){
		for(b = 0; b < Times ; b++){
		}
	}
}

