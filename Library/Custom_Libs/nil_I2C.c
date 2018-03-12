/*
 * I2C Library, extracted from STM32F3 demo code for the Discovery board(therefore you may see
 * LSM303DLHC in comments, its the compass module).
 * Usage for STM32F3
 *
 * Example of Usage:
 * Example show the read/write functionality and initialization, DS3231 RTC used in this example
#include "nil_I2C.h"

//Variables
uint8_t receive_data[7], send_data[7];
uint8_t second, minute, hour;
int count = 0;

int main(void)
{
	 //I2C Init
	//Init_I2C(I2C1, GPIO_Pin_6, GPIO_Pin_7, GPIOB);
	Init_I2C(I2C2, GPIO_Pin_9, GPIO_Pin_10, GPIOA);

	while(1){

		// Shows how to use the read function
		I2C_Read(I2C2, 0x68 << 1, 0x00, receive_data, 7);

		second=BCD2DEC(receive_data[0]);
		minute=BCD2DEC(receive_data[1]);
		hour=BCD2DEC(receive_data[2]);

		Delay_ms(100);

		// Shows how to do the write function
		if (count == 10){
			send_data[0]=DEC2BCD(12);
			send_data[1]=DEC2BCD(12);
			send_data[2]=DEC2BCD(12);

			I2C_Write(I2C2, 0x68 << 1, 0x00, send_data);
			I2C_Write(I2C2, 0x68 << 1, 0x01, send_data);
			I2C_Write(I2C2, 0x68 << 1, 0x02, send_data);
			count = 0;
		}
		count++;
	}
	return 0;
}
 *
 * Output:
Time: 12:12:39
Time: 12:12:40
Time: 12:12:40
Time: 12:12:40
Time: 12:12:41
Time: 12:12:12
Time: 12:12:12
Time: 12:12:12
Time: 12:12:12
Time: 12:12:13
Time: 12:12:13
Time: 12:12:13
Time: 12:12:14
 * */



/* Includes */
#include "nil_I2C.h"

/* I2C Private Variables */
__IO uint32_t  I2C_Timeout = I2C_LONG_TIMEOUT;

/**
  * @brief  Writes one byte to the LSM303DLHC.
  * @param  I2Cx : where x can be 1 or 2 to select the I2C peripheral.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the LSM303DLHC register to be written.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the LSM303DLH.
  * @retval LSM303DLHC Status
  */
uint16_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer)
{
  /* Test on BUSY Flag */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, DeviceAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Send Register address */
  I2C_SendData(I2Cx, (uint8_t) RegAddr);

  /* Wait until TCR flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TCR) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Write data to TXDR */
  I2C_SendData(I2Cx, *pBuffer);

  /* Wait until STOPF flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

  return 0;
}

/**
  * @brief  Reads a block of data from the LSM303DLHC.
  * @param  I2Cx : where x can be 1 or 2 to select the I2C peripheral.
  * @param  DeviceAddr : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
  * @param  RegAddr : specifies the LSM303DLHC internal address register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LSM303DLH.
  * @param  NumByteToRead : number of bytes to read from the LSM303DLH ( NumByteToRead >1  only for the Mgnetometer readinf).
  * @retval LSM303DLHC register value
  */
uint16_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
  /* Test on BUSY Flag */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_BUSY) != RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, DeviceAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

  /* Wait until TXIS flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TXIS) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  //if(NumByteToRead>1)
  //    RegAddr |= 0x80;


  /* Send Register address */
  I2C_SendData(I2Cx, (uint8_t)RegAddr);

  /* Wait until TC flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_TC) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Configure slave address, nbytes, reload, end mode and start or stop generation */
  I2C_TransferHandling(I2Cx, DeviceAddr, NumByteToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

  /* Wait until all data are received */
  while (NumByteToRead)
  {
    /* Wait until RXNE flag is set */
    I2C_Timeout = I2C_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2Cx, I2C_ISR_RXNE) == RESET)
    {
      if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
    }

    /* Read data from RXDR */
    *pBuffer = I2C_ReceiveData(I2Cx);
    /* Point to the next location where the byte read will be saved */
    pBuffer++;

    /* Decrement the read bytes counter */
    NumByteToRead--;
  }

  /* Wait until STOPF flag is set */
  I2C_Timeout = I2C_LONG_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_ISR_STOPF) == RESET)
  {
    if((I2C_Timeout--) == 0) return I2C_TIMEOUT_UserCallback();
  }

  /* Clear STOPF flag */
  I2C_ClearFlag(I2Cx, I2C_ICR_STOPCF);

  /* If all operations OK */
  return 0;
}


void Init_I2C(I2C_TypeDef* I2Cx,
						uint32_t GPIO_Pin_SCK,
						uint32_t GPIO_Pin_SDA,
						GPIO_TypeDef* GPIOx)
{

  /* Trying to keep the user input for the function short, I just listed all the GPIO Ports, GPIO Pins
   * and RCC_APB1Periph, you might not use all, but its there. This may not be the most elegant way
   * to do it, but it works and it makes it easier for the user to use.
   *  */
  uint32_t RCC_AHBPeriph_GPIOx;
  uint32_t GPIO_PinSource_SCK, GPIO_PinSource_SDA;
  uint32_t RCC_APB1Periph_I2Cx;
  /* For GPIO Ports */
  if 	  (GPIOx == GPIOA) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOA;
  else if (GPIOx == GPIOB) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOB;
  else if (GPIOx == GPIOC) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOC;
  else if (GPIOx == GPIOD) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOD;
  else if (GPIOx == GPIOE) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOE;
  else if (GPIOx == GPIOF) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOF;
  else if (GPIOx == GPIOG) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOG;
  else if (GPIOx == GPIOH) RCC_AHBPeriph_GPIOx = RCC_AHBPeriph_GPIOH;

  /* For GPIO Pins */
  if (GPIO_Pin_SCK == GPIO_Pin_0) GPIO_PinSource_SCK = GPIO_PinSource0;
  else if (GPIO_Pin_SCK == GPIO_Pin_1) GPIO_PinSource_SCK = GPIO_PinSource1;
  else if (GPIO_Pin_SCK == GPIO_Pin_2) GPIO_PinSource_SCK = GPIO_PinSource2;
  else if (GPIO_Pin_SCK == GPIO_Pin_3) GPIO_PinSource_SCK = GPIO_PinSource3;
  else if (GPIO_Pin_SCK == GPIO_Pin_4) GPIO_PinSource_SCK = GPIO_PinSource4;
  else if (GPIO_Pin_SCK == GPIO_Pin_5) GPIO_PinSource_SCK = GPIO_PinSource5;
  else if (GPIO_Pin_SCK == GPIO_Pin_6) GPIO_PinSource_SCK = GPIO_PinSource6;
  else if (GPIO_Pin_SCK == GPIO_Pin_7) GPIO_PinSource_SCK = GPIO_PinSource7;
  else if (GPIO_Pin_SCK == GPIO_Pin_8) GPIO_PinSource_SCK = GPIO_PinSource8;
  else if (GPIO_Pin_SCK == GPIO_Pin_9) GPIO_PinSource_SCK = GPIO_PinSource9;
  else if (GPIO_Pin_SCK == GPIO_Pin_10) GPIO_PinSource_SCK = GPIO_PinSource10;
  else if (GPIO_Pin_SCK == GPIO_Pin_11) GPIO_PinSource_SCK = GPIO_PinSource11;
  else if (GPIO_Pin_SCK == GPIO_Pin_12) GPIO_PinSource_SCK = GPIO_PinSource12;
  else if (GPIO_Pin_SCK == GPIO_Pin_13) GPIO_PinSource_SCK = GPIO_PinSource13;
  else if (GPIO_Pin_SCK == GPIO_Pin_14) GPIO_PinSource_SCK = GPIO_PinSource14;
  else if (GPIO_Pin_SCK == GPIO_Pin_15) GPIO_PinSource_SCK = GPIO_PinSource15;

  if (GPIO_Pin_SDA == GPIO_Pin_0) GPIO_PinSource_SDA = GPIO_PinSource0;
  else if (GPIO_Pin_SDA == GPIO_Pin_1) GPIO_PinSource_SDA = GPIO_PinSource1;
  else if (GPIO_Pin_SDA == GPIO_Pin_2) GPIO_PinSource_SDA = GPIO_PinSource2;
  else if (GPIO_Pin_SDA == GPIO_Pin_3) GPIO_PinSource_SDA = GPIO_PinSource3;
  else if (GPIO_Pin_SDA == GPIO_Pin_4) GPIO_PinSource_SDA = GPIO_PinSource4;
  else if (GPIO_Pin_SDA == GPIO_Pin_5) GPIO_PinSource_SDA = GPIO_PinSource5;
  else if (GPIO_Pin_SDA == GPIO_Pin_6) GPIO_PinSource_SDA = GPIO_PinSource6;
  else if (GPIO_Pin_SDA == GPIO_Pin_7) GPIO_PinSource_SDA = GPIO_PinSource7;
  else if (GPIO_Pin_SDA == GPIO_Pin_8) GPIO_PinSource_SDA = GPIO_PinSource8;
  else if (GPIO_Pin_SDA == GPIO_Pin_9) GPIO_PinSource_SDA = GPIO_PinSource9;
  else if (GPIO_Pin_SDA == GPIO_Pin_10) GPIO_PinSource_SDA = GPIO_PinSource10;
  else if (GPIO_Pin_SDA == GPIO_Pin_11) GPIO_PinSource_SDA = GPIO_PinSource11;
  else if (GPIO_Pin_SDA == GPIO_Pin_12) GPIO_PinSource_SDA = GPIO_PinSource12;
  else if (GPIO_Pin_SDA == GPIO_Pin_13) GPIO_PinSource_SDA = GPIO_PinSource13;
  else if (GPIO_Pin_SDA == GPIO_Pin_14) GPIO_PinSource_SDA = GPIO_PinSource14;
  else if (GPIO_Pin_SDA == GPIO_Pin_15) GPIO_PinSource_SDA = GPIO_PinSource15;

  /* For RCC_APB1Periph */
  if 	  (I2Cx == I2C1) RCC_APB1Periph_I2Cx = RCC_APB1Periph_I2C1;
  else if (I2Cx == I2C2) RCC_APB1Periph_I2Cx = RCC_APB1Periph_I2C2;


  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  /* For Interrupt */
  //NVIC_InitTypeDef NVIC_InitStructure;

  /* RCC I2C Clock */
  //RCC_I2CCLKConfig(RCC_I2C2CLK_SYSCLK);

  /* Enable the I2C Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE);

  /* Enable SCK and SDA GPIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOx | RCC_AHBPeriph_GPIOx , ENABLE);

  GPIO_PinAFConfig(GPIOx, GPIO_PinSource_SCK, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOx, GPIO_PinSource_SDA, GPIO_AF_4);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* I2C SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCK;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /* I2C SDA pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_SDA;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /* I2C configuration -------------------------------------------------------*/
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStructure.I2C_DigitalFilter = 0x00;
  I2C_InitStructure.I2C_OwnAddress1 = 0x00;		// May need to add this, for master and slave and address
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_Timing = 0x00902025;

  /* Apply I2C configuration */
  I2C_Init(I2Cx, &I2C_InitStructure);

  /* To enable interrupt */
  //I2C_ITConfig(I2C1, I2C_IT_RXI, ENABLE);
  //I2C_ITConfig(I2C1, I2C_FLAG_RXNE, ENABLE);
  //NVIC_EnableIRQ(I2C1_EV_IRQn);
  //NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);


  /* I2C Peripheral Enable */
  I2C_Cmd(I2Cx, ENABLE);
}

uint32_t I2C_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {
  }
}

/* Binary Coded Decimal 2 Decimal */
uint8_t BCD2DEC(uint8_t data)
{
	return (data>>4)*10 + (data&0x0f);
}

/* Decimal 2 Binary Coded Decimal */
uint8_t DEC2BCD(uint8_t data)
{
	return (data/10)<<4|(data%10);
}

