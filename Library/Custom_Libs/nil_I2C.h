#include "stm32f30x.h"

/* I2C Defines */
#define I2C_LONG_TIMEOUT             ((uint32_t)(10 * ((uint32_t)0x1000)))

/* I2C Functions */
uint32_t I2C_TIMEOUT_UserCallback(void);
void Init_I2C(I2C_TypeDef* I2Cx, uint32_t GPIO_Pin_SCK, uint32_t GPIO_Pin_SDA, GPIO_TypeDef* GPIOx);
uint16_t I2C_Write(I2C_TypeDef* I2Cx, uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);
uint16_t I2C_Read(I2C_TypeDef* I2Cx, uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
uint8_t BCD2DEC(uint8_t data);
uint8_t DEC2BCD(uint8_t data);
