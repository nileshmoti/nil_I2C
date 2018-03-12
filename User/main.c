/*
 * main.c
 *
 *  Created on: 14 Sep 2017
 *      Author: nmoti
 */

/* Delay Function */
void Delay_ms(int Times);
//void I2C1_IRQHandler(void);

/* Includes */
#include "nil_AIO.h"
#include "nil_I2C.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Variables */
uint8_t receive_data[7], send_data[7];
uint8_t second, minute, hour;
int count = 0;

int main(void)
{
	/* I2C Init */
	Init_I2C(I2C1, GPIO_Pin_6, GPIO_Pin_7, GPIOB);
	//Init_I2C(I2C2, GPIO_Pin_9, GPIO_Pin_10, GPIOA);


	while(1){

		/* Shows how to use the read function */
		I2C_Read(I2C1, 0x68 << 1, 0x00, receive_data, 7);

		second=BCD2DEC(receive_data[0]);
		minute=BCD2DEC(receive_data[1]);
		hour=BCD2DEC(receive_data[2]);

		Delay_ms(100);

		/* Shows how to do the write function */
		if (count == 10){
			send_data[0]=DEC2BCD(12);
			send_data[1]=DEC2BCD(12);
			send_data[2]=DEC2BCD(12);

			I2C_Write(I2C1, 0x68 << 1, 0x00, send_data);
			I2C_Write(I2C1, 0x68 << 1, 0x01, send_data);
			I2C_Write(I2C1, 0x68 << 1, 0x02, send_data);
			count = 0;
		}

		count++;

	}

	return 0;

}
/*
void I2C1_IRQHandler(void){

	I2C_Read(I2C1, 0x68 << 1, 0x00, receive_data, 7);

	second=BCD2DEC(receive_data[0]);
	minute=BCD2DEC(receive_data[1]);
	hour=BCD2DEC(receive_data[2]);

}
*/








void Delay_ms(int Times){

	int a, b;

	Times = Times * 2.267;

	for(a = 0; a < Times ; a++){
		for(b = 0; b < Times ; b++){
		}
	}
}

