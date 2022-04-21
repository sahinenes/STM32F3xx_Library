/*
 * 010_i2c_slave_tx_string.c
 *
 *  Created on: 8 Nis 2022
 *      Author: Enes
 */






#include<stdio.h>
#include<string.h>
#include "stm32f3xx_i2c.h"
#include "stm32f3xx_gpio.h"

#define MY_ADDR 0x69;

#define SLAVE_ADDR  0x68

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle I2C1Handle;
//rcv buffer
uint8_t rcv_buf[32];
//rcv buffer
uint8_t Tx_buf[32]  = "STM32 Slave mode testing..";

/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle I2CPins;

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx = GPIOB;
	I2CPins.Config.PinMode = GPIO_MODE_ALTERNATE;
	I2CPins.Config.PinOPType = GPIO_OTYPER_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.Config.PuPdControl = GPIO_PUPD_NO;
	I2CPins.Config.PinAltFunc = 4;
	I2CPins. Config.PinSpeed = GPIO_SPEED_HIGH;

	//scl
	I2CPins.Config.PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.Config.PinNumber = GPIO_PIN_9;

	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.AdressMode=I2C_ADRESS_7BIT;
	I2C1Handle.I2C_Config.SlaveAdress = SLAVE_ADDR;
	I2C1Handle.I2C_Config.OwnAdress=MY_ADDR;
	I2C1Handle.I2C_Config.GeneralCall=I2C_GENERAL_CALL_EN;
	I2C1Handle.I2C_Config.Speed=I2C_SPEED_STANDART;
	I2C1Handle.I2C_Config.NoStretch=I2C_NOSTRETCH_DIS;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.Config.PinNumber = GPIO_PIN_13;
	GPIOBtn.Config.PinMode = GPIO_MODE_INPUT;
	GPIOBtn.Config.PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.Config.PuPdControl = GPIO_PUPD_PD;

	GPIO_Init(&GPIOBtn);

}


int main(void)
{
	 uint8_t commandCode = 0;
	uint8_t len=0;

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);
	len=strlen((char*)Tx_buf);


	while(1)
	{
		I2C_SlaveReceiveData(&I2C1Handle, &commandCode, 1);
		delay();
		//Master wants some data. slave has to send it
		if(commandCode == 0x51)
		{
			//send the length information to the master
			I2C_SlaveTransmitData(&I2C1Handle,&len,1);
		}else if (commandCode == 0x52)
		{
			//Send the contents of Tx_buf
			I2C_SlaveTransmitData(&I2C1Handle,Tx_buf,strlen((char*)Tx_buf));

		}
		delay();
	}

}

