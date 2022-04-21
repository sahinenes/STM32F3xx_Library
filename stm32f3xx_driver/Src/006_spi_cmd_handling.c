 /*
 * 006_spi_cmd_handling.c
 *
 *  Created on: 13 Mar 2022
 *      Author: Enes
 */

#include "stm32f3xx_gpio.h"
#include "stm32f3xx_spi.h"
#include  <string.h>
#include  <stdio.h>
#include  <stdint.h>
#include  "stm32f3xx_cortex.h"

SPI_Handle SPI2Handle;

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

uint8_t buttonHandle=0;

void SPI2_GPIOInit() {


	GPIO_Handle SPIPins;

	SPIPins.pGPIOx=GPIOB;
	SPIPins.Config.PinMode=GPIO_MODE_ALTERNATE;
	SPIPins.Config.PinOPType=GPIO_OTYPER_PP;
	SPIPins.Config.PinSpeed=GPIO_SPEED_HIGH;
	SPIPins.Config.PuPdControl=GPIO_PUPD_NO;
	SPIPins.Config.PinAltFunc=5; //AF5

	SPIPins.Config.PinNumber=GPIO_PIN_13;//SCK
	GPIO_Init(&SPIPins);

	SPIPins.Config.PinNumber=GPIO_PIN_15;//MOSI
	GPIO_Init(&SPIPins);

	SPIPins.Config.PinNumber= GPIO_PIN_12; //NSS
	GPIO_Init(&SPIPins);

	SPIPins.Config.PinNumber= GPIO_PIN_14;  //MISO
	GPIO_Init(&SPIPins);


}

void SPI2Init() {

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.Config.BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.Config.Mode = SPI_MODE_MSTR;
	SPI2Handle.Config.baudRate = SPI_BAUD_RATE_16;
	SPI2Handle.Config.DataSize = SPI_DATA_SIZE_8BIT;
	SPI2Handle.Config.CPOL = SPI_CPOL_0;
	SPI2Handle.Config.CHPHA = SPI_CPHA_0;
	SPI2Handle.Config.SSM = SPI_SSM_DISABLE; //HARDWARE SLAVE MANAGMENT

	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit() {

	GPIO_Handle button, led;


	led.pGPIOx = GPIOA;
	led.Config.PinMode = GPIO_MODE_OUTPUT;
	led.Config.PinSpeed = GPIO_SPEED_LOW;
	led.Config.PinOPType = GPIO_OTYPER_PP;
	led.Config.PuPdControl = GPIO_PUPD_NO;
	led.Config.PinNumber = GPIO_PIN_5;

	GPIO_Init(&led);

	button.pGPIOx=GPIOC;
	button.Config.PinMode=GPIO_MODE_IT_FT;
	button.Config.PinNumber=GPIO_PIN_13;
	button.Config.PinOPType=GPIO_OTYPER_PP;
	button.Config.PinSpeed=GPIO_SPEED_LOW;
	button.Config.PuPdControl=GPIO_PUPD_PU;
	GPIO_Init(&button);

	NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}

uint8_t SPI_VerifyResponse(uint8_t ackbyte) {

	if (ackbyte == (uint8_t) 0xF5) {
		//ack
		return 1;
	}

	return 0;
}

int main(void) {


		uint8_t dummy_write = 0xff;
		uint8_t dummy_read;

		//initialise_monitor_handles();

		printf("Application is running\n");

		GPIO_ButtonInit();

		//this function is used to initialize the GPIO pins to behave as SPI2 pins
		SPI2_GPIOInit();

		//This function is used to initialize the SPI2 peripheral parameters
		SPI2Init();

		printf("SPI Init. done\n");

		/*
		* making SSOE 1 does NSS output enable.
		* The NSS pin is automatically managed by the hardware.
		* i.e when SPE=1 , NSS will be pulled to low
		* and NSS pin will be high when SPE=0
		*/
		SPI_SSOEConfig(SPI2,ENABLE);

		while(1)
		{
			//wait till button is pressed
			while(!buttonHandle);


			//enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,ENABLE);

		    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

			uint8_t commandcode = COMMAND_LED_CTRL;
			uint8_t ackbyte;
			uint8_t args[2];

			//send command
			SPI_TransmitData(&SPI2Handle,&commandcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(&SPI2Handle,&dummy_read,1);


			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_TransmitData(&SPI2Handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(&SPI2Handle,&ackbyte,1);



			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = LED_PIN;
				args[1] = LED_ON;

				//send arguments
				SPI_TransmitData(&SPI2Handle,args,2);
				// dummy read
				SPI_ReceiveData(&SPI2Handle,args,2);
				printf("COMMAND_LED_CTRL Executed\n");
			}

			buttonHandle=0;
			//end of COMMAND_LED_CTRL




			//2. CMD_SENOSR_READ   <analog pin number(1) >

			//wait till button is pressed
			while(!buttonHandle);



			commandcode = COMMAND_SENSOR_READ;

			//send command
			SPI_TransmitData(&SPI2Handle,&commandcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(&SPI2Handle,&dummy_read,1);


			//Send some dummy byte to fetch the response from the slave
			SPI_TransmitData(&SPI2Handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(&SPI2Handle,&ackbyte,1);

			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = ANALOG_PIN0;

				//send arguments
				SPI_TransmitData(&SPI2Handle,args,1); //sending one byte of

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

				//insert some delay so that slave can ready with the data
				delay();

				//Send some dummy bits (1 byte) fetch the response from the slave
				SPI_TransmitData(&SPI2Handle,&dummy_write,1);

				uint8_t analog_read;
				SPI_ReceiveData(&SPI2Handle,&analog_read,1);
				printf("COMMAND_SENSOR_READ %d\n",analog_read);
			}

			buttonHandle=0;


			//3.  CMD_LED_READ 	 <pin no(1) >

			//wait till button is pressed
			while(!buttonHandle);

			commandcode = COMMAND_LED_READ;

			//send command
			SPI_TransmitData(&SPI2Handle,&commandcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_TransmitData(&SPI2Handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(&SPI2Handle,&ackbyte,1);

			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = LED_PIN;

				//send arguments
				SPI_TransmitData(&SPI2Handle,args,1); //sending one byte of

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

				//insert some delay so that slave can ready with the data
				delay();

				//Send some dummy bits (1 byte) fetch the response from the slave
				SPI_TransmitData(&SPI2Handle,&dummy_write,1);

				uint8_t led_status;
				SPI_ReceiveData(&SPI2Handle,&led_status,1);
				printf("COMMAND_READ_LED %d\n",led_status);

			}

			buttonHandle=0;

			//4. CMD_PRINT 		<len(2)>  <message(len) >

			//wait till button is pressed
			while(!buttonHandle);

			commandcode = COMMAND_PRINT;

			//send command
			SPI_TransmitData(&SPI2Handle,&commandcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_TransmitData(&SPI2Handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(&SPI2Handle,&ackbyte,1);

			uint8_t message[] = "Hello ! How are you ??";
			if( SPI_VerifyResponse(ackbyte))
			{
				args[0] = strlen((char*)message);

				//send arguments
				SPI_TransmitData(&SPI2Handle,args,1); //sending length

				//do dummy read to clear off the RXNE
				SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

				delay();

				//send message
				for(int i = 0 ; i < args[0] ; i++){
					SPI_TransmitData(&SPI2Handle,&message[i],1);
					SPI_ReceiveData(&SPI2Handle,&dummy_read,1);
				}

				printf("COMMAND_PRINT Executed \n");

			}

			buttonHandle=0;

			//5. CMD_ID_READ
			//wait till button is pressed
			while(!buttonHandle);

			commandcode = COMMAND_ID_READ;

			//send command
			SPI_TransmitData(&SPI2Handle,&commandcode,1);

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(&SPI2Handle,&dummy_read,1);

			//Send some dummy byte to fetch the response from the slave
			SPI_TransmitData(&SPI2Handle,&dummy_write,1);

			//read the ack byte received
			SPI_ReceiveData(&SPI2Handle,&ackbyte,1);

			uint8_t id[11];
			uint32_t i=0;
			if( SPI_VerifyResponse(ackbyte))
			{
				//read 10 bytes id from the slave
				for(  i = 0 ; i < 10 ; i++)
				{
					//send dummy byte to fetch data from slave
					SPI_TransmitData(&SPI2Handle,&dummy_write,1);
					SPI_ReceiveData(&SPI2Handle,&id[i],1);
				}

				id[10] = '\0';

				printf("COMMAND_ID : %s \n",id);

			}

			//lets confirm SPI is not busy
			while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,DISABLE);

			printf("SPI Communication Closed\n");

			buttonHandle=0;
		}

}

void EXTI15_10_IRQHandler(void)
{
	buttonHandle=1;
	GPIO_IRQHandling(GPIO_PIN_13);
}
