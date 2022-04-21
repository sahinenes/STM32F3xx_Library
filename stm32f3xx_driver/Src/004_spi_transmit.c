/*
 * 004_spi_transmit.c
 *
 *  Created on: 12 Mar 2022
 *      Author: Enes
 */


#include "stm32f3xx_gpio.h"
#include "stm32f3xx_spi.h"
#include <string.h>

void SPI2_GPIOInit()
{
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




}
SPI_Handle SPI2Handle;
void SPI2Init()
{

	SPI2Handle.pSPIx=SPI2;
	SPI2Handle.Config.BusConfig=SPI_BUS_CONFIG_FD;
	SPI2Handle.Config.Mode=SPI_MODE_MSTR;
	SPI2Handle.Config.baudRate=SPI_BAUD_RATE_16;
	SPI2Handle.Config.DataSize=SPI_DATA_SIZE_8BIT;
	SPI2Handle.Config.CPOL=SPI_CPOL_1;
	SPI2Handle.Config.CHPHA=SPI_CPHA_0;
	SPI2Handle.Config.SSM=SPI_SSM_ENABLE;


	SPI_Init(&SPI2Handle);



}

int main(void)
{

	char data[]="Hello World!";

	SPI2_GPIOInit();

	SPI2Init();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralControl(SPI2, ENABLE);

	SPI_TransmitData(&SPI2Handle, (uint8_t*)data, strlen(data));

	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

	SPI_PeripheralControl(SPI2, DISABLE);

	while(1)
	{

	}





}
