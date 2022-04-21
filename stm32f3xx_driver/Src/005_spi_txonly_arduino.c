/*
 * 005_spi_txonly_arduino.c
 *
 *  Created on: 13 Mar 2022
 *      Author: Enes
 */

#include "stm32f3xx_gpio.h"
#include "stm32f3xx_spi.h"
#include "stm32f3xx_cortex.h"
#include <string.h>

SPI_Handle SPI2Handle;

uint8_t button=0;

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

void Button_GPIOInit() {
	GPIO_Handle button;
	button.pGPIOx = GPIOC;
	button.Config.PinMode = GPIO_MODE_IT_FT;
	button.Config.PinNumber = GPIO_PIN_13;
	button.Config.PinOPType = GPIO_OTYPER_PP;
	button.Config.PinSpeed = GPIO_SPEED_LOW;
	button.Config.PuPdControl = GPIO_PUPD_PU;
	GPIO_Init(&button);

	NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void delay(void) {
	for (uint32_t i = 0; i < 500000; i++)
		;
}

int main(void) {

	char data[] ="making SSOE 1 does NSS output enable. The NSS pin is automatically managed by the hardware. i.e when SPE=1 , NSS will be pulled to low and NSS pin will be high when SPE=0";

	SPI2_GPIOInit();

	Button_GPIOInit();

	SPI2Init();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * i.e when SPE=1 , NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while (1) {

		while (!button);

		delay();

		SPI_PeripheralControl(SPI2, ENABLE); // spı2 peripheral enable

		uint8_t dataLen = strlen(data);

		SPI_TransmitData(&SPI2Handle, &dataLen, 1); //send data len

		SPI_TransmitData(&SPI2Handle, (uint8_t*)data, strlen(data));

		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG))
			;

		SPI_PeriClockControl(SPI2, DISABLE);
		button=0;

	}

}

void EXTI15_10_IRQHandler(void)
{
	button=1;
	GPIO_IRQHandling(GPIO_PIN_13);
}
