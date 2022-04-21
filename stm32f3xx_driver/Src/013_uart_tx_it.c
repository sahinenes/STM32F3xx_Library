/*
 * 013_uart_tx_it.c
 *
 *  Created on: 14 Nis 2022
 *      Author: Enes
 */

#include "stm32f3xx_usart.h"
#include "stm32f3xx_gpio.h"
#include "stm32f3xx_cortex.h"
#include<stdio.h>
#include<string.h>


char msg[1024] = "UART Tx IT testing...\n\r";
USART_Handle usart2_handle;

void USART2_Init(void) {
	usart2_handle.pUSARTx = USART2;
	usart2_handle.Config.BaudRate = USART_BAUD_115200;
	usart2_handle.Config.Mode = USART_MODE_TX;
	usart2_handle.Config.StopBits = USART_STOP_1BIT;
	usart2_handle.Config.WordLength = USART_WORDLENGHT_8BIT;
	usart2_handle.Config.Parity = USART_PARITY_DISABLE;
	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void) {
	GPIO_Handle usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.Config.PinMode = GPIO_MODE_ALTERNATE;
	usart_gpios.Config.PinOPType = GPIO_OTYPER_PP;
	usart_gpios.Config.PuPdControl = GPIO_PUPD_PU;
	usart_gpios.Config.PinSpeed = GPIO_SPEED_HIGH;
	usart_gpios.Config.PinAltFunc = 7;

	//USART2 TX
	usart_gpios.Config.PinNumber = GPIO_PIN_2;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.Config.PinNumber = GPIO_PIN_3;
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void) {
	GPIO_Handle GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.Config.PinNumber = GPIO_PIN_13;
	GPIOBtn.Config.PinMode = GPIO_MODE_INPUT;
	GPIOBtn.Config.PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.Config.PuPdControl = GPIO_PUPD_PU;

	GPIO_Init(&GPIOBtn);

}

void delay(void) {
	for (uint32_t i = 0; i < 500000 / 2; i++)
		;
}

int main(void) {


	GPIO_ButtonInit();

	USART2_GPIOInit();

	USART2_Init();

	NVIC_SetPriority(USART2_IRQn, 0, 0);
	NVIC_EnableIRQ(USART2_IRQn);

	USART_PeripheralControl(USART2, ENABLE);

	while (1) {


		//wait till button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13))
			;

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		while(USART_TransmitDataIT(&usart2_handle,(uint8_t*)msg, strlen(msg))==USART_BUSY_IN_TX);

		while(USART_GetFlagStatus(&usart2_handle.pUSARTx, USART_FLAG_BUSY));

	}

	return 0;
}

void USART2_EXTI26_IRQHandler() {
	USART_IRQHandling(&usart2_handle);
}



