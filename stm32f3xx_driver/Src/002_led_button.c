/*
 * 002_led_button.c
 *
 *  Created on: 4 Mar 2022
 *      Author: Enes
 */




#include "stm32f3xx_gpio.h"
#include <stdbool.h>

void delay(void);

void delay(void)
{
	for(uint16_t i=0;i<65535;i++);

}

int main(void)
{



	GPIO_Handle led,button;
	led.pGPIOx=GPIOA;
	led.Config.PinMode=GPIO_MODE_OUTPUT;
	led.Config.PinNumber=5;
	led.Config.PinOPType=GPIO_OTYPER_PP;
	led.Config.PinSpeed=GPIO_SPEED_LOW;
	led.Config.PuPdControl=GPIO_PUPD_NO;
	GPIO_Init(&led);

	button.pGPIOx=GPIOC;
	button.Config.PinMode=GPIO_MODE_INPUT;
	button.Config.PinNumber=13;
	button.Config.PinOPType=GPIO_OTYPER_PP;
	button.Config.PinSpeed=GPIO_SPEED_LOW;
	button.Config.PuPdControl=GPIO_PUPD_PU;
	GPIO_Init(&button);




    /* Loop forever */
	while(1)
	{
		while(GPIO_ReadFromInputPin(GPIOC, 13));
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);



	}
}
