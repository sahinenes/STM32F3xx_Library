
/*
 * 001_led_high.c
 *
 *  Created on: 4 Mar 2022
 *      Author: Enes
 */
#include "stm32f3xx_gpio.h"



int main(void)
{


	GPIO_Handle led;
	led.pGPIOx=GPIOA;
	led.Config.PinMode=GPIO_MODE_OUTPUT;
	led.Config.PinNumber=5;
	led.Config.PinOPType=GPIO_OTYPER_PP;
	led.Config.PinSpeed=GPIO_SPEED_LOW;
	led.Config.PuPdControl=GPIO_PUPD_NO;
	GPIO_Init(&led);

	GPIO_WriteToOutputPin(GPIOA, 5, HIGH);







    /* Loop forever */
	for(;;)
	{

	}
}
