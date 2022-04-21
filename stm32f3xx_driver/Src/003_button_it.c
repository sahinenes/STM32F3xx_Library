/*
 * 003_button_it.c
 *
 *  Created on: 4 Mar 2022
 *      Author: Enes
 */


#include "stm32f3xx_gpio.h"
#include "stm32f3xx_cortex.h"


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
	button.Config.PinMode=GPIO_MODE_IT_FT;
	button.Config.PinNumber=GPIO_PIN_13;
	button.Config.PinOPType=GPIO_OTYPER_PP;
	button.Config.PinSpeed=GPIO_SPEED_LOW;
	button.Config.PuPdControl=GPIO_PUPD_PU;
	GPIO_Init(&button);

	NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	NVIC_EnableIRQ(EXTI15_10_IRQn);







    /* Loop forever */
	for(;;)
	{

	}
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_5);
	GPIO_IRQHandling(GPIO_PIN_13);
}
