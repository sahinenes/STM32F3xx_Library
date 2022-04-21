/*
 * stm32f3xx_gpio.h
 *
 *  Created on: 27 Şub 2022
 *      Author: Enes
 */

#ifndef INC_STM32F3XX_GPIO_H_
#define INC_STM32F3XX_GPIO_H_

#include "stm32f3xx.h"


/******************  Structs  ********************/
typedef struct
{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t PuPdControl;
	uint8_t PinOPType;
	uint8_t PinAltFunc;

}GPIO_Config;


typedef struct
{
	GPIO_Type* pGPIOx;
	GPIO_Config Config;

}GPIO_Handle;


/******************  Defines  ********************/
/*GPIO port mode*/
#define GPIO_MODE_INPUT     0
#define GPIO_MODE_OUTPUT    1
#define GPIO_MODE_ALTERNATE 2
#define GPIO_MODE_ANOLOG    3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*GPIO port output type*/
#define GPIO_OTYPER_PP     0
#define GPIO_OTYPER_OD     1

/*GPIO port output speed register*/
#define GPIO_SPEED_LOW    0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH   2

/*GPIO port pull-up/pull-down*/
#define GPIO_PUPD_NO    0
#define GPIO_PUPD_PU    1
#define GPIO_PUPD_PD    2
#define GPIO_PUPD_RESERVED     3

/*GPIO pins*/
#define GPIO_PIN_0 		0
#define GPIO_PIN_1 		1
#define GPIO_PIN_2 		2
#define GPIO_PIN_3 		3
#define GPIO_PIN_4 		4
#define GPIO_PIN_5 		5
#define GPIO_PIN_6 		6
#define GPIO_PIN_7 		7
#define GPIO_PIN_8 		8
#define GPIO_PIN_9 		9
#define GPIO_PIN_10 	10
#define GPIO_PIN_11 	11
#define GPIO_PIN_12 	12
#define GPIO_PIN_13 	13
#define GPIO_PIN_14 	14
#define GPIO_PIN_15 	15

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_Type *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle *pGPIOHandle);
void GPIO_DeInit(GPIO_Type *pGPIOx);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Type *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Type *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Type *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Type *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Type *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F3XX_GPIO_H_ */
