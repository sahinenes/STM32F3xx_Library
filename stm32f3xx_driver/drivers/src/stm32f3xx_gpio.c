/*
 * stm32f3xx_gpio.c
 *
 *  Created on: 27 Şub 2022
 *      Author: Enes
 */

#include "stm32f3xx_gpio.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - Enable Peripheral Clock
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	uint8_
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void GPIO_PeriClockControl(GPIO_Type *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			IOPA_PCLCK_EN();
		} else if (pGPIOx == GPIOB) {
			IOPB_PCLCK_EN();
		} else if (pGPIOx == GPIOC) {
			IOPC_PCLCK_EN();
		} else if (pGPIOx == GPIOD) {
			IOPD_PCLCK_EN();
		} else if (pGPIOx == GPIOE) {
			IOPE_PCLCK_EN();
		} else if (pGPIOx == GPIOF) {
			IOPF_PCLCK_EN();

		} else if (pGPIOx == GPIOG) {
			IOPG_PCLCK_EN();
		} else if (pGPIOx == GPIOH) {
			IOPH_PCLCK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			IOPA_PCLCK_DIS();
		} else if (pGPIOx == GPIOB) {
			IOPB_PCLCK_DIS();
		} else if (pGPIOx == GPIOC) {
			IOPC_PCLCK_DIS();
		} else if (pGPIOx == GPIOD) {
			IOPD_PCLCK_DIS();
		} else if (pGPIOx == GPIOE) {
			IOPE_PCLCK_DIS();
		} else if (pGPIOx == GPIOF) {
			IOPF_PCLCK_DIS();

		} else if (pGPIOx == GPIOG) {
			IOPG_PCLCK_DIS();
		} else if (pGPIOx == GPIOH) {
			IOPH_PCLCK_DIS();
		}
	}

}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Init GPIO
 *
 * @param[in]         - GPIO_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void GPIO_Init(GPIO_Handle *pGPIOHandle) {
	uint32_t temp = 0;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE); //Enabled clock

	if (pGPIOHandle->Config.PinMode <= GPIO_MODE_ANOLOG) {
		// gpio mode
		temp = (pGPIOHandle->Config.PinMode
				<< (2 * pGPIOHandle->Config.PinNumber));
		pGPIOHandle->pGPIOx->MODER &=
				~(3 << (2 * pGPIOHandle->Config.PinNumber)); //reset
		pGPIOHandle->pGPIOx->MODER |= temp; //set new settings
		temp = 0;

	} else {
		if (pGPIOHandle->Config.PinMode == GPIO_MODE_IT_FT) {
			//1. configure the FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->Config.PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->Config.PinNumber);

		} else if (pGPIOHandle->Config.PinMode == GPIO_MODE_IT_RT) {
			//1 . configure the RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->Config.PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->Config.PinNumber);

		} else if (pGPIOHandle->Config.PinMode == GPIO_MODE_IT_RFT) {
			//1. configure both FTSR and RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->Config.PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR1 |= (1 << pGPIOHandle->Config.PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->Config.PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->Config.PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLCK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR1 |= 1 << pGPIOHandle->Config.PinNumber;

	}

	if (pGPIOHandle->Config.PinMode == GPIO_MODE_ALTERNATE) {
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->Config.PinNumber / 8;
		temp2 = pGPIOHandle->Config.PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4 * temp2)); //clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->Config.PinAltFunc
				<< (4 * temp2));
	}

	//gpio otyper
	temp = (pGPIOHandle->Config.PinOPType << (pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << (pGPIOHandle->Config.PinNumber)); //reset
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//gpio output speed
	temp =
			(pGPIOHandle->Config.PinSpeed << (2 * pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(3 << (2 * pGPIOHandle->Config.PinNumber)); //reset
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// gpio pull up-down
	temp = (pGPIOHandle->Config.PuPdControl
			<< (2 * pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(3 << (2 * pGPIOHandle->Config.PinNumber)); //reset
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Reset Registers
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void GPIO_DeInit(GPIO_Type *pGPIOx) {

	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - Read from input pin
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - uint8_t
 *
 * @Note              - while(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_1));
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Type *pGPIOx, uint8_t PinNumber) {
	uint8_t state;
	state = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return state;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - Read from all port
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - uint8_t
 *
 * @Note              - while(GPIO_ReadFromInputPort(GPIOA));
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Type *pGPIOx) {
	uint16_t state;
	state = (uint16_t) (pGPIOx->IDR);
	return state;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - Write pin
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	uint8_t
 * @param[in]         - uint8_t
 *
 * @return            - none
 *
 * @Note              - GPIO_WriteToOutputPin(GPIOA,GPIO_PIN_13,ENABLE);
 *
 */
void GPIO_WriteToOutputPin(GPIO_Type *pGPIOx, uint8_t PinNumber, uint8_t Value) {
	if (Value == ENABLE) {
		pGPIOx->ODR |= (1 << PinNumber);

	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - write port
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	uint16_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void GPIO_WriteToOutputPort(GPIO_Type *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle pin
 *
 * @param[in]         - GPIO_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -  GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_13);
 *
 */
void GPIO_ToggleOutputPin(GPIO_Type *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - clear exti register
 *
 * @param[in]         - uint8_t
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber) {

	//clear the exti pr register corresponding to the pin number
	if (EXTI->PR1 & (1 << PinNumber)) {
		//clear
		EXTI->PR1 |= (1 << PinNumber);
	}
}
