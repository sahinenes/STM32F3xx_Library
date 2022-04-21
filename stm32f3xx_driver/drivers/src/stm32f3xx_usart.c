/*
 * stm32f3xx_usart.c
 *
 *  Created on: 10 Mar 2022
 *      Author: Enes
 */

#include "stm32f3xx_usart.h"


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - USART peripheral clock enable or disable
 *
 * @param[in]         - USART_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - USART_PeriClockControl(USART2,ENABLE);
 *
 */
void USART_PeriClockControl(USART_Type *pUSARTx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLCK_EN();
		} else if (pUSARTx == USART2) {
			USART2_PCLCK_EN();
		} else if (pUSARTx == USART3) {
			USART3_PCLCK_EN();
		} else if (pUSARTx == UART4) {
			UART4_PCLCK_EN();
		} else if (pUSARTx == UART5) {
			UART5_PCLCK_EN();
		}
	} else {
		if (pUSARTx == USART1) {
			USART1_PCLCK_DIS();
		} else if (pUSARTx == USART2) {
			USART2_PCLCK_DIS();
		} else if (pUSARTx == USART3) {
			USART3_PCLCK_DIS();
		} else if (pUSARTx == UART4) {
			UART4_PCLCK_DIS();
		} else if (pUSARTx == UART5) {
			UART5_PCLCK_DIS();
		}
	}

}


/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             - USART peripheral  enable or disable
 *
 * @param[in]         - USART_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_PeripheralControl(USART_Type *pUSARTx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pUSARTx->CR1 |= USART_CR1_UE; //USART enable
	} else {
		pUSARTx->CR1 &= ~USART_CR1_UE; //USART disable

	}
}


/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - USART init
 *
 * @param[in]         - USART_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_Init(USART_Handle *pUSARTHandle) {
	uint32_t temp = 0;

	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	pUSARTHandle->pUSARTx->CR1 &= ~USART_CR1_UE; //USART disable

	temp |= (pUSARTHandle->Config.WordLength << USART_CR1_M1_Pos); //Word length
	temp |= (pUSARTHandle->Config.OverSampling <<USART_CR1_OVER8_Pos); //Oversampling mode

	pUSARTHandle->pUSARTx->CR1 |= temp;

	temp = 0;

	pUSARTHandle->pUSARTx->CR2 |= (pUSARTHandle->Config.StopBits << USART_CR2_STOP_Pos); //STOP bits

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->Config.BaudRate);

	if (pUSARTHandle->Config.Mode == USART_MODE_TXRX) {
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RE; //Receiver enable
		pUSARTHandle->pUSARTx->CR1 |=USART_CR1_TE; //Transmitter enable
	} else if (pUSARTHandle->Config.Mode == USART_MODE_TX) {
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TE; //Transmitter enable
	} else if (pUSARTHandle->Config.Mode == USART_MODE_RX) {
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RE; //Receiver enable
	}

	if (pUSARTHandle->Config.Parity != USART_PARITY_DISABLE) {
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_PCE; //Parity control enable
		pUSARTHandle->pUSARTx->CR1 |= (pUSARTHandle->Config.Parity << USART_CR1_PS_Pos); // Parity selection
	}

	pUSARTHandle->pUSARTx->CR1 |= (1 << 0); //USART enable
}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - USART Deinit
 *
 * @param[in]         - USART_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_DeInit(USART_Type *pUSARTx) {
	//todo deinit
}


/*********************************************************************
 * @fn      		  - USART_TransmitData
 *
 * @brief             - USART transmit data
 *
 * @param[in]         - USART_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_TransmitData(USART_Handle *pUSARTx, uint8_t *pTxBuffer, uint32_t Len) {

	pUSARTx->pUSARTx->CR1 |= (1 << 3); //Transmitter enable
	uint16_t *pdata;
	while (Len > 0) {
		while (!(pUSARTx->pUSARTx->ISR & USART_ISR_TXE))
			; //Transmit data register empty

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if (pUSARTx->Config.WordLength == USART_WORDLENGHT_9BIT) {
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTx->pUSARTx->TDR = (*pdata & (uint16_t) 0x01FF);

			//check for ParityControl
			if (pUSARTx->Config.Parity == USART_PARITY_DISABLE) {
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			} else {
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else {
			//This is 8bit data transfer
			pUSARTx->pUSARTx->TDR = (*pTxBuffer & (uint8_t) 0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
		Len--;

	}
	while (!(pUSARTx->pUSARTx->ISR & USART_ISR_TC))
		;				//Transmission complete

	pUSARTx->pUSARTx->CR1 &= ~USART_CR1_TE; //Transmitter disable
}

/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - USART receive data
 *
 * @param[in]         - USART_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_ReceiveData(USART_Handle *pUSARTx, uint8_t *pRxBuffer, uint32_t Len) {
	pUSARTx->pUSARTx->CR1 |= USART_CR1_RE; //Receiver enable
	while (Len > 0) {

		while (!(pUSARTx->pUSARTx->ISR & USART_ISR_RXNE))
			;						//Read data register not empty

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if (pUSARTx->Config.WordLength == USART_WORDLENGHT_9BIT) {
			//We are going to receive 9bit data in a frame

			//Now, check are we using ParityControl control or not
			if (pUSARTx->Config.Parity == USART_PARITY_DISABLE) {
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTx->pUSARTx->RDR
						& (uint16_t) 0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			} else {
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTx->pUSARTx->RDR & (uint8_t) 0xFF);
				pRxBuffer++;
			}
		} else {
			//We are going to receive 8bit data in a frame

			//Now, check are we using ParityControl control or not
			if (pUSARTx->Config.Parity == USART_PARITY_DISABLE) {
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (uint8_t) (pUSARTx->pUSARTx->RDR & (uint8_t) 0xFF);
			}

			else {
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t) (pUSARTx->pUSARTx->RDR & (uint8_t) 0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;

		}
		Len--;
	}
	while ((pUSARTx->pUSARTx->ISR & USART_ISR_BUSY))
		;						//Busy flag
	pUSARTx->pUSARTx->CR1 &= ~USART_CR1_RE; //Receiver disable
}


/*********************************************************************
 * @fn      		  - USART_TransmitDataIT
 *
 * @brief             - USART transmit data with interrupts
 *
 * @param[in]         - USART_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - tx state
 *
 * @Note              - none
 *
 */
uint8_t USART_TransmitDataIT(USART_Handle *pUSARTHandle, uint8_t *pTxBuffer,
		uint32_t Len) {

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if (txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TXEIE;	//TXEIE: interrupt enable

		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TCIE;	//TCIE: Transmission complete interrupt enable

	}

	return txstate;

}

/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - USART receive data with interrupts
 *
 * @param[in]         - USART_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - rx state
 *
 * @Note              - none
 *
 */
uint8_t USART_ReceiveDataIT(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer,
		uint32_t Len) {
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if (rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RXNEIE;	//RXNEIE: RXNE interrupt enable

	}

	return rxstate;
}


/*********************************************************************
 * @fn      		  - USART_IRQHandling
 *
 * @brief             - USART IRQ handling
 *
 * @param[in]         - USART_Handle
 * @param[in]         - none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void USART_IRQHandling(USART_Handle *pUSARTHandle) {

	uint8_t temp1, temp2;
	uint16_t *pdata;

	/*************************transmit data ********************************************/

	temp1 = pUSARTHandle->pUSARTx->ISR & USART_ISR_TC;

	//Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_TCIE;

	if (temp1 && temp2) {
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			//Check the TxLen . If it is zero then close the data transmission
			if (pUSARTHandle->TxLen == 0) {
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->ICR |= USART_ICR_TCCF;
				pUSARTHandle->pUSARTx->CR1&=~USART_CR1_TCIE;
				pUSARTHandle->pUSARTx->CR1&=~USART_CR1_TXEIE;

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,
				USART_EVENT_TX_CMPLT);
			}
		}
	}

	// check TXE. if TXE flag is zero. transmit data

	temp1 = (pUSARTHandle->pUSARTx->ISR & USART_ISR_TXE); //TXE: Transmit data register empty
	temp2 = (pUSARTHandle->pUSARTx->CR1 & USART_CR1_TXEIE); //TXEIE: interrupt enable

	if (temp1 && temp2) //
			{
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			if (pUSARTHandle->TxLen > 0) {

				if (pUSARTHandle->Config.WordLength == USART_WORDLENGHT_9BIT) {
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t) (0x01ff));
					//check for ParityControl
					if (pUSARTHandle->Config.Parity == USART_PARITY_DISABLE) {
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					} else {
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				} else {
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer
							& (uint8_t) 0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}

			}

		}
	}
	/*************************recieve data ********************************************/

	temp1 = (pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE); //RXNEIE: RXNE interrupt enable
	temp2 = (pUSARTHandle->pUSARTx->ISR & USART_ISR_RXNE); //RXNE: Read data register not empty

	if (temp1 && temp2) {
		if (pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if (pUSARTHandle->RxLen > 0) {
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if (pUSARTHandle->Config.WordLength == USART_WORDLENGHT_9BIT) {
					//We are going to receive 9bit data in a frame

					//Now, check are we using ParityControl control or not
					if (pUSARTHandle->Config.Parity == USART_PARITY_DISABLE) {
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) =
								(pUSARTHandle->pUSARTx->RDR & (uint16_t) 0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					} else {
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR
								& (uint8_t) 0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 1;
					}
				} else {
					//We are going to receive 8bit data in a frame

					//Now, check are we using ParityControl control or not
					if (pUSARTHandle->Config.Parity == USART_PARITY_DISABLE) {
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer =
								(uint8_t) (pUSARTHandle->pUSARTx->RDR
										& (uint8_t) 0xFF);
					}

					else {
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer =
								(uint8_t) (pUSARTHandle->pUSARTx->RDR
										& (uint8_t) 0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}

			}

			if (pUSARTHandle->RxLen <= 0) {
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~USART_CR1_RXNEIE; //RXNEIE: RXNE interrupt disable
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,
				USART_EVENT_RX_CMPLT);
			}

		}
	}

	/*************************Check for Overrun detection flag ********************************************/

	temp1 = (pUSARTHandle->pUSARTx->ISR & USART_ISR_ORE); //ORE: Overrun error

	temp2 = (pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE); //RXNEIE: RXNE interrupt enable

	if (temp1 && temp2) {

		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/*************************Check for Error Flag ********************************************/
	temp2 = pUSARTHandle->pUSARTx->CR3 & USART_CR3_EIE; //EIE: Error interrupt enable

	if (temp2) {
		temp1 = pUSARTHandle->pUSARTx->ISR;
		if (temp1 & (1 << 1))				//FE: Framing error
				{

			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if (temp1 & (1 << 2))				//NF: START bit Noise detection flag

				{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if (temp1 & (1 << 3))				//ORE: Overrun error
				{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}
/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - return flag status
 *
 * @param[in]         - USART_Type
 * @param[in]         - uint32_t
 * @param[in]         - none
 *
 * @return            - flag status
 *
 * @Note              - none
 *
 */
uint8_t USART_GetFlagStatus(USART_Type *pUSARTx, uint32_t FlagName) {
	if (pUSARTx->ISR & FlagName) {
		return HIGH;
	} else {
		return LOW;
	}
}

/*
 * IRQ Configuration and ISR handling
 */

/*
 void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
 if (EnorDi == ENABLE) {
 if (IRQNumber <= 31) {
 //program ISER0 register
 *NVIC_ISER0 |= (1 << IRQNumber);

 } else if (IRQNumber > 31 && IRQNumber < 64) //32 to 63
 {
 //program ISER1 register
 *NVIC_ISER1 |= (1 << (IRQNumber % 32));
 } else if (IRQNumber >= 64 && IRQNumber < 96) {
 //program ISER2 register //64 to 95
 *NVIC_ISER2 |= (1 << (IRQNumber % 64));
 }
 } else {
 if (IRQNumber <= 31) {
 //program ICER0 register
 *NVIC_ICER0 |= (1 << IRQNumber);
 } else if (IRQNumber > 31 && IRQNumber < 64) {
 //program ICER1 register
 *NVIC_ICER1 |= (1 << (IRQNumber % 32));
 } else if (IRQNumber >= 64 && IRQNumber < 96) {
 //program ICER2 register
 *NVIC_ICER2 |= (1 << (IRQNumber % 64));
 }
 }

 }
 void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
 //1. first lets find out the ipr register
 uint8_t iprx = IRQNumber / 4;			// her biri 4 kutu yeri bul
 uint8_t iprx_section = IRQNumber % 4;  // biti bul

 uint8_t shift_amount = (8 * iprx_section) + (8 - 4); //

 *( NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
 }
 */


/*********************************************************************
 * @fn      		  - USART_ApplicationEventCallback
 *
 * @brief             - callback interrupts
 *
 * @param[in]         - USART_Handle
 * @param[in]         - uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
__weak void USART_ApplicationEventCallback(USART_Handle *pUSARTHandle,
		uint8_t event) {

}

/*
 * other
 */

void USART_SetBaudRate(USART_Type *pUSARTx, uint32_t BaudRate) {
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if (pUSARTx == USART1) {
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	} else {
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if (pUSARTx->CR1 & (1 << 15)) {
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else {
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv / 100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if (pUSARTx->CR1 & (1 << 15)) {
		//OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8) + 50) / 100) & ((uint8_t) 0x07);

	} else {
		//over sampling by 16
		F_part = (((F_part * 16) + 50) / 100) & ((uint8_t) 0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

uint16_t AHB_PreScaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t APB1_PreScaler[4] = { 2, 4, 8, 16 };

uint32_t RCC_GetPCLK1Value(void) {
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if (clksrc == 0) {
		SystemClk = 8000000;
	} else if (clksrc == 1) {
		SystemClk = 32000000;
	} else if (clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[temp - 8];
	}

	//apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4) {
		apb1p = 1;
	} else {
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
uint32_t RCC_GetPCLK2Value(void) {
	uint32_t SystemClock = 0, tmp, pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp, apb2p;

	if (clk_src == 0) {
		SystemClock = 8000000;
	} else {
		SystemClock = 32000000;
	}
	tmp = (RCC->CFGR >> 4) & 0xF;

	if (tmp < 0x08) {
		ahbp = 1;
	} else {
		ahbp = AHB_PreScaler[tmp - 8];
	}

	tmp = (RCC->CFGR >> 13) & 0x7;
	if (tmp < 0x04) {
		apb2p = 1;
	} else {
		apb2p = APB1_PreScaler[tmp - 4];
	}

	pclk2 = (SystemClock / ahbp) / apb2p;

	return pclk2;
}

uint32_t RCC_GetPLLOutputClock() {

	return 0;
}

