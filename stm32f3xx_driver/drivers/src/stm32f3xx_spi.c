/*
 * stm32f3xx_spi.c
 *
 *  Created on: 2 Mar 2022
 *      Author: Enes
 */

#include "stm32f3xx_spi.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - I2C peripheral clock enable or disable
 *
 * @param[in]         - I2C_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - SPI_PeriClockControl(SPI1,ENABLE);
 *
 */
void SPI_PeriClockControl(SPI_Type *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLCK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLCK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLCK_EN();

		} else if (pSPIx == SPI4) {
			SPI4_PCLCK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLCK_DIS();
		} else if (pSPIx == SPI2) {
			SPI2_PCLCK_DIS();
		} else if (pSPIx == SPI3) {
			SPI3_PCLCK_EN();

		} else if (pSPIx == SPI4) {
			SPI4_PCLCK_DIS();
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - SPI init
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_Init(SPI_Handle *SPIHandle) {

	uint32_t temp = 0;

	SPI_PeriClockControl(SPIHandle->pSPIx, ENABLE); //enable spix clock
	SPI_PeripheralControl(SPIHandle->pSPIx, DISABLE); //spix disable

	/* CR1 Settings */

	if (SPIHandle->Config.Mode == SPI_MODE_MSTR) {

		temp |= SPI_CR1_MSTR;
	}
	temp |= (SPIHandle->Config.baudRate << SPI_CR1_BR_Pos);
	temp |= (SPIHandle->Config.CHPHA << SPI_CR1_CPHA_Pos);
	temp |= (SPIHandle->Config.CPOL << SPI_CR1_CPOL_Pos);
	temp |= (SPIHandle->Config.Mode << SPI_CR1_MSTR_Pos);
	temp |= (SPIHandle->Config.SSM << SPI_CR1_SSM_Pos);
	SPIHandle->pSPIx->CR1 |= temp;

	if (SPIHandle->Config.BusConfig == SPI_BUS_CONFIG_FD) {
		SPIHandle->pSPIx->CR1 &= ~SPI_CR1_RXONLY; //RXONLY -> 0 (FULL DUBLEX MODE)
		SPIHandle->pSPIx->CR1 &= ~SPI_CR1_BIDIMODE; //2-line unidirectional data mode selected
	} else if (SPIHandle->Config.BusConfig == SPI_BUS_CONFIG_HD) {
		SPIHandle->pSPIx->CR1 |= SPI_CR1_BIDIMODE; //1-line bidirectional data mode selected
	} else if (SPIHandle->Config.BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		SPIHandle->pSPIx->CR1 &= ~SPI_CR1_BIDIMODE; //clear BIDIMODE
		SPIHandle->pSPIx->CR1 |= SPI_CR1_RXONLY; //Output disabled (Receive-only mode)
	}

	temp = 0;

	/* CR2 Settings */

	/* Align by default the rs fifo threshold on the data size */
	if (SPIHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {
		temp &= ~SPI_CR2_FRXTH;
	} else {
		temp |= SPI_CR2_FRXTH;
	}
	temp |= (SPIHandle->Config.DataSize << SPI_CR2_DS_Pos);
	temp |= SPI_CR2_NSSP; //NSS pulse management

	SPIHandle->pSPIx->CR2 |= temp;

	temp = 0;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - SPI deinit
 *
 * @param[in]         - SPI_Type
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_DeInit(SPI_Type *pSPIx) {

	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
		SPI3_REG_RESET();

	} else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}
/*********************************************************************
 * @fn      		  - SPI_TransmitData
 *
 * @brief             - SPI transmit data
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_TransmitData(SPI_Handle *pSPIxHandle, uint8_t *pTxBuffer, uint32_t Len) {

	if (pSPIxHandle->Config.BusConfig == SPI_BUS_CONFIG_HD) {
		SPI_PeripheralControl(pSPIxHandle->pSPIx, DISABLE);
		pSPIxHandle->pSPIx->CR1 |= SPI_CR1_BIDIOE; //BIDIOE: Output enable in bidirectional mode
		//1: Output enabled (transmit-only mode)
		SPI_PeripheralControl(pSPIxHandle->pSPIx, ENABLE);
	}

	/* Transmit data in 16 Bit mode */
	if (pSPIxHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {
		if (Len == 0x01U) {
			pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
			pTxBuffer += sizeof(uint16_t);
			Len--;
		}
		/* Transmit data in 16 Bit mode */
		while (Len > 0U) {
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_TXE_FLAG)) {
				pSPIxHandle->pSPIx->DR = *((uint16_t*) pTxBuffer);
				pTxBuffer += sizeof(uint16_t);
				Len--;

			}
		}
	}
	/* Transmit data in 8 Bit mode */
	else {
		if (Len == 0x01U) {
			if (Len > 1U) {
				pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
				pTxBuffer += sizeof(uint16_t);
				Len -= 2U;
			} else {
				*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR) = (*pTxBuffer);
				pTxBuffer++;
				Len--;
			}

		}
		/* Transmit data in 8 Bit mode */
		while (Len > 0U) {

			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_TXE_FLAG)) {

				if (Len > 1U) {
					pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
					pTxBuffer += sizeof(uint16_t);
					Len -= 2U;
				} else {
					*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR) =
							(*pTxBuffer);
					pTxBuffer++;
					Len--;
				}

			}
		}
	}

	/* Clear overrun flag in 2 Lines communication mode because received is not read */

	if (pSPIxHandle->Config.BusConfig == SPI_BUS_CONFIG_FD) {
		pSPIxHandle->pSPIx->SR &= ~SPI_SR_OVR;
		pSPIxHandle->pSPIx->SR &= ~SPI_SR_RXNE;
	}

}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - SPI receive data
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_ReceiveData(SPI_Handle *pSPIxHandle, uint8_t *pRxBuffer, uint32_t Len) {

	/* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
	/*
	 if ((pSPIxHandle->Config.Mode == SPI_MODE_MSTR)
	 && (pSPIxHandle->Config.BusConfig == SPI_BUS_CONFIG_FD)) {

	 SPI_TransmitReceive(pSPIxHandle, pRxBuffer, pRxBuffer, Len);
	 }
	 */
	if (pSPIxHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {

		/*RXNE event is generated if the FIFO level is greater than or equal to 1/2 (16-bit) */
		pSPIxHandle->pSPIx->CR2 &= ~SPI_CR2_FRXTH;

	} else {
		/*RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)*/
		pSPIxHandle->pSPIx->CR2 |= SPI_CR2_FRXTH;
	}

	if (pSPIxHandle->Config.BusConfig == SPI_BUS_CONFIG_HD) {
		SPI_PeripheralControl(pSPIxHandle->pSPIx, DISABLE);
		pSPIxHandle->pSPIx->CR1 &= ~SPI_CR1_BIDIOE;
		SPI_PeripheralControl(pSPIxHandle->pSPIx, ENABLE);
	}

	/* Receive data in 8 Bit mode */
	if (pSPIxHandle->Config.DataSize <= SPI_DATA_SIZE_8BIT) {
		while (Len > 0U) {

			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_RXNE_FLAG)) {
				(*(uint8_t*) pRxBuffer) =
						*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR);
				pRxBuffer += sizeof(uint8_t);
				Len--;

			}
		}
	} else {
		while (Len > 0U) {
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_RXNE_FLAG)) {
				*((uint16_t*) pRxBuffer) = (uint16_t) pSPIxHandle->pSPIx->DR;
				pRxBuffer += sizeof(uint16_t);
				Len--;
			}

		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_TransmitReceive
 *
 * @brief             - SPI transmit and receive data
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         -	uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_TransmitReceive(SPI_Handle *pSPIxHandle, uint8_t *pTxBuffer,
		uint8_t *pRxBuffer, uint32_t Len) {

	uint32_t txallowed = 1U;

	if (pSPIxHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {

		/*RXNE event is generated if the FIFO level is greater than or equal to 1/2 (16-bit) */
		pSPIxHandle->pSPIx->CR2 &= ~SPI_CR2_FRXTH;

	} else {
		/*RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)*/
		pSPIxHandle->pSPIx->CR2 |= SPI_CR2_FRXTH;
	}

	/* Transmit and Receive data in 16 Bit mode */
	if (pSPIxHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {
		if (Len == 0x01U) {
			pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
			pTxBuffer += sizeof(uint16_t);
			Len--;
		}
		/* Transmit data in 16 Bit mode */
		while (Len > 0U) {

			/* Check TXE flag */
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_TXE_FLAG)
					&& (txallowed == 1U)) {
				pSPIxHandle->pSPIx->DR = *((uint16_t*) pTxBuffer);
				pTxBuffer += sizeof(uint16_t);
				Len--;
				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;

			}
			/* Check RXNE flag */
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_RXNE_FLAG)) {
				*((uint16_t*) pRxBuffer) = (uint16_t) pSPIxHandle->pSPIx->DR;
				pRxBuffer += sizeof(uint16_t);
				Len--;
				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;
			}

		}
	}
	/* Transmit and Receive data in 8 Bit mode */
	else {
		if (Len == 0x01U) {
			if (Len > 1U) {
				pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
				pTxBuffer += sizeof(uint16_t);
				Len -= 2U;
			} else {
				*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR) = (*pTxBuffer);
				pTxBuffer++;
				Len--;
			}

		}

		while (Len > 0U) {
			/* Check TXE flag */
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_TXE_FLAG) && Len > 0U
					&& (txallowed == 1U)) {

				if (Len > 1U) {
					pSPIxHandle->pSPIx->DR = (*((uint16_t*) pTxBuffer));
					pTxBuffer += sizeof(uint16_t);
					Len -= 2U;
				} else {
					*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR) =
							(*pTxBuffer);
					pTxBuffer++;
					Len--;
				}
				/* Next Data is a reception (Rx). Tx not allowed */
				txallowed = 0U;
			}

			/* Wait until RXNE flag is reset */
			if (SPI_GetFlagStatus(pSPIxHandle->pSPIx, SPI_RXNE_FLAG)
					&& Len > 0U) {

				if (Len > 1U) {
					(*(uint16_t*) pRxBuffer) =
							(uint16_t) pSPIxHandle->pSPIx->DR;
					pRxBuffer += sizeof(uint16_t);
					Len -= 2U;

					if (Len <= 1) {
						/* Set RX Fifo threshold before to switch on 8 bit data size */
						pSPIxHandle->pSPIx->CR2 |= SPI_CR2_FRXTH;
					}
				} else {
					(*(uint8_t*) pRxBuffer) =
							*((volatile uint8_t*) &pSPIxHandle->pSPIx->DR);
					pRxBuffer += sizeof(uint8_t);
					Len--;
				}

				/* Next Data is a Transmission (Tx). Tx is allowed */
				txallowed = 1U;

			}

		}

	}

}
/*********************************************************************
 * @fn      		  - SPI_TransmitDataIT
 *
 * @brief             - SPI transmit and with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - uint8_t -> state
 *
 * @Note              -
 *
 */
uint8_t SPI_TransmitDataIT(SPI_Handle *pSPIxHandle, uint8_t *pTxBuffer,
		uint32_t Len) {

	uint8_t state = pSPIxHandle->TxState;

	pSPIxHandle->RxLen = 0;
	pSPIxHandle->pRxBuffer = (uint8_t*) NULL;
	pSPIxHandle->RxState = SPI_READY;

	pSPIxHandle->TxLen = Len;
	pSPIxHandle->pTxBuffer = pTxBuffer;
	pSPIxHandle->TxState = SPI_BUSY_IN_TX;

	pSPIxHandle->pSPIx->CR2 |= SPI_CR2_TXEIE; //TXEIE: Tx buffer empty interrupt enable
	//pSPIxHandle->pSPIx->CR2 |= (1 << 5); //ERRIE: Error interrupt enable

	return state;

}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - SPI receive and with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - uint8_t -> state
 *
 * @Note              -
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle *pSPIxHandle, uint8_t *pRxBuffer,
		uint32_t Len) {

	uint8_t state = pSPIxHandle->RxState;

	pSPIxHandle->TxLen = 0;
	pSPIxHandle->pTxBuffer = (uint8_t*) NULL;
	pSPIxHandle->TxState = SPI_READY;

	pSPIxHandle->RxLen = Len;
	pSPIxHandle->pRxBuffer = pRxBuffer;
	pSPIxHandle->RxState = SPI_BUSY_IN_RX;

	pSPIxHandle->pSPIx->CR2 |= SPI_CR2_RXNEIE; //RXNEIE: RX buffer not empty interrupt enable
	//pSPIxHandle->pSPIx->CR2 |= (1 << 5); //ERRIE: Error interrupt enable

	/* Align by default the rs fifo threshold on the data size */
	/*
	 if (pSPIxHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {
	 pSPIxHandle->pSPIx->CR2 &= ~(1 << 12);
	 } else {
	 pSPIxHandle->pSPIx->CR2 |= (1 << 12);
	 }
	 */
	return state;
}

/*
 * IRQ Configuration and ISR handling
 */
/*
 void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
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
 } else if (EnorDi == DISABLE) {
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

 void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
 //1. first lets find out the ipr register
 uint8_t iprx = IRQNumber / 4;			// her biri 4 kutu yeri bul
 uint8_t iprx_section = IRQNumber % 4;  // biti bul

 uint8_t shift_amount = (8 * iprx_section) + (8 - 4); //

 *( NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
 }
 */

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - SPI interrupt handling
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_IRQHandling(SPI_Handle *pHandle) {

	uint32_t itsource = pHandle->pSPIx->CR2;
	uint32_t itflag = pHandle->pSPIx->SR;

	/* SPI in mode Receiver ----------------------------------------------------*/
	if ((SPI_CHECK_FLAG(itflag, SPI_SR_OVR) == RESET)
			&& (SPI_CHECK_FLAG(itflag, SPI_SR_RXNE) != RESET)
			&& (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_RXNE) != RESET)) {

		/* Receive data in 8 Bit mode */
		if (pHandle->Config.DataSize <= SPI_DATA_SIZE_8BIT) {

			SPI_RxISR_8BIT(pHandle);
			/* Receive data in 16 Bit mode */
		} else {

			SPI_RxISR_16BIT(pHandle);

		}

		return;
	}

	/* SPI in mode Transmitter -------------------------------------------------*/
	if ((SPI_CHECK_FLAG(itflag, SPI_SR_TXE) != RESET)
			&& (SPI_CHECK_IT_SOURCE(itsource, SPI_IT_TXE) != RESET)) {

		/* Transmit data in 16 Bit mode */
		if (pHandle->Config.DataSize > SPI_DATA_SIZE_8BIT) {

			SPI_TxISR_16BIT(pHandle);

		}
		/* Transmit data in 8 Bit mode */
		else {

			SPI_TxISR_8BIT(pHandle);

		}
		return;

	}
	return;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - SPI peripheral control
 *
 * @param[in]         - SPI_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_PeripheralControl(SPI_Type *pSPIx, uint8_t EnOrDi) {
	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= SPI_CR1_SPE; //spi enable
	} else {
		pSPIx->CR2 &= ~SPI_CR1_SPE; //spi disable
	}

}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - SPI software slave management
 *
 * @param[in]         - SPI_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_SSIConfig(SPI_Type *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR1 |= SPI_CR1_SSI; //Software slave management enable
	} else {
		pSPIx->CR1 &= ~SPI_CR1_SSI; //Software slave management disable
	}

}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - SPI ss  control
 *
 * @param[in]         - SPI_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_SSOEConfig(SPI_Type *pSPIx, uint8_t EnOrDi) {

	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= SPI_CR2_SSOE; //SS output enable

	} else {
		pSPIx->CR2 &= ~SPI_CR2_SSOE; //SS output disable
	}

}
/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - SPI get flag status
 *
 * @param[in]         - SPI_Type
 * @param[in]         -	uint32_t
 * @param[in]         - none
 *
 * @return            - uint8_t -> flag status
 *
 * @Note              -
 *
 */
uint8_t SPI_GetFlagStatus(SPI_Type *pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return HIGH;
	}
	return LOW;
}

/*********************************************************************
 * @fn      		  - SPI_TxISR_16BIT
 *
 * @brief             - SPI transmit 16 bit data with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_TxISR_16BIT(SPI_Handle *pSPIHandle) {

	if (pSPIHandle->TxLen <= 0) {
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_TXEIE; //TXEIE: Tx buffer empty interrupt disable
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_ERRIE; //ERRIE: Error interrupt disable

		pSPIHandle->TxLen = 0;
		pSPIHandle->pTxBuffer = (uint8_t*) NULL;
		pSPIHandle->TxState = SPI_READY;

		while (SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_BUSY_FLAG))
			; //check busy flag
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		return;
	}
	/* Transmit data in 16 Bit mode */
	pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
	pSPIHandle->pTxBuffer += sizeof(uint16_t);
	pSPIHandle->TxLen--;

}

/*********************************************************************
 * @fn      		  - SPI_TxISR_8BIT
 *
 * @brief             - SPI transmit 8 bit data with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_TxISR_8BIT(SPI_Handle *pSPIHandle) {

	if (pSPIHandle->TxLen <= 0) {
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_TXEIE; //TXEIE: Tx buffer empty interrupt disable
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_ERRIE; //ERRIE: Error interrupt disable

		pSPIHandle->TxLen = 0;
		pSPIHandle->pTxBuffer = (uint8_t*) NULL;
		pSPIHandle->TxState = SPI_READY;

		while (SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_BUSY_FLAG))
			; //check busy flag
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		return;
	}
	*((volatile uint8_t*) &pSPIHandle->pSPIx->DR) = (*pSPIHandle->pTxBuffer);
	pSPIHandle->pTxBuffer++;
	pSPIHandle->TxLen--;

}

/*********************************************************************
 * @fn      		  - SPI_RxISR_16BIT
 *
 * @brief             - SPI receive 16 bit data with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_RxISR_16BIT(SPI_Handle *pSPIHandle) {

	if (pSPIHandle->RxLen <= 0) {
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_ERRIE; //ERRIE: Error interrupt disable
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_RXNEIE; //RXNEIE: RX buffer not empty interrupt disable

		pSPIHandle->RxLen = 0;
		pSPIHandle->pRxBuffer = (uint8_t*) NULL;
		pSPIHandle->RxState = SPI_READY;

		while (SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_BUSY_FLAG))
			; //check busy flag
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		return;
	}

	*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t) (pSPIHandle->pSPIx->DR);
	pSPIHandle->pRxBuffer += sizeof(uint16_t);
	pSPIHandle->RxLen--;

}

/*********************************************************************
 * @fn      		  - SPI_RxISR_8BIT
 *
 * @brief             - SPI receive 8 bit data with interrupts
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
void SPI_RxISR_8BIT(SPI_Handle *pSPIHandle) {

	if (pSPIHandle->RxLen <= 0) {
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_ERRIE; //ERRIE: Error interrupt disable
		pSPIHandle->pSPIx->CR2 &= ~SPI_CR2_RXNEIE; //RXNEIE: RX buffer not empty interrupt disable

		pSPIHandle->RxLen = 0;
		pSPIHandle->pRxBuffer = (uint8_t*) NULL;
		pSPIHandle->RxState = SPI_READY;

		while (SPI_GetFlagStatus(pSPIHandle->pSPIx, SPI_BUSY_FLAG))
			; //check busy flag
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		return;
	}
	*pSPIHandle->pRxBuffer = (*(volatile uint8_t*) &pSPIHandle->pSPIx->DR);
	pSPIHandle->pRxBuffer++;
	pSPIHandle->RxLen--;

}

/*********************************************************************
 * @fn      		  - SPI_ApplicationEventCallback
 *
 * @brief             - SPI callback function
 *
 * @param[in]         - SPI_Handle
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              -
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle *pSPIHandle, uint8_t AppEv) {

	//This is a weak implementation . the user application may override this function.
}

