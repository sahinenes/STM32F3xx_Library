/*
 * stm32f3xx_i2c.c
 *
 *  Created on: 3 Mar 2022
 *      Author: Enes
 */

#include "stm32f3xx_i2c.h"

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             - I2C peripheral clock enable or disable
 *
 * @param[in]         - I2C_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - I2C_PeriClockControl(I2C1,ENABLE);
 *
 */
void I2C_PeriClockControl(I2C_Type *pI2Cx, uint8_t EnorDi) {

	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLCK_EN();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLCK_EN();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLCK_EN();
		}
	} else {
		if (pI2Cx == I2C1) {
			I2C1_PCLCK_DIS();
		} else if (pI2Cx == I2C2) {
			I2C2_PCLCK_DIS();
		} else if (pI2Cx == I2C3) {
			I2C3_PCLCK_DIS();
		}
	}
}

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
 *
 * @brief             - I2C peripheral enable or disable
 *
 * @param[in]         - I2C_Type
 * @param[in]         -	uint8_t
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - I2C_PeripheralControl(I2C1,ENABLE);
 *
 */
void I2C_PeripheralControl(I2C_Type *pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		pI2Cx->CR1 |= (I2C_CR1_PE);
	} else {
		pI2Cx->CR1 &= ~(I2C_CR1_PE);
	}
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             - Init
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_Init(I2C_Handle *pI2CHandle) {

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE); //clock enable

	pI2CHandle->pI2Cx->CR1 &= ~(I2C_CR1_PE); //Peripheral disable

	/*speed*/
	if (pI2CHandle->I2C_Config.Speed == I2C_SPEED_STANDART) {
		pI2CHandle->pI2Cx->TIMINGR = 0x00101D2D & 0xF0FFFFFFU;
	} else if (pI2CHandle->I2C_Config.Speed == I2C_SPEED_FAST) {
		pI2CHandle->pI2Cx->TIMINGR = 0x0000020C & 0xF0FFFFFFU;
	} else if (pI2CHandle->I2C_Config.Speed == I2C_SPEED_FAST_PLUS) {
		pI2CHandle->pI2Cx->TIMINGR = 0x00000002 & 0xF0FFFFFFU;
	}

	/*own adress*/
	pI2CHandle->pI2Cx->OAR1 &= ~(I2C_OAR1_OA1EN); //Own Address 1 disable

	if (pI2CHandle->I2C_Config.AdressMode == I2C_ADRESS_7BIT) {

		pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.OwnAdress << 1);
		pI2CHandle->pI2Cx->OAR1 |= (I2C_OAR1_OA1EN); //Own Address 1 enables
	} else {

		pI2CHandle->pI2Cx->OAR1 |= (I2C_OAR1_OA1MODE); //Own Address 1 10-bit mode
		pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.OwnAdress << 0);
		pI2CHandle->pI2Cx->OAR1 |= (I2C_OAR1_OA1EN); //Own Address 1 enable

	}

	if (pI2CHandle->I2C_Config.AdressMode == I2C_ADRESS_10BIT) {
		pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_ADD10);
	}

	/* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
	pI2CHandle->pI2Cx->CR2 |= (I2C_CR2_AUTOEND) | (I2C_CR2_NACK);

	/* Disable Own Address2 before set the Own Address2 configuration */
	pI2CHandle->pI2Cx->OAR2 &= ~(I2C_OAR2_OA2EN);

	if (pI2CHandle->I2C_Config.AdressMode == I2C_ADRESS_7BIT) {

		pI2CHandle->pI2Cx->OAR2 |= (pI2CHandle->I2C_Config.OwnAdress << 1);
		pI2CHandle->pI2Cx->OAR2 |= (I2C_OAR2_OA2EN); //Own Address 1 enables
	} else {

		//pI2CHandle->pI2Cx->OAR1 |= (1 << 10); //Own Address 1 10-bit mode
		pI2CHandle->pI2Cx->OAR2 |= (pI2CHandle->I2C_Config.OwnAdress << 0);
		pI2CHandle->pI2Cx->OAR2 |= (I2C_OAR2_OA2EN); //Own Address 1 enable

	}

	/* Configure I2Cx: Generalcall and NoStretch mode */
	pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.GeneralCall
			<< I2C_CR1_GCEN_Pos)
			| (pI2CHandle->I2C_Config.NoStretch << I2C_CR1_NOSTRETCH_Pos);

	pI2CHandle->pI2Cx->CR1 |= (I2C_CR1_PE); //Peripheral enable

}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             - DeInit
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	none
 * @param[in]         - none
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_DeInit(I2C_Type *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
		pI2Cx->CR1 &= ~(1 << 0); //Peripheral disable
	} else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
		pI2Cx->CR1 &= ~(1 << 0); //Peripheral disable
	} else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
		pI2Cx->CR1 &= ~(1 << 0); //Peripheral disable
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterTransmitData
 *
 * @brief             - Transmit Data
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_MasterTransmitData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len) {
	uint16_t size;
	size = 0;
	while (I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_BUSY))
		;
	/* Send Slave Address */
	/* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
	if (Len > 255U) {
		size = 255U;
		I2C_TransferConfig(pI2CHandle, pI2CHandle->I2C_Config.SlaveAdress,
				(uint8_t) size,I2C_RELOAD_MODE,
				 I2C_GENERATE_START_WRITE);
	} else {
		size = Len;
		I2C_TransferConfig(pI2CHandle, pI2CHandle->I2C_Config.SlaveAdress,
				(uint8_t) size,  I2C_AUTOEND_MODE,
				 I2C_GENERATE_START_WRITE);
	}

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TXIS))
			; //Transmit interrupt status (transmitters)

		/* Write data to TXDR */
		pI2CHandle->pI2Cx->TXDR = *(pTxBuffer);
		pTxBuffer++;
		Len--;
		size--;

		if (size == 0 && Len != 0) //N>255
				{

			while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TCR))
				;
			if (Len > 255U) {
				size = 255U;
				I2C_TransferConfig(pI2CHandle,
						pI2CHandle->I2C_Config.SlaveAdress, (uint8_t) size,
						 I2C_RELOAD_MODE,
						 I2C_GENERATE_START_WRITE);
			} else {
				size = Len;
				I2C_TransferConfig(pI2CHandle,
						pI2CHandle->I2C_Config.SlaveAdress, (uint8_t) size,
						 I2C_AUTOEND_MODE,
						 I2C_GENERATE_START_WRITE);
			}
		}

	}
	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_STOPF))
		; //wait stop automatic
	pI2CHandle->pI2Cx->ICR &= ~(I2C_ICR_STOPCF); // Stop detection flag clear

	//reset CR2
	pI2CHandle->pI2Cx->CR2 &= ~((0x3FF << I2C_CR2_SADD_Pos) | I2C_CR2_HEAD10R
			| (0xFF << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             - Receive Data
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_MasterReceiveData(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len) {

	uint16_t size;
	while (I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_BUSY))
		;
	/* Send Slave Address */
	/* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
	if (Len > 255U) {
		size = 255U;
		I2C_TransferConfig(pI2CHandle, pI2CHandle->I2C_Config.SlaveAdress,
				(uint8_t) size,  I2C_RELOAD_MODE,
				I2C_GENERATE_START_READ);
	} else {

		size = Len;
		I2C_TransferConfig(pI2CHandle, pI2CHandle->I2C_Config.SlaveAdress,
				(uint8_t) size,I2C_AUTOEND_MODE,
				I2C_GENERATE_START_READ);
	}

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_RXNE))
			; //Transmit interrupt status (transmitters)

		/* Read data from RXDR */
		*pRxBuffer = (uint8_t) pI2CHandle->pI2Cx->RXDR;
		pRxBuffer++;
		Len--;
		size--;

		if (size == 0 && Len != 0) //N>255
				{

			while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TCR))
				;
			if (Len > 255U) {
				size = 255U;
				I2C_TransferConfig(pI2CHandle,
						pI2CHandle->I2C_Config.SlaveAdress, (uint8_t) size,
						 I2C_RELOAD_MODE,  I2C_NO_STARTSTOP);
			} else {

				size = Len;
				I2C_TransferConfig(pI2CHandle,
						pI2CHandle->I2C_Config.SlaveAdress, (uint8_t) size,
					 I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
			}
		}

	}
	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_STOPF))
		; //wait stop automatic
	pI2CHandle->pI2Cx->ICR &= ~(I2C_ICR_STOPCF); // Stop detection flag clear

	//reset CR2
	pI2CHandle->pI2Cx->CR2 &= ~((0x3FF << I2C_CR2_SADD_Pos) | I2C_CR2_HEAD10R
			| (0xFF << I2C_CR2_NBYTES_Pos) | I2C_CR2_RELOAD | I2C_CR2_RD_WRN);
}

/*********************************************************************
 * @fn      		  - I2C_SlaveTransmitData
 *
 * @brief             - Transmit Data
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_SlaveTransmitData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer,
		uint32_t Len) {

	//0: an ACK is sent after current received byte.
	pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_NACK;

	//ADDR: Address matched (slave mode)
	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR))
		;

	pI2CHandle->pI2Cx->ICR |= I2C_ICR_ADDRCF; //Address matched flag clear

	if (pI2CHandle->I2C_Config.AdressMode == I2C_ADRESS_10BIT) {
		while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR))
			;
		pI2CHandle->pI2Cx->ICR |= I2C_ICR_ADDRCF; //Address matched flag clear
	}

	//: Read transfer, slave enters transmitter mode.
	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_DIR))
		; //Transfer direction (Slave mode)

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TXIS))
			;
		pI2CHandle->pI2Cx->TXDR = *(pTxBuffer);
		pTxBuffer++;
		Len--;
	}

	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_STOPF))
		; //wait stop automatic
	pI2CHandle->pI2Cx->ICR |= I2C_ICR_STOPCF; // Stop detection flag clear

	while (I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_BUSY))
		;

}

/*********************************************************************
 * @fn      		  - I2C_SlaveReceiveData
 *
 * @brief             - Transmit Data
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint8_t -> data
 * @param[in]         - uint32_t -> data len
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_SlaveReceiveData(I2C_Handle *pI2CHandle, uint8_t *pRxBuffer,
		uint32_t Len) {

	//0: an ACK is sent after current received byte.
	pI2CHandle->pI2Cx->CR2 &= ~I2C_CR2_NACK;

	//ADDR: Address matched (slave mode)
	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR))
		;

	pI2CHandle->pI2Cx->ICR |= I2C_ICR_ADDRCF; //Address matched flag clear

	//: Read transfer, slave enters transmitter mode.
	while (I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_DIR))
		; //Transfer direction (Slave mode)

	while (Len > 0) {
		while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_RXNE))
			;
		*(pRxBuffer) = (uint8_t) pI2CHandle->pI2Cx->RXDR;
		pRxBuffer++;
		Len--;
	}

	while (!I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_STOPF))
		; //wait stop automatic
	pI2CHandle->pI2Cx->ICR |= I2C_ICR_STOPCF; // Stop detection flag clear

	while (I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_BUSY))
		;

}


/*********************************************************************
 * @fn      		  - I2C_TransferConfig
 *
 * @brief             - Config Transfer
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint16_t
 * @param[in]         - uint8_t
 * @param[in]         - uint8_t
 * @param[in]         - uint32_t
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
void I2C_TransferConfig(I2C_Handle *pI2CHandle, uint16_t DevAddress,
		uint8_t Size, uint8_t Mode, uint32_t Request) {

	/* update CR2 register */
	/*
	 MODIFY_REG(pI2CHandle->pI2Cx->CR2,
	 (((0x3FFUL << 0) | (0xFFUL << 16)| (0x1UL << 24) | (0x1UL << 25) | \
	               ((0x1UL << 10) & (uint32_t)(Request >> (31U - 10U))) | (0x1UL << 13) | (0x1UL << 14))), \
				   (uint32_t)  (((uint32_t)DevAddress & (0x3FFUL<<0))  |
	 (((uint32_t)Size <<16) & 0xFFUL) |  (uint32_t)Mode |  (uint32_t)Request));
	 */

	//reset register
	pI2CHandle->pI2Cx->CR2 = (uint32_t) 0x0000;

	//write register
	pI2CHandle->pI2Cx->CR2 |= (DevAddress << 1);
	pI2CHandle->pI2Cx->CR2 |= (Size << I2C_CR2_NBYTES_Pos);
	pI2CHandle->pI2Cx->CR2 |= Mode;
	pI2CHandle->pI2Cx->CR2 |= Request;
	pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_AUTOEND_Pos);

}

/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             - Get flag status
 *
 * @param[in]         - I2C_Handle
 * @param[in]         -	uint32_t
 *
 * @return            - none
 *
 * @Note              - none
 *
 */
uint8_t I2C_GetFlagStatus(I2C_Handle *pI2CHandle, uint32_t Flag) {
	if (pI2CHandle->pI2Cx->ISR & (Flag)) {
		return HIGH;
	} else {
		return LOW;
	}
}


