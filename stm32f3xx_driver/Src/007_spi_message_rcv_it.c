
/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */
#include<stdio.h>
#include<string.h>
#include "stm32f3xx_spi.h"
#include "stm32f3xx_gpio.h"
#include "stm32f3xx_cortex.h"




SPI_Handle SPI2handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;


volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.Config.PinMode = GPIO_MODE_ALTERNATE;
	SPIPins.Config.PinAltFunc= 5;
	SPIPins.Config.PinOPType = GPIO_OTYPER_PP;
	SPIPins.Config.PuPdControl=GPIO_PUPD_NO;
	SPIPins.Config.PinSpeed=GPIO_SPEED_HIGH;

	//SCLK
	SPIPins.Config.PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MOSI
    SPIPins.Config.PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.Config.PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);


	//NSS
	SPIPins.Config.PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Inits(void)
{
	SPI2handle.pSPIx = SPI2;
	SPI2handle.Config.BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.Config.Mode = SPI_MODE_MSTR;
	SPI2handle.Config.baudRate = SPI_BAUD_RATE_64;
	SPI2handle.Config.DataSize = SPI_DATA_SIZE_8BIT;
	SPI2handle.Config.CPOL = SPI_CPOL_0;
	SPI2handle.Config.CHPHA = SPI_CPHA_0;
	SPI2handle.Config.SSM = SPI_SSM_DISABLE; //Hardware slave management enabled for NSS pin

	SPI_Init(&SPI2handle);
}


/*This function configures the gpio pin over which SPI peripheral issues data available interrupt */
void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle spiIntPin;
	memset(&spiIntPin,0,sizeof(spiIntPin));



	//this is led gpio configuration
	spiIntPin.pGPIOx = GPIOB;
	spiIntPin.Config.PinNumber = GPIO_PIN_1;
	spiIntPin.Config.PinMode = GPIO_MODE_IT_FT;
	spiIntPin.Config.PinSpeed = GPIO_SPEED_LOW;
	spiIntPin.Config.PuPdControl = GPIO_PUPD_PU;

	GPIO_Init(&spiIntPin);

	NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	NVIC_EnableIRQ(EXTI1_IRQn);

}


int main(void)
{



	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);
	NVIC_SetPriority(SPI2_IRQn, 0, 0);
	NVIC_EnableIRQ(SPI2_IRQn);

	while(1){

		rcvStop = 0;

		while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)
		printf("button press\n");

		NVIC_DisableIRQ(EXTI1_IRQn);

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);


		while(!rcvStop)
		{
			/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
			while(SPI_TransmitDataIT(&SPI2handle,&dummy,1)==SPI_BUSY_IN_TX);
			while(SPI_ReceiveDataIT(&SPI2handle,&ReadByte,1)==SPI_BUSY_IN_RX);
		}


		// confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		NVIC_EnableIRQ(EXTI1_IRQn);

	}

	return 0;

}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI2_IRQHandler(void)
{
	printf("spi interrupt\n");

	SPI_IRQHandling(&SPI2handle);

}



void SPI_ApplicationEventCallback(SPI_Handle *pSPIHandle,uint8_t AppEv)
{
	printf("callback\n");
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{			printf("DATA:%c\n",ReadByte);
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}

	}


}

/* Slave data available interrupt handler */
void EXTI1_IRQHandler(void)
{
	printf("press\n");
	GPIO_IRQHandling(GPIO_PIN_1);
	dataAvailable = 1;
}

//monitor arm semihosting enable
//-specs=rdimon.specs -lc -lrdimon
