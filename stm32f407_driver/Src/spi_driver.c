/*
 * spi_driver.c
 *
 *  Created on: 13-Jan-2026
 *      Author: debian
 */
#include "stm32f407_spi_driver.h"

//************************************************ A P I  FOR USER  * ******************************************


/*
 *    PERIPHERAL CLK SETUP
 */
void SPI_periClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3 ){
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}

	}
	else{
		// disabling the spi
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3 ){
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
}
}

/*
 *  INIT and DE-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	// peripheral clock enable
	SPI_periClockControl(pSPIHandle->pSPIx, ENABLE);



uint32_t tempreg = 0;  //configuring multiple bit field with out clearing other bits

// 1. config the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode <<2;

//2, config the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode should be cleared
		tempreg &= ~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be set
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig ==  SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI Mode should be cleared
		tempreg &= ~(1<<15);
		// RXONLY BIT MUST BE SET
		tempreg |= (1<<10);
	}

	// 3. config the spi serial clock speed
	tempreg |= pSPIHandle ->SPIConfig.SPI_SclkSpeed <<3;
	//4. config the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF <<11;
	//5. config the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL <<1;
	//6. config the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA <<0;

	pSPIHandle->pSPIx->CR1 |= tempreg;
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){ // DO IT YOURSELF

	if (pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	// not done yet for the SPI4_reg_reset

}

/*
 *  Data send and receive
 */

//  STATUS REGISTER ARE DEFINED IN SPI STATUS REGISTER (SPI _SR)
uint8_t SPI_GETFLAGSTATUS(SPI_RegDef_t *pSPIx,uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return SET;
	}
	return RESET;
}

/*
 *  RE EVALUATE THE CODE  --- POINTER USE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){
	// DESCRIPTION :  THIS IS A BLOCKING CALL
	while(Len >0){
		//1. wait until TXE is set
		// POLING BASED CODE
		while (SPI_GETFLAGSTATUS(pSPIx,SPI_TXE_FLAG ) == RESET); //define in MACRO: SPI_TXE_FLAG == (1<<SPI_SR_TXE)
		// IN ORDER TO DEAL WITH IT WE NEED THE WATCH DOG TIMER, to get out from the completly exit form the poling
		//2. check the DFF bit
		if(pSPIx->CR1 &(1<<SPI_CR1_DFF) ){
			//16 bit format
			//1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			// incrementing the pointer
			(uint16_t*)pTxBuffer++;
		}
		else{
			// 8 bit format
			pSPIx->DR = *pTxBuffer;
			Len--;
			*pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);
/*
 *  IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
/*
 * other peripheral control APIs
 */




