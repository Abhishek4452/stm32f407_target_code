/*                           M C U     S P E C I F I C    H E A D E R   F I L E
 * stm32f407xx.h
 * NOTE - please include the path of this file from the setting -> gcc compiler -> add path -> ok and apply changes
 * this
 *  Created on: 04-Jan-2026
 *      Author: debian
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_


#include <stdint.h>
 // ******************************** PROCESSOR SPECIFIC DETAIL FROM THE CORTEX M4 **********************************************


// ------------------------------ NVIC ISERx register address ---------------------------------
#define NVIC_ISER0              ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1              ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2              ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3              ((volatile uint32_t*)0xE000E10C)

// ------------------------------ NVIC ICERx register address ---------------------------------
#define NVIC_ICER0              ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1              ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2              ((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3              ((volatile uint32_t*)0xE000E18C)

// ------------------------------  PRIORITY BASE CODE ------------------------------------------
#define NVIC_PR_BASE_ADDR        ((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED          4 // specific to the MCU

// HAL _, DRV_ these are the perfix which is used to easly identifing which layer code is it .
// while coding it is also possible that middleware also using the same micro name so to deal with such problem we use suffix

#define FLASH_BASEADDR        0x08000000U  // by default compiler will treat this as signed integer, but address can't be signed therfore either we type cast this or simply write U
#define SRAM_BASEADDR         0x20000000U
#define SRAM                  SRAM_BASEADDR
#define SRAM2_BASEADDR        0x2001C000U
#define ROM_BASEADDR          0x1FFF0000U  // ROM ALSO CALLED SYSTEM MEMORY

// ********************** BUS DOMAIN OF THE STM32f407 *************************************

#define PERI_ADDR             0x40000000U
#define APB1_PERI_ADDR        PERI_ADDR
#define APB2_PERI_ADDR        0x40010000U
#define AHB1_PERI_ADDR        0x40020000U
#define AHB2_PERI_ADDR        0x50000000U


// *********************** AHB1 BUS PERIPHERAL ******************************************* //


#define GPIOA_BASEADDR        AHB1_PERI_ADDR  // 0000 is our off set
#define GPIOB_BASEADDR        (AHB1_PERI_ADDR + 0x0400)
#define GPIOC_BASEADDR        (AHB1_PERI_ADDR + 0x0800)
#define GPIOD_BASEADDR        (AHB1_PERI_ADDR + 0x0C00)
#define GPIOE_BASEADDR        (AHB1_PERI_ADDR + 0x1000)
#define GPIOF_BASEADDR        (AHB1_PERI_ADDR + 0x1400)
#define GPIOG_BASEADDR        (AHB1_PERI_ADDR + 0x1800)
#define GPIOH_BASEADDR        (AHB1_PERI_ADDR + 0x1C00)
#define GPIOI_BASEADDR        (AHB1_PERI_ADDR + 0x2000)
// ------------------------------------- RCC ----------------------------------------------------
#define RCC_BASEADDR          (AHB1_PERI_ADDR + 0x3800)

#define SYSCFG  			  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* communication peripheral hanging on APB1 BUS
 *
 */

#define I2C1_BASEADDR            (PERI_ADDR+0x5400)
#define I2C2_BASEADDR            (PERI_ADDR+0x5800)
#define I2C3_BASEADDR            (PERI_ADDR+0x5C00)

#define SPI2_BASEADDR            (PERI_ADDR+0x3800)
#define SPI3_BASEADDR            (PERI_ADDR+0x3C00)


#define USART2_BASEADDR          (PERI_ADDR+0x4400)
#define USART3_BASEADDR          (PERI_ADDR+0x4800)
#define UART4_BASEADDR           (PERI_ADDR+0x4C00)
#define UART5_BASEADDR           (PERI_ADDR+0x5000)


/*
 * BUS HANGING ON APB2 BUS
 */

#define USART1_BASEADDR           (APB2_PERI_ADDR+0x1000)
#define USART6_BASEADDR			  (APB2_PERI_ADDR+0x1400)

#define SPI1_BASEADDR			  (APB2_PERI_ADDR+0x3000)
#define SPI4_BASEADDR             (APB2_PERI_ADDR+0x3400)
#define SYSCFG_BASEADDR			  (APB2_PERI_ADDR+0x3800)

#define EXTI_BASEADDR			  (APB2_PERI_ADDR+0x3C00)


/// ********************************************** PERIPHERAL REGISTER DECLEARATION ************************************************************
typedef struct
{
	volatile uint32_t MODER;    //                    GPIO port mode register                               offset -- 0x00
	volatile uint32_t OTYPER;   //                    GPIO port output type register                        offset -- T0x04
	volatile uint32_t OSPEEDR;  //                    GPIO port output speed register                       offset -- D0x08
	volatile uint32_t PUPDR;    //                    GPIO port pull-up/pull-down register                  offset -- R0x0C
	volatile uint32_t IDR;      //                    GPIO port input data register                         offset -- 0x10
	volatile uint32_t ODR;      //                    GPIO port output data register                        offset -- 0x14
	volatile uint32_t BSRR;     //                    GPIO port bit set/reset register                      offset -- 0x18
	volatile uint32_t LCKR;     //                    GPIO port configuration lock register                 offset -- 0x1C
	volatile uint32_t AFR[2];     //                    GPIO alternate function low and high register    offset -- 0x20 ,offset -- 0x24 for high

}GPIO_RegDef_t;

//  --------------------------------------------- REGISTER DECLERATION FOR PERIPHERAL --------------------------------------------------------------------

typedef struct
{
	volatile int32_t CR;  		 //				RCC clock control register     								    offset = 0x00
	volatile int32_t PLLCFGR;    //				RCC PLL configuration register 									offset = 0x04
	volatile int32_t CFGR; 	     //				RCC clock configuration register     							offset =  0x08
	volatile int32_t CIR; 	     //				RCC clock interrupt register    							     offset = 0x0C
	volatile int32_t AHB1RSTR;   //				RCC AHB1 peripheral reset register								offset = 0x10
	volatile int32_t AHB2RSTR;   //			 	RCC AHB2 peripheral reset register  							offset = 0x14
	volatile int32_t AHB3RSTR;   //				RCC AHB3 peripheral reset register 	offset = 0x18
	uint32_t RESERVED0;
	volatile int32_t APB1RSTR;   //				RCC APB1 peripheral reset register   							offset = 0x20
	volatile int32_t APB2RSTR;   //				RCC APB2 peripheral reset register 							    offset = 0x24
	uint32_t RESERVED1[2];
	volatile int32_t AHB1ENR;    //				RCC AHB1 peripheral clock enable register                       offset=  0x30
	volatile int32_t AHB2ENR;    //				RCC AHB2 peripheral clock enable register   				   	offset = 0x34
	volatile int32_t AHB3ENR;    //				RCC AHB3 peripheral clock enable register    					offset = 0x38
	uint32_t RESERVED2;
	volatile int32_t APB1ENR;    //				RCC APB1 peripheral clock enable register						offset = 0x40
	volatile int32_t APB2ENR;    //				RCC APB2 peripheral clock enable register 						offset = 0x44
	uint32_t RESERVED3[2];
	volatile int32_t AHB1LPENR;  //				RCC AHB1 peripheral clock enable in low power mode register 	offset = 0x50
	volatile int32_t AHB2LPENR;  //				RCC AHB2 peripheral clock enable in low power mode register     offset = 0x54
	volatile int32_t AHB3LPENR;  //				RCC AHB3 peripheral clock enable in low power mode register		offset = 0x58
	uint32_t RESERVED4;
	volatile int32_t APB1LPENR;  //				RCC APB1 peripheral clock enable in low power mode register	    offset = 0x60
	volatile int32_t APB2LPENR;  //				RCC APB2 peripheral clock enabled in low power mode  			offset = 0x64
	uint32_t RESERVED5[2];
	volatile int32_t BDCR;  	 //				RCC Backup domain control register  						    offset = 0x70
	volatile int32_t CSR;  		 //				RCC clock control & status register  						    offset = 0x74
	uint32_t RESERVED6[2];
	volatile int32_t SSCGR; 	 //				RCC spread spectrum clock generation register				    offset = 0x80
	volatile int32_t PLLI2SCFGR; //				RCC PLLI2S configuration register 							    offset = 0x84

}RCC_RegDef_t;


/*
 * peripheral register definition struct for EXTI
 */
typedef struct
{
	volatile uint32_t IMR;        //                    offset -- 0x00
	volatile uint32_t EMR;        //                    offset -- T0x04
	volatile uint32_t RTSR;       //                    offset -- D0x08
	volatile uint32_t FTSR;       //                    offset -- R0x0C
	volatile uint32_t SWIER;      //                    offset -- 0x10
	volatile uint32_t PR;         //                    offset -- 0x14

}EXTI_RegDef_t;


// ************************************************ SYSCFG REGISTER *************************************************
typedef struct{
	volatile uint32_t MEMRMP;           // offset - 0x00
	volatile uint32_t PMC;              // offset - 0x04
	volatile uint32_t EXTICR[4];          // offset - 0x08 to  0x14
    uint32_t RESERVED1[2];                  // offset 0x18 to 0x1c
	volatile uint32_t CMPCR;            // offset - 0x20
	uint32_t RESERVED2[2];                 // offset - 0x24-0x28
	volatile uint32_t CFGR;             // addr offset -0x2c;
}SYSCFG_RegDef_t;


//// ********************************************* PERIPHERAL DEFINATION  ( type casted value )************************************************************

#define GPIOA                         ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB                         ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC                         ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD                         ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE                         ((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF                         ((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG                         ((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH                         ((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI                         ((GPIO_RegDef_t*) GPIOI_BASEADDR)

#define RCC                           ((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI                          ((EXTI_RegDef_t*) EXTI_BASEADDR)   // handle by the APB2 bus

// ************************************************ CLK ENABLE MACRO FOR GPIOx ***********************************************************
#define GPIOA_PCLK_EN()                 (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() 				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()					(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()					(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()					(RCC->AHB1ENR |= (1<<8))

// ************************************************ CLK ENABLE MACRO FOR I2Cx ***********************************************************
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() 					(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1<<23))


// ************************************************ CLK ENABLE MACRO FOR USARTx ***********************************************************
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()					(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()						(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()						(RCC->APB1ENR |= (1<<20))
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1<<5))
// ************************************************ CLK ENABLE MACRO FOR SYSCFG ***********************************************************
#define SYSCFG_PCLK_EN() 					(RCC->APB2ENR |= (1<<14))


// ------------------------------------------------- CLK DISABLE FOR I2Cx peripheral -----------------------------------------------------------------
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() 					(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1<<23))
// ------------------------------------------------- CLK DISABLE FOR GPIOx peripheral -----------------------------------------------------------------
#define GPIOA_PCLK_DI()                 (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()					(RCC->AHB1ENR &= ~(1<<8))

// ************************************************ CLK DISABLE MACRO FOR SYSCFG ***********************************************************
#define SYSCFG_PCLK_DI() 					(RCC->APB2ENR &= ~(1<<14))
/*
 *  MACRO to reset  GPIOx Peripherals
 */
#define GPIOA_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<0));   (RCC->AHB1RSTR &= ~(1<<0));}while(0)  // using do while loop to do the things in one line
#define GPIOB_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<1));   (RCC->AHB1RSTR &= ~(1<<1));}while(0)  // reset and setting in one line using do while loop
#define GPIOC_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<2));   (RCC->AHB1RSTR &= ~(1<<2));}while(0)  // using do while loop to do the things in one line
#define GPIOD_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<3));   (RCC->AHB1RSTR &= ~(1<<3));}while(0)  // using do while loop to do the things in one line
#define GPIOE_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<4));   (RCC->AHB1RSTR &= ~(1<<4));}while(0)  // using do while loop to do the things in one line
#define GPIOF_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<5));   (RCC->AHB1RSTR &= ~(1<<5));}while(0)  // using do while loop to do the things in one line
#define GPIOG_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<6));   (RCC->AHB1RSTR &= ~(1<<6));}while(0)  // using do while loop to do the things in one line
#define GPIOH_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<7));   (RCC->AHB1RSTR &= ~(1<<7));}while(0)  // using do while loop to do the things in one line
#define GPIOI_REG_RESET()    do{ (RCC->AHB1RSTR |= (1<<8));   (RCC->AHB1RSTR &= ~(1<<8));}while(0)  // using do while loop to do the things in one line


/*
 * return port code for given GPIOx base addr
 * WE CAN USE THE IF ELSE STATEMENT ALSO
 * this macro return a code between 0 to7 for given GPIO base addr(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
									    (x == GPIOB) ? 3 : \
									    (x == GPIOA) ? 4 : \
										(x == GPIOB) ? 5 : \
										(x == GPIOA) ? 6 : \
										(x == GPIOB) ? 7 : 0 )
//  *********************************  IRQ NUMBER FOR DIFFERENT PERIPHERALS *************************************************
// IRQ(INTERRUPT REQUEST ) NUMBERS OF STM32f407 MCU

#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1	 	    7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40



// some generic Macros
#define ENABLE            1
#define DISABLE			  0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET

// ******************************************************* S P I { SERIAL PERIPHERAL INTERFACE } **************************************

// register defination for the SPI
 typedef struct
 {
	 volatile uint32_t CR1;						//    control register 1     	 offset - 0x00
	 volatile uint32_t CR2;						//    control register 2     	 offset - 0x04
	 volatile uint32_t SR;						//    status register        	 offset - 0x08
	 volatile uint32_t DR;				   		//    data register           	 offset - 0x0c
	 volatile uint32_t CRCPR;					//    CRC polynomial register    offset - 0x10
	 volatile uint32_t RXCRCR;					//    RX CRC register       	 offset - 0x14
	 volatile uint32_t TXCRCR;					//    TX CRC register            offset - 0x18
	 volatile uint32_t I2SCFGR;					//    I2S configuration registe  offset - 0x1c
	 volatile uint32_t I2SPR;					//    I2S prescaler register     offset - 0x20
 }SPI_RegDef_t;

  //--------------------------------------------   PERIPHERAL DEFINATION ----------------------------------------------
#define SPI1                                ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                                ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                                ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4                                ((SPI_RegDef_t*)SPI4_BASEADDR)

 // ************************************************ CLK ENABLE MACRO FOR SPIx ***********************************************************
 #define SPI2_PCLK_EN()                  (RCC->APB1ENR |= (1<<14))
 #define SPI3_PCLK_EN()                  (RCC->APB1ENR |= (1<<15))
 #define SPI1_PCLK_EN()					 (RCC->APB2ENR |= (1<<12))
#define  SPI4_PCLK_EN()                  (RCC->APB2ENR |= (1<<13)) // please check further, in my data sheet it was reserved
 // ------------------------------------------------- CLK DISABLE FOR SPIx peripheral -----------------------------------------------------------------
 #define SPI2_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<14))
 #define SPI3_PCLK_DI()                  (RCC->APB1ENR &= ~(1<<15))
 #define SPI1_PCLK_DI()				   	(RCC->APB2ENR &= ~(1<<12))
#define  SPI4_PCLK_DI()                  (RCC->APB2ENR &= ~(1<<13))

 // ----------------------------------------------
 //***********************************************************************************************************
//                                          BIT DEFINATION MACRO FOR SPI PERIPHERA
 ////***********************************************************************************************************

 /*
  *    BIT POSITION DEFINATION SPI_CR1
  */

#define SPI_CR1_CPHA                   0
#define SPI_CR1_CPOL                  1
#define SPI_CR1_MSTR                   2
#define SPI_CR1_BR                   3
#define SPI_CR1_SPE                   6
#define SPI_CR1_LSBFIRST             7
#define SPI_CR1_SSI                   8
#define SPI_CR1_SSM                   9
#define SPI_CR1_RXONLY                 10
#define SPI_CR1_DFF                    11
#define SPI_CR1_CRCNEXT                12
#define SPI_CR1_CRCEN                13
#define SPI_CR1_BIDIOE                 14
#define SPI_CR1_BIDIMODE                   15

 /*
  *    BIT POSITION DEFINATION SPI_SR
  */
#define SPI_SR_RXNE                        0
#define SPI_SR_TXE                       1
#define SPI_SR_CHSIDE                        2
#define SPI_SR_UDR                        3
#define SPI_SR_CRCERR                      4
#define SPI_SR_MODF                        5
#define SPI_SR_OVR                      6
 #define SPI_SR_BSY                        7
#define SPI_SR_FRE                        8

 /*
  *    BIT POSITION DEFINATION SPI_CR2
  */
#define SPI_CR2_RXNE                        0
#define SPI_CR2_TXE                       1
#define SPI_CR2_CHSIDE                        2
#define SPI_CR2_UDR                        3
#define SPI_CR2_CRCERR                      4
#define SPI_CR2_MODF                        5
#define SPI_CR2_OVR                      6
 #define SPI_CR2_BSY                        7
#define SPI_CR2_FRE                        8


#include "stm32f407_driver.h"
#include "stm32f407_spi_driver.h"
#endif /* STM32F407XX_H_ */


















