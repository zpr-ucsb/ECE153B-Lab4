#include "SPI.h"
#include "SysTimer.h"

void SPI1_GPIO_Init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;       // Enable Output Clock B

	GPIOB->MODER &= ~(GPIO_MODER_MODER3);      // 10 = Alternative Function mode
    GPIOB->MODER |= GPIO_MODER_MODER3_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3);      // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0;
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3);      // 0 = Push-Pull
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3);       // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3;		// 11 = very high speed
  
	GPIOB->MODER &= ~(GPIO_MODER_MODER4);      // 10 = Alternative Function mode
    GPIOB->MODER |= GPIO_MODER_MODER4_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL4);      // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0;
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT4);      // 0 = Push-Pull
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4);       // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED4;		// 11 = very high speed

	GPIOB->MODER &= ~(GPIO_MODER_MODER5);      // 10 = Alternative Function mode
    GPIOB->MODER |= GPIO_MODER_MODER5_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL5);      // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_0;
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT5);      // 0 = Push-Pull
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD5);       // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;		// 11 = very high speed
}

void SPI1_Init(void){
  // Enable SPI clock and Reset SPI
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

  // Disable SPI
  SPI1->CR1 &= ~SPI_CR1_SPE;
  // Configure for Full Duplex Communication
  SPI1->CR1 &= ~SPI_CR1_RXONLY;
  // Configure for 2-line Unidirectional Data Mode
  SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
  // Disable Output in Bidirectional Mode
  SPI1->CR1 &= ~SPI_CR1_BIDIOE;
  // Set Frame Format: MSB First, 16-bit, Motorola Mode
	SPI1->CR2 &= ~(SPI_CR2_DS);				// 0111 = 8-bit
	SPI1->CR2 |= SPI_CR2_DS_0;
	SPI1->CR2 |= SPI_CR2_DS_1;
	SPI1->CR2 |= SPI_CR2_DS_2;	
  SPI1->CR2 &= ~(SPI_CR2_FRF);
  SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
  // Configure Clock. Read DataSheet for required value
  SPI1->CR1 &= ~SPI_CR1_BR;
  SPI1->CR1 |= SPI_CR1_BR_2;
  // Disable Hardware CRC Calculation
  SPI1->CR1 &= ~SPI_CR1_CRCEN;
  // Set as Master
  SPI1->CR1 |= SPI_CR1_MSTR;
  // Disable Software Slave Management
  SPI1->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
  // Enable NSS Pulse Management
  SPI1->CR2 |= SPI_CR2_NSSP;
  // Enable Output
  SPI1->CR1 |= SPI_CR1_SPE;
  // Set FIFO Reception Threshold to 1/2
  SPI1->CR2 &= ~SPI_CR2_FRXTH;
  // Enable SPI
  SPI1->CR1 |= SPI_CR1_SPE;
}


uint16_t SPI_Transfer_Data(uint16_t write_data){
  // Wait for TXE (Transmit buffer empty)
  while (!(SPI1->SR & SPI_SR_TXE));

  // Write data
  SPI1->DR = write_data;

  // Wait for not busy
  while (SPI1->SR & SPI_SR_BSY);

  // Wait for RXNE (Receive buffer not empty)
  while (!(SPI1->SR & SPI_SR_RXNE));

  // Read data
  uint16_t read_data = SPI1->DR;

  return read_data;
}
