#include "SPI.h"
#include "SysTimer.h"

void SPI1_GPIO_Init(void) {
	//Initialize SPI1 GPIO pins
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;       // Enable Output Clock B

	GPIOB->MODER &= ~(GPIO_MODER_MODER3);      // 10 = Alternative Function mode
  GPIOB->MODER |= GPIO_MODER_MODER3_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3);      // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_0;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL3_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3);      // 0 = Push-Pull
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3);      // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED3;		// 11 = very high speed
  
	GPIOB->MODER &= ~(GPIO_MODER_MODER4);      // 10 = Alternative Function mode
  GPIOB->MODER |= GPIO_MODER_MODER4_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL4);      // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_0;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL4_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT4);      // 0 = Push-Pull
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4);      // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED4;		// 11 = very high speed

	GPIOB->MODER &= ~(GPIO_MODER_MODER5);     // 10 = Alternative Function mode
  GPIOB->MODER |= GPIO_MODER_MODER5_1;        
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL5);     // Choose alternative function 5
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_0;
  GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_2;       
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT5);      // 0 = Push-Pull
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD5);      // 00 = No PU PD. IS THIS WHAT NO PULL MEANS?
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED5;		// 11 = very high speed
}

void SPI1_Init(void){
	// (a) Enable the SPI clock.
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// (b) Set the RCC SPI reset bit, then clear it to reset the SPI1 or SPI2 peripheral.
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_SPI1RST);

	// (c) Disable the SPI enable bit. The peripheral must be configured while it is disabled.
	SPI1->CR1 &= ~(SPI_CR1_SPE);			// 0 = disabled

	// (d) Conigure the peripheral for full-duplex communication.
	SPI1->CR1 &= ~(SPI_CR1_RXONLY);			// 0 = full-duplex (Transmit and receive)

	// (e) Configure the peripheral for 2-line unidirectional data mode.
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);		// 0 = 2-line unidirectional data mode selected

	// (f) Disable output in bidirectional mode.
	SPI1->CR1 &= ~(SPI_CR1_BIDIOE); 		// 0 = Output disabled (receive-only mode) 

	// (g) Configure the frame format as MSB first.
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);		// 0 = data is transmitted / received with the MSB first

	// (h) Configure the frame format to 8-bit mode.
	SPI1->CR2 &= ~(SPI_CR2_DS);				// 0111 = 8-bit
	SPI1->CR2 |= SPI_CR2_DS_0;
	SPI1->CR2 |= SPI_CR2_DS_1;
	SPI1->CR2 |= SPI_CR2_DS_2;				

	// (i) Use Motorola SPI mode.
	SPI1->CR2 &= ~(SPI_CR2_FRF); 			// 0 = SPI Motorola mode

	// (j) Configure the clock to low polarity.
	SPI1->CR1 &= ~(SPI_CR1_CPOL);			// 0 = CK to 0 when idle

	// (k) Configure the clock to first clock transition.
	SPI1->CR1 &= ~(SPI_CR1_CPHA);			// 0 = The first clock transition is the first data capture edge

	// (l) Set the baud rate prescaler to 16.
	SPI1->CR1 &= ~(SPI_CR1_BR);				// 011 = fPCLK/16
	SPI1->CR1 |= SPI_CR1_BR_0;
	SPI1->CR1 |= SPI_CR1_BR_1;

	// (m) Disable hardware CRC calculation.
	SPI1->CR1 &= ~(SPI_CR1_CRCEN);			// 0 = CRC calculation disabled

	// (n) Set SPI1 to master mode and SPI2 to slave mode.
	SPI1->CR1 |= SPI_CR1_MSTR;				// 1 = Master

	// (o) Enable software SSM.
	SPI1->CR1 |= SPI_CR1_SSM;				// 1 = Software slave management enabled
	
	// (p) Enable NSS pulse generation.
	SPI1->CR2 |= SPI_CR2_NSSP;				// 1 = NSS pulse generated

	// (q)  Configure the internal slave select bit, 1 for master and 0 for slave.
	SPI1->CR1 |= SPI_CR1_SSI;

	// (r) Set the FIFO threshold to 1/4 (required for 8-bit mode).
	SPI1->CR2 |= SPI_CR2_FRXTH;				// 1 = 1/4

	// (s) Enable the SPI peripheral.
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
