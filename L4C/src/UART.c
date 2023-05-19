#include "UART.h"

void UART2_Init(void) {
	// (a) Enable the USART2 clock in the peripheral clock register.
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	// (b) Select the system clock as the USART2 clock source in the peripheral independent
	//     clock configuration register.
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);	// 01: SYSCLCK Selected 
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
}

void UART2_GPIO_Init(void) {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; 		// Enable GPIOA CLK

	// Configure PA2 and PA3 to operate as UART transmitters and receivers.
	GPIOA->MODER &= ~(GPIO_MODER_MODE2);    // Clear Port A, Pin 2 MODER
  	GPIOA->MODER |= GPIO_MODER_MODE2_1;   // Set Port A, Pin 2 MODER to Alternative Mode
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2);		// Clear AF for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0;		// Choose AF7 (USART2_TX) for Port A, Pin 2
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2;

	GPIOA->MODER &= ~(GPIO_MODER_MODE3);    // Clear Port A, Pin 3 MODER
  	GPIOA->MODER |= GPIO_MODER_MODE3_1;   // Set Port A, Pin 3 MODER to Alternative Mode
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL3);		// Clear AF for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_0;		// Choose AF7 (USART2_RX) for Port A, Pin 3
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2;

	// (a) Both GPIO pins should operate at very high speed.
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2); 	// 11 = very high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3);

	// (b) Both GPIO pins should have a push-pull output type.
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2); 	// 0 = push-pull, 1 = opendrain
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3);


	// (c) Configure both GPIO pins to use pull-up resistors for I/O.
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2);	// 01 = pull-up
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_0;
}

void USART_Init(USART_TypeDef* USARTx) {
	// Disabled before you start to modify the registers. 
	USARTx->CR1 &= ~(USART_CR1_UE);

	// In the control registers, set word length to 8 bits, oversampling mode to oversample

	USARTx->CR1 &= USART_CR1_M1;		// 00 = 8 bits
	USARTx->CR1 &= USART_CR1_M0;
	
	USARTx->CR1 &= ~(USART_CR1_OVER8); 	// 0 = oversampling by 16
	USARTx->CR2 &= ~(USART_CR2_STOP);  // 00 = 1 stop bit
	
	//Set the baud rate to 9600.
	USARTx->BRR = 8333; // USARTDIV = 80000000/9600 = 8333

	//In the control registers, enable both the transmitter and receiver.
	USARTx->CR1 |= USART_CR1_TE;
	USARTx->CR1 |= USART_CR1_RE;
	USARTx->CR1 |= USART_CR1_UE;
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}

