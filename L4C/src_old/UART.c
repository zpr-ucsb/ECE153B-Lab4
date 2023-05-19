#include "UART.h"

void UART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock
    USART2->CR1 |= USART_CR1_UE; // Enable USART2
}

void UART2_GPIO_Init(void) {
    // Configure GPIOs for USART2 TX and RX
    GPIOA->MODER |= GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1; // Set alternate function mode for PA2 and PA3
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL2_1 | GPIO_AFRL_AFRL3_1 | GPIO_AFRL_AFRL3_0; // Set alternate function AF7 (USART2) for PA2 and PA3
}

void USART_Init(USART_TypeDef* USARTx) {
    USARTx->BRR = SystemCoreClock / 9600; // Set baud rate to 9600
    USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
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
