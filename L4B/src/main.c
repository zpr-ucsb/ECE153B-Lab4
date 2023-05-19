/*
 * ECE 153B - Winter 2023
 *
 * Name(s):
 * Section:
 * Lab: 4B
 */

#include "stm32l476xx.h"
#include "I2C.h"
#include "SysClock.h"
#include "UART.h"
#include <string.h>
#include <stdio.h>

// Initializes USARTx
// USART2: UART Communication with Termite
// USART1: Bluetooth Communication with Phone
void Init_USARTx(int x) {
	if(x == 1) {
		UART1_Init();
		UART1_GPIO_Init();
		USART_Init(USART1);
	} else if(x == 2) {
		UART2_Init();
		UART2_GPIO_Init();
		USART_Init(USART2);
	} else {
		// Do nothing...
	}
}

int main(void) {
	
	printf("testing");
	
	System_Clock_Init(); // System Clock = 80 MHz
	
	// Initialize I2C
	I2C_GPIO_Init();
	I2C_Initialization();

	// Initialize UART -- change the argument depending on the part you are working on
	Init_USARTx(2);

	int i;
	char data[6];
	uint8_t SlaveAddress;
	uint8_t Data_Receive;
	uint8_t Data_Send;
	
	uint8_t sendingArray[30];
	uint8_t receivingArray[30];

	
	while(1) {	
		SlaveAddress = 0b1001000 << 1; // TMP102 Address: 1001000
		
		sendingArray[0] = 0;
		I2C_SendData(I2C1, SlaveAddress, sendingArray, 1);
		I2C_ReceiveData(I2C1, SlaveAddress, receivingArray, 1);
		Data_Receive = receivingArray[0];

		int temperature = (Data_Receive & 0x7F) - (((Data_Receive & 0x80) != 0) ? 128 : 0);

		sprintf(data, "The Current Temperature Is:%6d\n", temperature);
		printf("%s", data);
		
		// Some delay
		for(i = 0; i < 4500000; ++i); 
	}
}
