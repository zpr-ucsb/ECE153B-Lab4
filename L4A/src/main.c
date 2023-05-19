#include "stm32l476xx.h"
#include "LED.h"
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
    System_Clock_Init(); // Switch System Clock = 80 MHz
    LED_Init();
    
    // Initialize UART -- change the argument depending on the part you are working on
    Init_USARTx(1);
    
    char rxByte;
    while(1) {
			// Send data to the terminal that prompts the user to enter a command.
			// Use printf() to transmit data to Termite. The data that you send will be printed
			// on the Termite window.
			printf("Hello! Enter a command: ");

			// Wait for a response from the terminal. While there is no response,
			// scanf will load a 0 into rxByte as well as return 0.
			while (scanf("%c", &rxByte) == 0) {
			
			}

			// Receive the response from the terminal.
			// Use scanf() to receive data from Termite. Data can be sent from Termite by typing
			// a message into the terminal and pressing enter.
			if (rxByte == 'Y' || rxByte == 'y') {
				Green_LED_On();
				printf("The Green LED has been turned ON.");
			}
			else if (rxByte == 'N' || rxByte == 'n') {
				Green_LED_Off();
				printf("The Green LED has been turned OFF.");
			}
			else {
				printf("That command is not recognized. Please send a valid command.");
			}
    }
}
