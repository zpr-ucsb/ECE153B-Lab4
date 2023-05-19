#include "I2C.h"

extern void Error_Handler(void);

// Inter-integrated Circuit Interface (I2C)
// up to 100 Kbit/s in the standard mode, 
// up to 400 Kbit/s in the fast mode, and 
// up to 3.4 Mbit/s in the high-speed mode.

// Recommended external pull-up resistance is 
// 4.7 kOmh for low speed, 
// 3.0 kOmh for the standard mode, and 
// 1.0 kOmh for the fast mode
	
//===============================================================================
//                        I2C GPIO Initialization
//===============================================================================
void I2C_GPIO_Init(void) {
    // Enable GPIOB clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODE6);        // Clear Port B, Pin 6 MODER
  	GPIOB->MODER |= GPIO_MODER_MODE6_1;         // Set Port B, Pin 6 MODER to Alternative Mode
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6);		// Clear AF for Port B, Pin 6
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;		// Choose AF4 (I2C1_SCL) for Port B, Pin 6
	GPIOB->OTYPER |= GPIO_OTYPER_OT6;			// 1 = Open-Drain
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED6); 	// 11 = very high speed
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6);		// 01 = pull-up
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD6_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODE7);        // Clear Port B, Pin 7 MODER
  	GPIOB->MODER |= GPIO_MODER_MODE7_1;         // Set Port B, Pin 7 MODER to Alternative Mode
	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL7);		// Clear AF for Port B, Pin 7
	GPIOB->AFR[0] |= GPIO_AFRL_AFSEL7_2;		// Choose AF4 (I2C1_SDA) for Port B, Pin 7
	GPIOB->OTYPER |= GPIO_OTYPER_OT7;			// 1 = Open-Drain
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7); 	// 11 = very high speed
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);		// 01 = pull-up
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD7_0;
}
	
#define I2C_TIMINGR_PRESC_POS	28
#define I2C_TIMINGR_SCLDEL_POS	20
#define I2C_TIMINGR_SDADEL_POS	16
#define I2C_TIMINGR_SCLH_POS	8
#define I2C_TIMINGR_SCLL_POS	0

//===============================================================================
//                          I2C Initialization
//===============================================================================
void I2C_Initialization(void) {
uint32_t OwnAddr = 0x52;

	// [TODO]
	// 1. Set up the clock for I2C in the RCC registers. Refer to Section 8.4 of the STM32L4x6
	// Reference Manual for detailed information about the RCC registers and their bits.
	// (a) Enable the clock for I2C1 in the peripheral clock enable register.
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

	// (b) Set the system clock as the clock source for I2C1 in the peripherals independent clock
	// configuration register.
	RCC->CCIPR &= ~(RCC_CCIPR_I2C1SEL);	// 01: SYSCLCK Selected 
	RCC->CCIPR |= RCC_CCIPR_I2C1SEL_0;

	// (c) Reset I2C1 by setting bits in the peripheral reset register. After doing so, clear the
	// bits so that I2C1 does not remain in a reset state.
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C1RST;		// 1 = Reset;	
	RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_I2C1RST);		// 0 = No Effect (Clear);


	// 2. Configure the I2C registers for communication with the temperature sensor. Similar to
	// when we configured USART settings, ensure that I2C is disabled before modifying the
	// registers. Refer to Section 35.7 of the STM32L4x6 Reference Manual for more information
	// about the I2C registers and their bits.
	I2C1->CR1 &= ~(I2C_CR1_PE); 


	// (a) Enable the analog noise filter, disable the digital noise filter, enable error interrupts,
	// and enable clock stretching. Set the master to operate in 7-bit addressing mode.
	// Enable automatic end mode and NACK generation. (These settings are all in the
	// control registers.)
	I2C1->CR1 &= ~(I2C_CR1_ANFOFF); 	// 0 = Analog Noise Filter enabled
	I2C1->CR1 &= ~(I2C_CR1_DNF); 		// 0000 = Digital filter disabled
	I2C1->CR1 |= I2C_CR1_ERRIE;			// 1 = Error Interrupts enabled
	I2C1->CR1 &= ~(I2C_CR1_NOSTRETCH); 	// 0 = Clock Stretching enabled

	I2C1->CR2 &= ~(I2C_CR2_ADD10); 		// 0 = The master operates in 7-bit addressing mode
	I2C1->CR2 |= I2C_CR2_AUTOEND;		// 1 = Automatic end mode
	I2C1->CR2 |= I2C_CR2_NACK; 			// 1 = a NACK is sent after current received byte

	// (b) Set the values in the timing register. This guarantees correct data hold and setup
	// times that are used in master/peripheral modes. The timing register stores several
	// values:
	I2C1->TIMINGR &= ~(I2C_TIMINGR_PRESC | I2C_TIMINGR_SCLDEL | I2C_TIMINGR_SDADEL | I2C_TIMINGR_SCLH | I2C_TIMINGR_SCLL); //clear
	// PRESC – This value is the timing prescaler. It is used to generate the clock period
	// that will be used for the data setup, data hold, and SCL high/low counters.
		// Making TIMER CLK to be 40MHz (Period of 25ns). 80MHz / (PSC + 1) = 40MHz => PSC = 1
	I2C1->TIMINGR |= 7U << I2C_TIMINGR_PRESC_POS; // 7, 14, 14, 47, 47

	// SCLDEL – This value determines the minimum data setup time, which is a delay
	// between an SDA edge and the next SCL rising edge in transmission mode. The
	// generated delay time is
		// Minimum data setup time = 1000ns
		// Making t_SCLDEL to be 1000ns. 1000ns <= (SCLDEL + 1) * 25ns => SCLDEL = 39, but making it 40
	I2C1->TIMINGR |= 14U << I2C_TIMINGR_SCLDEL_POS;

	// SDADEL – This value determines the minimum data hold time, which is a delay
	// between the SCL falling edge and the next SDA edge in transmission mode. The
	// generated delay time is
		// Minimum data hold time = 1250ns
	I2C1->TIMINGR |= 14U << I2C_TIMINGR_SDADEL_POS;

	// SCLH – This value determines the minimum period for the high phase of the clock
	// signal. The generated high period is
		// Minimum period for the high clock = 4.0us
	I2C1->TIMINGR |= 49U << I2C_TIMINGR_SCLDEL_POS;

	// SCLL – This value determines the minimum period for the low phase of the clock
	// signal. The generated low period is
		// Minimum period for the low clock = 4.7us
	I2C1->TIMINGR |= 49U << I2C_TIMINGR_SCLL_POS;

	// 3. Set your own address in the own address registers. To modify the address, you must first
	// disable the own address. Do this for only Own Address 1 – we do not need Own Address
	// 2 (ensure that it remains disabled).
	I2C1->OAR1 &= ~(I2C_OAR1_OA1EN);		// 0 = disabled
	I2C1->OAR2 &= ~(I2C_OAR2_OA2EN);		// 0 = disabled
	I2C1->OAR1 &= ~(I2C_OAR1_OA1);			// Clear Own Address

	// (a) Set own address to 7-bit mode.
	I2C1->OAR1 &= ~(I2C_OAR1_OA1MODE);		// 0 = Own address 1 is a 7-bit address	

	// (b) Write the own address that you want to use – you are free to use OwnAddr = 0x52
	// that is provided in the code.
	I2C1->OAR1 |= (I2C1->OAR1 & ~(I2C_OAR1_OA1)) | OwnAddr;				// OwnAddr = 0x52.

	// (c) Enable own address.
	I2C1->OAR1 |= I2C_OAR1_OA1EN;			// 1 = enabled


	// 4. Enable I2C in the control register.
	I2C1->CR1 |= I2C_CR1_PE;				// 1 = enabled
}

//===============================================================================
//                           I2C Start
// Master generates START condition:
//    -- Slave address: 7 bits
//    -- Automatically generate a STOP condition after all bytes have been transmitted 
// Direction = 0: Master requests a write transfer
// Direction = 1: Master requests a read transfer
//=============================================================================== 
int8_t I2C_Start(I2C_TypeDef * I2Cx, uint32_t DevAddress, uint8_t Size, uint8_t Direction) {	
	
	// Direction = 0: Master requests a write transfer
	// Direction = 1: Master requests a read transfer
	
	uint32_t tmpreg = 0;
	
	// This bit is set by software, and cleared by hardware after the Start followed by the address
	// sequence is sent, by an arbitration loss, by a timeout error detection, or when PE = 0.
	tmpreg = I2Cx->CR2;
	
	tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));
	
	if (Direction == READ_FROM_SLAVE)
		tmpreg |= I2C_CR2_RD_WRN;  // Read from Slave
	else
		tmpreg &= ~I2C_CR2_RD_WRN; // Write to Slave
		
	tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16 ) & I2C_CR2_NBYTES));
	
	tmpreg |= I2C_CR2_START;
	
	I2Cx->CR2 = tmpreg; 
	
   	return 0;  // Success
}

//===============================================================================
//                           I2C Stop
//=============================================================================== 
void I2C_Stop(I2C_TypeDef * I2Cx){
	// Master: Generate STOP bit after the current byte has been transferred 
	I2Cx->CR2 |= I2C_CR2_STOP;								
	// Wait until STOPF flag is reset
	while( (I2Cx->ISR & I2C_ISR_STOPF) == 0 ); 
}

//===============================================================================
//                           Wait for the bus is ready
//=============================================================================== 
void I2C_WaitLineIdle(I2C_TypeDef * I2Cx){
	// Wait until I2C bus is ready
	while( (I2Cx->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY );	// If busy, wait
}

//===============================================================================
//                           I2C Send Data
//=============================================================================== 
int8_t I2C_SendData(I2C_TypeDef * I2Cx, uint8_t DeviceAddress, uint8_t *pData, uint8_t Size) {
	int i;
	
	if (Size <= 0 || pData == NULL) return -1;
	
	I2C_WaitLineIdle(I2Cx);
	
	if (I2C_Start(I2Cx, DeviceAddress, Size, WRITE_TO_SLAVE) < 0 ) return -1;

	// Send Data
	// Write the first data in DR register
	// while((I2Cx->ISR & I2C_ISR_TXE) == 0);
	// I2Cx->TXDR = pData[0] & I2C_TXDR_TXDATA;  

	for (i = 0; i < Size; i++) {
		// TXE is set by hardware when the I2C_TXDR register is empty. It is cleared when the next
		// data to be sent is written in the I2C_TXDR register.
		// while( (I2Cx->ISR & I2C_ISR_TXE) == 0 ); 

		// TXIS bit is set by hardware when the I2C_TXDR register is empty and the data to be
		// transmitted must be written in the I2C_TXDR register. It is cleared when the next data to be
		// sent is written in the I2C_TXDR register.
		// The TXIS flag is not set when a NACK is received.
		while((I2Cx->ISR & I2C_ISR_TXIS) == 0 );
		I2Cx->TXDR = pData[i] & I2C_TXDR_TXDATA;  // TXE is cleared by writing to the TXDR register.
	}
	
	// Wait until TC flag is set 
	while((I2Cx->ISR & I2C_ISR_TC) == 0 && (I2Cx->ISR & I2C_ISR_NACKF) == 0);
	
	if( (I2Cx->ISR & I2C_ISR_NACKF) != 0 ) return -1;

	I2C_Stop(I2Cx);
	return 0;
}


//===============================================================================
//                           I2C Receive Data
//=============================================================================== 
int8_t I2C_ReceiveData(I2C_TypeDef * I2Cx, uint8_t DeviceAddress, uint8_t *pData, uint8_t Size) {
	int i;
	
	if(Size <= 0 || pData == NULL) return -1;

	I2C_WaitLineIdle(I2Cx);

	I2C_Start(I2Cx, DeviceAddress, Size, READ_FROM_SLAVE); // 0 = sending data to the slave, 1 = receiving data from the slave
						  	
	for (i = 0; i < Size; i++) {
		// Wait until RXNE flag is set 	
		while( (I2Cx->ISR & I2C_ISR_RXNE) == 0 );
		pData[i] = I2Cx->RXDR & I2C_RXDR_RXDATA;
	}
	
	// Wait until TCR flag is set 
	while((I2Cx->ISR & I2C_ISR_TC) == 0);
	
	I2C_Stop(I2Cx);
	
	return 0;
}
