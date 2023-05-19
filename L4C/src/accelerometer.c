#include "SPI.h"
#include "SysTimer.h"
#include "accelerometer.h"

void accWrite(uint8_t addr, uint8_t val) {
    // The first byte will be command byte, with 1 R/W bit, 1 MB bit, and 6 command bits.
    // The second byte will be data byte.
    // All data read will be discarded. MB bit will always be low.
		//00 (addr) val
    uint16_t transferData = (uint16_t)(~(1 << 6 | 1 << 5)) | addr;
		transferData = (uint16_t)(transferData << 8) | val;
    SPI_Transfer_Data(transferData);
}

uint8_t accRead(uint8_t addr) {
    // The first byte will be command byte, with 1 R/W bit, 1 MB bit, and 6 command bits.
    // The second byte will be dummy data byte.
    // The third byte will be the received data byte.
    uint16_t transferData = (1 << 6) | (uint16_t)(~(1 << 5)) | (uint16_t)((addr << 3));
		transferData = (uint16_t)(transferData << 8); //dummy sedond byte (all zeros)
	
    uint16_t receivedData = SPI_Transfer_Data(transferData);
    return (uint8_t)receivedData;
}

void initAcc(void) {
    // Set full range mode
    accWrite(0x31, 0x01);  // data_format range= +- 4g
    // Enable measurement
		// (This command can be initiated by setting the measure bit (Bit D3) in the POWER_CTL register (Address 0x2D).)
    accWrite(0x2D, 0xD3);
}

void readValues(double* x, double* y, double* z) {
    // Find scaler from data sheet
		uint16_t receivedData = accRead(0x32);
	
    // Read values into x, y, z using accRead

    int16_t rawX = (accRead(0x33)<<8)|accRead(0x32);  
    int16_t rawY = (accRead(0x35)<<8)|accRead(0x34);
    int16_t rawZ = (accRead(0x37)<<8)|accRead(0x36);

    // Calculate the scaled values using the appropriate scaler
    *x = rawX * .0078;
    *y = rawY * .0078;
    *z = rawZ * .0078;
}
