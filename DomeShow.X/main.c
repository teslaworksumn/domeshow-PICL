/*
 * Dome Show Firmware
 * for Tesla Works
 * Authors: Ryan Fredlund, Katie Manderfeld, Ian Smith
 * Last updated: 3/5/2018
 * 
 * References:
 * - http://www.microchip.com/forums/m355304.aspx (UART ISR)
 */

#include "xc.h"
#include <libpic30.h>
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/mcc.h"
#include <p24Fxxxx.h>

// DMX constants
#define DMX_START_CODE 0

// Addressing
#define TOTAL_NUM_CHANNELS 512
#define NUM_BOARD_CHANNELS 16
#define DRIVER0_CHANNELS 8
#define DRIVER1_CHANNELS 8
uint8_t boardAddress;

// Set up delay stuff
#define FCY 16000000L  // define your instruction frequency, FCY = FOSC/2 = 16Mhz  
#define CYCLES_PER_MS ((unsigned long)(FCY * 0.001)) // instruction cycles per millisecond
#define __DELAY_MS(ms) __delay32(CYCLES_PER_MS * ((unsigned long) ms)); // __delay32 is provided by the compiler, delay some # of milliseconds

// Used in DMX RX state machine in interrupt
enum DMX_STATE {
    DMX_WAIT_FOR_BREAK,
    DMX_WAIT_FOR_START,
    DMX_READ_PACKET
};

// Start out waiting for a break
enum DMX_STATE dmxState = DMX_WAIT_FOR_BREAK;

// For storing channel values (0-255))
volatile uint8_t channelValues[NUM_BOARD_CHANNELS];
volatile int channelIndex = 0;

uint16_t counter = 0;

// I2C constants
#define SLAVE_I2C_GENERIC_RETRY_MAX 100
#define SLAVE_I2C_GENERIC_DEVICE_TIMEOUT 50
#define I2C_PENDING_DELAY_MS 0

// Forward declarations
void UART_DMX_RX_Handler(void);
void i2c_WriteData(uint16_t, uint8_t[], uint8_t);
uint8_t getBoardAddress();
void pwmDriverSendData(uint16_t, uint8_t*, uint8_t);
void updateLights();

int setup(void) {

    SYSTEM_Initialize();
    
    UART1_SetRxInterruptHandler(UART_DMX_RX_Handler);
    
    boardAddress = getBoardAddress();
    
    return 0;
}

void UART_DMX_RX_Handler(void) {
    
    char byte;
    uint16_t startIndex = boardAddress * NUM_BOARD_CHANNELS;
    uint16_t endIndex = (boardAddress + 1) * NUM_BOARD_CHANNELS - 1;
    
    switch (dmxState) {
        case DMX_WAIT_FOR_BREAK:            
            // We are trying to find the break condition.
            // Wait until a framing error is received (which indicates the break condition)
            if (U1STAbits.FERR == 1) {
                // Move on to next state
                dmxState = DMX_WAIT_FOR_START;

                // Clear framing error to let the next bytes be read
                byte = U1RXREG;                
            } else {
                // Discard valid bytes (we want a framing error)
                byte = U1RXREG;
            }
            
            break;
        case DMX_WAIT_FOR_START:            
            // Wait for a valid byte to be received
            if (U1STAbits.FERR == 1) {
                // Clear framing error and ignore byte
                byte = U1RXREG;
            } else {
                // Read start code
                byte = U1RXREG;
                
                // Check if it is a valid start code
                if (byte == DMX_START_CODE) {
                    // Initialize values
                    channelIndex = 0;
                    
                    // Move on to next state
                    dmxState = DMX_READ_PACKET;
                } else {
                    // Not a valid start code, so go back to waiting for the next packet
                    dmxState = DMX_WAIT_FOR_BREAK;
                }
            }
            
            break;
        case DMX_READ_PACKET:
            // If a framing error happens, ignore the rest of the frame and start over
            if (U1STAbits.FERR == 1) {
                byte = U1RXREG;
                dmxState = DMX_WAIT_FOR_BREAK;
            } else {
                // If no framing error, then read data byte
                byte = U1RXREG;
                
                // Check if out of bounds
                if (channelIndex >= TOTAL_NUM_CHANNELS) {
                    dmxState = DMX_WAIT_FOR_BREAK;
                    return;
                }
                
                // Check if this board cares
                if (channelIndex >= startIndex && startIndex <= endIndex) {
                    // Store in array
                    channelValues[channelIndex - startIndex] = byte;
                }                

                // Increment channel (value byte read)
                channelIndex++;
            }
            
            break;
    }
    
    IFS0bits.U1RXIF = 0;
}

/*
 * Gets the current board address
 */
uint8_t getBoardAddress() {
    // Preparing for using DIP switch
    
    return 0;
}

/*
 * Sends PWM data to one of the driver chips using i2c
 * address is the 7-bit address of the driver (right justified)
 * data is a pointer to the array of channel values
 * numChannels is the length of the data array (number of values to send)
 */
void pwmDriverSendData(uint16_t address, uint8_t* data, uint8_t numChannels) {
    // Build data to send
    uint8_t dataToSend[1 + numChannels]; // The +1 is the control byte
    
    // First, we set the control register of the driver
    dataToSend[0] = 0b10100010; // Auto-increment (101) starting with PWM0 (00010)
    
    // Now, add channel values
    uint8_t i;
    for (i = 0; i < numChannels; i++) {
        dataToSend[1 + i] = data[i];
    }
    
    // Finally, write the message to PWM driver
    i2c_WriteData(address, dataToSend, numChannels + 1);
}

void updateLights() {
    // Addresses chosen because they are clearly not reserved in the spec
    // They will be left shifted later, since the last bit tells if R/W
    uint16_t address0 = 16; // 0001 0000, which will left shift to 0010 000X
    uint16_t address1 = 17; // 0001 0001, which will left shift to 0010 001X
    
    // Get copy of channel values
    uint8_t data[NUM_BOARD_CHANNELS];
    int i;
    for (i = 0; i < NUM_BOARD_CHANNELS; i++) {
        data[i] = channelValues[i];
    }
    
    // Handle first PWM driver (channels 0-7)
    pwmDriverSendData(address0, data, DRIVER0_CHANNELS);
        
    // Handle second PWM driver (channels 8-15)
    pwmDriverSendData(address1, data + DRIVER0_CHANNELS, DRIVER1_CHANNELS);
}

/*
 * Writes some data to the PWM driver.
 * driverAddress should be the 7-bit address of the driver, with MSB=0.
 * (e.g. don't set the r/w bit. LSB of driverAddress should be the actual LSB of the address)
 */
void i2c_WriteData(uint16_t driverAddress, uint8_t* data, uint8_t dataLength) {
    
    // Now it is possible that the slave device will be slow.
    // As a work around on these slaves, the application can
    // retry sending the transaction
    uint16_t timeOut = 0;
    uint16_t slaveTimeOut = 0;
    
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;

    while(status != I2C1_MESSAGE_FAIL) {
        // Do the write
        I2C1_MasterWrite(data, dataLength, driverAddress, &status);

        // wait for the message to be sent or status has changed.
        while (status == I2C1_MESSAGE_PENDING) {
            // add some delay here
            __DELAY_MS(I2C_PENDING_DELAY_MS);

            // timeout checking
            // check for max retry and skip this byte
            if (slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT) {                
                break;
            } else {
                slaveTimeOut++;
            }
        } 
        if ((slaveTimeOut == SLAVE_I2C_GENERIC_DEVICE_TIMEOUT) || 
            (status == I2C1_MESSAGE_COMPLETE)) {

            break;
        }

        // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
        //               or I2C1_DATA_NO_ACK,
        // The device may be busy and needs more time for the last
        // write so we can retry writing the data, this is why we
        // use a while loop here

        // check for max retry and skip this byte
        if (timeOut == SLAVE_I2C_GENERIC_RETRY_MAX) {
            break;
        } else {
            timeOut++;
        }
    }

    if (status == I2C1_MESSAGE_FAIL)
    {
        // TODO: do something here. Currently ignoring.
        return;
    }
}   

int main(void) {
    setup();
    
    while(1) {
        //updateLights();
        IO_RB14_Toggle();
        
        __DELAY_MS(1);
    }
    
    return 0;
}
