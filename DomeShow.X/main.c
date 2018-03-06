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
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/mcc.h"
#include <p24Fxxxx.h>

// Some constants
#define DMX_START_CODE 0
#define NUM_CHANNELS 512

// Used in DMX RX state machine in interrupt
enum DMX_STATE {
    DMX_WAIT_FOR_BREAK,
    DMX_WAIT_FOR_START,
    DMX_READ_PACKET
};

// Start out waiting for a break
enum DMX_STATE dmxState = DMX_WAIT_FOR_BREAK;

// For storing channel values (0-255))
volatile uint8_t channelValues[NUM_CHANNELS];
volatile int channelIndex = 0;

// Forward declaration
void UART_DMX_RX_Handler(void);

int setup(void) {

    SYSTEM_Initialize();
    
    UART1_SetRxInterruptHandler(UART_DMX_RX_Handler);
    
    return 0;
}

void UART_DMX_RX_Handler(void) {
    
    char byte;
    
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
                if (channelIndex >= NUM_CHANNELS) {
                    dmxState = DMX_WAIT_FOR_BREAK;
                    return;
                }
                
                // Store in array
                channelValues[channelIndex] = byte;
                channelIndex++;
            }
            
            break;
    }
    
    IFS0bits.U1RXIF = 0;
}

void updateLights() {
    // TODO: write PWM driver I2C stuff here
}

int main(void) {
    setup();
    
    while(1) {
        updateLights();
    }
    
    return 0;
}
