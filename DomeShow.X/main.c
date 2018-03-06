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

int setup(void) {

    OSCILLATOR_Initialize();
    SYSTEM_Initialize();
    
    return 0;
}

int main(void) {
    setup();
    
    while(1) {
        // Do stuff
    }
    
    return 0;
}
