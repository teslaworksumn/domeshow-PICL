/*
 * Dome Show Firmware
 */

#include "xc.h"

// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF           // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config ICS = PGx1           // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GCP = OFF            // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF         // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI        // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF        // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON        // Primary Oscillator I/O Function (CLKO functions as I/O pin)
#pragma config FCKSM = CSECME       // Clock Switching and Monitor (Clock switching is enabled, 
                                    // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL       // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))


int setup(void) {
    CLKDIVbits.RCDIV = 0;   //Postscaler to 8MHz
    AD1PCFG = 0x9fff;       //All pins digital
    TRISA = 0;
    TRISB = 0;
    
    //UART Setup
    TRISBbits.TRISB9 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR18bits.U1RXR = 9; //Use Pin RP9 = "9", for UART Rx (Table 10-2)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
    //OC Setup
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR5bits.RP11R = 18;
    RPOR6bits.RP12R = 19;
    RPOR6bits.RP13R = 20;
    RPOR7bits.RP14R = 21;
    RPOR7bits.RP15R = 22;
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
    OC1CON1 = 0;
    OC1CON1bits.OCTSEL = ___;
    OC1CON1bits.OCM = 110;
    OC1CON2 = 0;
    //OCxCON2bits.OCINV lets you invert the output.
    
    
}

int main(void) {
    return 0;
}
