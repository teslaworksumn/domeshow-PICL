/*
 * Dome Show Firmware
 * for Tesla Works
 * Authors: Ryan Fredlund, Katie Manderfeld, Ian Smith
 * Last updated: 9/7/2017
 * 
 * References:
 * - http://www.microchip.com/forums/m355304.aspx (UART ISR)
 */

#include "xc.h"
#include <p24Fxxxx.h>

// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF           // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config ICS = PGx1           // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config GCP = OFF            // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF         // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI        // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF        // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = 1         // Primary Oscillator I/O Function (CLKO functions as I/O pin)
#pragma config FCKSM = CSECME       // Clock Switching and Monitor (Clock switching is enabled, 
                                    // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL       // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))


int setup(void) {
    // Set up clock
    // Clock routing from FRC Oscillator to
    // post scaler through PLL handled in
    // configuration bits above (FNOSC = FRCPLL)
    CLKDIVbits.RCDIV = 0;           // FRC oscillator post scaler 1:1
    CLKDIVbits.DOZEN = 1;
    CLKDIVbits.DOZE = 0;            // Peripheral post scaler 1:1
    
    //Pin Configuration
    AD1PCFG = 0x9fff;               // All pins digital.
    TRISA = 0;                      // Take care of specific I/O later
    TRISB = 0;                      // UART will override, as will PPS for OC
    
    //UART Setup
    TRISBbits.TRISB9 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR18bits.U1RXR = 9;          // Use Pin RP9 = "9", for UART Rx (Table 10-2)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    IEC0bits.U1RXIE = 1;            // Enable receive interrupt
    IPC2bits.U1RXIP = 7;            // Set interrupt priority
    U1MODEbits.UARTEN = 1;
    U1MODEbits.UEN = 0;
    U1MODEbits.BRGH = 1;            // Use high speed baud rate generation
    U1BRG = 15;                     // 250 kilobit per second baud rate
    U1MODEbits.PDSEL = 0;           // 8-bit data, no parity
    U1MODEbits.STSEL = 1;           // 2 stop bits
    U1STAbits.UTXEN = 0;            // Disable transmission
    U1STAbits.URXISEL = 2;          // TODO: Double-check this!!
    
    //OC Setup
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR5bits.RP11R = 18;           // OC1
    RPOR6bits.RP12R = 19;           // OC2
    RPOR6bits.RP13R = 20;           // OC3
    RPOR7bits.RP14R = 21;           // OC4
    RPOR7bits.RP15R = 22;           // OC5
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
    //Not using this yet because configuration is different between PICs
    /*
    OC1CON1 = 0;
    OC1CON1bits.OCTSEL = ___;
    OC1CON1bits.OCM = 110;
    OC1CON2 = 0;
    //OCxCON2bits.OCINV lets you invert the output. 
    */
    
    return 0;
}

char break_seen = 0;

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    char temp;

    _U1RXIF = 0;
    if (U1STAbits.FERR == 1) {      // Framing error, discard this byte   
        temp = (char) U1RXREG;      // Read receive register
        break_seen = 1;             // Remember that we have seen a break
        U1STAbits.FERR = 0;         // Don't save character anywhere
        
        asm("btg LATB, #5");        // Remove this when finished with dev
    }
    else {                          // No framing error, use this byte
        if (break_seen) {           // Last character was a break, so new frame
            break_seen = 0;         // Clear the break flag.
        }
        temp = (char) U1RXREG;      // Use this data. Implementation to come.
    }
    
    // Remove this once the above is verified to work.
    // This is the part where OERR isn't set while receiving data from the 16 channels.. Interesting
    /*
    if(_U1RXIF == 1) {   // If interrupt on UART receive
        //_U1RXIF = 0;
        if(U1STAbits.OERR == 1) {
            U1STAbits.OERR = 0;
        //if(U1STAbits.FERR == 1) {
            asm("btg LATB, #5");
        }
    }
    */
}

int main(void) {
    setup();
    while(1) {
    }
    return 0;
}
