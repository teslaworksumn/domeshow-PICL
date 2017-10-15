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

#define CHIP_CHANNELS 4
#define BOARD_CHANNELS 16

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
//#pragma config POSCMOD = EC         // Use External Clock
//#pragma config POSCMOD = XT         // Use External Clock
//#pragma config FNOSC = 0b10         // Oscillator Select (External Clock)
//#pragma config FNOSC = PRI         // Oscillator Select (External Clock)
#pragma config FNOSC = FRCPLL

int channel = 0;                    // Channel we're currently at in DMX frame
int start_address, end_address;     // Start and end DMX addresses on chip

int board_address = 0;             // Address of board for channel calculations
int chip_number = 0;               // Chip number on board
int chip_channel = 0;              // Channel on chip
int levels[CHIP_CHANNELS];         // Hold new output levels

int tempLevel = 0;

char break_seen = 0;                // If we've seen a frame break
char temp;                          // Where trash DMX bytes get stored
char dataByte;                      // Where good DMX bytes get stored

int setup(void) {
    // Set up clock
    // Clock routing from FRC Oscillator to
    // post scaler through PLL handled in
    // configuration bits above (FNOSC = FRCPLL)
    CLKDIVbits.RCDIV = 0;           // FRC oscillator post scaler 1:1
    CLKDIVbits.DOZEN = 1;
    CLKDIVbits.DOZE = 0;            // Peripheral post scaler 1:1
    //OSCTUNbits.TUN = 0b011111;
    
    // Pin Configuration
    AD1PCFG = 0x9fff;               // All pins digital.
    TRISA = 0;                      // Take care of specific I/O later
    TRISB = 0;                      // UART will override, as will PPS for OC   
    
    // DIP Switch address calculation
    // Pin assignments done via spreadsheet in Google Drive
    /*
    TRISA |= 0x001f; //Set up inputs
    TRISB |= 0x0018; //Set up inputs
     
    // Might need to do some stuff with pull-up resistors here?
    chip_number = (PORTAbits.RA1<<1) + (PORTAbits.RA0);
    board_address = (PORTAbits.RA4<<4) + (PORTBbits.RB4<<3) + (PORTAbits.RA3<<2) 
            + (PORTAbits.RA2<<1) + (PORTBbits.RB3);
    */
    
    board_address = 0;
    chip_number = 0;
    
    // These get set based off the previous two variables
    // Adding the +1s for 1 indexing in DMX. If we get mismatches, remove those.
    start_address = (board_address*BOARD_CHANNELS + chip_number*CHIP_CHANNELS) + 1;
    end_address = (board_address*BOARD_CHANNELS + chip_number*CHIP_CHANNELS + CHIP_CHANNELS) + 1;
    
    // UART Setup
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
    U1MODEbits.WAKE = 1;            // Wake on receiving start bit
    U1MODEbits.STSEL = 1;           // 2 stop bits
    U1STAbits.UTXEN = 0;            // Disable transmission
    U1STAbits.URXISEL = 1;          // Interrupt when there's one thing in the buffer
    
    // Timer Setup
    T1CONbits.TCS = 0;              // Use 1/2 Fosc (16 MHz)
    T1CONbits.TCKPS = 0b00;         // 1:1 with system clock
    TMR1 = 0;
    PR1 = 256;
    T1CONbits.TON = 1;
    
    T2CONbits.T32 = 0;
    T2CONbits.TCS = 0;              // Use 1/2 Fosc (16 MHz)
    T2CONbits.TCKPS = 0b00;         // 1/8 system clock
    TMR2 = 0;
    PR2 = 65535;
    _T2IE = 1;
    T2CONbits.TON = 1;
    
    // OC Setup
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR5bits.RP11R = 18;           // OC1
    RPOR6bits.RP12R = 19;           // OC2
    RPOR6bits.RP13R = 20;           // OC3
    RPOR7bits.RP14R = 21;           // OC4
    //RPOR7bits.RP15R = 22;           // OC5
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
    OC1CON1 = 0;
    OC1CON1bits.OCTSEL = 4;         // Use Timer1
    OC1CON1bits.OCM = 0b110;          // Edge-aligned PWM
    OC1CON2bits.SYNCSEL = 0b01011;
    OC1CON2 = 0x0000;
//    OC1CON2bits.OCINV = 1; // lets you invert the output. 
//    OC2CON2bits.OCINV = 1; // lets you invert the output. 
//    OC3CON2bits.OCINV = 1; // lets you invert the output. 
//    OC4CON2bits.OCINV = 1; // lets you invert the output. 
//    OC5CON2bits.OCINV = 1; // lets you invert the output. 

    OC2CON1 = 0x1006;
    OC2CON2 = 0x0000;
    OC3CON1 = 0x1006;
    OC3CON2 = 0x0000;
    OC4CON1 = 0x1006;
    OC4CON2 = 0x0000;
    //OC5CON1 = 0x1006;
    //OC5CON2 = 0;
    OC2CON2bits.SYNCSEL = 0b01011;
    OC3CON2bits.SYNCSEL = 0b01011;
    OC4CON2bits.SYNCSEL = 0b01011;
    
    OC1RS = 255;
    OC2RS = 255;
    OC3RS = 255;
    OC4RS = 255;
    //OC5RS = 255;
    
    return 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt() {
    _T2IF = 0;
    
    /*
    tempLevel += 5;
    OC4R = tempLevel;
    if(tempLevel == 255) {
        tempLevel = 0;
    }
    */
    // Write to OCRx registers
    
    OC1R = levels[0];
    OC2R = levels[1];
    OC3R = levels[2];
    OC4R = levels[3];
    //OC5R = levels[4];
    asm("btg LATB, #15");
    
}


void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    _U1RXIF = 0;
    if (U1STAbits.FERR == 1) {      // Framing error 
//        U1STAbits.FERR = 0;         // Clear framing error
        break_seen = 1;             // Remember that we have seen a break
        
        temp = (char) U1RXREG;      // Read receive register
        
        // TODO: Remove this when finished with dev
        asm("btg LATB, #5");
    }
    else {                          // No framing error, use this byte
        if (break_seen) {           // Last character was a break, so new frame
            break_seen = 0;         // Clear the break flag.
            
            // I'm fairly sure this line goes here, but not 100%
            channel = 0;            // Reset channel count
            chip_channel = 0;       // Reset chip channel count
            asm("btg LATB, #6");
        }
        else {
            // If we're not starting over at the beginning of the frame, move on
            channel++;
        }
        
        dataByte = (char) U1RXREG;  // Get the data
        
        // If channels are within the range on this chip, write them to local
        if (start_address <= channel && channel < end_address) {
            levels[chip_channel] = dataByte;
            chip_channel++;
        }
    }
}

int main(void) {
    setup();
    //OC4R = 255;
    //OC3R = 125;
    while(1) {
        //OC4R = tempLevel;
    }
    return 0;
}
