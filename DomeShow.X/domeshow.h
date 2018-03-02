/*
 * Dome Show Firmware
 * for Tesla Works
 * Authors: Ryan Fredlund, Katie Manderfeld, Ian Smith
 * Last updated: 3/2/2018
 * 
 */

#ifndef DOMESHOW_H
#define	DOMESHOW_H

#ifdef	__cplusplus
extern "C" {
#endif

#define BOARD_ADDRESS 0
#define BOARD_CHANNELS 15
#define PAYLOAD_SIZE 120        // Size of payload in r2 protocol
#define RX_BUFFER_SIZE 0x03ff   // 1024 (it's a nice high number)

// State of the domeshow RX protocol
typedef enum {
    DSCOM_STATE_WAITING,
    DSCOM_STATE_MAGIC,
    DSCOM_STATE_READING,
    DSCOM_STATE_PROCESSING
} DSCOM_RX_STATE_t;

// Start in WAITING state
DSCOM_RX_STATE_t dscom_rx_state = DSCOM_STATE_WAITING;

// Managing the CRC
unsigned char crc_start = 0;
unsigned char crc_end = 0;

// Magic bytes for protocol frame
uint8_t magic[6] = {0xFF, 0x00, 0xFF, 0x00, 0x00, 0xFF};

void setupUART() {
    TRISBbits.TRISB9 = 1;
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR18bits.U1RXR = 9;          // Use Pin RP9 = "9", for UART Rx (Table 10-2)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    IEC0bits.U1RXIE = 1;            // Enable receive interrupt
    IPC2bits.U1RXIP = 7;            // Set interrupt priority
    U1MODEbits.UARTEN = 1;
    U1MODEbits.UEN = 0;
    U1MODEbits.BRGH = 1;            // Use high speed baud rate generation
    U1BRG = 103;                     // 15=>250 kilobit per second baud rate
    U1MODEbits.PDSEL = 0;           // 8-bit data, no parity
    U1MODEbits.STSEL = 0;           // 1=>2 stop bits
    U1STAbits.UTXEN = 0;            // Disable transmission
    U1STAbits.URXISEL = 2;          // TODO: Double-check this!!
}

void setupPWM() {
    // Timer Setup
    T1CON = 0;
    TMR1 = 0;
    PR1 = 256;
    
    T2CONbits.T32 = 0;
    T2CONbits.TCS = 0;              // Use 1/2 Fosc (16 MHz)
    T2CONbits.TCKPS = 0b00;         // 1/8 system clock
    TMR2 = 0;
    PR2 = 65535;
    _T2IE = 1;
    T2CONbits.TON = 1;
    
    OC1CON1bits.OCTSEL = 4;         // Use Timer1
    OC2CON1bits.OCTSEL = 4;         // Use Timer1
    OC3CON1bits.OCTSEL = 4;         // Use Timer1
    OC4CON1bits.OCTSEL = 4;         // Use Timer1
    
    OC1CON1bits.OCM = 0b110;          // Edge-aligned PWM
    OC2CON1bits.OCM = 0b110;          // Edge-aligned PWM
    OC3CON1bits.OCM = 0b110;          // Edge-aligned PWM
    OC4CON1bits.OCM = 0b110;          // Edge-aligned PWM
    
    OC1CON2bits.SYNCSEL = 0b01011;  // Sync with Timer 1
    OC2CON2bits.SYNCSEL = 0b01011;  // Sync with Timer 1
    OC3CON2bits.SYNCSEL = 0b01011;  // Sync with Timer 1
    OC4CON2bits.SYNCSEL = 0b01011;  // Sync with Timer 1
    
    // OC Setup
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR5bits.RP11R = 18;           // OC1
    RPOR6bits.RP12R = 19;           // OC2
    RPOR6bits.RP13R = 20;           // OC3
    RPOR7bits.RP14R = 21;           // OC4
    //RPOR7bits.RP15R = 22;           // OC5
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
    
    T1CONbits.TON = 1;
    
    
    //OC1CON2 = 0x0000;
    //OCxCON2bits.OCINV lets you invert the output. 
    
//    OC2CON1 = 0x1006;
//    OC2CON2 = 0x0000;
//    OC3CON1 = 0x1006;
//    OC3CON2 = 0x0000;
//    OC4CON1 = 0x1006;
//    OC4CON2 = 0x0000;
    //OC5CON1 = 0x1006;
    //OC5CON2 = 0;
    
    OC1RS = 255;
    OC2RS = 255;
    OC3RS = 255;
    OC4RS = 255;
    //OC5RS = 255;

}


#ifdef	__cplusplus
}
#endif

#endif	/* DOMESHOW_H */

