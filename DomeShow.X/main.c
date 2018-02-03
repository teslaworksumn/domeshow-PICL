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
#include "stdint.h"
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

#define BOARD_ADDRESS 0
#define BOARD_CHANNELS 15
#define MAX_PAYLOAD_SIZE 120
#define RX_BUFFER_SIZE 0x03ff // 1024

// State of the domeshow RX
typedef enum {
    DSCOM_STATE_READY,
    DSCOM_STATE_PRE_PROCESSING,
    DSCOM_STATE_PROCESSING
} DSCOM_RX_STATE_t;

uint8_t startChannel = BOARD_ADDRESS * BOARD_CHANNELS;
uint8_t channelValues[MAX_PAYLOAD_SIZE];
DSCOM_RX_STATE_t dscom_rx_state = DSCOM_STATE_READY;
volatile uint8_t rxData[RX_BUFFER_SIZE];
uint16_t head = 0;
volatile uint16_t tail;
unsigned char crc_start = 0;
unsigned char crc_end = 0;
uint8_t num_magic_found = 0;
uint8_t magic[4] = {0xDE, 0xAD, 0xBE, 0xEF};

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
    
    return 0;
}

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    char temp;

    _U1RXIF = 0;
    if (U1STAbits.FERR == 1) {      // Framing error, discard this byte   
        temp = (char) U1RXREG;      // Read receive register
        U1STAbits.FERR = 0;         // Don't save character anywhere
        
        asm("btg LATB, #5");        // Remove this when finished with dev
    }
    else {                          // No framing error, use this byte
        temp = (char) U1RXREG;      // Use this data. Implementation to come.
    }
}

/*
 * interrupt from pic18
    uint8_t rxByte;
    uint16_t t;

    if(RC1IE & RC1IF)
    {
        RC1IF=0; // Clear interrupt flag

        // Check for framing error
        if(RCSTA1bits.FERR)
        {
            rxByte = RCREG1; // Clear framing error
        } else if (RCSTA1bits.OERR) {
            // clear error
        } else {
            rxByte = RCREG1;
            t = tail;
            rxData[t] = rxByte;
            t = (t + 1) & RX_BUFFER_SIZE;
            tail = t;
        }
    }
 */

uint16_t get_tail() {
    RC1IE = 0; // Disable interrupts to read value
    uint16_t t = tail;
    RC1IE = 1; // Re-enable interrupts
    return t;
}

/*
 * Returns the number of bytes in the ring buffer that can be used currently.
 * Intentionally off by one to account for race conditions with interrupts
 * (giving a one byte buffer between head and tail)
 */
uint16_t bytes_available() {
    uint16_t t = get_tail();
    uint16_t available = t - head;
    if (t < head) { // Looped around ring buffer
        available = RX_BUFFER_SIZE - head + t;
    }
    return available;
}

/*
 * Reads in the next byte from the ring buffer and properly increments head.
 * Assumes that there is at least one byte to read
 */
__inline uint8_t read_byte() {
    RC1IE = 0; // Disable interrupts to read value
    uint8_t byte = rxData[head];
    RC1IE = 1; // Re-enable interrupts
    head = (head + 1) & RX_BUFFER_SIZE;
    return byte;
}

/*
 * Reads the next two bytes from the ring buffer as a uint16.
 * Assumes that there are at least two bytes to read
 */
__inline uint16_t read_two_bytes() {
    uint16_t highByte = read_byte();
    uint16_t lowByte = read_byte();
    return (highByte << 8) | lowByte;
}

void read_packet(uint16_t length) {
    unsigned int i = 0;
    while (i < length) {
        channelValues[i] = read_byte();
        i++;
    }
}

/*
 * Write the signal
 */
void write() {
    return; // TODO
}

int main(void) {
    setup();
    
    uint8_t rxByte;
    uint16_t length;
    uint16_t num_bytes;
    
    while(1) {
        switch (dscom_rx_state) {
            case DSCOM_STATE_READY:
                // Wait for magic bytes
                num_bytes = bytes_available();
                if (num_bytes > 0) {
                    rxByte = read_byte();
                    if (rxByte == magic[num_magic_found]) {
                        num_magic_found++;
                        // If all magic found, move on
                        if (num_magic_found == 4) {
                            dscom_rx_state = DSCOM_STATE_PRE_PROCESSING;
                            num_magic_found = 0;
                        }
                    } else {
                        // Not a magic sequence. Start over
                        num_magic_found = 0;
                    }
                }
                break;
            case DSCOM_STATE_PRE_PROCESSING:
                // Decode length (two bytes)
                if (bytes_available() >= 2) {
                    length = read_two_bytes();
                    // Check for invalid length
                    if (length > MAX_PAYLOAD_SIZE) {
                        dscom_rx_state = DSCOM_STATE_READY;
                    } else {
                        dscom_rx_state = DSCOM_STATE_PROCESSING;
                    }
                }
                break;
            case DSCOM_STATE_PROCESSING:
                // Decode packet (most of the data)
                if (bytes_available() >= length) {
                    // Load data into "ready" array
                    read_packet(length);
                    
                    // Verify using XMODEM 16 CRC
                    uint16_t readCrc = read_two_bytes();
//                    uint16_t calculatedCrc = crc16xmodem(channelValues, length);
//                    if (readCrc == calculatedCrc) {
//                        // Valid packet (no corruption)
                        write();
//                    }
                    dscom_rx_state = DSCOM_STATE_READY;
                }
                break;
            default:
                break;
        }
    }
    return 0;
}
