/*
 * Dome Show Firmware
 * for Tesla Works
 * Authors: Ryan Fredlund, Katie Manderfeld, Ian Smith
 * Last updated: 3/2/2018
 * 
 */

#include "xc.h"
#include "stdint.h"
#include <p24Fxxxx.h>
#include "domeshow.h"
#include "crc16_xmodem.h"

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


// Addressing
uint8_t startChannel = BOARD_ADDRESS * BOARD_CHANNELS;

// Storage for channel values (make pointers for easier switching)
uint8_t currValues[PAYLOAD_SIZE];
uint8_t nextValues[PAYLOAD_SIZE];

// Ring buffer for received data
volatile uint8_t rxData[RX_BUFFER_SIZE];

// Managing the ring buffer
uint16_t head = 0;
volatile uint16_t tail;

// Keeping track of magic
uint8_t num_magic_found = 0;


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
    
    setupUART();
    
    setupPWM();
    
    // Set up arrays
    int i;
    for (i = 0; i < PAYLOAD_SIZE; i++) {
        currValues[i] = 255;
    }
    
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
    /*
    OC1R = channelValues[0];
    OC2R = channelValues[1];
    OC3R = channelValues[2];
    OC4R = channelValues[3];*/
    //OC5R = levels[4];
    
    //TODO:Quick/hacky fix... try only updating PWM every 3 UART frame receives
}

void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    //char temp;

    /*
    // interrupt old version
    _U1RXIF = 0;
    if (U1STAbits.FERR == 1) {      // Framing error, discard this byte   
        temp = (char) U1RXREG;      // Read receive register
        U1STAbits.FERR = 0;         // Don't save character anywhere
        
        asm("btg LATB, #5");        // Remove this when finished with dev
    }
    else {                          // No framing error, use this byte
        temp = (char) U1RXREG;      // Use this data. Implementation to come.
    }
    */

    // interrupt from pic18
    uint8_t rxByte;
    uint16_t t;

    if(_U1RXIE & _U1RXIF)
    {
        
        _U1RXIF=0; // Clear interrupt flag

        // Check for framing error
        if(U1STAbits.FERR)
        {
            rxByte = U1RXREG; // Clear framing error
            num_magic_found = 0;
            dscom_rx_state = DSCOM_STATE_WAITING; // Go to Waiting state on framing error
        } else if (U1STAbits.OERR) {
            // Clear overflow error and go to Waiting state
            rxByte = U1RXREG; // Clear framing error
            num_magic_found = 0;
            dscom_rx_state = DSCOM_STATE_WAITING; // Go to Waiting state on framing error
        } else {
            asm("btg LATB, #15");
            rxByte = (char) U1RXREG; //add cast to char????

//            _U1RXIE = 0; // Disable interrupts to set new tail value

            // Add byte to rxData ring buffer
            t = tail;
            rxData[t] = rxByte;
            t = (t + 1) & RX_BUFFER_SIZE;
            tail = t;
        
//            _U1RXIE = 1; // Re-enable interrupts
        }
    }

}

uint16_t get_tail() {
    _U1RXIE = 0; // Disable interrupts to read value
    uint16_t t = tail;
    _U1RXIE = 1; // Re-enable interrupts
    return t;
}

/*
 * Returns the number of bytes in the ring buffer that can be used currently.
 */
uint16_t bytes_available() {
    uint16_t t = get_tail();
    uint16_t available = t - head + 1;
    if (t < head) { // Looped around ring buffer
        available = RX_BUFFER_SIZE - head + t + 1;
    }
    return available;
}

/*
 * Reads in the next byte from the ring buffer and properly increments head.
 * Assumes that there is at least one byte to read
 */
__inline uint8_t read_byte() {
    _U1RXIE = 0; // Disable interrupts to read value
    uint8_t byte = rxData[head];
    _U1RXIE = 1; // Re-enable interrupts
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

    _U1RXIE = 0; // Disable interrupts to read values
    while (i < length) {
        // Read
        nextValues[i] = rxData[head];
        
        // Update head and i
        head = (head + 1) & RX_BUFFER_SIZE;
        i++;
    }
    _U1RXIE = 1; // Re-enable interrupts
}

/*
 * Write the signal
 */
void write() {
    OC1R = currValues[0];
    OC2R = currValues[1];
    OC3R = currValues[2];
    OC4R = currValues[3];
}

int main(void) {
    setup();
    
    uint8_t rxByte;
    uint8_t i;
    uint16_t num_bytes;
    uint16_t readCrc;
    uint16_t calculatedCrc;
    
    while(1) {
        // Write to output each time
        write();
        
        switch (dscom_rx_state) {
            case DSCOM_STATE_WAITING:
                // Wait for magic bytes
                num_bytes = bytes_available();
                if (num_bytes > 0) {
                    // If we have a byte, check it
                    rxByte = read_byte();
                    if (rxByte == magic[0]) {
                        num_magic_found += 1;
                        dscom_rx_state = DSCOM_STATE_MAGIC;
                    }
                }
                
                break;
            case DSCOM_STATE_MAGIC:
                // Wait for magic bytes
                num_bytes = bytes_available();
                if (num_bytes > 0) {
                    rxByte = read_byte();
                    if (rxByte == magic[num_magic_found]) {
                        num_magic_found++;
                        // If all magic found, move on
                        if (num_magic_found == 6) {
                            dscom_rx_state = DSCOM_STATE_READING;
                            num_magic_found = 0;
                        }
                    } else {
                        // Not a magic sequence. Start over
                        num_magic_found = 0;
                        dscom_rx_state = DSCOM_STATE_WAITING;
                    }
                }
                
                break;
            case DSCOM_STATE_READING:
                // Decode packet (most of the data)
                if (bytes_available() >= PAYLOAD_SIZE) {
                    // Load data into "next" array
                    read_packet(PAYLOAD_SIZE);
                    
                    // Process packet
                    dscom_rx_state = DSCOM_STATE_PROCESSING;
                }
                
                break;
            case DSCOM_STATE_PROCESSING:
                // Verify using XMODEM 16 CRC
                readCrc = read_two_bytes();
                calculatedCrc = crc16xmodem(nextValues, PAYLOAD_SIZE);

                // Check for valid packet (no corruption)
                if (readCrc == calculatedCrc) {

                    // Switch current and next arrays
                    // TODO: do this by switching pointers (no copying)
                    for (i = 0; i < PAYLOAD_SIZE; i++) {
                        currValues[i] = nextValues[i];
                    }
                }

                // Go back to waiting for next packet
                dscom_rx_state = DSCOM_STATE_WAITING;

                break;
            default:
                break;
        }
    }
    return 0;
}
