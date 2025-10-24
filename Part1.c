#include <msp430.h>
#include <stdint.h>

#define I2C_DEV7_AT24C16   0x50      // AT24C16, block/page 000 -> 0b1010_000 = 0x50

void i2c_init(void) {
    // Select I2C function on P1.6/1.7
    P1SEL  |= BIT6 | BIT7;
    P1SEL2 |= BIT6 | BIT7;

    UCB0CTL1 = UCSWRST;                    // hold USCI_B0 in reset
    UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;  // Master, I2C, synchronous
    UCB0CTL1 = UCSWRST + UCSSEL_2;         // SMCLK

    UCB0BR0 = 10;
    UCB0BR1 = 0;

    UCB0I2CSA = I2C_DEV7_AT24C16;          // 7-bit slave address
    UCB0CTL1 &= ~UCSWRST;                  // release for operation
}

int i2c_start_write_then_wait_addr_ack(void) {
    // Wait until address phase finishes or a NACK shows up
    while (UCB0CTL1 & UCTXSTT);
    UCB0CTL1 |= UCTR + UCTXSTT;        // Transmitter mode + START
    return 0;
}

int i2c_send_byte(uint8_t b) {
    while (!(IFG2 & UCB0TXIFG));
    UCB0TXBUF = b;
    // wait for the byte shift (TXIFG set again for next byte)
    while (!(IFG2 & UCB0TXIFG));
    return 0;
}

int i2c_repeated_start_read_then_wait_addr_ack(void) {
    UCB0CTL1 &= ~UCTR;    // Receiver mode
    UCB0CTL1 |= UCTXSTT;  // Repeated START
    // Wait for address to complete (START cleared)
    while (UCB0CTL1 & UCTXSTT);
    return 0;
}

uint8_t i2c_rx_one_with_stop(void) {
    //For single-byte read: set STOP immediately after START sent completes
    UCB0CTL1 |= UCTXSTP;                 // NACK last byte + STOP after it arrives
    while (!(IFG2 & UCB0RXIFG)) ;        // wait for the byte
    return UCB0RXBUF;
}

uint8_t i2c_rx_next(void) {
    while (!(IFG2 & UCB0RXIFG)) ;
    return UCB0RXBUF;
}

uint8_t i2c_rx_last_with_stop(void) {
    //Generate NACK+STOP BEFORE reading the last byte
    UCB0CTL1 |= UCTXSTP;
    while (!(IFG2 & UCB0RXIFG)) ;
    return UCB0RXBUF;
}

int eeprom_read_n(uint8_t word_addr, uint8_t *buf, uint8_t n) {
    //Phase 1: START + Write + WordAddr
    if (i2c_start_write_then_wait_addr_ack() != 0) return -1;
    if (i2c_send_byte(word_addr) != 0) return -2;
    // Phase 2: REPEATED START + Read
    if (i2c_repeated_start_read_then_wait_addr_ack() != 0) return -3;

    // Read N-1 bytes with ACK (USCI auto-ACKs intermediate bytes)
    int i = 0;
    for (i = 0; i < n - 1; ++i) {
        buf[i] = i2c_rx_next();
    }
    // Last byte: NACK + STOP
    buf[n - 1] = i2c_rx_last_with_stop();
    while (UCB0CTL1 & UCTXSTP) ; // ensure STOP sent

    return 0;
}

void step1_start_addr(void) {
    i2c_start_write_then_wait_addr_ack();  // START + DevAddr(W)
    UCB0CTL1 |= UCTXSTP;                         // STOP to close it out
    while (UCB0CTL1 & UCTXSTP) ;
}

void step2_send_word_addr(uint8_t word) {
    if (i2c_start_write_then_wait_addr_ack() == 0 && i2c_send_byte(word) == 0) {
        UCB0CTL1 |= UCTXSTP;
        while (UCB0CTL1 & UCTXSTP) ;
    }
}

void step3_rs_addr_read(void) {
    if (i2c_start_write_then_wait_addr_ack() == 0) {
        i2c_send_byte(0xE0);               // prime a word address
        i2c_repeated_start_read_then_wait_addr_ack(); // repeated START + Addr(R)
        UCB0CTL1 |= UCTXSTP;                     // close without reading
        while (UCB0CTL1 & UCTXSTP) ;
    }
}

void step4_read_second_byte(void) {
    uint8_t buf[3];
    eeprom_read_n(0xE0, buf, 2); // read 0xE0, 0xE1
}

void step5_read_second_byte(void) {
    uint8_t buf[3];
    eeprom_read_n(0xE0, buf, 3); // read 0xE0, 0xE1, 0XE2
}

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    i2c_init();
    // --- STEP 1: START + Device Address (Write) + ACK
    // step1_start_addr();
    // --- STEP 2: + Word Address byte (0xE0)
    // step2_send_word_addr(0xE0);
    // // --- STEP 3: Repeated START + Device Address (Read) + first byte 
    // step3_rs_addr_read();
    // --- STEP 4: Read next byte 
    // step4_read_second_byte();
    // // --- STEP 5: Read next byte 
    step5_read_second_byte();
    while (1);
}
