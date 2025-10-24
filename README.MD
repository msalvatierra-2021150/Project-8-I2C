# MSP430G2553 ↔ AT24C16 I²C EEPROM

A compact reference and quick-start guide for the EE3123 Project 8 implementation of I²C using the MSP430G2553 (master) to read bytes from an AT24C16 EEPROM (slave).

---

## Overview

This project demonstrates **five incremental I²C steps** on the MSP430G2553 using the USCI_B0 peripheral to communicate with an **AT24C16** EEPROM:

1. **START + Device Address (Write)** → ACK → STOP
2. **Write Word Address (0xE0)** to set the internal pointer
3. **Repeated START + Device Address (Read)** (random read setup)
4. **Read the next byte** (current-address read at 0xE1)
5. **Read three consecutive bytes** (0xE0, 0xE1, 0xE2) using sequential read with auto-increment

All transactions were **validated on the oscilloscope**, confirming correct START/STOP timing, address phases, and ACK/NACK behavior.

> **Expected demo output:** the three EEPROM bytes at 0xE0..0xE2 decode to ASCII **`S`, `i`, `!`** (message: “**Si!**”).

---

## Hardware

* **MCU:** TI MSP430G2553 (LaunchPad)
* **EEPROM:** Microchip/Atmel AT24C16 (I²C, 16 Kbit)
* **Bus:** I²C @ ~100 kHz
* **Lines:** P1.6 = SCL, P1.7 = SDA
* **Pull‑ups:** 4.7 kΩ–10 kΩ recommended on SDA, SCL
* **Power:** 3.3 V (typ.)

> The 7‑bit slave address used is **`0x50`** (A2..A0 = 000). Word address **0xE0** is the first location read; device auto‑increments to **0xE1**, **0xE2** during sequential reads.

---

## Firmware Highlights

**Clocking & Bit Rate**

* SMCLK assumed ≈ **1 MHz** (calibrated DCO).
* I²C speed set via **`UCB0BR1:BR0 = 0:10`** → **~100 kHz** SCL.

**Core helpers (USCI_B0):**

```c
void i2c_init(void);
int  i2c_start_write_then_wait_addr_ack(void);
int  i2c_send_byte(uint8_t b);
int  i2c_repeated_start_read_then_wait_addr_ack(void);
uint8_t i2c_rx_next(void);
static uint8_t i2c_rx_last_with_stop(void);
int  eeprom_read_n(uint8_t word_addr, uint8_t *buf, uint8_t n);
```

**Key patterns:**

* **Random Read:** START + Addr(W) → WordAddr → Repeated START → Addr(R) → read N bytes → NACK+STOP on last byte.
* **Current-Address Read:** If a prior read left the pointer at `X`, a new read fetches from `X` (no word-addr write needed).

---

## Build & Flash

### Option A — Code Composer Studio (CCS)

1. Create an MSP430G2553 project (C), add `src/` files.
2. Set target device to **MSP430G2553**.
3. Build & debug; use on‑board eZ‑FET.

### Option B — msp430-gcc (CLI)

```bash
# Example commands (adapt to your paths/toolchain)
msp430-gcc -mmcu=msp430g2553 -Os -o main.elf src/*.c
mspdebug rf2500 "prog main.elf"
```

---

## Wiring

* **P1.6 (SCL)** → EEPROM SCL
* **P1.7 (SDA)** → EEPROM SDA
* **VCC** → 3.3 V
* **GND** → GND
* **SDA/SCL pull‑ups** → 3.3 V

> Ensure common ground between LaunchPad and EEPROM board.

---

## How to Run the 5 Steps

Each step can be toggled via `main()` by calling one function at a time.

1. **Step 1:** `step1_start_addr();`
   Sends START + `0x50(W)`; expects ACK; issues STOP.
2. **Step 2:** `step2_send_word_addr(0xE0);`
   Writes word address **0xE0** (sets EEPROM pointer), then STOP.
3. **Step 3:** `step3_rs_addr_read();`
   Does Random-Read setup: write 0xE0, repeated START, `0x50(R)`, then STOP.
4. **Step 4:** `step4_read_second_byte();`
   Performs a 2‑byte read starting at 0xE0 → reads 0xE0, 0xE1.
5. **Step 5:** `step5_read_second_byte();`
   Performs a 3‑byte sequential read → 0xE0, 0xE1, 0xE2.

> On the **last** byte, the master sets **`UCTXSTP` before reading** to cause **NACK + STOP** after the byte arrives (per USCI sequence).

---

## Validation on the Oscilloscope (What to Look For)

* **START:** SDA falls while SCL high.
* **Address Frame:** `0x50` + R/W bit; **ACK** as SDA low from slave on 9th clock.
* **Word Address Byte:** `0xE0` on SDA MSB→LSB; ACK low from EEPROM.
* **Repeated START:** Another START without a prior STOP.
* **Data Bytes:** Each 8‑bit frame followed by ACK/NACK from master.
* **STOP:** SDA rises while SCL high.

If lines latch **low**, power‑cycle or reset the MCU to free the bus (some slaves hold SDA when mid‑transaction).

---

## Troubleshooting

* **No ACK on address:** Verify **pull‑ups**, wiring, and correct **7‑bit address (0x50)** with R/W bit added by hardware.
* **Wrong data:** Confirm **word address** loaded (Step 2) before read; mind EEPROM **write‑cycle times** if you wrote earlier.
* **Bus stuck low:** Reset MCU and/or power‑cycle slave; check for shorted pull‑ups.
* **I²C too fast/slow:** Adjust `UCB0BR0/BR1` (e.g., BR0=10 for ≈100 kHz at SMCLK=1 MHz).
* **Last byte behavior:** Ensure **`UCTXSTP` is set before reading** final byte to generate **NACK+STOP**.

---

## FAQ

**Q: Is read = 1 and write = 0?**
A: Yes. The USCI appends the R/W bit to the 7‑bit address when you choose **transmitter (write)** or **receiver (read)** mode.

**Q: Do I need a STOP between pointer set and read?**
A: Not for **random read**—use a **repeated START** directly after writing the word address.

**Q: Why don’t I see 0xE0 on the bus during the address phase?**
A: `0xE0` is a **data** byte (word address), not the **device address**. The device address is `0x50` with the R/W bit added by hardware.

---

## Acknowledgments

* **Grace examples 12.19 & 12.21** used conceptually for reference.

---

## License

This reference is for academic use in EE3123. Adapt as needed for coursework.

---

## References

* Unsalan, C., & Gurhan, H. D. (2013). *Programmable Microcontrollers with Applications: MSP430 LaunchPad with CCS and Grace*. McGraw‑Hill. (Examples 12.19 & 12.21.)
