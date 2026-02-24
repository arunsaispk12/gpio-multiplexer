# Wiring Guide

Pin assignments, ASCII wiring diagrams, and connection notes for all major ICs connected to ESP32.

> All diagrams assume **3.3 V operation** compatible with ESP32/ESP8266.
> GPIO pin numbers are examples — remap as needed for your board layout.

---

## 74HC4051 — 8-Channel Analog Mux

### Pinout (DIP-16)

```
          74HC4051
        ┌────────────┐
   Y4 ──┤ 1       16 ├── VCC (3.3 V)
   Y6 ──┤ 2       15 ├── Y2
   Y7 ──┤ 3       14 ├── Y1
   Y5 ──┤ 4       13 ├── Y0
   COM ─┤ 5       12 ├── Y3   ← Common I/O (to ADC/signal)
   INH ─┤ 6       11 ├── S0   ← Select 0
   VEE ─┤ 7       10 ├── S1   ← Select 1
   GND ─┤ 8        9 ├── S2   ← Select 2
        └────────────┘
```

> **VEE** = negative supply for bipolar operation. Tie to GND for single-supply (3.3 V) use.
> **INH** = inhibit, active high. Tie to GND to keep mux always enabled.

### ESP32 Connection

```
ESP32                74HC4051
──────               ──────────────────
GPIO4  ──────────── S0 (pin 11)
GPIO5  ──────────── S1 (pin 10)
GPIO6  ──────────── S2 (pin 9)
GND    ──────────── INH (pin 6)  ← disable inhibit
GND    ──────────── VEE (pin 7)  ← single supply
GND    ──────────── GND (pin 8)
3.3V   ──────────── VCC (pin 16)

ADC1_CH0 (GPIO36) ─ COM (pin 5)  ← shared ADC input

           Y0–Y7 (pins 13,14,15,12,1,4,2,3) → sensors/signals
```

### Schematic

```
3.3V ─────────────────────────────── VCC (pin 16)
                                        │
                                     [0.1µF]  ← decoupling cap
                                        │
GND ──────────────────────────────── GND (pin 8)
                                        │
GND ───────────────────────────────── VEE (pin 7)
GND ───────────────────────────────── INH (pin 6)

GPIO4 ──────────────────────────────  S0 (pin 11)
GPIO5 ──────────────────────────────  S1 (pin 10)
GPIO6 ──────────────────────────────  S2 (pin 9)

ADC pin ────────────────────────────  COM (pin 5)

             ┌───── Y0 → Sensor 0
             ├───── Y1 → Sensor 1
             ├───── Y2 → Sensor 2
        COM  ├───── Y3 → Sensor 3
    (shared) ├───── Y4 → Sensor 4
             ├───── Y5 → Sensor 5
             ├───── Y6 → Sensor 6
             └───── Y7 → Sensor 7
```

---

## CD74HC4067 — 16-Channel Analog Mux

### Pinout (DIP-24)

```
          CD74HC4067
        ┌────────────┐
   C0  ─┤ 1       24 ├── VCC (3.3 V)
   C1  ─┤ 2       23 ├── C15
   C2  ─┤ 3       22 ├── C14
   C3  ─┤ 4       21 ├── C13
   C4  ─┤ 5       20 ├── C12
   C5  ─┤ 6       19 ├── C11
   C6  ─┤ 7       18 ├── C10
   C7  ─┤ 8       17 ├── C9
   EN  ─┤ 9       16 ├── C8   ← Enable (active low)
   SIG ─┤ 10      15 ├── S3   ← Common signal
   GND ─┤ 11      14 ├── S2
   S0  ─┤ 12      13 ├── S1
        └────────────┘
```

### ESP32 Connection

```
ESP32                CD74HC4067
──────               ──────────────────
GPIO4  ──────────── S0 (pin 12)
GPIO5  ──────────── S1 (pin 13)
GPIO6  ──────────── S2 (pin 14)
GPIO7  ──────────── S3 (pin 15)
GND    ──────────── EN  (pin 9)   ← tie low to enable always
GND    ──────────── GND (pin 11)
3.3V   ──────────── VCC (pin 24)

ADC1_CH0 (GPIO36) ─ SIG (pin 10)

                    C0–C15 → up to 16 sensors
```

---

## MCP23017 — 16-bit I2C I/O Expander

### Pinout (DIP-28)

```
          MCP23017
        ┌────────────┐
  GPB0 ─┤ 1       28 ├── GPA7
  GPB1 ─┤ 2       27 ├── GPA6
  GPB2 ─┤ 3       26 ├── GPA5
  GPB3 ─┤ 4       25 ├── GPA4
  GPB4 ─┤ 5       24 ├── GPA3
  GPB5 ─┤ 6       23 ├── GPA2
  GPB6 ─┤ 7       22 ├── GPA1
  GPB7 ─┤ 8       21 ├── GPA0
  VCC  ─┤ 9       20 ├── INTA
  GND  ─┤ 10      19 ├── INTB
   CS  ─┤ 11      18 ├── RESET (active low)
  SCL  ─┤ 12      17 ├── A2
  SDA  ─┤ 13      16 ├── A1
  NC   ─┤ 14      15 ├── A0
        └────────────┘
```

### ESP32 Connection

```
ESP32                MCP23017
──────               ──────────────────
GPIO21 (SDA) ─────── SDA (pin 13)
GPIO22 (SCL) ─────── SCL (pin 12)
GND          ─────── GND (pin 10)
3.3V         ─────── VCC (pin 9)
3.3V         ─────── RESET (pin 18)  ← pull high = normal operation
GPIO34       ─────── INTA (pin 20)   ← optional interrupt
GPIO35       ─────── INTB (pin 19)   ← optional interrupt

Address pins (for address 0x20 = all low):
GND          ─────── A0 (pin 15)
GND          ─────── A1 (pin 16)
GND          ─────── A2 (pin 17)
```

### I2C Address Configuration

```
A2  A1  A0 | I2C Address
 0   0   0 | 0x20  ← default
 0   0   1 | 0x21
 0   1   0 | 0x22
 0   1   1 | 0x23
 1   0   0 | 0x24
 1   0   1 | 0x25
 1   1   0 | 0x26
 1   1   1 | 0x27
```

### Full Schematic

```
3.3V ──┬─────────────── VCC (pin 9)
       │                              MCP23017
      [4.7kΩ]◄── SDA pull-up
       │                       ┌──── GPA0 → LED/Button 0
3.3V ──┼───── SCL pull-up     │
      [4.7kΩ]                  ├──── GPA1 → LED/Button 1
       │                       │
GPIO21 ┤ SDA ──────────────── SDA (pin 13)
GPIO22 ┤ SCL ──────────────── SCL (pin 12)
                               │
GND ──────────────────────── GND (pin 10)
                               │
3.3V ─────────────────────── RESET (pin 18)  + [0.1µF to GND]
                               │
GND ──────────────────────── A0, A1, A2  (address = 0x20)
                               │
                              [0.1µF] bypass cap between VCC and GND
```

> **Always add 4.7 kΩ pull-up resistors on SDA and SCL to 3.3 V.**
> One pair per I2C bus is sufficient (not one per IC).

---

## PCF8574 — 8-bit I2C I/O Expander

### Pinout (DIP-16)

```
          PCF8574
        ┌────────────┐
   A0  ─┤ 1       16 ├── VCC (3.3 V)
   A1  ─┤ 2       15 ├── SDA
   A2  ─┤ 3       14 ├── SCL
   P0  ─┤ 4       13 ├── INT (active low)
   P1  ─┤ 5       12 ├── P7
   P2  ─┤ 6       11 ├── P6
   P3  ─┤ 7       10 ├── P5
   GND ─┤ 8        9 ├── P4
        └────────────┘
```

### ESP32 Connection

```
ESP32                PCF8574
──────               ──────────────────
GPIO21 (SDA) ─────── SDA (pin 15)
GPIO22 (SCL) ─────── SCL (pin 14)
GND          ─────── GND (pin 8)
3.3V         ─────── VCC (pin 16)
GPIO34       ─────── INT (pin 13)   ← optional

Address 0x20 (all low):
GND          ─────── A0 (pin 1)
GND          ─────── A1 (pin 2)
GND          ─────── A2 (pin 3)
```

---

## 74HC595 — 8-bit Shift Register (Output)

### Pinout (DIP-16)

```
          74HC595
        ┌────────────┐
   Q1  ─┤ 1       16 ├── VCC (3.3 V)
   Q2  ─┤ 2       15 ├── Q0
   Q3  ─┤ 3       14 ├── DS     ← Serial Data In
   Q4  ─┤ 4       13 ├── OE     ← Output Enable (active low)
   Q5  ─┤ 5       12 ├── RCLK   ← Latch (Storage Clock)
   Q6  ─┤ 6       11 ├── SRCLK  ← Shift Clock
   Q7  ─┤ 7       10 ├── SRCLR  ← Clear (active low)
   GND ─┤ 8        9 ├── Q7'    ← Serial Out (cascade)
        └────────────┘
```

### ESP32 Connection

```
ESP32                74HC595
──────               ──────────────────
GPIO14 ──────────── DS    (pin 14)  ← Data
GPIO13 ──────────── SRCLK (pin 11)  ← Clock
GPIO27 ──────────── RCLK  (pin 12)  ← Latch
GND    ──────────── OE    (pin 13)  ← tie to GND (always enable outputs)
3.3V   ──────────── SRCLR (pin 10)  ← tie to VCC (never clear)
GND    ──────────── GND   (pin 8)
3.3V   ──────────── VCC   (pin 16)

           Q0–Q7 → LEDs (with 330Ω resistors), relays, etc.
```

### Cascading Two 74HC595

```
ESP32         First 74HC595        Second 74HC595
──────        ─────────────        ──────────────
GPIO14 ──── DS (pin 14)
GPIO13 ──── SRCLK  ──────────────── SRCLK
GPIO27 ──── RCLK   ──────────────── RCLK
           Q7' (pin 9) ──────────── DS (pin 14)

Send 16 bits to control 16 outputs total.
```

---

## 74HC165 — 8-bit Shift Register (Input)

### Pinout (DIP-16)

```
          74HC165
        ┌────────────┐
   SL  ─┤ 1       16 ├── VCC
   CLK ─┤ 2       15 ├── CLK INH (tie to GND)
   E   ─┤ 3       14 ├── D
   F   ─┤ 4       13 ├── C
   G   ─┤ 5       12 ├── B
   H   ─┤ 6       11 ├── A
   QH' ─┤ 7       10 ├── SER   ← Serial in (cascade)
   GND ─┤ 8        9 ├── QH    ← Serial out
        └────────────┘
```

> **SL (PL)** = Parallel Load (active low). Pulse low to latch all inputs.
> **CLK INH** = Clock inhibit. Tie to GND to enable clocking.

### ESP32 Connection

```
ESP32                74HC165
──────               ──────────────────
GPIO14 ──────────── CLK (pin 2)
GPIO12 ──────────── QH  (pin 9)   ← read serial data
GPIO27 ──────────── SL  (pin 1)   ← pulse low to latch
GND    ──────────── CLK INH (pin 15)
GND    ──────────── SER (pin 10)  ← tie to GND (no cascade)
GND    ──────────── GND (pin 8)
3.3V   ──────────── VCC (pin 16)

           A–H (pins 11,12,13,14,3,4,5,6) → buttons/switches
           (use pull-up or pull-down resistors on inputs)
```

---

## TCA9548A — 8-Channel I2C Bus Mux

### Pinout (TSSOP-24)

```
        TCA9548A
      ┌────────────┐
 A0  ─┤ 1       24 ├── VCC
 A1  ─┤ 2       23 ├── SD0  ─► I2C Bus 0 SDA
 A2  ─┤ 3       22 ├── SC0  ─► I2C Bus 0 SCL
RESET─┤ 4       21 ├── SD1
 SDA ─┤ 5       20 ├── SC1
 SCL ─┤ 6       19 ├── SD2
 GND ─┤ 7       18 ├── SC2
      │             │
      │  ...        │   SD3–SD7, SC3–SC7
      └────────────┘
```

### ESP32 Connection

```
ESP32                TCA9548A
──────               ──────────────────
GPIO21 (SDA) ─────── SDA (pin 5)
GPIO22 (SCL) ─────── SCL (pin 6)
3.3V         ─────── VCC (pin 24)
GND          ─────── GND (pin 7)
3.3V         ─────── RESET (pin 4)  ← pull high = normal

Address 0x70 (all low):
GND          ─────── A0 (pin 1)
GND          ─────── A1 (pin 2)
GND          ─────── A2 (pin 3)

Sub-bus devices:
              SC0/SD0 ── Device 0 (SCL/SDA)
              SC1/SD1 ── Device 1 (SCL/SDA)
              ...
              SC7/SD7 ── Device 7 (SCL/SDA)
```

### Multi-Device Example (8× BME280 at same address)

```
ESP32 ──I2C──► TCA9548A (0x70)
               ├── SD0/SC0 ──► BME280 #0 (0x76)
               ├── SD1/SC1 ──► BME280 #1 (0x76)
               ├── SD2/SC2 ──► BME280 #2 (0x76)
               ├── SD3/SC3 ──► BME280 #3 (0x76)
               ├── SD4/SC4 ──► BME280 #4 (0x76)
               ├── SD5/SC5 ──► BME280 #5 (0x76)
               ├── SD6/SC6 ──► BME280 #6 (0x76)
               └── SD7/SC7 ──► BME280 #7 (0x76)

All 8 sensors share the same I2C address — no conflict!
```

---

## MCP23S17 — 16-bit SPI I/O Expander

### ESP32 SPI Connection

```
ESP32                MCP23S17
──────               ──────────────────
GPIO18 (CLK)  ────── SCK  (pin 1)
GPIO23 (MOSI) ────── SI   (pin 2)   ← SPI data in
GPIO19 (MISO) ────── SO   (pin 3)   ← SPI data out
GPIO5  (CS)   ────── CS   (pin 11)  ← chip select (active low)
3.3V          ────── VCC  (pin 9)
GND           ────── GND  (pin 10)
3.3V          ────── RESET (pin 18)

Address 0 (A1=0, A0=0):
GND           ────── A0   (pin 15)
GND           ────── A1   (pin 16)
```

> SPI opcode for MCP23S17: `0x40 | (address << 1)` for write, `0x41 | (address << 1)` for read.

---

## Decoupling Capacitors — Essential Rule

**Always place a 100 nF (0.1 µF) ceramic capacitor between VCC and GND of every IC, as close to the power pins as possible.**

```
VCC pin ──────┬──── to circuit VCC
             [0.1µF]
              │
GND pin ──────┴──── to circuit GND
```

For high-speed or noisy environments, add a 10 µF electrolytic in parallel.

---

## I2C Pull-up Resistors

Required on every I2C bus. One pair covers the whole bus.

```
3.3V ──┬──── [4.7kΩ] ──── SDA ──── all SDA pins
       └──── [4.7kΩ] ──── SCL ──── all SCL pins
```

| Bus Speed | Recommended Pull-up |
|---|---|
| 100 kHz (Standard) | 4.7 kΩ |
| 400 kHz (Fast) | 2.2 kΩ |
| 1 MHz (Fast-mode Plus) | 1 kΩ |

Stronger pull-up = faster rise time = supports higher speeds. But too low (< 1 kΩ) wastes current and may violate driver specs.
