# GPIO Multiplexer Types

A breakdown of every major GPIO multiplexer and I/O expander category with key specs, operating principles, and IC examples.

---

## Category 1 — Analog Multiplexers

Analog muxes route one of N analog (or digital) input signals to a single output. The MCU selects which channel is active by setting address pins. Only one channel is connected at a time.

**Key property:** bidirectional — they can mux inputs OR outputs.

### 74HC4051

8-channel single-ended analog multiplexer/demultiplexer.

| Property | Value |
|---|---|
| Channels | 8 (1 of 8) |
| Supply voltage | 2 V – 6 V (use 3.3 V for ESP32) |
| On-resistance (Ron) | ~70 Ω @ 3.3 V |
| Signal bandwidth | ~200 MHz |
| Control pins | 3 (S0, S1, S2) + INH (inhibit) |
| Package | DIP-16, SOIC-16, TSSOP-16 |
| Bidirectional | Yes |

**Channel selection truth table:**

```
S2  S1  S0 | Active Channel
 0   0   0 | Y0
 0   0   1 | Y1
 0   1   0 | Y2
 0   1   1 | Y3
 1   0   0 | Y4
 1   0   1 | Y5
 1   1   0 | Y6
 1   1   1 | Y7
```

**Typical use:** 8 analog sensors → 1 ADC pin, 8 analog outputs from 1 DAC pin.

---

### 74HC4052

Dual 4-channel analog multiplexer/demultiplexer.

| Property | Value |
|---|---|
| Channels | 2 independent banks × 4 channels |
| Supply voltage | 2 V – 6 V |
| Control pins | 2 (S0, S1) shared between both banks |
| Package | DIP-16, SOIC-16 |

**Typical use:** Stereo audio routing, differential signal muxing.

---

### 74HC4053

Triple 2-channel analog multiplexer/demultiplexer.

| Property | Value |
|---|---|
| Channels | 3 independent banks × 2 channels |
| Supply voltage | 2 V – 6 V |
| Control pins | 3 (one per bank) |
| Package | DIP-16, SOIC-16 |

**Typical use:** Signal source switching, I2S bus routing.

---

### CD74HC4067 {#cd74hc4067}

16-channel single-ended analog multiplexer/demultiplexer.

| Property | Value |
|---|---|
| Channels | 16 (1 of 16) |
| Supply voltage | 2 V – 6 V |
| On-resistance (Ron) | ~80 Ω @ 3.3 V |
| Control pins | 4 (S0–S3) + EN (enable, active low) |
| Package | DIP-24, SOIC-24, SSOP-24 |
| Bidirectional | Yes |

**Channel selection (partial):**

```
S3  S2  S1  S0 | Active Channel
 0   0   0   0 | C0
 0   0   0   1 | C1
 ...           |
 1   1   1   1 | C15
```

**Typical use:** 16 analog sensors → 1 ADC pin. Most popular choice when you need > 8 channels.

> **Note:** Available as a breakout board (SparkFun BOB-09056, similar clones) — easy to prototype with.

---

## Category 2 — I2C I/O Expanders

These ICs add GPIO pins over a 2-wire I2C bus. Each pin can independently be configured as input or output. Multiple expanders can share the same I2C bus (up to address limit).

### PCF8574 {#pcf8574}

8-bit I2C I/O expander.

| Property | Value |
|---|---|
| GPIO pins | 8 |
| Supply voltage | 2.5 V – 6 V |
| I2C speed | up to 100 kHz (Standard mode) |
| I2C addresses | 8 (A0–A2 pins), 0x20–0x27 |
| Interrupt pin | Yes (INT, active low) |
| Current per pin | 25 mA sink, 1 mA source |
| Package | DIP-16, SOIC-16, SSOP-16 |

> **Important:** PCF8574 pins are quasi-bidirectional — write 1 to use as input, write 0 to drive low. No push-pull output mode. Low source current (1 mA) is a significant limitation.

**Typical use:** Keypad scanning, status LEDs (with resistors), relay control (via transistor).

**PCF8574A variant:** Same IC, different I2C address range (0x38–0x3F), allowing 16 total expanders on one bus.

---

### MCP23017 {#mcp23017}

16-bit I2C I/O expander — the most capable general-purpose expander.

| Property | Value |
|---|---|
| GPIO pins | 16 (Port A: GPA0–GPA7, Port B: GPB0–GPB7) |
| Supply voltage | 1.8 V – 5.5 V |
| I2C speed | up to 1.7 MHz (Fast-mode Plus) |
| I2C addresses | 8 (A0–A2 pins), 0x20–0x27 |
| Interrupt pins | 2 (INTA, INTB — one per port) |
| Internal pull-ups | Yes (100 kΩ, configurable per pin) |
| Current per pin | 25 mA source and sink |
| Package | DIP-28, SOIC-28, SSOP-28, QFN-28 |

**Key registers:**

| Register | Function |
|---|---|
| IODIRA/B | Direction (1=input, 0=output) |
| GPPUA/B | Pull-up enable (1=enable 100kΩ) |
| GPIOA/B | Read input / write output |
| OLATA/B | Output latch (set without reading) |
| GPINTENA/B | Interrupt-on-change enable |
| INTCONA/B | Interrupt compare (vs previous or vs DEFVAL) |
| DEFVALA/B | Default compare value for interrupts |

**Typical use:** Button matrices, relay banks, LED arrays, general GPIO expansion — the go-to expander for most projects.

---

### MCP23008

8-bit version of MCP23017. Same register map, single port (GPA0–GPA7).

| Property | Value |
|---|---|
| GPIO pins | 8 |
| I2C addresses | 8 (0x20–0x27) |
| Package | DIP-18, SOIC-18 |

---

## Category 3 — SPI I/O Expanders

### MCP23S17 {#mcp23s17}

16-bit SPI I/O expander — SPI counterpart to MCP23017, identical register map.

| Property | Value |
|---|---|
| GPIO pins | 16 |
| Supply voltage | 1.8 V – 5.5 V |
| SPI speed | up to 10 MHz |
| SPI addresses | 4 (A0–A1 pins, hardware address bits) |
| Interrupt pins | 2 (INTA, INTB) |
| Internal pull-ups | Yes |
| Package | DIP-28, SOIC-28 |

**Advantage over I2C version:** Higher bus speed (10 MHz vs 1.7 MHz) and daisy-chainable on SPI bus. Better for real-time applications.

**Typical use:** Same as MCP23017, preferred when I2C bus is already congested or faster response needed.

---

## Category 4 — Shift Registers

Shift registers extend outputs or inputs serially. They are not true multiplexers but are the most efficient way to drive many digital outputs (LEDs, relays) or scan many digital inputs (buttons).

### 74HC595 {#74hc595}

8-bit serial-in, parallel-out shift register (output expansion).

| Property | Value |
|---|---|
| Output pins | 8 (Q0–Q7) |
| Supply voltage | 2 V – 6 V |
| Output current | 35 mA per pin, 70 mA total |
| SPI pins needed | 3 (CLK, DATA, LATCH) + optional OE |
| Daisy-chainable | Yes (cascade via Q7' pin) |
| Package | DIP-16, SOIC-16 |

**Operation:**
1. Shift 8 bits in on DATA pin, clocked by CLK.
2. Pulse LATCH (RCLK) to transfer shift register → output latches.
3. All 8 outputs update simultaneously.

**Cascading:** Connect Q7' of first 595 to DATA of second 595. Send 16 bits for 16 outputs (2 × 595), and so on.

**Typical use:** 8/16/24+ LED drivers, relay banks, 7-segment display driving.

---

### 74HC165 {#74hc165}

8-bit parallel-in, serial-out shift register (input expansion).

| Property | Value |
|---|---|
| Input pins | 8 (A–H) |
| Supply voltage | 2 V – 6 V |
| SPI pins needed | 3 (CLK, DATA, LOAD) |
| Daisy-chainable | Yes |
| Package | DIP-16, SOIC-16 |

**Operation:**
1. Pulse LOAD (PL, active low) to latch all 8 inputs simultaneously.
2. Clock out 8 bits on Qh (serial output).

**Typical use:** 8/16/24+ button or switch scanning.

---

## Category 5 — I2C Bus Multiplexers

Used when multiple I2C devices share the same 7-bit address and cannot be wired to the same bus simultaneously.

### TCA9548A {#tca9548a}

8-channel I2C bus switch/multiplexer.

| Property | Value |
|---|---|
| Sub-channels | 8 independent I2C buses |
| Supply voltage | 1.65 V – 5.5 V |
| I2C speed | up to 400 kHz per channel |
| I2C addresses | 8 (A0–A2 pins), 0x70–0x77 |
| Level translation | Yes (supports mixed-voltage buses) |
| Reset pin | Yes (active low) |
| Package | TSSOP-24, VQFN-24 |

**Operation:** Write a single byte to the TCA9548A to select which sub-channel(s) are active. Multiple channels can be enabled simultaneously.

**Control byte:**

```
Bit 7 6 5 4 3 2 1 0
    |               |
    SC7             SC0   (1 = enable that sub-channel)
```

**Typical use:** Multiple identical sensors (BME280, SSD1306 OLEDs, VL53L0X, etc.) on the same board.

---

### PCA9547

Single-channel I2C mux (only one channel active at a time, unlike TCA9548A which allows multiple).

| Property | Value |
|---|---|
| Sub-channels | 8 |
| I2C addresses | 8 (0x70–0x77) |
| Active channels | 1 at a time |
| Package | SOIC-16, TSSOP-16 |

---

## Category 6 — Analog Switch ICs (Single Gate)

For simpler routing tasks — single or dual channel analog/digital switches.

### TS5A3157 / TS5A23157

Single/dual 1:2 analog switch. SPDT (single-pole double-throw).

| Property | Value |
|---|---|
| Channels | 1 (3157) or 2 (23157) |
| Ron | ~1 Ω |
| Supply voltage | 1.65 V – 5.5 V |
| Control | 1 pin per switch |

**Typical use:** USB switch, audio signal routing, power path selection.

---

## Summary Table

| IC | Type | Channels/Pins | Interface | VCC Range | Key Limitation |
|---|---|---|---|---|---|
| 74HC4051 | Analog mux | 8 ch | 3 GPIO | 2–6 V | One channel at a time |
| 74HC4052 | Analog mux | 2×4 ch | 2 GPIO | 2–6 V | Shared select |
| 74HC4053 | Analog mux | 3×2 ch | 3 GPIO | 2–6 V | Only 2 per bank |
| CD74HC4067 | Analog mux | 16 ch | 4 GPIO | 2–6 V | One channel at a time |
| PCF8574 | I2C expander | 8 pins | I2C | 2.5–6 V | 1 mA source only |
| MCP23017 | I2C expander | 16 pins | I2C | 1.8–5.5 V | 400 kHz std speed |
| MCP23008 | I2C expander | 8 pins | I2C | 1.8–5.5 V | — |
| MCP23S17 | SPI expander | 16 pins | SPI | 1.8–5.5 V | Needs CS pin |
| 74HC595 | Shift reg | 8 out | SPI-like | 2–6 V | Output only |
| 74HC165 | Shift reg | 8 in | SPI-like | 2–6 V | Input only |
| TCA9548A | I2C bus mux | 8 buses | I2C | 1.65–5.5 V | 400 kHz per channel |
| PCA9547 | I2C bus mux | 8 buses | I2C | 2.3–5.5 V | 1 channel at a time |
