# GPIO Multiplexer Comparison & Best Picks

Use this guide to quickly find the right IC for your application.

---

## Master Comparison Table

| IC | Channels | Interface | ESP32 Pins Used | Speed | VCC | Price | Complexity |
|---|---|---|---|---|---|---|---|
| **74HC4051** | 8 analog | 3 GPIO | 3–4 | Fast | 3.3 V | Very low | Low |
| **74HC4052** | 2×4 analog | 2 GPIO | 2–3 | Fast | 3.3 V | Very low | Low |
| **74HC4053** | 3×2 analog | 3 GPIO | 3–4 | Fast | 3.3 V | Very low | Low |
| **CD74HC4067** | 16 analog | 4 GPIO | 4–5 | Fast | 3.3 V | Low | Low |
| **PCF8574** | 8 digital | I2C | 2 (shared) | Slow (100 kHz) | 3.3–5 V | Very low | Low |
| **MCP23017** | 16 digital | I2C | 2 (shared) | Medium (1.7 MHz) | 3.3 V | Low | Medium |
| **MCP23008** | 8 digital | I2C | 2 (shared) | Medium (1.7 MHz) | 3.3 V | Low | Medium |
| **MCP23S17** | 16 digital | SPI | 4 (shared) | Fast (10 MHz) | 3.3 V | Low | Medium |
| **74HC595** | 8 out | SPI-like | 3 (shared) | Very fast | 3.3 V | Very low | Low |
| **74HC165** | 8 in | SPI-like | 3 (shared) | Very fast | 3.3 V | Very low | Low |
| **TCA9548A** | 8 I2C buses | I2C | 2 (shared) | Medium | 3.3 V | Low | Low |
| **PCA9547** | 8 I2C buses | I2C | 2 (shared) | Medium | 3.3 V | Low | Low |

> **ESP32 pins used** = dedicated pins needed (shared means multiple ICs share same bus pins).

---

## Best Pick by Use Case

### "I want to read more than 8 analog sensors"

**Winner: CD74HC4067**

- 16 channels, only 4 ESP32 GPIO needed
- Bidirectional — can also mux analog outputs
- Widely available, cheap breakout boards exist
- Cascadable: two CD74HC4067 + 1 demux = 32 channels

**Runner-up: 74HC4051 × 2** — if you need separate channel grouping

---

### "I want to read 4–8 analog sensors"

**Winner: 74HC4051**

- 8 channels, 3 GPIO, extremely simple
- Very low cost (<$0.10 per IC in quantity)
- Ron ~70 Ω is fine for most sensor applications

---

### "I want to expand digital outputs (LEDs, relays)"

**Winner for simplicity: 74HC595**

- 3 shared SPI pins drive unlimited outputs (cascade)
- Very fast update (SPI up to 10+ MHz)
- Low cost, universally available
- Limitation: output only, no feedback/read

**Winner for full control: MCP23017**

- 16 bidirectional pins with read and write
- Internal pull-ups, interrupt capability
- 8 expanders on one I2C bus = 128 pins
- Best choice when you need to also read state back

---

### "I want to read many digital inputs (buttons, switches)"

**Winner for bulk scanning: 74HC165**

- 3 shared SPI pins read unlimited inputs (cascade)
- Fastest update rate of any option
- Can combine with 74HC595 on same SPI bus

**Winner for interrupt-driven: MCP23017**

- Interrupt pins (INTA/INTB) notify ESP32 of changes
- No need to poll continuously
- Internal pull-ups eliminate external resistors
- Best for keyboards, alarm inputs, event-driven buttons

---

### "I need both inputs and outputs, minimal pins"

**Winner: MCP23017**

- 16 independently configurable pins
- 2 I2C pins shared across up to 8 expanders
- Interrupt support on both ports
- With 8× MCP23017: 128 bidirectional GPIO on 2 pins + interrupt

---

### "I have multiple I2C devices with the same address"

**Winner: TCA9548A**

- Route identical sensors to separate sub-buses
- Enable/disable channels in software
- Level translation built in (mix 3.3 V and 5 V I2C devices)
- Stack up to 8 TCA9548A = 64 independent I2C sub-buses

---

### "I need the fastest possible GPIO expansion over SPI"

**Winner: MCP23S17**

- 10 MHz SPI vs 1.7 MHz I2C of MCP23017
- Identical register map to MCP23017
- 4 ICs per CS line using hardware address bits
- Best for real-time applications, PWM, fast bit-banging

---

### "Low power / battery-operated device"

**Winner: PCF8574 or MCP23017**

- Both support low-power standby
- MCP23017 has lower quiescent current at 3.3 V
- Avoid analog muxes in sleep mode (Ron consumes power when signal present)

---

### "I need to route I2S, SPI, or UART signals"

**Winner: 74HC4051 or 74HC4053**

- Analog muxes pass high-frequency digital signals transparently
- 74HC4053 (3× SPDT) is ideal for routing differential or bus signals
- Bandwidth > 100 MHz at 3.3 V — suitable for most digital protocols

---

## Scalability Comparison

How many peripherals can you reach with N chips?

### Analog Mux (CD74HC4067)

```
1 IC  → 16 channels  (4 select pins)
2 ICs → 32 channels  (5 select pins, 1 demux for IC select)
4 ICs → 64 channels  (6 select pins)
```

### I2C Expander (MCP23017)

```
1 IC  → 16 GPIO   (same 2 I2C pins)
2 ICs → 32 GPIO   (same 2 I2C pins)
8 ICs → 128 GPIO  (same 2 I2C pins!)
```

### Shift Register (74HC595)

```
1 IC  → 8 outputs  (3 SPI pins)
2 ICs → 16 outputs (3 SPI pins, cascaded)
4 ICs → 32 outputs (3 SPI pins, cascaded)
N ICs → N×8 outputs (3 SPI pins, cascaded)
```

---

## ESP32 GPIO Pin Budget

| Strategy | Pins Used | Max Peripherals |
|---|---|---|
| Direct GPIO | 1 per peripheral | ~34 (ESP32) |
| 74HC4051 | 4 (3 sel + 1 SIG) | 8 analog |
| CD74HC4067 | 5 (4 sel + 1 SIG) | 16 analog |
| 74HC595 ×4 | 3 (SPI) | 32 outputs |
| MCP23017 ×8 | 2 (I2C) | 128 bidirectional |
| TCA9548A + 8 sensors | 2 (I2C) | 64 same-addr sensors |

---

## Recommended Starter Kit

For a versatile GPIO expansion setup:

| IC | Qty | Purpose |
|---|---|---|
| CD74HC4067 | 1–2 | Analog sensor muxing |
| MCP23017 | 2–4 | Digital I/O expansion |
| 74HC595 | 2 | LED/relay output expansion |
| 74HC165 | 1 | Button/switch input scanning |
| TCA9548A | 1 | I2C bus isolation |

Total: ~$5–10 in parts, covers virtually any expansion need.
