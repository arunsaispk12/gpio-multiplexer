# GPIO Multiplexer — Reference Guide

A comprehensive reference for GPIO multiplexers and I/O expanders, focused on ESP32/ESP8266 with ESP-IDF, but applicable to any embedded platform.

## What Is a GPIO Multiplexer?

A GPIO multiplexer (mux) lets a microcontroller communicate with more peripherals than it has physical GPIO pins. Instead of needing one MCU pin per device, a mux routes many signals through a small number of control pins.

There are two fundamentally different approaches:

| Approach | How It Works | Best For |
|---|---|---|
| **Analog/Digital Mux** | One signal path, selected by address bits | ADC inputs, analog sensors, digital buses |
| **I/O Expander** | Adds fully independent GPIO pins via I2C/SPI | Buttons, LEDs, relays, general GPIO |
| **Shift Register** | Serial-to-parallel or parallel-to-serial | Bulk LED driving, button scanning |
| **I2C Bus Mux** | Shares one I2C bus across multiple sub-buses | Multiple devices with same address |

## Documentation Index

| Document | Description |
|---|---|
| [Types](docs/types.md) | Every multiplexer category explained with IC examples |
| [Comparison](docs/comparison.md) | Side-by-side table and best-pick recommendations |
| [Wiring Guide](docs/wiring-guide.md) | Pinouts and ASCII wiring diagrams for all major ICs |
| [Usage Guidelines](docs/usage-guidelines.md) | Electrical best practices, dos and don'ts |
| [ESP-IDF Examples](docs/esp-idf-examples.md) | Ready-to-use C code for ESP-IDF |
| [Troubleshooting](docs/troubleshooting.md) | Common problems and how to fix them |

## Quick Picks

**I need more analog inputs (ADC)** → [CD74HC4067](docs/types.md#cd74hc4067) (16-ch) or [74HC4051](docs/types.md#74hc4051) (8-ch)

**I need more digital outputs (LEDs, relays)** → [74HC595](docs/types.md#74hc595) shift register or [MCP23017](docs/types.md#mcp23017) I2C expander

**I need more digital inputs (buttons)** → [74HC165](docs/types.md#74hc165) shift register or [MCP23017](docs/types.md#mcp23017)

**I have multiple I2C sensors with the same address** → [TCA9548A](docs/types.md#tca9548a) I2C bus mux

**I need general-purpose I/O expansion (I2C)** → [MCP23017](docs/types.md#mcp23017) (16 pins) or [PCF8574](docs/types.md#pcf8574) (8 pins)

**Low pin count, simple SPI** → [MCP23S17](docs/types.md#mcp23s17)

## Quick Wiring Overview

```
ESP32 → [MUX/EXPANDER] → Multiple Peripherals
         ↑
     3 select pins (analog mux)
     OR 2-wire I2C
     OR 3-wire SPI
     OR 3-wire shift register
```

## Compatibility Notes

- Most ICs listed operate at **3.3 V** (compatible with ESP32/ESP8266 directly)
- ICs marked with `*` require level shifting or have 5 V variants
- ESP32 GPIO is **not 5 V tolerant** — never connect 5 V logic directly

## License

Documentation released under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
