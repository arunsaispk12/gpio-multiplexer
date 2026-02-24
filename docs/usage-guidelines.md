# Usage Guidelines & Best Practices

Electrical and software best practices for GPIO multiplexers and I/O expanders.

---

## General Electrical Rules

### 1. Always Use Decoupling Capacitors

Place 100 nF ceramic capacitors between VCC and GND on every IC, as close to the power pins as possible.

- **Why:** Digital switching creates current spikes that corrupt analog readings. Without bypass caps, multiplexed ADC readings will be noisy and erratic.
- **Rule:** One 0.1 µF cap per IC at minimum. Add 10 µF bulk cap at the power supply entry point.

### 2. VCC Must Match Logic Levels

| System VCC | Compatible ICs | Notes |
|---|---|---|
| 3.3 V | 74HC series, MCP23017, TCA9548A, PCF8574 | Direct connect to ESP32 |
| 5 V | 74HC series (most), PCF8574 | Needs level shifter to ESP32! |

**ESP32 GPIO is NOT 5 V tolerant.** Connecting 5 V signals directly will damage the ESP32.

### 3. Level Shifting for 5 V Devices

If a peripheral or IC operates at 5 V:

```
ESP32 (3.3V) ──[BSS138 MOSFET level shifter]── 5V device
```

Use a bidirectional level shifter for SDA/SCL (I2C). Use a simple resistor divider or buffer for unidirectional signals.

### 4. Ground Everything Together

All GND pins of all ICs must connect to a common ground plane. Avoid ground loops — use star topology or proper PCB ground plane.

---

## Analog Multiplexer Guidelines

### Channel Settling Time

When you switch channels on an analog mux, the output takes time to settle to the new voltage. This is affected by:
- On-resistance (Ron) of the mux: ~70–80 Ω for 74HC devices
- Load capacitance (ADC input capacitance + PCB parasitic)

**Rule:** Add a settling delay after changing the channel before reading the ADC.

```c
// After selecting channel:
gpio_set_level(S0, ch & 1);
gpio_set_level(S1, (ch >> 1) & 1);
gpio_set_level(S2, (ch >> 2) & 1);

vTaskDelay(pdMS_TO_TICKS(1));  // 1 ms settle time (adjust as needed)

// Then read ADC
int raw = adc1_get_raw(ADC1_CHANNEL_0);
```

For fast switching, characterize your specific setup — often 10–100 µs is sufficient.

### Signal Integrity

- Keep analog signal traces short. Long traces pick up noise.
- Route analog signal traces away from digital clock lines (SPI, I2C CLK, PWM).
- If possible, shield analog traces with ground fills.
- The 74HC4051/4067 Ron (~70 Ω) forms an RC filter with input capacitance. For high-impedance sensors, buffer the output with an op-amp.

### Input Range Must Stay Within VCC/GND

The analog signal must stay between GND and VCC at all times. Signals outside this range will:
- Cause the mux to behave incorrectly
- Potentially damage the IC

For sensors exceeding 3.3 V, use a resistor voltage divider before the mux input.

### Do Not Leave Unused Channels Floating

Unconnected channels on an analog mux can pick up noise and inject it into the selected channel through capacitive coupling.

**Fix:** Connect unused channel pins to GND (for input muxing) through a 10 kΩ resistor.

---

## I2C Expander Guidelines

### Pull-up Resistors Are Mandatory

I2C lines are open-drain — without pull-ups, the bus cannot work.

- 4.7 kΩ to 3.3 V is the standard value at 100 kHz
- Use 2.2 kΩ at 400 kHz
- **Only one pair of pull-ups per bus** — not one per device

### Bus Capacitance Limits

I2C has a maximum bus capacitance of 400 pF. Each device and trace adds capacitance.

- Long cables and many devices degrade signal edges
- If running I2C over long cables (>30 cm), use an I2C buffer/extender IC (e.g., PCA9600, LTC4311)
- For many devices on one bus, prefer the TCA9548A to split the load

### Address Planning

Map out I2C addresses before building:

```
Device          Address Range    Configurable Pins
MCP23017        0x20–0x27        A0, A1, A2
PCF8574         0x20–0x27        A0, A1, A2
PCF8574A        0x38–0x3F        A0, A1, A2
TCA9548A        0x70–0x77        A0, A1, A2
BME280          0x76 or 0x77     SDO pin
SSD1306 OLED    0x3C or 0x3D     SA0 pin
```

**Watch for conflicts:** MCP23017 and PCF8574 share the same address space (0x20–0x27). Do not mix them at the same addresses.

### Interrupt Usage (MCP23017)

For button/switch inputs, use interrupt-driven reads instead of polling:

1. Configure GPINTEN register to enable interrupt on desired pins.
2. Configure INTCON to interrupt on change (vs. DEFVAL).
3. Wire INTA/INTB to ESP32 GPIO with `GPIO_INTR_NEGEDGE`.
4. In the ISR, read INTFA/INTFB (which pin) and INTCAPA/INTCAPB (captured value at interrupt time).

Benefits: no polling overhead, instant response, CPU sleeps between events.

### Reset Pin

Always connect the RESET pin of MCP23017 to VCC (active low, so pull high for normal operation). Optionally connect to a GPIO for software reset:

```c
gpio_set_level(RESET_PIN, 0);  // reset
vTaskDelay(pdMS_TO_TICKS(1));
gpio_set_level(RESET_PIN, 1);  // release
```

---

## Shift Register Guidelines

### 74HC595 — Output Shift Register

- **Latch on final bit:** Always pulse RCLK after all bits are shifted. Outputs update atomically on the latch pulse.
- **Output Enable (OE):** Tie to GND for always-on. Or use a GPIO to blank outputs during update (glitch-free switching).
- **Clear (SRCLR):** Tie to VCC. Pulling it low clears the shift register (not the output latch).
- **Current limits:** Each Q pin sinks up to 35 mA. Total IC sink current: 70 mA. For more LEDs, use transistors or MOSFET drivers.

### 74HC165 — Input Shift Register

- **Debounce buttons:** 74HC165 reads instantaneous state. For buttons, debounce in software (e.g., read twice with 10 ms gap and compare).
- **Pull-up or pull-down:** Floating inputs cause erratic reads. Always add pull-up (buttons to GND) or pull-down resistors.
- **SER pin:** In a single-IC setup, tie SER (serial input) to GND or VCC. In cascade, connect Q7'/QH' of the previous IC.

### Cascade SPI Sharing

74HC595 and 74HC165 can share the same SPI CLK and CS lines. Use separate MOSI (for 595) and MISO (for 165) signals. Latch control (RCLK for 595, PL for 165) uses separate GPIO pins.

---

## TCA9548A — I2C Bus Mux

### Channel Selection Best Practice

```c
// Always deselect all channels when done with a device
// This prevents bus contention
tca9548a_select_channel(i2c_port, TCA_ADDR, 0x00);  // none active
```

### Multiple TCA9548A Chips

Up to 8 TCA9548A can coexist on one I2C bus (addresses 0x70–0x77). This gives 64 independent I2C sub-buses from 2 ESP32 pins.

### Level Translation

TCA9548A supports mixed-voltage I2C buses. Connect VCC to the higher voltage side. For 3.3 V ESP32 and 5 V I2C devices:
- VCC = 5 V
- I2C pull-ups on sub-bus side = 5 V
- I2C pull-ups on ESP32 side = 3.3 V

---

## Software Best Practices

### Debouncing Digital Inputs

```
Raw input:   _____|‾|_|‾‾‾‾‾‾‾‾‾‾‾‾‾
Debounced:   ________________|‾‾‾‾‾‾
```

Always debounce in software for any mechanical input (buttons, reed switches, encoders).

Minimum debounce window: 10–50 ms for most buttons.

### Batch I2C Operations

Minimize I2C transactions — reading/writing the full port register at once is faster than individual pin access:

```c
// Slow: set pins one at a time (N I2C transactions)
// Fast: build byte then write once (1 I2C transaction)
uint8_t port_val = 0;
if (led0_on) port_val |= (1 << 0);
if (led1_on) port_val |= (1 << 1);
// ...
mcp23017_write_register(GPIOA, port_val);  // single transaction
```

### Error Handling on I2C

Always check I2C return values. A missing device (not ACKing) or bus lockup (SDA stuck low) will cause timeouts:

```c
esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C error: %s", esp_err_to_name(ret));
    // handle or retry
}
```

### Analog Mux Channel Map

Define your channel map as a constant array — do not hardcode magic numbers:

```c
typedef enum {
    SENSOR_TEMP    = 0,  // MUX Y0
    SENSOR_HUMID   = 1,  // MUX Y1
    SENSOR_LIGHT   = 2,  // MUX Y2
    SENSOR_BATT    = 3,  // MUX Y3
} mux_channel_t;
```

---

## Common Dos and Don'ts

| Do | Don't |
|---|---|
| Add 0.1 µF decoupling caps to every IC | Leave power pins without bypass |
| Pull RESET high (MCP23017, TCA9548A) | Leave RESET floating |
| Add pull-up resistors to I2C bus | Use I2C without pull-ups |
| Tie unused analog mux channels to GND via 10 kΩ | Leave mux inputs floating |
| Use voltage-compatible ICs (3.3 V) | Connect 5 V signals directly to ESP32 |
| Debounce mechanical inputs | Read raw button state without debounce |
| Use interrupt-driven I/O when possible | Poll I2C expanders constantly |
| Plan I2C addresses before wiring | Mix conflicting address devices |
| Latch 595 outputs atomically | Update outputs mid-shift (glitches) |
| Check ESP-IDF I2C return values | Ignore I2C errors silently |
