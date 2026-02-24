# Troubleshooting Guide

Common problems with GPIO multiplexers and I/O expanders, and how to fix them.

---

## I2C Issues

### Problem: `ESP_ERR_TIMEOUT` on every I2C transaction

**Causes and fixes:**

1. **Missing pull-up resistors** — Most common cause.
   - Add 4.7 kΩ resistors from SDA and SCL to 3.3 V.
   - Without pull-ups, the bus never goes high — every transaction times out.

2. **Wrong I2C address** — Device not acknowledging.
   - Run an I2C scanner to discover actual addresses:
   ```c
   for (uint8_t addr = 1; addr < 127; addr++) {
       i2c_cmd_handle_t cmd = i2c_cmd_link_create();
       i2c_master_start(cmd);
       i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
       i2c_master_stop(cmd);
       esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(10));
       i2c_cmd_link_delete(cmd);
       if (ret == ESP_OK) printf("Found device at 0x%02X\n", addr);
   }
   ```

3. **SDA stuck LOW (bus lockup)** — A failed transaction left SDA pulled low.
   - Hardware fix: Reset the device (pulse RESET pin low then high).
   - Software fix (clock out the stuck slave):
   ```c
   // Toggle SCL up to 9 times to free stuck SDA
   gpio_set_direction(I2C_SCL_PIN, GPIO_MODE_OUTPUT);
   for (int i = 0; i < 9; i++) {
       gpio_set_level(I2C_SCL_PIN, 0);
       esp_rom_delay_us(5);
       gpio_set_level(I2C_SCL_PIN, 1);
       esp_rom_delay_us(5);
   }
   // Re-install I2C driver
   i2c_driver_delete(I2C_PORT);
   i2c_master_init();
   ```

4. **Too-low pull-up value** — Pull-ups that are too strong (< 1 kΩ) exceed the GPIO driver current rating.
   - Use 4.7 kΩ at 100 kHz, 2.2 kΩ at 400 kHz.

5. **Long I2C wires** — Cable capacitance slows rise times.
   - Keep I2C bus length under 30 cm for 400 kHz.
   - For longer runs, use I2C buffers (e.g., LTC4311, PCA9600) or reduce speed to 100 kHz.

---

### Problem: I2C works sometimes, randomly fails

**Causes:**

1. **Bus capacitance too high** — Too many devices or long traces.
   - Add I2C bus buffer. Or reduce speed.

2. **Power supply noise** — Missing or inadequate decoupling capacitors.
   - Add 100 nF caps on every IC's VCC pin.
   - Add 10 µF bulk cap at power supply input.

3. **EMI / RF interference** — Nearby transmitter (WiFi, BT, motor).
   - Route I2C traces away from high-current paths.
   - Shorten cable length.
   - Lower I2C clock speed.

---

### Problem: MCP23017 interrupt (INTA/INTB) never fires

1. **INT pins not configured correctly.**
   - Ensure `GPINTEN` register bits are set for the pins you want.
   - Ensure `INTCON` is correct (0 = compare to previous, 1 = compare to DEFVAL).

2. **ESP32 ISR not triggering.**
   - Check GPIO direction: interrupt pin must be configured as input.
   - Use `GPIO_INTR_NEGEDGE` (INT is active low, open-drain by default).
   - Pull up the INT pin with 10 kΩ resistor (open-drain output needs pull-up).

3. **Interrupt latches not cleared.**
   - Read `INTCAPA`/`INTCAPB` or `GPIOA`/`GPIOB` to clear the interrupt flag.
   - If not read, INTA/INTB stays asserted and no new interrupt fires.

---

## Analog Mux Issues

### Problem: ADC reads are noisy / unstable

1. **No decoupling caps on mux IC.**
   - Add 100 nF between VCC and GND of the 74HC4051/4067.

2. **Insufficient settling time.**
   - Increase the delay between channel select and ADC read.
   - Start with 10 ms, then reduce while checking stability.

3. **Floating unused channels.**
   - Tie all unused channel pins to GND via 10 kΩ resistors.

4. **Long traces or cables on channel inputs.**
   - Keep sensor wiring short, or buffer sensor output with op-amp.

5. **ESP32 ADC noise.**
   - ESP32's internal ADC has inherent noise, especially on WiFi-active chips.
   - Average multiple reads: take 8–16 samples and average.
   - Use `esp_adc_cal_characterize()` and `esp_adc_cal_raw_to_voltage()` for calibrated readings.
   - Run ADC when WiFi is idle or use external ADC (e.g., ADS1115).

---

### Problem: All channels read the same value

1. **Select pins (S0/S1/S2/S3) not changing.**
   - Verify GPIO are configured as output (`GPIO_MODE_OUTPUT`).
   - Check GPIO numbers match your wiring.

2. **EN/INH pin not driven correctly.**
   - 74HC4051: INH must be LOW to enable.
   - CD74HC4067: EN must be LOW to enable.
   - If floating, behavior is undefined.

3. **COM/SIG pin floating.**
   - Check physical connection from mux COM to ESP32 ADC pin.

---

### Problem: Only some channels work

1. **Address bit (S pin) not driving high enough.**
   - Confirm GPIO output voltage reaches VCC (3.3 V). Use a multimeter.

2. **Partial continuity failure** — one Y pin not connected or broken trace.

---

## Shift Register Issues

### Problem: 74HC595 outputs don't update

1. **LATCH (RCLK) not pulsing.**
   - The shift register loads data on CLK edges, but outputs only update on a RCLK rising edge.
   - Ensure you pulse RCLK high after shifting all bits.

2. **OE (Output Enable) pulled high.**
   - OE is active low. Tie to GND or drive low to enable outputs.

3. **SRCLR (Clear) pulled low.**
   - SRCLR is active low. Tie to VCC for normal operation.

4. **Bit order wrong.**
   - First bit clocked in is the last to appear on Q7. Shift MSB first.

---

### Problem: 74HC165 reads wrong values

1. **LOAD (PL) not pulsing.**
   - PL must pulse low before clocking to latch parallel inputs.
   - If PL stays high, 74HC165 shifts serial input (SER), not parallel inputs.

2. **CLK INH not tied to GND.**
   - CLK INH = 1 inhibits clocking. Tie to GND.

3. **Floating inputs.**
   - Inputs A–H will float and read random values without pull-ups or pull-downs.

4. **SER pin floating (single IC).**
   - In a single-165 setup, SER must be tied to a defined level (GND or VCC).

---

## TCA9548A Issues

### Problem: Can't reach devices on sub-channels

1. **Channel not selected.**
   - Write the channel select byte before accessing devices: `1 << channel_num`.

2. **Pull-ups missing on sub-channel.**
   - Each sub-bus needs its own I2C pull-ups to VCC.

3. **Address conflict with TCA9548A.**
   - TCA9548A itself occupies 0x70–0x77. If a device on the sub-bus also has address 0x70–0x77, writes intended for the device will re-configure the TCA9548A.
   - Use TCA9548A at a different address or avoid conflicting addresses on sub-bus.

4. **Multiple channels active simultaneously causing bus conflict.**
   - Never enable two channels where sub-bus devices share addresses.

---

## Power Issues

### Problem: IC gets hot

1. **VCC too high** — Verify supply voltage is within spec (3.3 V for ESP32-compatible ICs).
2. **Short circuit on output pin** — Disconnect outputs and recheck wiring.
3. **Excessive load current** — Each 74HC595 Q pin can sink max 35 mA. Use drivers (transistors/MOSFETs) for high-current loads.

### Problem: Random resets / brownout

1. **Insufficient power supply current.**
   - Large LED arrays or relay activation can cause voltage dips.
   - Use capacitors (100 µF–1000 µF) on the 3.3 V rail near high-current loads.
   - Use a separate power supply for high-current peripherals.

2. **Ground bounce from switching loads.**
   - Ensure all GNDs connect at a single point (star topology) or proper ground plane.

---

## General Debugging Checklist

```
[ ] Decoupling caps (0.1µF) on every IC VCC pin
[ ] I2C pull-up resistors (4.7kΩ) on SDA and SCL
[ ] VCC voltage is correct (3.3V for ESP32 compatibility)
[ ] RESET pins pulled to VCC (MCP23017, TCA9548A)
[ ] EN/INH/OE pins in correct state (enabled)
[ ] GPIO configured as OUTPUT for control pins
[ ] GPIO configured as INPUT for read pins
[ ] I2C address confirmed with scanner
[ ] Channel selected before reading (mux + TCA9548A)
[ ] Settling delay added after channel change (analog mux)
[ ] Latch pulsed after shifting (74HC595)
[ ] LOAD pulsed before reading (74HC165)
[ ] Unused mux channels tied to GND via 10kΩ
[ ] All inputs have pull-up or pull-down resistors
[ ] No floating pins anywhere in the circuit
```
