# ESP-IDF Code Examples

Ready-to-use C code for all major GPIO multiplexer ICs using ESP-IDF.

> Tested with ESP-IDF v5.x. Most code is compatible with v4.4+.
> All examples use standard `driver/i2c.h`, `driver/gpio.h`, and `driver/spi_master.h`.

---

## Common Setup — I2C Bus Init

Used by all I2C expander examples (MCP23017, PCF8574, TCA9548A).

```c
#include "driver/i2c.h"

#define I2C_PORT         I2C_NUM_0
#define I2C_SDA_PIN      GPIO_NUM_21
#define I2C_SCL_PIN      GPIO_NUM_22
#define I2C_FREQ_HZ      400000      // 400 kHz fast mode

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,  // use external 4.7k
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}
```

---

## 74HC4051 — 8-Channel Analog Mux

```c
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MUX_S0  GPIO_NUM_4
#define MUX_S1  GPIO_NUM_5
#define MUX_S2  GPIO_NUM_6
#define MUX_ADC ADC1_CHANNEL_0   // GPIO36 on ESP32

static void mux4051_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MUX_S0) | (1ULL << MUX_S1) | (1ULL << MUX_S2),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MUX_ADC, ADC_ATTEN_DB_12);
}

static void mux4051_select(uint8_t channel)
{
    // channel: 0–7
    gpio_set_level(MUX_S0, (channel >> 0) & 1);
    gpio_set_level(MUX_S1, (channel >> 1) & 1);
    gpio_set_level(MUX_S2, (channel >> 2) & 1);
}

static int mux4051_read(uint8_t channel)
{
    mux4051_select(channel);
    vTaskDelay(pdMS_TO_TICKS(1));  // settling time
    return adc1_get_raw(MUX_ADC);
}

void app_main(void)
{
    mux4051_init();

    while (1) {
        for (int ch = 0; ch < 8; ch++) {
            int raw = mux4051_read(ch);
            printf("Channel %d: %d (%.2f V)\n", ch, raw,
                   raw * 3.3f / 4095.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

---

## CD74HC4067 — 16-Channel Analog Mux

```c
#include "driver/gpio.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MUX_S0  GPIO_NUM_4
#define MUX_S1  GPIO_NUM_5
#define MUX_S2  GPIO_NUM_6
#define MUX_S3  GPIO_NUM_7
#define MUX_EN  GPIO_NUM_15      // EN pin — active low
#define MUX_ADC ADC1_CHANNEL_0

static void mux4067_init(void)
{
    uint64_t pin_mask = (1ULL << MUX_S0) | (1ULL << MUX_S1) |
                        (1ULL << MUX_S2) | (1ULL << MUX_S3) |
                        (1ULL << MUX_EN);
    gpio_config_t io = {
        .pin_bit_mask = pin_mask,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(MUX_EN, 0);  // enable (active low)

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MUX_ADC, ADC_ATTEN_DB_12);
}

static void mux4067_select(uint8_t channel)
{
    // channel: 0–15
    gpio_set_level(MUX_S0, (channel >> 0) & 1);
    gpio_set_level(MUX_S1, (channel >> 1) & 1);
    gpio_set_level(MUX_S2, (channel >> 2) & 1);
    gpio_set_level(MUX_S3, (channel >> 3) & 1);
}

static int mux4067_read(uint8_t channel)
{
    mux4067_select(channel);
    esp_rom_delay_us(100);  // 100 µs settling time
    return adc1_get_raw(MUX_ADC);
}

void app_main(void)
{
    mux4067_init();

    while (1) {
        for (int ch = 0; ch < 16; ch++) {
            int raw = mux4067_read(ch);
            printf("Ch%02d: %4d (%.3fV)\n", ch, raw, raw * 3.3f / 4095.0f);
        }
        printf("---\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## MCP23017 — 16-bit I2C I/O Expander

```c
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MCP23017";

// I2C settings
#define I2C_PORT     I2C_NUM_0
#define I2C_SDA      GPIO_NUM_21
#define I2C_SCL      GPIO_NUM_22
#define I2C_FREQ     400000

// MCP23017 address (A2=0, A1=0, A0=0)
#define MCP_ADDR     0x20

// MCP23017 registers (IOCON.BANK = 0, default)
#define REG_IODIRA   0x00
#define REG_IODIRB   0x01
#define REG_IPOLA    0x02
#define REG_IPOLB    0x03
#define REG_GPINTENA 0x04
#define REG_GPINTENB 0x05
#define REG_DEFVALA  0x06
#define REG_DEFVALB  0x07
#define REG_INTCONA  0x08
#define REG_INTCONB  0x09
#define REG_IOCON    0x0A
#define REG_GPPUA    0x0C
#define REG_GPPUB    0x0D
#define REG_INTFA    0x0E
#define REG_INTFB    0x0F
#define REG_INTCAPA  0x10
#define REG_INTCAPB  0x11
#define REG_GPIOA    0x12
#define REG_GPIOB    0x13
#define REG_OLATA    0x14
#define REG_OLATB    0x15

// ---- Low-level I2C helpers ----

static esp_err_t mcp_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mcp_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // repeated start
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ---- MCP23017 init ----

static void mcp23017_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA,
        .scl_io_num       = I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // Port A: all outputs (IODIRA = 0x00)
    ESP_ERROR_CHECK(mcp_write_reg(MCP_ADDR, REG_IODIRA, 0x00));
    // Port B: all inputs with pull-ups (IODIRB = 0xFF, GPPUB = 0xFF)
    ESP_ERROR_CHECK(mcp_write_reg(MCP_ADDR, REG_IODIRB, 0xFF));
    ESP_ERROR_CHECK(mcp_write_reg(MCP_ADDR, REG_GPPUB,  0xFF));

    // Initialize Port A to all-off
    ESP_ERROR_CHECK(mcp_write_reg(MCP_ADDR, REG_OLATA, 0x00));
}

// ---- Port A output helpers ----

static void mcp23017_set_porta(uint8_t value)
{
    ESP_ERROR_CHECK(mcp_write_reg(MCP_ADDR, REG_OLATA, value));
}

static void mcp23017_set_pin(uint8_t pin, bool state)
{
    // pin: 0–7 for Port A
    uint8_t cur;
    mcp_read_reg(MCP_ADDR, REG_OLATA, &cur);
    if (state) cur |=  (1 << pin);
    else       cur &= ~(1 << pin);
    mcp_write_reg(MCP_ADDR, REG_OLATA, cur);
}

// ---- Port B input helper ----

static uint8_t mcp23017_read_portb(void)
{
    uint8_t val = 0;
    mcp_read_reg(MCP_ADDR, REG_GPIOB, &val);
    return val;
}

// ---- Example: LED bar + button read ----

void app_main(void)
{
    mcp23017_init();

    uint8_t led_pattern = 0x01;
    while (1) {
        // Rotate LED on Port A
        mcp23017_set_porta(led_pattern);
        led_pattern = (led_pattern << 1) | (led_pattern >> 7);  // rotate left

        // Read buttons on Port B
        uint8_t buttons = mcp23017_read_portb();
        ESP_LOGI(TAG, "LEDs: 0x%02X  Buttons: 0x%02X", led_pattern, ~buttons & 0xFF);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

---

## PCF8574 — 8-bit I2C I/O Expander

```c
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "PCF8574";

#define I2C_PORT   I2C_NUM_0
#define PCF_ADDR   0x20   // A0=A1=A2=0

// PCF8574 is extremely simple: write 1 byte to set outputs,
// read 1 byte to read inputs.
// Pins driven HIGH are inputs (quasi-bidirectional).

static esp_err_t pcf8574_write(uint8_t addr, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t pcf8574_read(uint8_t addr, uint8_t *value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    // i2c_master_init(); // call your I2C init here

    // Set P0–P3 as outputs (LOW), P4–P7 as inputs (HIGH = quasi-input)
    pcf8574_write(PCF_ADDR, 0xF0);

    while (1) {
        uint8_t val;
        pcf8574_read(PCF_ADDR, &val);
        uint8_t inputs = (val >> 4) & 0x0F;  // upper nibble = inputs

        ESP_LOGI(TAG, "Input nibble: 0x%X", inputs);

        // Drive lower nibble based on input
        pcf8574_write(PCF_ADDR, 0xF0 | inputs);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## TCA9548A — 8-Channel I2C Bus Mux

```c
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "TCA9548A";

#define I2C_PORT   I2C_NUM_0
#define TCA_ADDR   0x70   // A0=A1=A2=0

// Select one or more channels (bitmask: bit N = channel N)
// Pass 0x00 to disable all channels
static esp_err_t tca9548a_select(uint8_t addr, uint8_t channels)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, channels, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Example: read a BME280 (0x76) on each of 8 channels
#define BME280_ADDR   0x76
#define BME280_ID_REG 0xD0  // chip ID register, should return 0x60

static uint8_t read_bme280_id(void)
{
    // Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BME280_ID_REG, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BME280_ADDR << 1) | I2C_MASTER_READ, true);
    uint8_t id = 0;
    i2c_master_read_byte(cmd, &id, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return id;
}

void app_main(void)
{
    // i2c_master_init(); // call your I2C init here

    for (int ch = 0; ch < 8; ch++) {
        // Select sub-channel
        tca9548a_select(TCA_ADDR, 1 << ch);
        vTaskDelay(pdMS_TO_TICKS(10));  // allow bus to settle

        // Read BME280 on this channel
        uint8_t id = read_bme280_id();
        if (id == 0x60) {
            ESP_LOGI(TAG, "Channel %d: BME280 found (ID=0x%02X)", ch, id);
        } else {
            ESP_LOGW(TAG, "Channel %d: no device or wrong ID (0x%02X)", ch, id);
        }
    }

    // Deselect all channels when done
    tca9548a_select(TCA_ADDR, 0x00);
    ESP_LOGI(TAG, "Scan complete");
}
```

---

## 74HC595 — Output Shift Register (SPI)

```c
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "74HC595";

#define PIN_CLK    GPIO_NUM_18   // SRCLK
#define PIN_MOSI   GPIO_NUM_23   // DS (data)
#define PIN_LATCH  GPIO_NUM_5    // RCLK (latch)

// Number of cascaded 595s
#define NUM_595    2             // = 16 output bits

static spi_device_handle_t spi_dev;
static uint8_t output_buf[NUM_595];

static void hc595_init(void)
{
    // Configure LATCH pin separately (SPI CS would auto-deassert)
    gpio_config_t latch_cfg = {
        .pin_bit_mask = (1ULL << PIN_LATCH),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&latch_cfg);
    gpio_set_level(PIN_LATCH, 0);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = PIN_MOSI,
        .miso_io_num     = -1,       // no MISO needed for 595
        .sclk_io_num     = PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,   // 1 MHz
        .mode           = 0,          // CPOL=0, CPHA=0
        .spics_io_num   = -1,         // manual latch control
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev));
}

static void hc595_update(void)
{
    spi_transaction_t t = {
        .length    = NUM_595 * 8,   // bits to transfer
        .tx_buffer = output_buf,    // MSB first (most recent 595 first)
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_dev, &t));

    // Pulse latch to transfer shift register → output
    gpio_set_level(PIN_LATCH, 1);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_LATCH, 0);
}

static void hc595_set_pin(int pin, bool state)
{
    // pin 0–7 = first 595, pin 8–15 = second 595
    int byte_idx = pin / 8;
    int bit_idx  = pin % 8;
    if (state) output_buf[byte_idx] |=  (1 << bit_idx);
    else       output_buf[byte_idx] &= ~(1 << bit_idx);
}

void app_main(void)
{
    hc595_init();
    memset(output_buf, 0, sizeof(output_buf));

    // Knight Rider effect across 16 LEDs
    int pos = 0;
    int dir = 1;
    while (1) {
        memset(output_buf, 0, sizeof(output_buf));
        hc595_set_pin(pos, true);
        hc595_update();

        pos += dir;
        if (pos >= 15) dir = -1;
        if (pos <= 0)  dir =  1;

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```

---

## 74HC165 — Input Shift Register (SPI)

```c
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "74HC165";

#define PIN_CLK    GPIO_NUM_18   // CLK
#define PIN_MISO   GPIO_NUM_19   // QH (serial data out)
#define PIN_LOAD   GPIO_NUM_4    // PL (parallel load, active low)

#define NUM_165    1             // number of cascaded 165s (1 = 8 inputs)

static spi_device_handle_t spi_dev;

static void hc165_init(void)
{
    gpio_config_t load_cfg = {
        .pin_bit_mask = (1ULL << PIN_LOAD),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&load_cfg);
    gpio_set_level(PIN_LOAD, 1);  // idle high

    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = -1,         // no MOSI needed
        .miso_io_num   = PIN_MISO,
        .sclk_io_num   = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 1000000,
        .mode           = 0,
        .spics_io_num   = -1,
        .queue_size     = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev));
}

static uint8_t hc165_read(void)
{
    // Pulse LOAD low to latch parallel inputs
    gpio_set_level(PIN_LOAD, 0);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_LOAD, 1);

    uint8_t rx_buf = 0;
    spi_transaction_t t = {
        .length    = NUM_165 * 8,
        .rxlength  = NUM_165 * 8,
        .rx_buffer = &rx_buf,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi_dev, &t));
    return rx_buf;
}

void app_main(void)
{
    hc165_init();

    while (1) {
        uint8_t inputs = hc165_read();
        ESP_LOGI(TAG, "Inputs: 0b%08b (0x%02X)", inputs, inputs);
        // Bit 7 = pin A, Bit 0 = pin H (MSB first)
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
```

---

## Combining 595 + 165 on Shared SPI Bus

Both ICs can share SCLK. Use separate GPIO for LOAD (165) and LATCH (595):

```c
// SPI bus: CLK shared, separate MOSI and MISO
// MOSI → 595 DS
// MISO ← 165 QH
// GPIO_LOAD → 165 PL
// GPIO_LATCH → 595 RCLK

// Two separate spi_device_handle_t — same SPI bus, different devices
// spi_bus_add_device() twice on SPI2_HOST
```

---

## ESP-IDF Component: `idf_component.yml`

If building as a component or using ESP-IDF component manager, a minimal component descriptor:

```yaml
## idf_component.yml
version: "1.0.0"
description: "GPIO multiplexer drivers (74HC4051, CD74HC4067, MCP23017, PCF8574, TCA9548A, 74HC595, 74HC165)"
targets:
  - esp32
  - esp32s2
  - esp32s3
  - esp32c3
  - esp8266

dependencies:
  idf: ">=4.4"
```

---

## CMakeLists.txt (ESP-IDF Project)

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gpio_multiplexer_example)
```

```cmake
## main/CMakeLists.txt
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_rom freertos
)
```
