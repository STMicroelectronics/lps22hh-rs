# LPS22HH Pressure and Temperature Data Acquisition on STM32F401RE Using Embassy Framework

This example demonstrates how to interface the **LPS22HH** pressure sensor with an **STM32F401RE** microcontroller board using the I2C bus with interrupt-driven event handling. The program reads pressure and temperature data from the sensor at 10 Hz and outputs the results over UART.

The code is written in Rust using the `embassy-stm32` hardware abstraction layer and the `lps22hh` sensor driver crate. It showcases sensor initialization, configuration, and polling-based data reading with UART output.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LPS22HH pressure sensor (I2C interface)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode with interrupt-driven event handling
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                  |
|--------------|-----------------|------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)  |
| I2C1_SDA     | PB9             | I2C data line (open-drain)   |
| USART2_TX    | PA2             | UART transmit for debug output|

The LPS22HH sensor is connected to the STM32F401RE's I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals using the `embassy-stm32` runtime.
- UART is configured on PA2 at 115200 baud for serial output.
- The I2C bus is configured for 100 kHz Standard Mode with interrupt-driven event handling.
- Interrupt handlers for USART2 and I2C1 events and errors are bound.
- A delay provider is used for sensor startup timing.

### Sensor Setup

- The LPS22HH sensor is initialized over I2C with the high address variant.
- The device ID is read and printed over UART to confirm sensor presence.
- The sensor is reset and the program waits until reset completes.
- Block Data Update (BDU) is enabled to ensure data consistency.
- Output Data Rate (ODR) is set to 10 Hz in low noise mode.

### Data Acquisition Loop

- The program continuously polls the sensor status register.
- When new pressure data is available, it reads the raw pressure, converts it to hPa, and prints it over UART.
- When new temperature data is available, it reads the raw temperature, converts it to Celsius, and prints it over UART.
- UART writes are blocking; asynchronous DMA-based UART is not used in this example.
- A 1-second delay is inserted between readings.

---

## Usage

1. Connect the LPS22HH sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect UART TX pin PA2 to a serial terminal interface.
3. Build and flash the Rust firmware onto the STM32F401RE.
4. Open a serial terminal at 115200 baud.
5. Observe pressure and temperature data printed every second.

---

## Notes

- This example uses polling to check for new sensor data.
- UART output uses blocking writes; DMA or async UART is not implemented.
- The sensor driver handles low-level register access and configuration.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications using the Embassy runtime.
- Panic behavior is set to use `panic_probe`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LPS22HH Sensor Datasheet](https://www.st.com/resource/en/datasheet/lps22hh.pdf)
- [embassy-stm32 Rust crate](https://docs.rs/embassy-stm32)

---

*This README provides a detailed explanation of the embedded Rust program for pressure and temperature data acquisition on STM32F401RE using the LPS22HH sensor via I2C with UART output.*