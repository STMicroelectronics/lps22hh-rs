# LPS22HH Pressure and Temperature Data Acquisition on STM32F401RE with External Interrupt

This example demonstrates how to interface the **LPS22HH** pressure sensor with an **STM32F401RE** microcontroller board using the I2C bus and an external interrupt pin. The program configures the sensor to generate an interrupt when new pressure data is available, reads pressure and temperature data upon interrupt, and outputs the results over UART.

The code is written in Rust using the `stm32f4xx-hal` hardware abstraction layer and the `lps22hh` sensor driver crate. It showcases sensor initialization, interrupt configuration, and event-driven data acquisition with serial output.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LPS22HH pressure sensor (I2C interface)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB10 configured as input with rising edge interrupt

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                  |
|--------------|-----------------|------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)  |
| I2C1_SDA     | PB9             | I2C data line (open-drain)   |
| USART2_TX    | PA2             | UART transmit for debug output|
| INT (IRQ)    | PB10            | External interrupt from sensor|

The LPS22HH sensor is connected to the STM32F401RE's I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's interrupt output is connected to PB10, configured to trigger an interrupt on rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals, including clocks, GPIO pins, I2C, UART, and external interrupt.
- The I2C bus is configured for 100 kHz Standard Mode using pins PB8 (SCL) and PB9 (SDA).
- UART is configured on PA2 at 115200 baud for serial output.
- The PB10 pin is configured as an input and set up to generate an interrupt on rising edge.
- The external interrupt line is enabled in the NVIC and linked to the `EXTI15_10` interrupt handler.
- The interrupt pin is stored in a global mutex-protected static for safe access in the interrupt handler.

### Sensor Setup

- The LPS22HH sensor is initialized over I2C with the high address variant.
- The device ID is read and printed over UART to confirm sensor presence.
- The sensor is reset and the program waits until reset completes.
- Interrupt notification is configured as pulsed.
- The interrupt routing is set to enable pressure data-ready interrupt on the sensor's INT pin.
- Block Data Update (BDU) is enabled to ensure data consistency.
- Output Data Rate (ODR) is set to 10 Hz in low noise mode.

### Data Acquisition Loop

- The main loop waits for interrupts using the `wfi` (Wait For Interrupt) instruction to reduce power consumption.
- When an interrupt occurs, the program checks if new pressure or temperature data is ready.
- If pressure data is ready, it reads the raw pressure, converts it to hPa, and prints it over UART.
- If temperature data is ready, it reads the raw temperature, converts it to Celsius, and prints it over UART.

### Interrupt Handler

- The `EXTI15_10` interrupt handler clears the interrupt pending bit on PB10 to allow further interrupts.

---

## Usage

1. Connect the LPS22HH sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's interrupt output to PB10 on the STM32F401RE.
3. Connect UART TX pin PA2 to a serial terminal interface.
4. Build and flash the Rust firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud.
6. Observe pressure and temperature data printed upon sensor interrupts.

---

## Notes

- This example uses an external interrupt to detect new sensor data availability, improving power efficiency by avoiding continuous polling.
- The interrupt pin is configured for rising edge detection and handled safely in an interrupt context.
- The sensor driver handles low-level register access and configuration.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic (`panic_halt`).

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LPS22HH Sensor Datasheet](https://www.st.com/resource/en/datasheet/lps22hh.pdf)
- [stm32f4xx-hal Rust crate](https://docs.rs/stm32f4xx-hal)

---

*This README provides a detailed explanation of the embedded Rust program for interrupt-driven pressure and temperature data acquisition on STM32F401RE using the LPS22HH sensor via I2C with UART output.*