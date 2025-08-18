# LPS22HH Sensor FIFO Data Acquisition on STM32F401RE with External Interrupt

This example demonstrates how to interface the **LPS22HH** pressure sensor with an **STM32F401RE** microcontroller board using the I2C bus and the sensor's FIFO feature with interrupt-driven data reading. The program configures the sensor to generate an interrupt when the FIFO watermark level is reached, reads multiple pressure and temperature samples from the FIFO, and outputs the results over UART.

The code is written in Rust using the `embassy-stm32` hardware abstraction layer and the `lps22hh` sensor driver crate. It showcases sensor initialization, FIFO configuration, external interrupt setup, and event-driven data acquisition with serial output.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LPS22HH pressure sensor (I2C interface)
- **Communication Interface:** I2C1 at 100 kHz Standard Mode with interrupt-driven event handling
- **UART:** USART2 for serial output at 115200 baud
- **Interrupt Pin:** PB10 configured as input with external interrupt for FIFO watermark notification

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                  |
|--------------|-----------------|------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)  |
| I2C1_SDA     | PB9             | I2C data line (open-drain)   |
| USART2_TX    | PA2             | UART transmit for debug output|
| INT (IRQ)    | PB10            | External interrupt from sensor FIFO watermark |

The LPS22HH sensor is connected to the STM32F401RE's I2C1 peripheral on pins PB8 (SCL) and PB9 (SDA). The sensor's interrupt output is connected to PB10, configured to trigger an external interrupt on rising edge. UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals using the `embassy-stm32` runtime.
- UART is configured on PA2 at 115200 baud for serial output.
- The I2C bus is configured for 100 kHz Standard Mode with interrupt-driven event handling.
- The PB10 pin is configured as an input with external interrupt capability using EXTI10.
- Interrupt handlers for USART2 and I2C1 events and errors are bound.
- A delay provider is used for sensor startup timing.

### Sensor Setup

- The LPS22HH sensor is initialized over I2C with the high address variant.
- The device ID is read and printed over UART to confirm sensor presence.
- The sensor is reset and the program waits until reset completes.
- FIFO watermark level is set to 32 samples.
- FIFO stop-on-watermark mode is enabled.
- FIFO mode is set to Stream Mode.
- Interrupt notification is configured as latched.
- The interrupt routing is set to enable FIFO watermark interrupt on the sensor's INT pin.
- Output Data Rate (ODR) is set to 10 Hz.

### Data Acquisition Loop

- The main loop asynchronously waits for a rising edge on the interrupt pin signaling FIFO watermark reached.
- When the interrupt occurs, the program reads the number of samples in the FIFO.
- It then reads pressure and temperature data for each sample from the FIFO.
- Pressure data is converted to hPa and temperature data to Celsius, then printed over UART.
- UART writes are blocking; asynchronous DMA-based UART is not used in this example.

---

## Usage

1. Connect the LPS22HH sensor to the STM32F401RE Nucleo board via I2C on pins PB8 (SCL) and PB9 (SDA).
2. Connect the sensor's interrupt output to PB10 on the STM32F401RE.
3. Connect UART TX pin PA2 to a serial terminal interface.
4. Build and flash the Rust firmware onto the STM32F401RE.
5. Open a serial terminal at 115200 baud.
6. Observe batches of pressure and temperature data printed when the FIFO watermark is reached.

---

## Notes

- This example uses the sensor's FIFO feature to buffer multiple samples, reducing I2C traffic.
- External interrupt on PB10 signals when FIFO watermark is reached, enabling efficient data reading.
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

*This README provides a detailed explanation of the embedded Rust program for FIFO-based pressure and temperature data acquisition on STM32F401RE using the LPS22HH sensor via I2C with external interrupt and UART output.*