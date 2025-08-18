# lps22hh-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/lps22hh-rs
[crates-url]: https://crates.io/crates/lps22hh-rs
[bsd-badge]: https://img.shields.io/crates/l/lps22hh-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

This crate provides a platform-agnostic driver for the ST LPS22HH pressure and temperature sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The LPS22HH is an ultra-compact piezoresistive
absolute pressure sensor which functions as a
digital output barometer. The device comprises a
sensing element and an IC interface which
communicates through I²C, MIPI I3CSM or SPI
from the sensing element to the application.

The sensing element, which detects absolute
pressure, consists of a suspended membrane
manufactured using a dedicated process
developed by ST.

The LPS22HH is available in a full-mold, holed
LGA package (HLGA). It is guaranteed to operate
over a temperature range extending from -40 °C
to +85 °C. The package is holed to allow external
pressure to reach the sensing element

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lps22hh.html](https://www.st.com/en/mems-and-sensors/lps22hh.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lps22hh-rs = "0.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add lps22hh-rs
```

## Usage

Include the crate and its prelude
```rust
use lps22hh_rs as lps22hh;
use lps22hh::*;
use lps22hh::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance.

An example with I2C:

```rust
let mut sensor = Lps22hh::new_i2c(i2c, I2CAddress::AddressH);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.reset_set(1).unwrap();
loop {
    if sensor.reset_get().unwrap() == 0 {
        break;
    }
}

// Enable Block Data Update
sensor.block_data_update_set(1).unwrap();

// Set Output Data Rate
sensor.data_rate_set(Odr::_10hzLowNoise).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**