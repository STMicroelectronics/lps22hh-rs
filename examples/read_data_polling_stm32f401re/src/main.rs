#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;

use cortex_m_rt::entry;

use panic_halt as _;

use lps22hh_rs::{prelude::*, I2CAddress, Lps22hh};
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::Config,
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8;
    let sda = gpiob.pb9;

    let i2c: I2c<pac::I2C1> = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = dp
        .USART2
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

    delay.delay_ms(5);

    let mut sensor = Lps22hh::new_i2c(i2c, I2CAddress::AddressH);
    let whoami = sensor.device_id_get();
    match whoami {
        Ok(value) => writeln!(tx, "LPS22HH id: {:#02x}", value).unwrap(),
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    let _ = sensor.reset_set(1);

    loop {
        if sensor.reset_get().unwrap() == 0 {
            break;
        }
    }

    let _ = sensor.block_data_update_set(1);
    let _ = sensor.data_rate_set(Odr::_10hzLowNoise);

    loop {
        let status = sensor.status_reg_get().unwrap();

        if status.p_da() == 1 {
            let press_raw = sensor.pressure_raw_get();

            match press_raw {
                Ok(value) => {
                    writeln!(tx, "press: {} hPa", lps22hh_rs::from_lsb_to_hpa(value)).unwrap()
                }
                Err(e) => writeln!(tx, "Error in reading pressure: {:?}", e).unwrap(),
            }
        }

        if status.t_da() == 1 {
            let temp_raw = sensor.temperature_raw_get();
            match temp_raw {
                Ok(value) => writeln!(
                    tx,
                    "temp: {:.2} C",
                    lps22hh_rs::from_lsb_to_celsius(value)
                )
                .unwrap(),
                Err(e) => writeln!(tx, "Error in reading temperature: {:?}", e).unwrap(),
            }
        }

        delay.delay_ms(1000);
    }
}
