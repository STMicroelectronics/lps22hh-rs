#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use lps22hh_rs::{prelude::*, I2CAddress, Lps22hh};

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    // Configure the interrupt pin (if needed) and obtain handler.
    // On the Nucleo FR401 the interrupt pin is connected to pin PB10.
    let interrupt = Input::new(p.PB10, Pull::None);
    let mut interrupt = ExtiInput::new(interrupt, p.EXTI10);

    let mut msg: String<64> = String::new();

    delay.delay_ms(5_u32);

    let mut sensor = Lps22hh::new_i2c(i2c, I2CAddress::AddressH);
    let whoami = sensor.device_id_get();
    match whoami {
        Ok(value) => {
            writeln!(&mut msg, "LPS22HH id: {:#02x}", value).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
        Err(e) => {
            writeln!(&mut msg, "Error in reading id: {:?}", e).unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }
    }

    let _ = sensor.reset_set(1);

    loop {
        if sensor.reset_get().unwrap() == 0 {
            break;
        }
    }

    let _ = sensor.int_notification_set(Lir::Pulsed);

    let mut int_route = sensor.pin_int_route_get().unwrap();
    int_route.drdy_pres = true;
    let _ = sensor.pin_int_route_set(int_route);

    let _ = sensor.block_data_update_set(1);
    let _ = sensor.data_rate_set(Odr::_10hzLowNoise);

    loop {
        interrupt.wait_for_rising_edge().await;

        let p_drdy = sensor.press_flag_data_ready_get().unwrap();
        let t_drdy = sensor.temp_flag_data_ready_get().unwrap();

        if p_drdy == 1 {
            let press_raw = sensor.pressure_raw_get();

            match press_raw {
                Ok(value) => {
                    writeln!(
                        &mut msg,
                        "press: {} hPa",
                        lps22hh_rs::from_lsb_to_hpa(value)
                    )
                    .unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
                Err(e) => {
                    writeln!(&mut msg, "Error in reading pressure: {:?}", e).unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
            }
        }

        if t_drdy == 1 {
            let temp_raw = sensor.temperature_raw_get();
            match temp_raw {
                Ok(value) => {
                    writeln!(
                        &mut msg,
                        "temp: {:.2} C",
                        lps22hh_rs::from_lsb_to_celsius(value)
                    )
                    .unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
                Err(e) => {
                    writeln!(&mut msg, "Error in reading temperature: {:?}", e).unwrap();
                    let _ = tx.blocking_write(msg.as_bytes());
                    msg.clear();
                }
            }
        }
    }
}
