#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;

use panic_halt as _;

use lps22hh_rs::{prelude::*, I2CAddress, Lps22hh};
use stm32f4xx_hal::{
    gpio::{self, Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    pac::{self, interrupt},
    prelude::*,
    serial::Config,
};

type IntPin = gpio::PB10<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
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

    let mut int_pin = gpiob.pb10.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI);

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

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

    let _ = sensor.int_notification_set(Lir::Pulsed);

    let mut int_route = sensor.pin_int_route_get().unwrap();
    int_route.drdy_pres = true;
    let _ = sensor.pin_int_route_set(int_route);

    /* Set Output Data Rate */
    let _ = sensor.block_data_update_set(1);
    let _ = sensor.data_rate_set(Odr::_10hzLowNoise);

    loop {
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }

        let p_drdy = sensor.press_flag_data_ready_get().unwrap();
        let t_drdy = sensor.temp_flag_data_ready_get().unwrap();

        if p_drdy == 1 {
            let press_raw = sensor.pressure_raw_get();

            match press_raw {
                Ok(value) => {
                    writeln!(tx, "press: {} hPa", lps22hh_rs::from_lsb_to_hpa(value)).unwrap()
                }
                Err(e) => writeln!(tx, "Error in reading pressure: {:?}", e).unwrap(),
            }
        }

        if t_drdy == 1 {
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
    }
}

#[interrupt]
fn EXTI15_10() {
    // Start a Critical Section
    cortex_m::interrupt::free(|cs| {
        // Obtain access to Peripheral and Clear Interrupt Pending Flag
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    });
}
