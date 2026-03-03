use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, _irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use lps22hh::prelude::*;
    use lps22hh::*;

    info!("Configuring the sensor");
    let mut sensor = Lps22hh::from_bus(bus);

    // boot time
    delay.delay_ms(5).await;

    // Check device ID
    let id = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    sensor.reset_set(1).await.unwrap();

    loop {
        if sensor.reset_get().await.unwrap() == 0 {
            break;
        }
    }

    sensor.block_data_update_set(1).await.unwrap();
    sensor.data_rate_set(Odr::_10hzLowNoise).await.unwrap();

    loop {
        let status = sensor.status_reg_get().await.unwrap();

        if status.p_da() == 1 {
            let press_raw = sensor.pressure_raw_get().await;

            match press_raw {
                Ok(value) => {
                    writeln!(tx, "press: {} hPa", from_lsb_to_hpa(value)).unwrap()
                }
                Err(e) => writeln!(tx, "Error in reading pressure: {:?}", e).unwrap(),
            }
        }

        if status.t_da() == 1 {
            let temp_raw = sensor.temperature_raw_get().await;
            match temp_raw {
                Ok(value) => writeln!(
                    tx,
                    "temp: {:.2} C",
                    from_lsb_to_celsius(value)
                )
                .unwrap(),
                Err(e) => writeln!(tx, "Error in reading temperature: {:?}", e).unwrap(),
            }
        }

        delay.delay_ms(1000).await;
    }
}
