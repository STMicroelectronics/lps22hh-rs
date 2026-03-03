use defmt::info;
use maybe_async::maybe_async;
use crate::*;
#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, mut delay: D, mut int_pin: I) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
    I: InterruptPin
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

    sensor.fifo_watermark_set(32).await.unwrap();
    sensor.fifo_stop_on_wtm_set(1).await.unwrap();
    sensor.fifo_mode_set(FMode::StreamMode).await.unwrap();

    sensor.int_notification_set(Lir::Latched).await.unwrap();

    let mut int_route = sensor.pin_int_route_get().await.unwrap();
    int_route.fifo_th = true;
    sensor.pin_int_route_set(int_route).await.unwrap();

    /* Set Output Data Rate */
    sensor.data_rate_set(Odr::_10hz).await.unwrap();

    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;

        let wtm_flag = sensor.fifo_wtm_flag_get().await.unwrap();

        if wtm_flag == 1 {
            let num = sensor.fifo_data_level_get().await.unwrap();
            writeln!(tx, "-- FIFO interrupt {}", num).unwrap();
            for _i in 0..num {
                let press_raw = sensor.fifo_pressure_raw_get().await;
                match press_raw {
                    Ok(value) => {
                        writeln!(tx, "press: {} hPa", from_lsb_to_hpa(value))
                            .unwrap()
                    }
                    Err(e) => writeln!(tx, "Error in reading pressure: {:?}", e).unwrap(),
                }

                let temp_raw = sensor.fifo_temperature_raw_get().await;
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
            writeln!(tx, "--").unwrap();
        }
    }

}
