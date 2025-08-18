#![no_std]
#![doc = include_str!("../README.md")]

pub mod prelude;
pub mod register;

use core::fmt::Debug;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;

use prelude::*;
use st_mems_bus::*;

/// Driver for LPS22HH sensor.
///
/// The struct takes a bus object to write to the registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
pub struct Lps22hh<B> {
    /// The bus driver.
    bus: B,
}

/// Driver errors.
#[derive(Debug)]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    UnexpectedValue, // Unexpected value read from a register
}
impl<P> Lps22hh<i2c::I2cBus<P>>
where
    P: I2c,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self { bus }
    }
}
impl<P> Lps22hh<spi::SpiBus<P>>
where
    P: SpiDevice,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self { bus }
    }
}
impl<B: BusOperation> Lps22hh<B> {
    /// Read Register Data
    ///
    /// Reads multiple bytes from a specified register into a buffer by sending the register address and receiving the data.
    pub fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_from_register(reg, buf).map_err(Error::Bus)
    }

    /// Write Register Data
    ///
    /// Writes multiple bytes to a specified register, splitting the data into chunks if necessary to comply with bus limitations.
    pub fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_to_register(reg, buf).map_err(Error::Bus)
    }

    /// Reset Autozero Function
    ///
    /// Sets the RESET_AZ bit in the interrupt configuration register to reset the Autozero function, clearing AUTOZERO and reference pressure registers.
    pub fn autozero_rst_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_reset_az(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Autozero Reset Status
    ///
    /// Reads the current state of the RESET_AZ bit from the interrupt configuration register.
    pub fn autozero_rst_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = InterruptCfg::read(self).map(|reg| reg.reset_az())?;

        Ok(val)
    }
    /// Enable Autozero Function
    ///
    /// Enables or disables the Autozero function. When enabled, the sensor uses the current pressure as a reference and stores it internally. The AUTOZERO bit clears automatically after the first measurement.
    pub fn autozero_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_autozero(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Autozero Enable Status
    ///
    /// Retrieves the current state of the Autozero enable bit.
    pub fn autozero_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = InterruptCfg::read(self).map(|reg| reg.autozero())?;

        Ok(val)
    }
    /// Reset AutoRefP Function
    ///
    /// Sets the RESET_ARP bit to reset the AutoRefP function, clearing the AutoRefP interrupt reference.
    pub fn pressure_snap_rst_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_reset_arp(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get AutoRefP Reset Status
    ///
    /// Reads the current state of the RESET_ARP bit from the interrupt configuration register.
    pub fn pressure_snap_rst_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = InterruptCfg::read(self).map(|reg| reg.reset_arp())?;

        Ok(val)
    }
    /// Enable AutoRefP Function
    ///
    /// Enables or disables the AutoRefP function. When enabled, differential pressure is
    /// used for interrupt generation without modifying the pressure output registers.
    pub fn pressure_snap_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_autorefp(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get AutoRefP Enable Status
    ///
    /// Retrieves the current state of the AutoRefP enable bit.
    pub fn pressure_snap_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = InterruptCfg::read(self).map(|reg| reg.autorefp())?;

        Ok(val)
    }
    /// Set Block Data Update (BDU)
    ///
    /// Controls whether output registers are updated continuously or only after both
    /// MSB and LSB are read, ensuring data consistency.
    pub fn block_data_update_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_bdu(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get Block Data Update (BDU) Status
    ///
    /// Reads the current setting of the block data update feature.
    pub fn block_data_update_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg1::read(self).map(|reg| reg.bdu())?;

        Ok(val)
    }
    /// Set Output Data Rate (ODR)
    ///
    /// Configures the sensor's output data rate and related modes such as
    /// low-noise and one-shot.
    pub fn data_rate_set(&mut self, val: Odr) -> Result<(), Error<B::Error>> {
        // Read the current values of the control registers
        let mut ctrl_reg1 = CtrlReg1::read(self)?;
        let mut ctrl_reg2 = CtrlReg2::read(self)?;

        // Update the ODR field in the control register 1
        ctrl_reg1.set_odr((val as u8) & 0x07);
        ctrl_reg1.write(self)?;

        // Update the low noise and one shot fields in the control register 2
        //let val_u8 = val as u8;
        ctrl_reg2.set_low_noise_en((val as u8 & 0x10) >> 4);
        ctrl_reg2.set_one_shot((val as u8 & 0x08) >> 3);
        ctrl_reg2.write(self)?;

        Ok(())
    }
    /// Get Output Data Rate (ODR)
    ///
    /// Reads the current output data rate and mode settings from the sensor.
    pub fn data_rate_get(&mut self) -> Result<Odr, Error<B::Error>> {
        let ctrl_reg1 = CtrlReg1::read(self)?;
        let ctrl_reg2 = CtrlReg2::read(self)?;

        let combined_value =
            (ctrl_reg2.low_noise_en() << 4) + (ctrl_reg2.one_shot() << 3) + ctrl_reg1.odr();

        let val = Odr::try_from(combined_value).unwrap_or_default();

        Ok(val)
    }
    /// Set Reference Pressure
    ///
    /// Writes a 16-bit (2's complement reference pressure value used in
    /// AUTOZERO or AUTOREFP modes.    
    pub fn pressure_ref_set(&mut self, val: i16) -> Result<(), Error<B::Error>> {
        let ref_p = RefP::new().with_ref_p(val);
        ref_p.write(self)
    }
    /// Get Reference Pressure
    ///
    /// Reads the 16-bit (2's complement) reference pressure value used in
    /// AUTOZERO or AUTOREFP modes.
    pub fn pressure_ref_get(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(RefP::read(self)?.ref_p())
    }
    /// Set Pressure Offset
    ///
    /// Writes a 16-bit pressure offset value used for one-point calibration (OPC)
    /// after soldering.    
    pub fn pressure_offset_set(&mut self, val: i16) -> Result<(), Error<B::Error>> {
        Rpds::new().with_rpds(val).write(self)
    }
    /// Get Pressure Offset
    ///
    /// Reads the 16-bit pressure offset value used for one-point calibration.
    pub fn pressure_offset_get(&mut self) -> Result<i16, Error<B::Error>> {
        Rpds::read(self).map(|reg| reg.rpds())
    }
    /// Read All Interrupt and Status Flags
    ///
    /// Retrieves the current interrupt source, FIFO status, and general status flags from the sensor.    
    pub fn all_sources_get(&mut self) -> Result<(IntSource, FifoStatus2, Status), Error<B::Error>> {
        Ok((
            IntSource::read(self)?,
            FifoStatusReg::read(self)?.into(),
            Status::read(self)?,
        ))
    }
    /// Read Status Register
    ///
    /// Reads the status register which indicates data availability and overrun conditions.    
    pub fn status_reg_get(&mut self) -> Result<Status, Error<B::Error>> {
        Status::read(self)
    }
    /// Check Pressure Data Ready Flag
    ///
    /// Indicates if new pressure data is available to read.
    pub fn press_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Status::read(self).map(|reg| reg.p_da())?;

        Ok(val)
    }
    /// Check Temperature Data Ready Flag
    ///
    /// Indicates if new temperature data is available to read.
    pub fn temp_flag_data_ready_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = Status::read(self).map(|reg| reg.t_da())?;

        Ok(val)
    }
    /// Read Raw Pressure Data
    ///
    /// Reads the 24-bit raw pressure output from the sensor registers.
    pub fn pressure_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        let reg = PressOut::read(self)?;
        Ok(reg.pressure())
    }
    /// Read Raw Temperature Data
    ///
    /// Reads the 16-bit raw temperature output from the sensor registers.
    pub fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        let val = TempOut::read(self)?;
        Ok(val.temperature())
    }
    /// Read Raw Pressure Data from FIFO
    ///
    /// Reads the 24-bit raw pressure data stored in the FIFO buffer.
    pub fn fifo_pressure_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        let reg = FifoDataOutPress::read(self)?;
        Ok(reg.pressure())
    }
    /// Read Raw Temperature Data from FIFO
    ///
    /// Reads the 16-bit raw temperature data stored in the FIFO buffer.
    pub fn fifo_temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        let fifo_temp = FifoDataOutTemp::read(self)?;
        Ok(fifo_temp.temperature())
    }
    /// Read Device ID
    ///
    /// Reads the device identification register to verify sensor identity.
    pub fn device_id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).map(|reg| reg.who_am_i())
    }
    /// Perform Software Reset
    ///
    /// Sets the software reset bit to restore default values in user registers.
    pub fn reset_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_swreset(val);
        reg.write(self)
    }
    /// Read Software Reset Status
    ///
    /// Reads the software reset bit to check if a reset is in progress or completed.
    pub fn reset_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg2::read(self)?.swreset();

        Ok(val)
    }
    /// Enable or Disable Register Auto-Increment
    ///
    /// Controls whether the register address automatically increments during multi-byte serial interface accesses.
    pub fn auto_increment_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_if_add_inc(val);
        reg.write(self)
    }
    /// Get Register Auto-Increment Status
    ///
    /// Reads the current setting of the register address auto-increment feature.
    pub fn auto_increment_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg2::read(self)?.if_add_inc();

        Ok(val)
    }
    /// Reload Calibration Parameters
    ///
    /// Triggers a reboot of memory content to reload factory calibration parameters from internal flash.
    pub fn boot_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_boot(val);
        reg.write(self)
    }
    /// Get Calibration Reload Status
    ///
    /// Reads the status of the boot bit indicating if calibration parameters are being reloaded.
    pub fn boot_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = CtrlReg2::read(self)?.boot();

        Ok(val)
    }
    /// Configure Low-Pass Filter Bandwidth
    ///
    /// Selects the low-pass filter bandwidth to reduce noise on pressure data.
    pub fn lp_bandwidth_set(&mut self, val: LpfpCfg) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_lpfp_cfg(val as u8);
        reg.write(self)
    }
    /// Get Low-Pass Filter Bandwidth Setting
    ///
    /// Reads the current low-pass filter bandwidth configuration.
    pub fn lp_bandwidth_get(&mut self) -> Result<LpfpCfg, Error<B::Error>> {
        let reg = CtrlReg1::read(self)?;

        let val = LpfpCfg::try_from(reg.lpfp_cfg()).unwrap_or_default();

        Ok(val)
    }
    /// Enable or Disable I2C Interface
    ///
    /// Controls whether the I2C interface is enabled or disabled.
    pub fn i2c_interface_set(&mut self, val: I2cMode) -> Result<(), Error<B::Error>> {
        let mut reg = IfCtrl::read(self)?;
        reg.set_i2c_disable(val as u8);
        reg.write(self)
    }
    /// Get I2C Interface Status
    ///
    /// Reads whether the I2C interface is currently enabled or disabled.
    pub fn i2c_interface_get(&mut self) -> Result<I2cMode, Error<B::Error>> {
        let reg = IfCtrl::read(self)?;

        let val = I2cMode::try_from(reg.i2c_disable()).unwrap_or_default();
        Ok(val)
    }
    /// Configure MIPI I3C Interface and Interrupt Pin
    ///
    /// Enables or disables the MIPI I3C communication protocol and configures the interrupt pin.
    pub fn i3c_interface_set(&mut self, val: I3cMode) -> Result<(), Error<B::Error>> {
        let mut reg = IfCtrl::read(self)?;
        reg.set_i3c_disable((val as u8) & 0x01);
        reg.set_int_en_i3c(((val as u8) & 0x10) >> 4);
        reg.write(self)
    }
    /// Get MIPI I3C Interface and Interrupt Pin Status
    ///
    /// Reads the current configuration of the MIPI I3C interface and interrupt pin.  
    pub fn i3c_interface_get(&mut self) -> Result<I3cMode, Error<B::Error>> {
        let reg = IfCtrl::read(self)?;
        let reg_int_en_i3c = reg.int_en_i3c();
        let reg_i3c_disable = reg.i3c_disable();
        let val = (reg_int_en_i3c << 4) + reg_i3c_disable;
        let val = I3cMode::try_from(val).unwrap_or_default();

        Ok(val)
    }
    /// Enable or Disable Pull-Up on SDO Pin
    ///
    /// Controls the internal pull-up resistor connection on the SDO pin.
    pub fn sdo_sa0_mode_set(&mut self, val: PullUp) -> Result<(), Error<B::Error>> {
        let mut reg = IfCtrl::read(self)?;
        reg.set_sdo_pu_en(val as u8);
        reg.write(self)
    }
    /// Get Pull-Up Status on SDO Pin
    ///
    /// Reads whether the internal pull-up resistor on the SDO pin is connected or disconnected.    
    pub fn sdo_sa0_mode_get(&mut self) -> Result<PullUp, Error<B::Error>> {
        let tmp: u8 = IfCtrl::read(self)?.sdo_pu_en();
        let val = PullUp::try_from(tmp).unwrap_or_default();
        Ok(val)
    }
    /// Enable or Disable Pull-Up on SDA Pin
    ///
    /// Controls the internal pull-up resistor connection on the SDA pin.
    pub fn sda_mode_set(&mut self, val: PullUp) -> Result<(), Error<B::Error>> {
        let mut reg = IfCtrl::read(self)?;
        reg.set_sda_pu_en(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Pull-Up Status on SDA Pin
    ///
    /// Reads whether the internal pull-up resistor on the SDA pin is connected or disconnected.
    pub fn sda_mode_get(&mut self) -> Result<PullUp, Error<B::Error>> {
        let reg = IfCtrl::read(self)?;

        let val = PullUp::try_from(reg.sda_pu_en()).unwrap_or_default();
        Ok(val)
    }
    /// Set SPI Interface Mode
    ///
    /// Selects between 4-wire and 3-wire SPI interface modes for communication.
    pub fn spi_mode_set(&mut self, val: Sim) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg1::read(self)?;
        reg.set_sim(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get SPI Interface Mode
    ///
    /// Reads the current SPI interface mode setting (4-wire or 3-wire).
    pub fn spi_mode_get(&mut self) -> Result<Sim, Error<B::Error>> {
        let reg = CtrlReg1::read(self)?;

        let val = Sim::try_from(reg.sim()).unwrap_or_default();
        Ok(val)
    }
    /// Set Interrupt Request Latch Mode
    ///
    /// Configures whether interrupt requests are pulsed or latched.
    pub fn int_notification_set(&mut self, val: Lir) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_lir(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Interrupt Request Latch Mode
    ///
    /// Reads the current interrupt request latch configuration.
    pub fn int_notification_get(&mut self) -> Result<Lir, Error<B::Error>> {
        let reg = InterruptCfg::read(self)?;

        let val = Lir::try_from(reg.lir()).unwrap_or_default();
        Ok(val)
    }
    /// Set Interrupt Pin Output Mode
    ///
    /// Selects push-pull or open-drain configuration for interrupt output pads.    
    pub fn pin_mode_set(&mut self, val: PpOd) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_pp_od(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Interrupt Pin Output Mode
    ///
    /// Reads the current push-pull or open-drain configuration of interrupt pads.
    pub fn pin_mode_get(&mut self) -> Result<PpOd, Error<B::Error>> {
        let reg = CtrlReg2::read(self)?;

        let val = PpOd::try_from(reg.pp_od()).unwrap_or_default();
        Ok(val)
    }
    /// Set Interrupt Active Level
    ///
    /// Configures whether interrupts are active-high or active-low.
    pub fn pin_polarity_set(&mut self, val: IntHL) -> Result<(), Error<B::Error>> {
        let mut reg = CtrlReg2::read(self)?;
        reg.set_int_h_l(val as u8);
        reg.write(self)?;

        Ok(())
    }
    /// Get Interrupt Active Level
    ///
    /// Reads the current interrupt active-high or active-low configuration.
    pub fn pin_polarity_get(&mut self) -> Result<IntHL, Error<B::Error>> {
        let reg = CtrlReg2::read(self)?;

        let val = IntHL::try_from(reg.int_h_l()).unwrap_or_default();

        Ok(val)
    }
    /// Configure Interrupt Signal Routing on INT1 Pin
    ///
    /// Routes various interrupt signals such as data-ready, FIFO watermark, FIFO overrun, and
    /// FIFO full flags to the INT1 pin.    
    pub fn pin_int_route_set(&mut self, val: PinIntRoute) -> Result<(), Error<B::Error>> {
        let mut ctrl_reg3 = CtrlReg3::read(self)?;

        ctrl_reg3.set_drdy(val.drdy_pres as u8);
        ctrl_reg3.set_int_f_wtm(val.fifo_th as u8);
        ctrl_reg3.set_int_f_ovr(val.fifo_ovr as u8);
        ctrl_reg3.set_int_f_full(val.fifo_full as u8);

        ctrl_reg3.write(self)?;

        Ok(())
    }
    /// Get Interrupt Signal Routing on INT1 Pin
    ///
    /// Reads which interrupt signals are currently routed to the INT1 pin.
    pub fn pin_int_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        let ctrl_reg3 = CtrlReg3::read(self)?;

        let val = PinIntRoute {
            drdy_pres: ctrl_reg3.drdy() != 0,
            fifo_th: ctrl_reg3.int_f_wtm() != 0,
            fifo_ovr: ctrl_reg3.int_f_ovr() != 0,
            fifo_full: ctrl_reg3.int_f_full() != 0,
        };

        Ok(val)
    }
    /// Enable Interrupt on Pressure Threshold Events
    ///
    /// Configures interrupt generation on pressure low/high threshold events.
    /// Disables interrupt generation if no threshold is selected.    
    pub fn int_on_threshold_set(&mut self, val: Pe) -> Result<(), Error<B::Error>> {
        let mut reg = InterruptCfg::read(self)?;
        reg.set_pe(val as u8);

        if (val as u8) == (Pe::NoThreshold as u8) {
            reg.set_diff_en(0_u8);
        } else {
            reg.set_diff_en(1_u8);
        }

        reg.write(self)?;

        Ok(())
    }
    /// Get Interrupt on Pressure Threshold Status
    ///
    /// Reads the current configuration of interrupt generation on pressure threshold events.
    pub fn int_on_threshold_get(&mut self) -> Result<Pe, Error<B::Error>> {
        let reg = InterruptCfg::read(self)?;

        let val = Pe::try_from(reg.pe()).unwrap_or_default();
        Ok(val)
    }
    /// Set Pressure Interrupt Threshold
    ///
    /// Writes a 15-bit user-defined threshold value for pressure interrupt events.
    /// The threshold value is scaled as threshold (hPa) × 16.
    pub fn int_threshold_set(&mut self, buff: u16) -> Result<(), Error<B::Error>> {
        ThsP::from_bits(buff).write(self)
    }
    /// Get Pressure Interrupt Threshold
    ///
    /// Reads the 15-bit user-defined threshold value for pressure interrupt events.
    pub fn int_threshold_get(&mut self) -> Result<u16, Error<B::Error>> {
        let val = ThsP::read(self)?;
        Ok(val.ths())
    }
    /// Set FIFO Mode
    ///
    /// Selects the FIFO operating mode such as bypass, FIFO, continuous stream, or triggered modes.
    pub fn fifo_mode_set(&mut self, val: FMode) -> Result<(), Error<B::Error>> {
        let mut reg = FifoCtrl::read(self)?;
        reg.set_f_mode(val as u8);
        reg.write(self)
    }
    /// Get FIFO Mode
    ///
    /// Reads the current FIFO operating mode.
    pub fn fifo_mode_get(&mut self) -> Result<FMode, Error<B::Error>> {
        let reg = FifoCtrl::read(self)?;
        let val = FMode::try_from(reg.f_mode()).unwrap_or_default();

        Ok(val)
    }
    /// Enable or Disable FIFO Stop on Watermark
    ///
    /// Configures whether FIFO stops filling when the watermark level is reached.
    pub fn fifo_stop_on_wtm_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = FifoCtrl::read(self)?;
        reg.set_stop_on_wtm(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get FIFO Stop on Watermark Status
    ///
    /// Reads whether FIFO stop on watermark is enabled.
    pub fn fifo_stop_on_wtm_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoCtrl::read(self).map(|reg| reg.stop_on_wtm())?;

        Ok(val)
    }
    /// Set FIFO Watermark Level
    ///
    /// Sets the FIFO watermark level which triggers interrupts when reached.
    pub fn fifo_watermark_set(&mut self, val: u8) -> Result<(), Error<B::Error>> {
        let mut reg = FifoWtm::read(self)?;
        reg.set_wtm(val);
        reg.write(self)?;

        Ok(())
    }
    /// Get FIFO Watermark Level
    ///
    /// Reads the current FIFO watermark level.
    pub fn fifo_watermark_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoWtm::read(self).map(|reg| reg.wtm())?;

        Ok(val)
    }
    /// Get FIFO Data Level
    ///
    /// Reads the number of unread samples currently stored in the FIFO buffer.
    pub fn fifo_data_level_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val = FifoStatusReg::read(self)?.into_bits();
        Ok(val.to_le_bytes()[0])
    }
    /// Get FIFO Status Flags
    ///
    /// Reads FIFO status flags including full, overrun, and watermark interrupt active flags.
    pub fn fifo_src_get(&mut self) -> Result<FifoStatus2, Error<B::Error>> {
        let val = FifoStatusReg::read(self)?;
        Ok(val.into())
    }
    /// Get FIFO Full Flag
    ///
    /// Indicates if the FIFO buffer is completely full.
    pub fn fifo_full_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoStatusReg::read(self).map(|reg| reg.fifo_full_ia())?;

        Ok(val)
    }
    /// Get FIFO Overrun Flag
    ///
    /// Indicates if FIFO data has been overwritten due to overrun
    pub fn fifo_ovr_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoStatusReg::read(self).map(|reg| reg.fifo_ovr_ia())?;

        Ok(val)
    }
    /// Get FIFO Watermark Flag
    ///
    /// Indicates if the FIFO watermark level has been reached.
    pub fn fifo_wtm_flag_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoStatusReg::read(self).map(|reg| reg.fifo_wtm_ia())?;

        Ok(val)
    }
}

pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    (lsb as f32) / 100.0
}

pub fn from_lsb_to_hpa(lsb: u32) -> f32 {
    (lsb as f32) / 4096.0
}

/// I2CAddress - Available I2C addresses for the  sensor
///
/// Variants:
/// - `AddressH` (0x5D): High address
/// - `AddressL` (0x5C): Low address
#[repr(u8)]
pub enum I2CAddress {
    AddressH = 0x5D,
    AddressL = 0x5C,
}

/// Device ID for the  sensor
pub const ID: u8 = 0xB3;
