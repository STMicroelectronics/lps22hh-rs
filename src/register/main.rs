use crate::Error;
use crate::Lps22hh;
use bitfield_struct::bitfield;
use derive_more::TryFrom;
use st_mem_bank_macro::register;
use st_mems_bus::BusOperation;

/// Register addresses for LPS22HH sensor
///
/// This enum represents the 8-bit register addresses embedded in the device.
/// Registers marked as Reserved must not be changed to avoid permanent damage.
/// The content of registers loaded at boot contains factory calibration values and should not be changed.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Reg {
    /// Interrupt configuration register
    InterruptCfg = 0x0B,
    /// Pressure threshold low register (least significant bits)
    ThsPL = 0x0C,
    /// Pressure threshold high register (most significant bits)
    ThsPH = 0x0D,
    /// Interface control register
    IfCtrl = 0x0E,
    /// Device identification register
    WhoAmI = 0x0F,
    /// Control register 1
    CtrlReg1 = 0x10,
    /// Control register 2
    CtrlReg2 = 0x11,
    /// Control register 3
    CtrlReg3 = 0x12,
    /// FIFO control register
    FifoCtrl = 0x13,
    /// FIFO watermark level register
    FifoWtm = 0x14,
    /// Reference pressure low register
    RefPL = 0x15,
    /// Reference pressure high register
    RefPH = 0x16,
    /// Pressure offset low register
    RpdsL = 0x18,
    /// Pressure offset high register
    RpdsH = 0x19,
    /// Interrupt source register
    IntSource = 0x24,
    /// FIFO status register 1
    FifoStatus1 = 0x25,
    /// FIFO status register 2
    FifoStatus2 = 0x26,
    /// Status register
    Status = 0x27,
    /// Pressure output least significant byte
    PressOutXl = 0x28,
    /// Pressure output middle byte
    PressOutL = 0x29,
    /// Pressure output most significant byte
    PressOutH = 0x2A,
    /// Temperature output least significant byte
    TempOutL = 0x2B,
    /// Temperature output most significant byte
    TempOutH = 0x2C,
    /// FIFO pressure output least significant byte
    FifoDataOutPressXl = 0x78,
    /// FIFO pressure output middle byte
    FifoDataOutPressL = 0x79,
    /// FIFO pressure output most significant byte
    FifoDataOutPressH = 0x7A,
    /// FIFO temperature output least significant byte
    FifoDataOutTempL = 0x7B,
    /// FIFO temperature output most significant byte
    FifoDataOutTempH = 0x7C,
}

/// INTERRUPT_CFG (0x0B)
///
/// Interrupt configuration register (R/W)
///
/// Configures interrupt generation based on pressure threshold events,
/// autozero and autoreference reset and enable functions, and interrupt latching.
#[register(address = Reg::InterruptCfg, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InterruptCfg {
    /// Pressure event interrupt enable (2 bits)
    /// 00: No threshold (interrupt disabled)
    /// 01: Positive threshold event enabled
    /// 10: Negative threshold event enabled
    /// 11: Both positive and negative threshold events enabled
    #[bits(2)]
    pub pe: u8,
    /// Latch interrupt request to INT_SOURCE register (1 bit)
    /// 0: Interrupt request not latched (pulsed)
    /// 1: Interrupt request latched
    #[bits(1)]
    pub lir: u8,
    /// Differential pressure interrupt enable (1 bit)
    /// 0: Interrupt generation disabled
    /// 1: Interrupt generation enabled
    #[bits(1)]
    pub diff_en: u8,
    /// Reset Autozero function (1 bit)
    /// Writing 1 resets AUTOZERO and REF_P registers to 0
    #[bits(1)]
    pub reset_az: u8,
    /// Enable Autozero function (1 bit)
    /// When set, measured pressure is used as reference and stored in REF_P
    /// AUTOZERO bit auto-clears after first conversion
    #[bits(1)]
    pub autozero: u8,
    /// Reset AutoRefP function (1 bit)
    /// Writing 1 resets AUTOREFP function
    #[bits(1)]
    pub reset_arp: u8,
    /// Enable AutoRefP function (1 bit)
    /// When set, differential pressure is used for interrupt generation without changing PRESS_OUT registers
    #[bits(1)]
    pub autorefp: u8,
}

/// THS_P_L - THS_P_H (0x0C - 0x0D)
///
/// User-defined threshold value for pressure interrupt event (R/W)
///
/// Contains the 15-bit threshold pressure value used for interrupt generation.
/// Threshold value = Desired interrupt threshold (hPa) × 16
#[register(address = Reg::ThsPL, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct ThsP {
    /// Threshold pressure low bits [7:0] (8 bits)
    #[bits(15)]
    pub ths: u16,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// IF_CTRL (0x0E)
///
/// Interface control register (R/W)
///
/// Controls enabling/disabling of I2C and MIPI I3C interfaces, pull-up and pull-down resistors,
/// and interrupt pin configuration.
#[register(address = Reg::IfCtrl, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IfCtrl {
    /// I2C interface disable (1 bit)
    /// - 0: I2C enabled
    /// - 1: I2C disabled
    #[bits(1)]
    pub i2c_disable: u8,
    /// MIPI I3C interface disable (1 bit)
    /// - 0: MIPI I3C enabled
    /// - 1: MIPI I3C disabled
    #[bits(1)]
    pub i3c_disable: u8,
    /// Pull-down disable on INT1 pin (1 bit)
    /// - 0: INT1 pin with pull-down enabled
    /// - 1: INT1 pin pull-down disabled
    #[bits(1)]
    pub pd_dis_int1: u8,
    /// SDO pull-up enable (1 bit)
    /// - 0: Pull-up disconnected
    /// - 1: Pull-up connected
    #[bits(1)]
    pub sdo_pu_en: u8,
    /// SDA pull-up enable (1 bit)
    /// - 0: Pull-up disconnected
    /// - 1: Pull-up connected
    #[bits(1)]
    pub sda_pu_en: u8,
    #[bits(2, access = RO)]
    pub not_used_01: u8,
    /// Interrupt enable on INT1 pin for MIPI I3C (1 bit)
    /// - 0: INT1 disabled with MIPI I3C
    /// - 1: INT1 enabled with MIPI I3C
    #[bits(1)]
    pub int_en_i3c: u8,
}

/// WHO_AM_I (0x0F)
///
/// The identification register is used to identity the device (R)
#[register(address = Reg::WhoAmI, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WhoAmI {
    #[bits(8, access = RO)]
    pub who_am_i: u8,
}

/// CTRL_REG1 (0x10)
///
/// Control register 1 (R/W)
///
/// Controls output data rate, low-pass filter, block data update, and SPI interface mode.
#[register(address = Reg::CtrlReg1, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg1 {
    /// SPI Serial Interface Mode selection (1 bit)
    /// - 0: 4-wire SPI interface
    /// - 1: 3-wire SPI interface
    #[bits(1)]
    pub sim: u8,
    /// Block Data Update (1 bit)
    /// - 0: Continuous update of output registers
    /// - 1: Output registers not updated until MSB and LSB are read
    #[bits(1)]
    pub bdu: u8,
    /// Low-pass filter configuration (2 bits)
    /// - 00: Disabled (bandwidth = ODR/2)
    /// - 01: Enabled (bandwidth = ODR/9)
    /// - 10: Enabled (bandwidth = ODR/20)
    #[bits(2)]
    pub lpfp_cfg: u8,
    /// Output data rate selection (3 bits)
    /// - 000: Power-down mode
    /// - 001: 3 Hz
    /// - 010: 10 Hz
    /// - 011: 25 Hz
    /// - 100: 50 Hz
    /// - 101: 75 Hz
    /// - 110: 100 Hz (low-noise mode disabled automatically)
    /// - 111: 200 Hz (low-noise mode disabled automatically)
    #[bits(3)]
    pub odr: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}
/// CTRL_REG2 (0x11)
///
/// Control register 2 (R/W)
///
/// Controls one-shot mode, low noise mode, software reset, address auto-increment,
/// interrupt pin configuration, and reboot memory content.
#[register(address = Reg::CtrlReg2, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg2 {
    /// One-shot mode enable (1 bit)
    /// - 0: Idle mode
    /// - 1: Triggers a single acquisition when device is in power-down mode
    #[bits(1)]
    pub one_shot: u8,
    /// Low noise enable (1 bit)
    /// - 0: Low current mode
    /// - 1: Low noise mode (only valid if ODR < 100 Hz)
    #[bits(1)]
    pub low_noise_en: u8,
    /// Software reset (1 bit)
    /// Writing 1 resets volatile registers to default values; bit self-clears after reset
    #[bits(1)]
    pub swreset: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
    /// Register address auto-increment (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled (default)
    #[bits(1)]
    pub if_add_inc: u8,
    /// Push-pull/open-drain selection on interrupt pads (1 bit)
    /// - 0: Push-pull
    /// - 1: Open-drain
    #[bits(1)]
    pub pp_od: u8,
    /// Interrupt active level (1 bit)
    /// - 0: Active high
    /// - 1: Active low
    #[bits(1)]
    pub int_h_l: u8,
    /// Reboot memory content (1 bit)
    /// Writing 1 reloads calibration parameters from internal flash; bit auto-clears
    #[bits(1)]
    pub boot: u8,
}

/// CTRL_REG3 (0x12)
///
/// Control register 3 - INT_DRDY pin control register (R/W)
///
/// Controls interrupt signals routed to the INT_DRDY pin.
#[register(address = Reg::CtrlReg3, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg3 {
    /// Data signal on INT_DRDY pin control bits (2 bits)
    /// - 00: Data signal (priority: DRDY or FIFO watermark or FIFO overrun or FIFO full)
    /// - 01: Pressure high event
    /// - 10: Pressure low event
    /// - 11: Pressure low OR high event
    #[bits(2)]
    pub int_s: u8,
    /// Data-ready signal on INT_DRDY pin (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled
    #[bits(1)]
    pub drdy: u8,
    /// FIFO overrun interrupt on INT_DRDY pin (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled
    #[bits(1)]
    pub int_f_ovr: u8,
    /// FIFO watermark status on INT_DRDY pin (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled
    #[bits(1)]
    pub int_f_wtm: u8,
    /// FIFO full flag on INT_DRDY pin (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled
    #[bits(1)]
    pub int_f_full: u8,
    #[bits(2, access = RO)]
    pub not_used_01: u8,
}

/// FIFO_CTRL (0x13)
///
/// FIFO control register (R/W)
///
/// Controls FIFO mode selection and watermark stop behavior.
#[register(address = Reg::FifoCtrl, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl {
    /// FIFO mode selection (3 bits)
    /// - 000: Bypass mode
    /// - 001: FIFO mode
    /// - 01x: Continuous (Dynamic-Stream) mode
    /// - 101: Bypass-to-FIFO mode
    /// - 110: Bypass-to-Continuous (Dynamic-Stream) mode
    /// - 111: Continuous (Dynamic-Stream)-to-FIFO mode
    #[bits(3)]
    pub f_mode: u8,
    /// Stop on FIFO watermark (1 bit)
    /// - 0: Disabled
    /// - 1: Enabled (FIFO depth limited to watermark level)
    #[bits(1)]
    pub stop_on_wtm: u8,
    #[bits(4, access = RO)]
    pub not_used_01: u8,
}

/// FIFO_WTM (0x14)
///
/// FIFO watermark setting register (R/W)
///
/// Sets the FIFO watermark level for interrupt generation.
#[register(address = Reg::FifoWtm, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoWtm {
    /// FIFO watermark level (7 bits)
    #[bits(7)]
    pub wtm: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// REF_P_L - REF_P_H (0x15 - 0x16)
///
/// Reference pressure registers (R/W)
///
/// Contains the 16-bit (expressed in 2's complement) reference pressure value used in AUTOZERO or AUTOREFP modes.
#[register(address = Reg::RefPL, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct RefP {
    /// Reference pressure
    #[bits(16)]
    pub ref_p: i16,
}

/// RPDS_L - RPDS_H (0x18 - 0x19)
///
/// Pressure offset low and high register (R/W)
///
/// Contains the 16-bit (2's complement) pressure offset value used for one-point calibration (OPC).
#[register(address = Reg::RpdsL, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct Rpds {
    /// Pressure offset (16 bits)
    #[bits(16)]
    pub rpds: i16,
}

/// INT_SOURCE (0x24)
///
/// Interrupt source register (R)
#[register(address = Reg::IntSource, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntSource {
    /// Differential pressure high event flag (1 bit)
    #[bits(1)]
    pub ph: u8,
    /// Differential pressure low event flag (1 bit)
    #[bits(1)]
    pub pl: u8,
    /// Interrupt active flag (1 bit)
    #[bits(1)]
    pub ia: u8,
    #[bits(4, access = RO)]
    pub not_used_01: u8,
    /// Boot phase indicator (1 bit)
    #[bits(1)]
    pub boot_on: u8,
}

/// FIFO_STATUS1 - FIFO_STATUS2 (0x25 - 0x26)
///
/// FIFO status register 1 (R)
#[register(address = Reg::FifoStatus1, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FifoStatusReg {
    /// FIFO stored data level, number of unread samples stored in FIFO (8 bits)
    #[bits(8)]
    pub fss: u8,
    #[bits(5, access = RO)]
    pub not_used_01: u8,
    /// FIFO full interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_full_ia: u8,
    /// FIFO overrun interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_ovr_ia: u8,
    /// FIFO watermark interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_wtm_ia: u8,
}

impl From<FifoStatusReg> for FifoStatus1 {
    fn from(value: FifoStatusReg) -> Self {
        let bytes = value.into_bits().to_le_bytes();
        Self::from_bits(bytes[0])
    }
}

impl From<FifoStatusReg> for FifoStatus2 {
    fn from(value: FifoStatusReg) -> Self {
        let bytes = value.into_bits().to_le_bytes();
        Self::from_bits(bytes[1])
    }
}

/// FIFO_STATUS1 (0x25)
///
/// FIFO status register 1 (R)
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus1 {
    /// FIFO stored data level, number of unread samples stored in FIFO (8 bits)
    #[bits(8)]
    pub fss: u8,
}

/// FIFO_STATUS2 (0x26)
///
/// FIFO status register 2 (R)
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus2 {
    #[bits(5, access = RO)]
    pub not_used_01: u8,
    /// FIFO full interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_full_ia: u8,
    /// FIFO overrun interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_ovr_ia: u8,
    /// FIFO watermark interrupt active flag (1 bit)
    #[bits(1)]
    pub fifo_wtm_ia: u8,
}

/// STATUS (0x27)
///
/// Status register (R)
#[register(address = Reg::Status, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Status {
    /// Pressure data available flag (1 bit)
    #[bits(1)]
    pub p_da: u8,
    /// Temperature data available flag (1 bit)
    #[bits(1)]
    pub t_da: u8,
    #[bits(2, access = RO)]
    pub not_used_01: u8,
    /// Pressure data overrun flag (1 bit)
    #[bits(1)]
    pub p_or: u8,
    /// Temperature data overrun flag (1 bit)
    #[bits(1)]
    pub t_or: u8,
    #[bits(2, access = RO)]
    pub not_used_02: u8,
}

/// PRESS_OUT_XL - PRESS_OUT_L - PRESS_OUT_H (0x28 - 0x2A)
///
/// Pressure output value (R)
#[register(address = Reg::PressOutXl, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u32, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u32, order = Lsb))]
pub struct PressOut {
    #[offset_before(8)]
    /// Pressure output (3 byte)
    #[bits(32)]
    pub pressure: u32,
}

/// TempOut (0x2B - 0x2C)
///
/// Temperature register of the sensor in 2's complement format
#[register(address = Reg::TempOutL, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct TempOut {
    /// Temperature output
    #[bits(16)]
    pub temperature: i16,
}

/// FIFO_DATA_OUT_PRESS_XL - FIFO_DATA_OUT_PRESS_L - FIFO_DATA_OUT_PRESS_H (0x78 - 0x7A)
///
/// FIFO pressure output (3 byte) (R)
#[register(address = Reg::FifoDataOutPressXl, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u32, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u32, order = Lsb))]
pub struct FifoDataOutPress {
    #[offset_before(8)]
    /// FIFO pressure output (24 bits)
    #[bits(32)]
    pub pressure: u32,
}

/// FIFO_DATA_OUT_TEMP_L - FIFO_DATA_OUT_TEMP_H (0x7B - 0x7C)
///
/// FIFO temperature output two's complement 16 bit format(R)
#[register(address = Reg::FifoDataOutTempL, access_type = Lps22hh, generics = 1)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct FifoDataOutTemp {
    /// FIFO temperature output (16 bits)
    #[bits(16)]
    pub temperature: i16,
}

/// Output data rate (ODR) settings for LPS22HH sensor
///
/// Controls the frequency of pressure and temperature data acquisition.
/// When set to '000', device is in Power-down mode.
/// Other values correspond to continuous acquisition at specified Hz.
///
/// The device is in Power-down mode when ODR bits are set to '000'.
/// Continuous mode acquires data at frequencies from 1 Hz to 200 Hz.
/// Low-noise mode is enabled only if ODR < 100 Hz and LOW_NOISE_EN bit is set.
/// ODR 100 Hz and 200 Hz disable low-noise mode automatically.
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum Odr {
    /// Power-down mode, no continuous data acquisition.
    #[default]
    PowerDown = 0x00,
    /// One-shot mode triggers a single acquisition.
    OneShoot = 0x08,
    /// 1 Hz output data rate.
    _1hz = 0x01,
    /// 10 Hz output data rate.
    _10hz = 0x02,
    /// 25 Hz output data rate.
    _25hz = 0x03,
    /// 50 Hz output data rate.
    _50hz = 0x04,
    /// 75 Hz output data rate.
    _75hz = 0x05,
    /// 1 Hz output data rate with low-noise mode enabled.
    _1hzLowNoise = 0x11,
    /// 10 Hz output data rate with low-noise mode enabled.
    _10hzLowNoise = 0x12,
    /// 25 Hz output data rate with low-noise mode enabled.
    _25hzLowNoise = 0x13,
    /// 50 Hz output data rate with low-noise mode enabled.
    _50hzLowNoise = 0x14,
    /// 75 Hz output data rate with low-noise mode enabled.
    _75hzLowNoise = 0x15,
    /// 100 Hz output data rate (low-noise mode disabled automatically).
    _100hz = 0x06,
    /// 200 Hz output data rate (low-noise mode disabled automatically).
    _200hz = 0x07,
}

/// Low-pass filter configurations for LPS22HH sensor
///
/// Controls additional low-pass filter status and device bandwidth.
/// - 0: Disabled, bandwidth = ODR/2
/// - 1: Enabled, bandwidth = ODR/9
/// - 2: Enabled, bandwidth = ODR/20
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum LpfpCfg {
    /// Low-pass filter disabled; device bandwidth = ODR/2
    #[default]
    OdrDiv2 = 0,
    /// Low-pass filter enabled; device bandwidth = ODR/9
    OdrDiv9 = 1,
    /// Low-pass filter enabled; device bandwidth = ODR/20
    OdrDiv20 = 2,
}

/// I2C interface enable/disable for LPS22HH sensor
///
/// Controls whether the I2C interface is enabled or disabled.
/// - 0: I2C enabled
/// - 1: I2C disabled
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum I2cMode {
    /// I2C interface enabled
    #[default]
    Enable = 0,
    /// I2C interface disabled
    Disable = 1,
}

/// MIPI I3C interface enable/disable for LPS22HH sensor
///
/// Controls MIPI I3C communication protocol and interrupt pin.
/// - 0x00: I3C enabled, interrupt pin disabled
/// - 0x10: I3C enabled, interrupt pin enabled
/// - 0x11: I3C disabled
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum I3cMode {
    /// I3C enabled, interrupt pin disabled
    #[default]
    Enable = 0x00,
    /// I3C enabled, interrupt pin enabled
    EnableIntPinEnable = 0x10,
    /// I3C disabled
    Disable = 0x11,
}

/// Pull-up enable for SDO or SDA pins on LPS22HH sensor
///
/// Controls connection of internal pull-up resistors.
/// - 0: Pull-up disconnected
/// - 1: Pull-up connected
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum PullUp {
    /// Pull-up disconnected
    #[default]
    Disconnect = 0,
    /// Pull-up connected
    Connect = 1,
}

/// SPI interface mode selection for LPS22HH sensor
///
/// Selects between 4-wire and 3-wire SPI interface modes.
/// - 0: 4-wire SPI interface
/// - 1: 3-wire SPI interface
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum Sim {
    /// 4-wire SPI interface mode
    #[default]
    Spi4Wire = 0,
    /// 3-wire SPI interface mode
    Spi3Wire = 1,
}

/// Interrupt request latch mode for LPS22HH sensor
///
/// Controls whether interrupt requests are pulsed or latched.
/// - 0: Interrupt request not latched (pulsed)
/// - 1: Interrupt request latched
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum Lir {
    /// Interrupt request pulsed
    #[default]
    Pulsed = 0,
    /// Interrupt request latched
    Latched = 1,
}

/// Interrupt pin output mode for LPS22HH sensor
///
/// Controls push-pull or open-drain configuration on interrupt pads.
/// - 0: Push-pull
/// - 1: Open-drain
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum PpOd {
    /// Push-pull output mode
    #[default]
    PushPull = 0,
    /// Open-drain output mode
    OpenDrain = 1,
}

/// Interrupt active level for LPS22HH sensor
///
/// Controls whether interrupt is active-high or active-low.
/// - 0: Active high
/// - 1: Active low
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum IntHL {
    /// Interrupt active high
    #[default]
    ActiveHigh = 0,
    /// Interrupt active low
    ActiveLow = 1,
}

/// Pressure event interrupt enable for LPS22HH sensor
///
/// Controls interrupt generation on pressure threshold events.
/// - 0: No threshold (interrupt disabled)
/// - 1: Positive threshold event enabled
/// - 2: Negative threshold event enabled
/// - 3: Both positive and negative threshold events enabled
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum Pe {
    /// No threshold interrupt (interrupt disabled)
    #[default]
    NoThreshold = 0,
    /// Interrupt on positive pressure threshold event
    Positive = 1,
    /// Interrupt on negative pressure threshold event
    Negative = 2,
    /// Interrupt on both positive and negative pressure threshold events
    Both = 3,
}

/// FIFO mode selection for LPS22HH sensor FIFO buffer operation
///
/// The FIFO buffer can work according to six different modes:
/// - 0: Bypass mode (FIFO not operational, remains empty)
/// - 1: FIFO mode (data stored until FIFO full)
/// - 2,3: Continuous (Dynamic-Stream) mode (streaming data)
/// - 5: Bypass-to-FIFO mode (switches on interrupt)
/// - 6: Bypass-to-Continuous (Dynamic-Stream) mode (switches on interrupt)
/// - 7: Continuous (Dynamic-Stream)-to-FIFO mode (switches on interrupt)
#[repr(u8)]
#[derive(Clone, Copy, Default, TryFrom)]
#[try_from(repr)]
pub enum FMode {
    /// Bypass mode: FIFO is not operational and remains empty.
    #[default]
    BypassMode = 0,
    /// FIFO mode: Data stored in FIFO until full.
    FifoMode = 1,
    /// Continuous (Dynamic-Stream) mode: FIFO behaves as a stream buffer.
    StreamMode = 2,
    /// Continuous (Dynamic-Stream) mode: alternate encoding.
    DynamicStreamMode = 3,
    /// Bypass-to-FIFO mode: FIFO switches from Bypass to FIFO mode on interrupt.
    BypassToFifoMode = 5,
    /// Bypass-to-Continuous (Dynamic-Stream) mode: FIFO switches from Bypass to Continuous mode on interrupt.
    BypassToStreamMode = 6,
    /// Continuous (Dynamic-Stream)-to-FIFO mode: FIFO switches from Continuous to FIFO mode on interrupt.
    StreamToFifoMode = 7,
}

#[derive(Default)]
pub struct PinIntRoute {
    pub drdy_pres: bool,
    pub fifo_th: bool,
    pub fifo_ovr: bool,
    pub fifo_full: bool,
}
