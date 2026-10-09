#![no_std]

//! Board-independent setup code for the Bosch BMM350 magnetic sensor.
//!
//! The register behavior, startup sequence, OTP layout, and compensation
//! decoding are adapted from the Bosch Sensortec BMM350 SensorAPI. The license
//! notice is stored in this crate's `LICENSE` file.

pub const DEFAULT_I2C_ADDRESS: u8 = 0x14;
pub const ALTERNATE_I2C_ADDRESS: u8 = 0x15;

const CHIP_ID: u8 = 0x33;
const REG_CHIP_ID: u8 = 0x00;
const REG_PMU_CMD_AGGR_SET: u8 = 0x04;
const REG_PMU_CMD_AXIS_EN: u8 = 0x05;
const REG_PMU_CMD: u8 = 0x06;
const REG_PMU_CMD_STATUS_0: u8 = 0x07;
const REG_OTP_CMD: u8 = 0x50;
const REG_OTP_DATA_MSB: u8 = 0x52;
const REG_OTP_DATA_LSB: u8 = 0x53;
const REG_OTP_STATUS: u8 = 0x55;
const REG_CMD: u8 = 0x7e;

const CMD_SOFT_RESET: u8 = 0xb6;
const OTP_DIRECT_READ: u8 = 0x20;
const OTP_POWER_OFF: u8 = 0x80;
const OTP_WORD_COUNT: usize = 32;
const OTP_STATUS_DONE: u8 = 0x01;
const OTP_STATUS_ERROR_MASK: u8 = 0xe0;
const OTP_POLL_ATTEMPTS: usize = 100;

const POWER_MODE_SUSPEND: u8 = 0x00;
pub const POWER_MODE_NORMAL: u8 = 0x01;
const PMU_CMD_UPDATE_ODR_AVERAGING: u8 = 0x02;
const PMU_CMD_FLUX_GUIDE_RESET: u8 = 0x05;
const PMU_CMD_BIT_RESET: u8 = 0x07;

const PMU_STATUS_BUSY: u8 = 0x01;
const PMU_STATUS_NORMAL: u8 = 0x08;
const PMU_STATUS_ILLEGAL_COMMAND: u8 = 0x10;
const PMU_STATUS_COMMAND_SHIFT: u8 = 5;

pub const ODR_100_HZ: u8 = 0x04;
pub const AVERAGING_4: u8 = 0x02;
pub const AXES_XYZ: u8 = 0x07;

const MAX_REGISTER_READ_LENGTH: usize = 12;
const I2C_DUMMY_BYTE_COUNT: usize = 2;

/// Low-level operations required by the BMM350 driver.
///
/// An application implements this trait for its own board or I2C peripheral.
/// The sensor code can then run without knowing anything about that platform.
pub trait Bmm350Bus {
    type Error;

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error>;

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8])
        -> Result<(), Self::Error>;

    fn delay_us(&mut self, microseconds: u16);
    fn delay_ms(&mut self, milliseconds: u16);
}

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    Bus(E),
    InvalidAddress(u8),
    WrongChipId(u8),
    RegisterReadTooLong,
    OtpTimeout(u8),
    OtpStatus(u8),
    PmuCommand { expected: u8, status: u8 },
    ConfigurationRejected,
}

// These values are read during setup and will be applied when measurement
// support is added. Keeping them here makes the state part of the reusable
// sensor driver instead of the RRIV wrapper.
#[allow(dead_code)]
#[derive(Copy, Clone)]
struct Compensation {
    temperature_offset: f32,
    offset_x: f32,
    offset_y: f32,
    offset_z: f32,
    temperature_sensitivity: f32,
    sensitivity_x: f32,
    sensitivity_y: f32,
    sensitivity_z: f32,
    tco_x: f32,
    tco_y: f32,
    tco_z: f32,
    tcs_x: f32,
    tcs_y: f32,
    tcs_z: f32,
    reference_temperature: f32,
    cross_x_y: f32,
    cross_y_x: f32,
    cross_z_x: f32,
    cross_z_y: f32,
}

/// Persistent state for one BMM350 sensor.
///
/// The bus is passed to each operation rather than stored here. That allows
/// RRIV to lend its shared board object to the sensor only when it is needed.
pub struct Bmm350 {
    address: u8,
    compensation: Option<Compensation>,
    variant_id: Option<u8>,
    initialized: bool,
}

impl Bmm350 {
    pub const fn new(address: u8) -> Self {
        Self {
            address,
            compensation: None,
            variant_id: None,
            initialized: false,
        }
    }

    pub const fn address(&self) -> u8 {
        self.address
    }

    pub const fn variant_id(&self) -> Option<u8> {
        self.variant_id
    }

    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    pub fn reset_runtime_state(&mut self) {
        self.compensation = None;
        self.variant_id = None;
        self.initialized = false;
    }

    /// Reset, identify, calibrate, and configure the sensor for normal mode.
    pub fn initialize<B: Bmm350Bus>(&mut self, bus: &mut B) -> Result<(), Error<B::Error>> {
        self.reset_runtime_state();

        if !matches!(self.address, DEFAULT_I2C_ADDRESS | ALTERNATE_I2C_ADDRESS) {
            return Err(Error::InvalidAddress(self.address));
        }

        bus.delay_ms(3);
        self.write_register(bus, REG_CMD, CMD_SOFT_RESET)?;
        bus.delay_ms(24);

        let chip_id = self.read_register(bus, REG_CHIP_ID)?;
        if chip_id != CHIP_ID {
            return Err(Error::WrongChipId(chip_id));
        }

        let otp = self.read_otp(bus)?;
        let compensation = Compensation::from_otp(&otp);
        let variant_id = ((otp[30] & 0x7f00) >> 9) as u8;

        self.write_register(bus, REG_OTP_CMD, OTP_POWER_OFF)?;
        self.magnetic_reset(bus)?;
        self.configure_sensor(bus)?;

        self.compensation = Some(compensation);
        self.variant_id = Some(variant_id);
        self.initialized = true;
        Ok(())
    }

    fn write_register<B: Bmm350Bus>(
        &self,
        bus: &mut B,
        register: u8,
        value: u8,
    ) -> Result<(), Error<B::Error>> {
        bus.write(self.address, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_register<B: Bmm350Bus>(
        &self,
        bus: &mut B,
        register: u8,
    ) -> Result<u8, Error<B::Error>> {
        let mut value = [0; 1];
        self.read_registers(bus, register, &mut value)?;
        Ok(value[0])
    }

    fn read_registers<B: Bmm350Bus>(
        &self,
        bus: &mut B,
        register: u8,
        values: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        if values.len() > MAX_REGISTER_READ_LENGTH {
            return Err(Error::RegisterReadTooLong);
        }

        let mut response = [0; MAX_REGISTER_READ_LENGTH + I2C_DUMMY_BYTE_COUNT];
        let response_length = values.len() + I2C_DUMMY_BYTE_COUNT;
        bus.write_read(self.address, &[register], &mut response[..response_length])
            .map_err(Error::Bus)?;

        values.copy_from_slice(&response[I2C_DUMMY_BYTE_COUNT..response_length]);
        Ok(())
    }

    fn read_otp_word<B: Bmm350Bus>(
        &self,
        bus: &mut B,
        word_address: u8,
    ) -> Result<u16, Error<B::Error>> {
        self.write_register(bus, REG_OTP_CMD, OTP_DIRECT_READ | (word_address & 0x1f))?;

        let mut command_done = false;
        for _ in 0..OTP_POLL_ATTEMPTS {
            bus.delay_us(300);
            let status = self.read_register(bus, REG_OTP_STATUS)?;
            let otp_error = status & OTP_STATUS_ERROR_MASK;
            if otp_error != 0 {
                return Err(Error::OtpStatus(otp_error));
            }
            if status & OTP_STATUS_DONE != 0 {
                command_done = true;
                break;
            }
        }

        if !command_done {
            return Err(Error::OtpTimeout(word_address));
        }

        let msb = self.read_register(bus, REG_OTP_DATA_MSB)?;
        let lsb = self.read_register(bus, REG_OTP_DATA_LSB)?;
        Ok(u16::from_be_bytes([msb, lsb]))
    }

    fn read_otp<B: Bmm350Bus>(
        &self,
        bus: &mut B,
    ) -> Result<[u16; OTP_WORD_COUNT], Error<B::Error>> {
        let mut otp = [0; OTP_WORD_COUNT];
        for (address, word) in otp.iter_mut().enumerate() {
            *word = self.read_otp_word(bus, address as u8)?;
        }
        Ok(otp)
    }

    fn verify_pmu_command<B: Bmm350Bus>(
        &self,
        bus: &mut B,
        expected_command: u8,
    ) -> Result<u8, Error<B::Error>> {
        let status = self.read_register(bus, REG_PMU_CMD_STATUS_0)?;
        let completed_command = status >> PMU_STATUS_COMMAND_SHIFT;
        if status & (PMU_STATUS_BUSY | PMU_STATUS_ILLEGAL_COMMAND) != 0
            || completed_command != expected_command
        {
            return Err(Error::PmuCommand {
                expected: expected_command,
                status,
            });
        }
        Ok(status)
    }

    fn magnetic_reset<B: Bmm350Bus>(&self, bus: &mut B) -> Result<(), Error<B::Error>> {
        let status = self.read_register(bus, REG_PMU_CMD_STATUS_0)?;
        if status & PMU_STATUS_NORMAL != 0 {
            self.write_register(bus, REG_PMU_CMD, POWER_MODE_SUSPEND)?;
            bus.delay_ms(6);
            self.verify_pmu_command(bus, POWER_MODE_SUSPEND)?;
        }

        self.write_register(bus, REG_PMU_CMD, PMU_CMD_BIT_RESET)?;
        bus.delay_ms(14);
        self.verify_pmu_command(bus, PMU_CMD_BIT_RESET)?;

        self.write_register(bus, REG_PMU_CMD, PMU_CMD_FLUX_GUIDE_RESET)?;
        bus.delay_ms(18);
        self.verify_pmu_command(bus, PMU_CMD_FLUX_GUIDE_RESET)?;
        Ok(())
    }

    fn configure_sensor<B: Bmm350Bus>(&self, bus: &mut B) -> Result<(), Error<B::Error>> {
        let odr_and_averaging = (AVERAGING_4 << 4) | ODR_100_HZ;
        self.write_register(bus, REG_PMU_CMD_AGGR_SET, odr_and_averaging)?;
        self.write_register(bus, REG_PMU_CMD, PMU_CMD_UPDATE_ODR_AVERAGING)?;
        bus.delay_ms(1);
        self.verify_pmu_command(bus, PMU_CMD_UPDATE_ODR_AVERAGING)?;

        let stored_odr_and_averaging = self.read_register(bus, REG_PMU_CMD_AGGR_SET)?;
        if stored_odr_and_averaging & 0x3f != odr_and_averaging {
            return Err(Error::ConfigurationRejected);
        }

        self.write_register(bus, REG_PMU_CMD_AXIS_EN, AXES_XYZ)?;
        if self.read_register(bus, REG_PMU_CMD_AXIS_EN)? & AXES_XYZ != AXES_XYZ {
            return Err(Error::ConfigurationRejected);
        }

        self.write_register(bus, REG_PMU_CMD, POWER_MODE_NORMAL)?;
        bus.delay_ms(38);
        let status = self.verify_pmu_command(bus, POWER_MODE_NORMAL)?;
        if status & PMU_STATUS_NORMAL == 0 {
            return Err(Error::ConfigurationRejected);
        }
        Ok(())
    }
}

impl Compensation {
    fn from_otp(otp: &[u16; OTP_WORD_COUNT]) -> Self {
        let offset_x = otp[0x0e] & 0x0fff;
        let offset_y = ((otp[0x0e] & 0xf000) >> 4) | (otp[0x0f] & 0x00ff);
        let offset_z = (otp[0x0f] & 0x0f00) | (otp[0x10] & 0x00ff);

        Self {
            temperature_offset: sign_extend((otp[0x0d] & 0x00ff) as u32, 8) as f32 / 5.0,
            offset_x: sign_extend(offset_x as u32, 12) as f32,
            offset_y: sign_extend(offset_y as u32, 12) as f32,
            offset_z: sign_extend(offset_z as u32, 12) as f32,
            temperature_sensitivity: sign_extend((otp[0x0d] >> 8) as u32, 8) as f32 / 512.0,
            sensitivity_x: sign_extend((otp[0x10] >> 8) as u32, 8) as f32 / 256.0,
            sensitivity_y: sign_extend((otp[0x11] & 0x00ff) as u32, 8) as f32 / 256.0,
            sensitivity_z: sign_extend((otp[0x11] >> 8) as u32, 8) as f32 / 256.0,
            tco_x: sign_extend((otp[0x12] & 0x00ff) as u32, 8) as f32 / 32.0,
            tco_y: sign_extend((otp[0x13] & 0x00ff) as u32, 8) as f32 / 32.0,
            tco_z: sign_extend((otp[0x14] & 0x00ff) as u32, 8) as f32 / 32.0,
            tcs_x: sign_extend((otp[0x12] >> 8) as u32, 8) as f32 / 16384.0,
            tcs_y: sign_extend((otp[0x13] >> 8) as u32, 8) as f32 / 16384.0,
            tcs_z: sign_extend((otp[0x14] >> 8) as u32, 8) as f32 / 16384.0,
            reference_temperature: sign_extend(otp[0x18] as u32, 16) as f32 / 512.0 + 23.0,
            cross_x_y: sign_extend((otp[0x15] & 0x00ff) as u32, 8) as f32 / 800.0,
            cross_y_x: sign_extend((otp[0x15] >> 8) as u32, 8) as f32 / 800.0,
            cross_z_x: sign_extend((otp[0x16] & 0x00ff) as u32, 8) as f32 / 800.0,
            cross_z_y: sign_extend((otp[0x16] >> 8) as u32, 8) as f32 / 800.0,
        }
    }
}

fn sign_extend(value: u32, bit_count: u8) -> i32 {
    let shift = 32 - u32::from(bit_count);
    ((value << shift) as i32) >> shift
}

#[cfg(test)]
mod tests {
    use super::*;

    struct FakeBus {
        address: u8,
        register: u8,
    }

    impl Bmm350Bus for FakeBus {
        type Error = ();

        fn write(&mut self, _address: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write_read(
            &mut self,
            address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.address = address;
            self.register = write[0];
            read.copy_from_slice(&[0x00, 0x00, 0x12, 0x34][..read.len()]);
            Ok(())
        }

        fn delay_us(&mut self, _microseconds: u16) {}
        fn delay_ms(&mut self, _milliseconds: u16) {}
    }

    #[test]
    fn register_reads_use_the_bus_and_remove_i2c_dummy_bytes() {
        let sensor = Bmm350::new(DEFAULT_I2C_ADDRESS);
        let mut bus = FakeBus {
            address: 0,
            register: 0,
        };
        let mut values = [0; 2];

        sensor.read_registers(&mut bus, 0x40, &mut values).unwrap();

        assert_eq!(bus.address, DEFAULT_I2C_ADDRESS);
        assert_eq!(bus.register, 0x40);
        assert_eq!(values, [0x12, 0x34]);
    }

    #[test]
    fn invalid_address_is_rejected_before_the_bus_is_used() {
        let mut sensor = Bmm350::new(0x16);
        let mut bus = FakeBus {
            address: 0,
            register: 0,
        };

        assert_eq!(
            sensor.initialize(&mut bus),
            Err(Error::InvalidAddress(0x16))
        );
        assert!(!sensor.is_initialized());
    }

    #[test]
    fn sign_extension_handles_otp_field_widths() {
        assert_eq!(sign_extend(0x7f, 8), 127);
        assert_eq!(sign_extend(0x80, 8), -128);
        assert_eq!(sign_extend(0x7ff, 12), 2047);
        assert_eq!(sign_extend(0x800, 12), -2048);
        assert_eq!(sign_extend(0xffff, 16), -1);
    }

    #[test]
    fn compensation_decodes_packed_signed_offsets() {
        let mut otp = [0; OTP_WORD_COUNT];
        otp[0x0e] = 0x7800;
        otp[0x0f] = 0x08ff;
        otp[0x10] = 0x0000;

        let compensation = Compensation::from_otp(&otp);
        assert_eq!(compensation.offset_x, -2048.0);
        assert_eq!(compensation.offset_y, 2047.0);
        assert_eq!(compensation.offset_z, -2048.0);
    }
}
