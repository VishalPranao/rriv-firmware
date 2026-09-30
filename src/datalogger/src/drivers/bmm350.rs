// BMM350 register behavior, startup flow, OTP layout, and compensation decoding
// are adapted from the Bosch Sensortec BMM350 SensorAPI. The upstream notice
// and BSD-3-Clause terms are retained in LICENSES/Bosch-BMM350-SensorAPI.txt.

use serde_json::{json, Value};

use super::types::{
    copy_config_into_partition, getters, SensorDriver, SensorDriverGeneralConfiguration,
    SENSOR_SETTINGS_PARTITION_SIZE,
};

pub const TYPE_NAME: &str = "bmm350";

const CONFIG_VERSION: u8 = 1;
const DEFAULT_I2C_ADDRESS: u8 = 0x14;
const ALTERNATE_I2C_ADDRESS: u8 = 0x15;

const CONFIG_VERSION_INDEX: usize = 0;
const CONFIG_ADDRESS_INDEX: usize = 1;
const CONFIG_POWER_MODE_INDEX: usize = 2;
const CONFIG_ODR_INDEX: usize = 3;
const CONFIG_AVERAGING_INDEX: usize = 4;
const CONFIG_AXES_INDEX: usize = 5;
const CONFIG_RESERVED_START: usize = 6;

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
const POWER_MODE_NORMAL: u8 = 0x01;
const PMU_CMD_UPDATE_ODR_AVERAGING: u8 = 0x02;
const PMU_CMD_FLUX_GUIDE_RESET: u8 = 0x05;
const PMU_CMD_BIT_RESET: u8 = 0x07;

const PMU_STATUS_BUSY: u8 = 0x01;
const PMU_STATUS_NORMAL: u8 = 0x08;
const PMU_STATUS_ILLEGAL_COMMAND: u8 = 0x10;
const PMU_STATUS_COMMAND_SHIFT: u8 = 5;

const ODR_100_HZ: u8 = 0x04;
const AVERAGING_4: u8 = 0x02;
const AXES_XYZ: u8 = 0x07;

const MAX_REGISTER_READ_LENGTH: usize = 12;
const I2C_DUMMY_BYTE_COUNT: usize = 2;

#[repr(transparent)]
#[derive(Copy, Clone)]
pub struct BMM350SpecialConfiguration {
    bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
}

impl BMM350SpecialConfiguration {
    pub fn parse_from_values(values: Value) -> Result<Self, &'static str> {
        let address = match values.get("address") {
            Some(value) => Self::parse_address(value)?,
            None => DEFAULT_I2C_ADDRESS,
        };

        let mut bytes = [0; SENSOR_SETTINGS_PARTITION_SIZE];
        bytes[CONFIG_VERSION_INDEX] = CONFIG_VERSION;
        bytes[CONFIG_ADDRESS_INDEX] = address;
        bytes[CONFIG_POWER_MODE_INDEX] = POWER_MODE_NORMAL;
        bytes[CONFIG_ODR_INDEX] = ODR_100_HZ;
        bytes[CONFIG_AVERAGING_INDEX] = AVERAGING_4;
        bytes[CONFIG_AXES_INDEX] = AXES_XYZ;
        Ok(Self { bytes })
    }

    pub fn new_from_bytes(bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE]) -> Self {
        Self { bytes }
    }

    fn parse_address(value: &Value) -> Result<u8, &'static str> {
        let address = value
            .as_u64()
            .and_then(|number| u8::try_from(number).ok())
            .ok_or("BMM350 address must be an integer")?;

        if address != DEFAULT_I2C_ADDRESS && address != ALTERNATE_I2C_ADDRESS {
            return Err("BMM350 address must be 0x14 (20) or 0x15 (21)");
        }

        Ok(address)
    }

    fn address(&self) -> u8 {
        self.bytes[CONFIG_ADDRESS_INDEX]
    }

    fn is_valid(&self) -> bool {
        self.bytes[CONFIG_VERSION_INDEX] == CONFIG_VERSION
            && matches!(self.address(), DEFAULT_I2C_ADDRESS | ALTERNATE_I2C_ADDRESS)
            && self.bytes[CONFIG_POWER_MODE_INDEX] == POWER_MODE_NORMAL
            && self.bytes[CONFIG_ODR_INDEX] == ODR_100_HZ
            && self.bytes[CONFIG_AVERAGING_INDEX] == AVERAGING_4
            && self.bytes[CONFIG_AXES_INDEX] == AXES_XYZ
            && self.bytes[CONFIG_RESERVED_START..]
                .iter()
                .all(|byte| *byte == 0)
    }
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
enum BMM350Error {
    Communication,
    InvalidConfiguration,
    WrongChipId(u8),
    RegisterReadTooLong,
    OtpTimeout(u8),
    OtpStatus(u8),
    PmuCommand { expected: u8, status: u8 },
    ConfigurationRejected,
}

// These coefficients are loaded in setup and are used by the compensated
// measurement calculation added in step 5.
#[allow(dead_code)]
#[derive(Copy, Clone)]
struct BMM350Compensation {
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

pub struct BMM350 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: BMM350SpecialConfiguration,
    measured_parameter_values: [Option<f64>; 4],
    compensation: Option<BMM350Compensation>,
    variant_id: Option<u8>,
    initialized: bool,
}

impl BMM350 {
    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        special_config: BMM350SpecialConfiguration,
    ) -> Self {
        Self {
            general_config,
            special_config,
            measured_parameter_values: [None; 4],
            compensation: None,
            variant_id: None,
            initialized: false,
        }
    }

    fn write_register(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
        register: u8,
        value: u8,
    ) -> Result<(), BMM350Error> {
        board
            .ic2_write(self.special_config.address(), &[register, value])
            .map_err(|_| BMM350Error::Communication)
    }

    fn read_register(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
        register: u8,
    ) -> Result<u8, BMM350Error> {
        let mut value = [0; 1];
        self.read_registers(board, register, &mut value)?;
        Ok(value[0])
    }

    fn read_registers(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
        register: u8,
        values: &mut [u8],
    ) -> Result<(), BMM350Error> {
        if values.len() > MAX_REGISTER_READ_LENGTH {
            return Err(BMM350Error::RegisterReadTooLong);
        }

        let mut response = [0; MAX_REGISTER_READ_LENGTH + I2C_DUMMY_BYTE_COUNT];
        let response_length = values.len() + I2C_DUMMY_BYTE_COUNT;
        board
            .ic2_write_read(
                self.special_config.address(),
                &[register],
                &mut response[..response_length],
            )
            .map_err(|_| BMM350Error::Communication)?;

        values.copy_from_slice(&response[I2C_DUMMY_BYTE_COUNT..response_length]);
        Ok(())
    }

    fn read_otp_word(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
        word_address: u8,
    ) -> Result<u16, BMM350Error> {
        self.write_register(board, REG_OTP_CMD, OTP_DIRECT_READ | (word_address & 0x1f))?;

        let mut command_done = false;
        for _ in 0..OTP_POLL_ATTEMPTS {
            board.delay_us(300);
            let status = self.read_register(board, REG_OTP_STATUS)?;
            let otp_error = status & OTP_STATUS_ERROR_MASK;
            if otp_error != 0 {
                return Err(BMM350Error::OtpStatus(otp_error));
            }
            if status & OTP_STATUS_DONE != 0 {
                command_done = true;
                break;
            }
        }

        if !command_done {
            return Err(BMM350Error::OtpTimeout(word_address));
        }

        let msb = self.read_register(board, REG_OTP_DATA_MSB)?;
        let lsb = self.read_register(board, REG_OTP_DATA_LSB)?;
        Ok(u16::from_be_bytes([msb, lsb]))
    }

    fn read_otp(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
    ) -> Result<[u16; OTP_WORD_COUNT], BMM350Error> {
        let mut otp = [0; OTP_WORD_COUNT];
        for (address, word) in otp.iter_mut().enumerate() {
            *word = self.read_otp_word(board, address as u8)?;
        }
        Ok(otp)
    }

    fn verify_pmu_command(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
        expected_command: u8,
    ) -> Result<u8, BMM350Error> {
        let status = self.read_register(board, REG_PMU_CMD_STATUS_0)?;
        let completed_command = status >> PMU_STATUS_COMMAND_SHIFT;
        if status & (PMU_STATUS_BUSY | PMU_STATUS_ILLEGAL_COMMAND) != 0
            || completed_command != expected_command
        {
            return Err(BMM350Error::PmuCommand {
                expected: expected_command,
                status,
            });
        }
        Ok(status)
    }

    fn magnetic_reset(&self, board: &mut dyn rriv_board::RRIVBoard) -> Result<(), BMM350Error> {
        let status = self.read_register(board, REG_PMU_CMD_STATUS_0)?;
        if status & PMU_STATUS_NORMAL != 0 {
            self.write_register(board, REG_PMU_CMD, POWER_MODE_SUSPEND)?;
            board.delay_ms(6);
            self.verify_pmu_command(board, POWER_MODE_SUSPEND)?;
        }

        self.write_register(board, REG_PMU_CMD, PMU_CMD_BIT_RESET)?;
        board.delay_ms(14);
        self.verify_pmu_command(board, PMU_CMD_BIT_RESET)?;

        self.write_register(board, REG_PMU_CMD, PMU_CMD_FLUX_GUIDE_RESET)?;
        board.delay_ms(18);
        self.verify_pmu_command(board, PMU_CMD_FLUX_GUIDE_RESET)?;
        Ok(())
    }

    fn configure_sensor(&self, board: &mut dyn rriv_board::RRIVBoard) -> Result<(), BMM350Error> {
        let odr_and_averaging = (AVERAGING_4 << 4) | ODR_100_HZ;
        self.write_register(board, REG_PMU_CMD_AGGR_SET, odr_and_averaging)?;
        self.write_register(board, REG_PMU_CMD, PMU_CMD_UPDATE_ODR_AVERAGING)?;
        board.delay_ms(1);
        self.verify_pmu_command(board, PMU_CMD_UPDATE_ODR_AVERAGING)?;

        let stored_odr_and_averaging = self.read_register(board, REG_PMU_CMD_AGGR_SET)?;
        if stored_odr_and_averaging & 0x3f != odr_and_averaging {
            return Err(BMM350Error::ConfigurationRejected);
        }

        self.write_register(board, REG_PMU_CMD_AXIS_EN, AXES_XYZ)?;
        if self.read_register(board, REG_PMU_CMD_AXIS_EN)? & AXES_XYZ != AXES_XYZ {
            return Err(BMM350Error::ConfigurationRejected);
        }

        self.write_register(board, REG_PMU_CMD, POWER_MODE_NORMAL)?;
        board.delay_ms(38);
        let status = self.verify_pmu_command(board, POWER_MODE_NORMAL)?;
        if status & PMU_STATUS_NORMAL == 0 {
            return Err(BMM350Error::ConfigurationRejected);
        }
        Ok(())
    }

    fn initialize_sensor(
        &self,
        board: &mut dyn rriv_board::RRIVBoard,
    ) -> Result<(BMM350Compensation, u8), BMM350Error> {
        if !self.special_config.is_valid() {
            return Err(BMM350Error::InvalidConfiguration);
        }

        board.delay_ms(3);
        self.write_register(board, REG_CMD, CMD_SOFT_RESET)?;
        board.delay_ms(24);

        let chip_id = self.read_register(board, REG_CHIP_ID)?;
        if chip_id != CHIP_ID {
            return Err(BMM350Error::WrongChipId(chip_id));
        }

        let otp = self.read_otp(board)?;
        let compensation = BMM350Compensation::from_otp(&otp);
        let variant_id = ((otp[30] & 0x7f00) >> 9) as u8;

        self.write_register(board, REG_OTP_CMD, OTP_POWER_OFF)?;
        self.magnetic_reset(board)?;
        self.configure_sensor(board)?;
        Ok((compensation, variant_id))
    }
}

impl BMM350Compensation {
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

impl SensorDriver for BMM350 {
    getters!();

    fn get_configuration_json(&mut self) -> Value {
        let mut id_bytes = self.general_config.id;
        let id = util::str_from_utf8(&mut id_bytes).unwrap_or_default();
        json!({
            "id": id,
            "type": TYPE_NAME,
            "address": self.special_config.address(),
            "operating_mode": "normal",
            "output_data_rate_hz": 100,
            "averaging": 4,
            "axes": "xyz",
            "initialized": self.initialized,
            "variant_id": self.variant_id,
            "config_valid": self.special_config.is_valid()
        })
    }

    fn update(&mut self, _values: Value) -> Result<(), &'static str> {
        Err("BMM350 runtime configuration is not implemented")
    }

    fn setup(&mut self, board: &mut dyn rriv_board::RRIVBoard) {
        self.measured_parameter_values = [None; 4];
        self.compensation = None;
        self.variant_id = None;
        self.initialized = false;

        match self.initialize_sensor(board) {
            Ok((compensation, variant_id)) => {
                self.compensation = Some(compensation);
                self.variant_id = Some(variant_id);
                self.initialized = true;
                board.serial_debug(format_args!(
                    "BMM350 initialized at 0x{:02x}, variant {}",
                    self.special_config.address(),
                    variant_id
                ));
            }
            Err(error) => {
                board.serial_debug(format_args!("BMM350 setup failed: {:?}", error));
            }
        }
    }

    fn take_measurement(&mut self, _board: &mut dyn rriv_board::RRIVBoard) {
        // Step 5 will perform one burst read and apply the OTP compensation.
        self.measured_parameter_values = [None; 4];
    }

    fn get_measured_parameter_count(&mut self) -> usize {
        self.measured_parameter_values.len()
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        self.measured_parameter_values
            .get(index)
            .copied()
            .flatten()
            .ok_or(())
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8; 16] {
        let identifier: &[u8] = match index {
            0 => b"mag_x",
            1 => b"mag_y",
            2 => b"mag_z",
            3 => b"temperature",
            _ => b"invalid",
        };
        let mut result = [0; 16];
        result[..identifier.len()].copy_from_slice(identifier);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_extension_handles_otp_field_widths() {
        assert_eq!(sign_extend(0x7f, 8), 127);
        assert_eq!(sign_extend(0x80, 8), -128);
        assert_eq!(sign_extend(0x7ff, 12), 2047);
        assert_eq!(sign_extend(0x800, 12), -2048);
        assert_eq!(sign_extend(0xffff, 16), -1);
    }

    #[test]
    fn configuration_accepts_only_bmm350_addresses() {
        let default_config = BMM350SpecialConfiguration::parse_from_values(json!({})).unwrap();
        assert_eq!(default_config.address(), DEFAULT_I2C_ADDRESS);
        assert!(default_config.is_valid());

        assert!(
            BMM350SpecialConfiguration::parse_from_values(json!({ "address": 0x15 }))
                .unwrap()
                .is_valid()
        );
        assert!(BMM350SpecialConfiguration::parse_from_values(json!({ "address": 0x16 })).is_err());
    }

    #[test]
    fn compensation_decodes_packed_signed_offsets() {
        let mut otp = [0; OTP_WORD_COUNT];
        otp[0x0e] = 0x7800;
        otp[0x0f] = 0x08ff;
        otp[0x10] = 0x0000;

        let compensation = BMM350Compensation::from_otp(&otp);
        assert_eq!(compensation.offset_x, -2048.0);
        assert_eq!(compensation.offset_y, 2047.0);
        assert_eq!(compensation.offset_z, -2048.0);
    }
}
