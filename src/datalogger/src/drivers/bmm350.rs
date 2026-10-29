use bmm350_driver::i2c_bus::{Delay, I2CBus};
use bmm350_driver::{
    Bmm350 as Bmm350Device, Error as Bmm350Error, ALTERNATE_I2C_ADDRESS, AVERAGING_4, AXES_XYZ,
    DEFAULT_I2C_ADDRESS, ODR_100_HZ, POWER_MODE_NORMAL,
};
use rriv_board::RRIVBoard;
use serde_json::{json, Value};

use super::types::{
    copy_config_into_partition, getters, SensorDriver, SensorDriverGeneralConfiguration,
    SENSOR_SETTINGS_PARTITION_SIZE,
};

pub const TYPE_NAME: &str = "bmm350";

/// Lends the RRIV board's I2C and delay operations to the forked driver.
struct BMM350I2CBus<'a> {
    board: &'a mut dyn RRIVBoard,
}

impl<'a> BMM350I2CBus<'a> {
    fn new(board: &'a mut dyn RRIVBoard) -> Self {
        Self { board }
    }
}

impl I2CBus for BMM350I2CBus<'_> {
    type Error = ();

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.board.ic2_write(address, bytes)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.board.ic2_write_read(address, write, read)
    }
}

impl Delay for BMM350I2CBus<'_> {
    fn delay_us(&mut self, microseconds: u16) {
        self.board.delay_us(microseconds);
    }

    fn delay_ms(&mut self, milliseconds: u16) {
        self.board.delay_ms(milliseconds);
    }
}

fn report_pmu_diagnostic(board: &mut dyn RRIVBoard, device: &Bmm350Device) {
    let diagnostic = {
        let mut bus = BMM350I2CBus::new(board);
        device.pmu_diagnostic(&mut bus)
    };

    // serial_debug has a short output buffer, so keep each line compact.
    board.serial_debug(format_args!(
        "BMM350 PMU after={:02x} err={:02x}",
        diagnostic.status_after.unwrap_or(0xff),
        diagnostic.error.unwrap_or(0xff)
    ));
    board.serial_debug(format_args!(
        "BMM350 PMU req={:02x} cmd={:02x} eff={:02x}",
        diagnostic.requested.unwrap_or(0xff),
        diagnostic.command.unwrap_or(0xff),
        diagnostic.effective.unwrap_or(0xff)
    ));
}

const CONFIG_VERSION: u8 = 1;
const CONFIG_VERSION_INDEX: usize = 0;
const CONFIG_ADDRESS_INDEX: usize = 1;
const CONFIG_POWER_MODE_INDEX: usize = 2;
const CONFIG_ODR_INDEX: usize = 3;
const CONFIG_AVERAGING_INDEX: usize = 4;
const CONFIG_AXES_INDEX: usize = 5;
const CONFIG_RESERVED_START: usize = 6;

// The initial driver uses one fixed sensor configuration. These fields reserve a stable
// EEPROM layout so runtime configuration can be added later without moving the
// address or changing existing saved settings.
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

        if !matches!(address, DEFAULT_I2C_ADDRESS | ALTERNATE_I2C_ADDRESS) {
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

/// RRIV-facing wrapper around the reusable BMM350 sensor driver.
pub struct BMM350 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: BMM350SpecialConfiguration,
    measured_parameter_values: [Option<f64>; 4],
    device: Bmm350Device,
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
            device: Bmm350Device::new(special_config.address()),
        }
    }
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
            "initialized": self.device.is_initialized(),
            "variant_id": self.device.variant_id(),
            "config_valid": self.special_config.is_valid()
        })
    }

    fn update(&mut self, _values: Value) -> Result<(), &'static str> {
        Err("BMM350 runtime configuration is not implemented")
    }

    fn setup(&mut self, board: &mut dyn rriv_board::RRIVBoard) {
        self.measured_parameter_values = [None; 4];
        self.device.reset_runtime_state();

        if !self.special_config.is_valid() {
            board.serial_debug(format_args!("BMM350 setup failed: invalid configuration"));
            return;
        }

        // This is the only RRIV-specific connection needed by the sensor code.
        // The adapter borrow ends after initialize(), so setup can use the
        // board again to report the result.
        let result = {
            let mut bus = BMM350I2CBus::new(board);
            self.device.initialize(&mut bus)
        };

        match result {
            Ok(()) => board.serial_debug(format_args!(
                "BMM350 initialized at 0x{:02x}, variant {}",
                self.device.address(),
                self.device.variant_id().unwrap_or_default()
            )),
            Err(Bmm350Error::PmuCommand { expected, status }) => {
                board.serial_debug(format_args!(
                    "BMM350 PMU exp={} st={:02x}",
                    expected, status
                ));
                report_pmu_diagnostic(board, &self.device);
            }
            Err(Bmm350Error::ConfigurationRejected) => {
                board.serial_debug(format_args!("BMM350 setup: configuration rejected"));
                report_pmu_diagnostic(board, &self.device);
            }
            Err(error) => {
                board.serial_debug(format_args!("BMM350 setup failed: {:?}", error));
            }
        }
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::RRIVBoard) {
        // Clear every output first so a failed read can never leave an older
        // sample looking current.
        self.measured_parameter_values = [None; 4];

        let result = {
            let mut bus = BMM350I2CBus::new(board);
            self.device.measure(&mut bus)
        };

        match result {
            Ok(measurement) => {
                self.measured_parameter_values = [
                    Some(f64::from(measurement.x)),
                    Some(f64::from(measurement.y)),
                    Some(f64::from(measurement.z)),
                    Some(f64::from(measurement.temperature)),
                ];
            }
            Err(error) => {
                board.serial_debug(format_args!("BMM350 measurement failed: {:?}", error));
            }
        }
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
}
