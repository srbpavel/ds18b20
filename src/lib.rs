#![no_std]

//! # Test Test

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use one_wire_bus::{self, Address, OneWire, OneWireError, OneWireResult};

pub const FAMILY_CODE: u8 = 0x28;

pub mod commands;
mod resolution;

use one_wire_bus::crc::check_crc8;
pub use resolution::Resolution;

/// All of the data that can be read from the sensor.
#[derive(Debug)]
pub struct SensorData {
    /// Temperature in degrees Celsius. Defaults to 85 on startup
    pub temperature: f32,

    /// The current resolution configuration
    pub resolution: Resolution,

    /// If the last recorded temperature is lower than this, the sensor is put in an alarm state
    pub alarm_temp_low: i8,

    /// If the last recorded temperature is higher than this, the sensor is put in an alarm state
    pub alarm_temp_high: i8,

    /// Raw temperature after positive/negative verification
    pub raw_temp: u16,

    /// Scratchpad
    pub scratchpad: [u8; 9],
}

pub struct Ds18b20 {
    address: Address,
}

impl Ds18b20 {
    /// Checks that the given address contains the correct family code, reads
    /// configuration data, then returns a device
    pub fn new<E>(address: Address) -> OneWireResult<Ds18b20, E> {
        if address.family_code() == FAMILY_CODE {
            Ok(Ds18b20 { address })
        } else {
            Err(OneWireError::FamilyCodeMismatch)
        }
    }

    /// Returns the device address
    pub fn address(&self) -> &Address {
        &self.address
    }

    /// Starts a temperature measurement for just this device
    /// You should wait for the measurement to finish before reading the measurement.
    /// The amount of time you need to wait depends on the current resolution configuration
    pub fn start_temp_measurement<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayUs<u16>,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        onewire.send_command(commands::CONVERT_TEMP, Some(&self.address), delay)?;
        Ok(())
    }

    pub fn read_data<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayUs<u16>,
    ) -> OneWireResult<SensorData, E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        let data = read_data(&self.address, onewire, delay)?;
        Ok(data)
    }

    pub fn set_config<T, E>(
        &self,
        alarm_temp_low: i8,
        alarm_temp_high: i8,
        resolution: Resolution,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayUs<u16>,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        onewire.send_command(commands::WRITE_SCRATCHPAD, Some(&self.address), delay)?;
        onewire.write_byte(alarm_temp_high.to_ne_bytes()[0], delay)?;
        onewire.write_byte(alarm_temp_low.to_ne_bytes()[0], delay)?;
        onewire.write_byte(resolution.to_config_register(), delay)?;
        Ok(())
    }

    pub fn save_to_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayUs<u16>,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        save_to_eeprom(Some(&self.address), onewire, delay)
    }

    pub fn recall_from_eeprom<T, E>(
        &self,
        onewire: &mut OneWire<T>,
        delay: &mut impl DelayUs<u16>,
    ) -> OneWireResult<(), E>
    where
        T: InputPin<Error = E>,
        T: OutputPin<Error = E>,
    {
        recall_from_eeprom(Some(&self.address), onewire, delay)
    }
}

/// Starts a temperature measurement for all devices on this one-wire bus, simultaneously
pub fn start_simultaneous_temp_measurement<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.skip_address(delay)?;
    onewire.write_byte(commands::CONVERT_TEMP, delay)?;
    Ok(())
}

/// Read the contents of the EEPROM config to the scratchpad for all devices simultaneously.
pub fn simultaneous_recall_from_eeprom<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    recall_from_eeprom(None, onewire, delay)
}

/// Read the config contents of the scratchpad memory to the EEPROMfor all devices simultaneously.
pub fn simultaneous_save_to_eeprom<T, E>(
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    save_to_eeprom(None, onewire, delay)
}

pub fn read_scratchpad<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<[u8; 9], E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.reset(delay)?;
    onewire.match_address(address, delay)?;
    onewire.write_byte(commands::READ_SCRATCHPAD, delay)?;
    let mut scratchpad = [0; 9];
    onewire.read_bytes(&mut scratchpad, delay)?;
    check_crc8(&scratchpad)?;
    Ok(scratchpad)
}


fn read_data<T, E>(
    address: &Address,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<SensorData, E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    let scratchpad = read_scratchpad(address, onewire, delay)?;

    let resolution = if let Some(resolution) = Resolution::from_config_register(scratchpad[4]) {
        resolution
    } else {
        return Err(OneWireError::CrcMismatch);
    };

    let (raw_temp, temperature): (u16, f32) = verify_sign(u16::from_le_bytes([scratchpad[0], scratchpad[1]]),
                                                          resolution,
    );
    
    Ok(SensorData {
        temperature,
        resolution,
        alarm_temp_high: i8::from_le_bytes([scratchpad[2]]),
        alarm_temp_low: i8::from_le_bytes([scratchpad[3]]),
        raw_temp, 
        scratchpad,
    })
}

fn recall_from_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.send_command(commands::RECALL_EEPROM, address, delay)?;

    // wait for the recall to finish (up to 10ms)
    let max_retries = (10000 / one_wire_bus::READ_SLOT_DURATION_MICROS) + 1;
    for _ in 0..max_retries {
        if onewire.read_bit(delay)? == true {
            return Ok(());
        }
    }
    Err(OneWireError::Timeout)
}

fn save_to_eeprom<T, E>(
    address: Option<&Address>,
    onewire: &mut OneWire<T>,
    delay: &mut impl DelayUs<u16>,
) -> OneWireResult<(), E>
where
    T: InputPin<Error = E>,
    T: OutputPin<Error = E>,
{
    onewire.send_command(commands::COPY_SCRATCHPAD, address, delay)?;
    delay.delay_us(10000); // delay 10ms for the write to complete
    Ok(())
}


//
fn verify_sign(raw_temp: u16,
               resolution: Resolution) -> (u16, f32) {
    let (sign, raw_temp): (f32, u16) = if (raw_temp & 0x8000).eq(&0x8000) {
        (-1.0, (raw_temp ^ 0xFFFF) + 1)
    } else {
        (1.0, raw_temp)
    };
    
    let temperature = match resolution {
        Resolution::Bits12 => (raw_temp as f32) / 16.0,
        Resolution::Bits11 => (raw_temp as f32) / 8.0,
        Resolution::Bits10 => (raw_temp as f32) / 4.0,
        Resolution::Bits9 => (raw_temp as f32) / 2.0,
    };

    (raw_temp, sign * temperature)
}


#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn negative_value() {
        assert_eq!(verify_sign(0b1111_1111_1111_1000, Resolution::Bits12),
                   (0b0000_0000_0000_1000, -0.5),
        );
        
        assert_eq!(verify_sign(0b1111_1110_0110_1111, Resolution::Bits12),
                   (0b110010001, -25.0625),
        );

        assert_eq!(verify_sign(0b1111_1100_1001_0000, Resolution::Bits12),
                   (0b0000_0011_0111_0000, -55.0),
        );

        assert_eq!(verify_sign(0b1111_1110_1101_0111, Resolution::Bits12),
                   (0b0000_0001_0010_1001, -18.5625),
        );
    }

    #[test]
    fn zero_value() {
        assert_eq!(verify_sign(0b0, Resolution::Bits12),
                   (0b0, 0.0),
        );
    }

    #[test]
    fn positive_value() {
        assert_eq!(verify_sign(0b0000_0000_0000_1000, Resolution::Bits12),
                   (0b0000_0000_0000_1000, 0.5),
        );

        assert_eq!(verify_sign(0b0000_0001_0010_1000, Resolution::Bits12),
                   (0b0000_0001_0010_1000, 18.5),
        );

        assert_eq!(verify_sign(0b0000_0001_1001_0001, Resolution::Bits12),
                   (0b0000_0001_1001_0001, 25.0625),
        );

        assert_eq!(verify_sign(0b0000_0111_1101_0000, Resolution::Bits12),
                   (0b0000_0111_1101_0000, 125.0),
        );
    }
}
