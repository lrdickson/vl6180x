/*

Copyright (c) 2021 Lyndsey Dickson (lyndseyrd@gmail.com)

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

-----------------------------------------------------------------

Copyright (c) 2016-2021 Pololu Corporation (www.pololu.com)

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

*/
// #![allow(dead_code)]
#![no_std]

use cast::u16;
use hal::blocking::i2c::{Write, WriteRead};
use nb;

const ADDRESS_DEFAULT: u8 = 0x29;

// RANGE_SCALER values for 1x, 2x, 3x scaling - see STSW-IMG003 core/src/vl6180x_api.c (ScalerLookUP[])
const SCALER_VALUES: [u16; 4] = [0, 253, 127, 84];

pub struct VL6180X<I2C: hal::blocking::i2c::WriteRead> {
    bus: I2C,
    address: u8,
    scaling: u8,
    ptp_offset: u8,
    io_timeout: u16,
    did_timeout: bool,
}

// MPU Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// WHO_AM_I returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Underlying bus error.
    BusError(E),
    /// Timeout
    Timeout,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

impl<I2C, E> VL6180X<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    // Public Methods ////////////////////////////////////////////

    // Creates new driver with default address.
    pub fn new(i2c: I2C) -> VL6180X<I2C>
    where
        I2C: hal::blocking::i2c::WriteRead<Error = E>,
    {
        let sensor = VL6180X {
            bus: i2c,
            address: ADDRESS_DEFAULT,
            scaling: 0,
            ptp_offset: 0,
            io_timeout: 0,
            did_timeout: false,
        };
        return sensor;
    }


    pub fn set_address(&mut self, new_addr: u8) -> Result<(), E>
    {
        self.write_register(RegisterAddress::I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F)?;
        self.address = new_addr;
        Ok(())
    }

    // Initialize sensor with settings from ST application note AN4545, section
    // "SR03 settings" - "Mandatory : private registers"
    pub fn init(&mut self) -> Result<(), E>
    {
      // Store part-to-part range offset so it can be adjusted if scaling is changed
      self.ptp_offset = self.read_register(RegisterAddress::SYSRANGE__PART_TO_PART_RANGE_OFFSET)?;

      if self.read_register(RegisterAddress::SYSTEM__FRESH_OUT_OF_RESET)? == 1
      {
        self.scaling = 1;

        self.write_8bit(0x207, 0x01)?;
        self.write_8bit(0x208, 0x01)?;
        self.write_8bit(0x096, 0x00)?;
        self.write_8bit(0x097, 0xFD)?; // RANGE_SCALER = 253
        self.write_8bit(0x0E3, 0x01)?;
        self.write_8bit(0x0E4, 0x03)?;
        self.write_8bit(0x0E5, 0x02)?;
        self.write_8bit(0x0E6, 0x01)?;
        self.write_8bit(0x0E7, 0x03)?;
        self.write_8bit(0x0F5, 0x02)?;
        self.write_8bit(0x0D9, 0x05)?;
        self.write_8bit(0x0DB, 0xCE)?;
        self.write_8bit(0x0DC, 0x03)?;
        self.write_8bit(0x0DD, 0xF8)?;
        self.write_8bit(0x09F, 0x00)?;
        self.write_8bit(0x0A3, 0x3C)?;
        self.write_8bit(0x0B7, 0x00)?;
        self.write_8bit(0x0BB, 0x3C)?;
        self.write_8bit(0x0B2, 0x09)?;
        self.write_8bit(0x0CA, 0x09)?;
        self.write_8bit(0x198, 0x01)?;
        self.write_8bit(0x1B0, 0x17)?;
        self.write_8bit(0x1AD, 0x00)?;
        self.write_8bit(0x0FF, 0x05)?;
        self.write_8bit(0x100, 0x05)?;
        self.write_8bit(0x199, 0x05)?;
        self.write_8bit(0x1A6, 0x1B)?;
        self.write_8bit(0x1AC, 0x3E)?;
        self.write_8bit(0x1A7, 0x1F)?;
        self.write_8bit(0x030, 0x00)?;

        self.write_register(RegisterAddress::SYSTEM__FRESH_OUT_OF_RESET, 0)?;
      }
      else
      {
        // Sensor has already been initialized, so try to get scaling settings by
        // reading registers.

        let scaling = self.read_register_16bit(RegisterAddress::RANGE_SCALER)?;

        if      scaling == SCALER_VALUES[3] { self.scaling = 3; }
        else if scaling == SCALER_VALUES[2] { self.scaling = 2; }
        else                           { self.scaling = 1; }

        // Adjust the part-to-part range offset value read earlier to account for
        // existing scaling. If the sensor was already in 2x or 3x scaling mode,
        // precision will be lost calculating the original (1x) offset, but this can
        // be resolved by resetting the sensor and Arduino again.
        self.ptp_offset *= self.scaling;
      }

      Ok(())
    }

    // Configure some settings for the sensor's default behavior from AN4545 -
    // "Recommended : Public registers" and "Optional: Public registers"
    //
    // Note that this function does not set up GPIO1 as an interrupt output as
    // suggested, though you can do so by calling:
    // writeReg(SYSTEM__MODE_GPIO1, 0x10);
    pub fn configure_default(&mut self) -> Result<(), E>
    {
      // "Recommended : Public registers"

      // readout__averaging_sample_period = 48
      self.write_register(RegisterAddress::READOUT__AVERAGING_SAMPLE_PERIOD, 0x30)?;

      // sysals__analogue_gain_light = 6 (ALS gain = 1 nominal, actually 1.01 according to table "Actual gain values" in datasheet)
      self.write_register(RegisterAddress::SYSALS__ANALOGUE_GAIN, 0x46)?;

      // sysrange__vhv_repeat_rate = 255 (auto Very High Voltage temperature recalibration after every 255 range measurements)
      self.write_register(RegisterAddress::SYSRANGE__VHV_REPEAT_RATE, 0xFF)?;

      // sysals__integration_period = 99 (100 ms)
      self.write_register_16bit(RegisterAddress::SYSALS__INTEGRATION_PERIOD, 0x0063)?;

      // sysrange__vhv_recalibrate = 1 (manually trigger a VHV recalibration)
      self.write_register(RegisterAddress::SYSRANGE__VHV_RECALIBRATE, 0x01)?;


      // "Optional: Public registers"

      // sysrange__intermeasurement_period = 9 (100 ms)
      self.write_register(RegisterAddress::SYSRANGE__INTERMEASUREMENT_PERIOD, 0x09)?;

      // sysals__intermeasurement_period = 49 (500 ms)
      self.write_register(RegisterAddress::SYSALS__INTERMEASUREMENT_PERIOD, 0x31)?;

      // als_int_mode = 4 (ALS new sample ready interrupt)?; range_int_mode = 4 (range new sample ready interrupt)
      self.write_register(RegisterAddress::SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24)?;


      // Reset other settings to power-on defaults

      // sysrange__max_convergence_time = 49 (49 ms)
      self.write_register(RegisterAddress::SYSRANGE__MAX_CONVERGENCE_TIME, 0x31)?;

      // disable interleaved mode
      self.write_register(RegisterAddress::INTERLEAVED_MODE__ENABLE, 0)?;

      // reset range scaling factor to 1x
      self.set_scaling(1)?;

      Ok(())
    }

    // Implemented using ST's VL6180X API as a reference (STSW-IMG003); see
    // VL6180x_UpscaleSetScaling() in vl6180x_api.c.
    pub fn set_scaling(&mut self, new_scaling: u8) -> Result<(), E>
    {
      let default_crosstalk_valid_height: u8 = 20; // default value of SYSRANGE__CROSSTALK_VALID_HEIGHT

      // do nothing if scaling value is invalid
      if new_scaling < 1 || new_scaling > 3 { return Ok(()); }

      self.scaling = new_scaling;
      self.write_register_16bit(RegisterAddress::RANGE_SCALER, SCALER_VALUES[self.scaling as usize])?;

      // apply scaling on part-to-part offset
      self.write_register(
          RegisterAddress::SYSRANGE__PART_TO_PART_RANGE_OFFSET,
          self.ptp_offset / self.scaling)?;

      // apply scaling on CrossTalkValidHeight
      self.write_register(
          RegisterAddress::SYSRANGE__CROSSTALK_VALID_HEIGHT,
          default_crosstalk_valid_height / self.scaling)?;

      // This function does not apply scaling to RANGE_IGNORE_VALID_HEIGHT.

      // enable early convergence estimate only at 1x scaling
      let rce: u8 = self.read_register(RegisterAddress::SYSRANGE__RANGE_CHECK_ENABLES)?;
      self.write_register(RegisterAddress::SYSRANGE__RANGE_CHECK_ENABLES, (rce & 0xFE) | (self.scaling == 1) as u8)?;

      Ok(())
    }

    // Performs a single-shot ranging measurement
    pub fn read_range_single(&mut self) -> Result<u8, E>
    {
      self.write_register(RegisterAddress::SYSRANGE__START, 0x01)?;
      return self.read_range_continuous()?;
    }

    // Performs a single-shot ambient light measurement
    pub fn read_ambient_single(&mut self) -> Result<u16, E>
    {
      self.write_register(RegisterAddress::SYSALS__START, 0x01)?; return self.read_ambient_continuous()?;
    }

    // Starts continuous ranging measurements with the given period in ms
    // (10 ms resolution; defaults to 100 ms if not specified).
    //
    // The period must be greater than the time it takes to perform a
    // measurement. See section "Continuous mode limits" in the datasheet
    // for details.
    pub fn start_range_continuous(&mut self, period: u16) -> Result<(), E>
    {
      let mut period_reg: u16 = (period / 10) - 1;
      period_reg = constrain(period_reg, 0, 254);

      writeReg(SYSRANGE__INTERMEASUREMENT_PERIOD, period_reg);
      writeReg(SYSRANGE__START, 0x03);

      Ok(())
    }

    // Private Methods ///////////////////////////////////////////

    // Writes 8-bit
    fn write_8bit(&mut self, reg_addr: u16, value: u8)
        -> Result<(), E>
    {
        self.bus.write(self.address, &[
            (reg_addr >> 8) as u8,
            reg_addr as u8,
            value])?;
        Ok(())
    }

    // Writes a 8-bit register
    fn write_register(&mut self, reg_addr: RegisterAddress, value: u8)
        -> Result<(), E>
    {
        self.write_8bit(reg_addr as u16, value)?;
        Ok(())
    }

    // Writes a 16-bit register
    fn write_register_16bit(&mut self, reg_addr: RegisterAddress, value: u16)
        -> Result<(), E>
    {
        let reg_addr = reg_addr as u16;
        self.bus.write(self.address, &[
            (reg_addr >> 8) as u8,
            reg_addr as u8,
            (value >> 8) as u8,
            value as u8])?;
        Ok(())
    }

    // Reads a 8-bit register
    fn read_register(&mut self, reg_addr: RegisterAddress) -> Result<u8, E>
    {
        let mut value: [u8; 1] = [0];
        let reg_addr = reg_addr as u16;
        self.bus.write_read(self.address, &[
            (reg_addr >> 8) as u8,
            reg_addr as u8],
            &mut value)?;
        Ok(value[0])
    }

    // Reads a 16-bit register
    fn read_register_16bit(&mut self, reg_addr: RegisterAddress) -> Result<u16, E>
    {
        let mut value: [u8; 2] = [0, 0];
        let reg_addr = reg_addr as u16;
        self.bus.write_read(self.address, &[
            (reg_addr >> 8) as u8,
            reg_addr as u8],
            &mut value)?;
        let value: u16 = ((value[0] as u16) << 8) | (value[1] as u16);
        Ok(value)
    }
}

#[allow(non_camel_case_types)]
enum RegisterAddress
{
    IDENTIFICATION__MODEL_ID              = 0x000,
    IDENTIFICATION__MODEL_REV_MAJOR       = 0x001,
    IDENTIFICATION__MODEL_REV_MINOR       = 0x002,
    IDENTIFICATION__MODULE_REV_MAJOR      = 0x003,
    IDENTIFICATION__MODULE_REV_MINOR      = 0x004,
    IDENTIFICATION__DATE_HI               = 0x006,
    IDENTIFICATION__DATE_LO               = 0x007,
    IDENTIFICATION__TIME                  = 0x008, // 16-bit

    SYSTEM__MODE_GPIO0                    = 0x010,
    SYSTEM__MODE_GPIO1                    = 0x011,
    SYSTEM__HISTORY_CTRL                  = 0x012,
    SYSTEM__INTERRUPT_CONFIG_GPIO         = 0x014,
    SYSTEM__INTERRUPT_CLEAR               = 0x015,
    SYSTEM__FRESH_OUT_OF_RESET            = 0x016,
    SYSTEM__GROUPED_PARAMETER_HOLD        = 0x017,

    SYSRANGE__START                       = 0x018,
    SYSRANGE__THRESH_HIGH                 = 0x019,
    SYSRANGE__THRESH_LOW                  = 0x01A,
    SYSRANGE__INTERMEASUREMENT_PERIOD     = 0x01B,
    SYSRANGE__MAX_CONVERGENCE_TIME        = 0x01C,
    SYSRANGE__CROSSTALK_COMPENSATION_RATE = 0x01E, // 16-bit
    SYSRANGE__CROSSTALK_VALID_HEIGHT      = 0x021,
    SYSRANGE__EARLY_CONVERGENCE_ESTIMATE  = 0x022, // 16-bit
    SYSRANGE__PART_TO_PART_RANGE_OFFSET   = 0x024,
    SYSRANGE__RANGE_IGNORE_VALID_HEIGHT   = 0x025,
    SYSRANGE__RANGE_IGNORE_THRESHOLD      = 0x026, // 16-bit
    SYSRANGE__MAX_AMBIENT_LEVEL_MULT      = 0x02C,
    SYSRANGE__RANGE_CHECK_ENABLES         = 0x02D,
    SYSRANGE__VHV_RECALIBRATE             = 0x02E,
    SYSRANGE__VHV_REPEAT_RATE             = 0x031,

    SYSALS__START                         = 0x038,
    SYSALS__THRESH_HIGH                   = 0x03A,
    SYSALS__THRESH_LOW                    = 0x03C,
    SYSALS__INTERMEASUREMENT_PERIOD       = 0x03E,
    SYSALS__ANALOGUE_GAIN                 = 0x03F,
    SYSALS__INTEGRATION_PERIOD            = 0x040,

    RESULT__RANGE_STATUS                  = 0x04D,
    RESULT__ALS_STATUS                    = 0x04E,
    RESULT__INTERRUPT_STATUS_GPIO         = 0x04F,
    RESULT__ALS_VAL                       = 0x050, // 16-bit
    RESULT__HISTORY_BUFFER_0              = 0x052, // 16-bit
    RESULT__HISTORY_BUFFER_1              = 0x054, // 16-bit
    RESULT__HISTORY_BUFFER_2              = 0x056, // 16-bit
    RESULT__HISTORY_BUFFER_3              = 0x058, // 16-bit
    RESULT__HISTORY_BUFFER_4              = 0x05A, // 16-bit
    RESULT__HISTORY_BUFFER_5              = 0x05C, // 16-bit
    RESULT__HISTORY_BUFFER_6              = 0x05E, // 16-bit
    RESULT__HISTORY_BUFFER_7              = 0x060, // 16-bit
    RESULT__RANGE_VAL                     = 0x062,
    RESULT__RANGE_RAW                     = 0x064,
    RESULT__RANGE_RETURN_RATE             = 0x066, // 16-bit
    RESULT__RANGE_REFERENCE_RATE          = 0x068, // 16-bit
    RESULT__RANGE_RETURN_SIGNAL_COUNT     = 0x06C, // 32-bit
    RESULT__RANGE_REFERENCE_SIGNAL_COUNT  = 0x070, // 32-bit
    RESULT__RANGE_RETURN_AMB_COUNT        = 0x074, // 32-bit
    RESULT__RANGE_REFERENCE_AMB_COUNT     = 0x078, // 32-bit
    RESULT__RANGE_RETURN_CONV_TIME        = 0x07C, // 32-bit
    RESULT__RANGE_REFERENCE_CONV_TIME     = 0x080, // 32-bit

    RANGE_SCALER                          = 0x096, // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

    READOUT__AVERAGING_SAMPLE_PERIOD      = 0x10A,
    FIRMWARE__BOOTUP                      = 0x119,
    FIRMWARE__RESULT_SCALER               = 0x120,
    I2C_SLAVE__DEVICE_ADDRESS             = 0x212,
    INTERLEAVED_MODE__ENABLE              = 0x2A3,
}
