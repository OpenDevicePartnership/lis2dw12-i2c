//! This is a platform-agnostic Rust driver for the STMicroelectronis LIS2DW12
//! 3-axis Accelerometer based on the [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! For further details of the device architecture and operation, please refer
//! to the official [`Datasheet`].
//!
//! [`Datasheet`]: https://www.st.com/resource/en/datasheet/lis2dw12.pdf

#![cfg_attr(not(test), no_std)]

use embedded_hal_async::i2c::I2c;

pub mod registers;
pub use registers::*;

/// SA0 pin logic level representation.
pub enum SA0 {
    /// SA0 tied to GND (default).
    Gnd,
    /// SA0 tied to V+.
    Vplus,
}

impl Default for SA0 {
    fn default() -> Self {
        Self::Gnd
    }
}

impl From<SA0> for u8 {
    fn from(connection: SA0) -> Self {
        match connection {
            SA0::Gnd => 0b001_1000,   // 0x18
            SA0::Vplus => 0b001_1001, // 0x19
        }
    }
}

pub struct Lis2dw12<I2C: I2c> {
    i2c: I2C,
    addr: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Lis2dw12<I2C> {
    /// Create a new LIS2DW12 instance. Address determined by connection to SA0
    pub fn new(i2c: I2C, sa0: SA0) -> Self {
        Self { i2c, addr: sa0.into() }
    }

    /// Create a new LIS2DW12 instance with SA0 tied to GND, resulting in an
    /// instance responding to address `0x18`.
    pub fn new_with_sa0_gnd(i2c: I2C) -> Self {
        Self::new(i2c, SA0::Gnd)
    }

    /// Create a new LIS2DW12 instance with SA0 tied to V+, resulting in an
    /// instance responding to address `0x19`.
    pub fn new_with_sa0_vplus(i2c: I2C) -> Self {
        Self::new(i2c, SA0::Vplus)
    }

    /// Destroy the driver instance, return the I2C bus instance.
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Read LIS2DW12 register
    pub async fn read_reg(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut read_byte: [u8; 1] = [0; 1];
        self.i2c.write_read(self.addr, &[reg as u8], &mut read_byte).await?;
        Ok(read_byte[0])
    }

    /// Write to LIS2DW12 register
    pub async fn write_reg(&mut self, reg: Register, val: u8) -> Result<(), I2C::Error> {
        let data: &[u8; 2] = &[reg as u8, val];
        self.i2c.write(self.addr, data).await
    }

    /// Modifies the specified register by first reading then setting or resetting specified bits
    /// If a bit is marked in both set and reset masks, then that bit will not be updated
    pub async fn modify_reg_bits(
        &mut self,
        reg: Register,
        bits_to_reset: u8,
        bits_to_set: u8,
    ) -> Result<(), I2C::Error> {
        // Filter masks to clear overlap bits
        let both_mask: u8 = bits_to_reset & bits_to_set;
        let reset_mask: u8 = bits_to_reset & !both_mask;
        let set_mask: u8 = bits_to_set & !both_mask;

        // Read current value of that register
        let mut current: u8 = self.read_reg(reg).await?;

        // Update current register value with specified reset/set bits
        current &= !reset_mask;
        current |= set_mask;

        self.write_reg(reg, current).await
    }

    /// Modifies the specified register by first reading then replacing the masked bits with the value's bits
    pub async fn modify_reg_field(&mut self, reg: Register, val: u8, mask: u8) -> Result<(), I2C::Error> {
        // Read current value of that register
        let mut current: u8 = self.read_reg(reg).await?;

        // Update the register value with the new masked bits
        current = (current & !mask) | (val & mask);

        self.write_reg(reg, current).await
    }

    /// Reads the device temperature with 12 bit precision. The LSB bits 3..0 are unused 0
    /// Offset: 0 LSB = 25 deg C
    /// Scale: 16 LSB / deg C (Note: LSB bits 3..0 are unused 0, so the LSB is at bit 4)
    pub async fn temp_12bit(&mut self) -> Result<i16, I2C::Error> {
        let mut temp: i16 = 0;
        temp += ((self.read_reg(Register::TempOutHigh).await?) as i16) << 8;
        temp += (self.read_reg(Register::TempOutLow).await?) as i16;
        Ok(temp)
    }

    /// Reads the device temperature with 8 bit precision
    /// Offset: 0 LSB = 25 deg C
    /// Scale: 1 deg C / LSB
    pub async fn temp_8bit(&mut self) -> Result<i8, I2C::Error> {
        Ok(self.read_reg(Register::TempOut).await? as i8)
    }

    /// Reads the current temperature and returns the value in degrees Celcius
    pub async fn temp_celcius(&mut self) -> Result<f32, I2C::Error> {
        Ok(Lis2dw12::<I2C>::convert_temp_reg_to_celcius(self.temp_12bit().await?))
    }

    /// Reads the device acceleration register in the X axis
    pub async fn acc_x(&mut self) -> Result<i16, I2C::Error> {
        let mut accx: i16 = 0;
        accx += ((self.read_reg(Register::XOutHigh).await?) as i16) << 8;
        accx += (self.read_reg(Register::XOutLow).await?) as i16;
        Ok(accx)
    }

    /// Reads the device acceleration register in the Y axis
    pub async fn acc_y(&mut self) -> Result<i16, I2C::Error> {
        let mut accy: i16 = 0;
        accy += ((self.read_reg(Register::YOutHigh).await?) as i16) << 8;
        accy += (self.read_reg(Register::YOutLow).await?) as i16;
        Ok(accy)
    }

    /// Reads the device acceleration register in the Z axis
    pub async fn acc_z(&mut self) -> Result<i16, I2C::Error> {
        let mut accz: i16 = 0;
        accz += ((self.read_reg(Register::ZOutHigh).await?) as i16) << 8;
        accz += (self.read_reg(Register::ZOutLow).await?) as i16;
        Ok(accz)
    }

    /// Reads the 3D device acceleration from registers
    pub async fn acc(&mut self) -> Result<(i16, i16, i16), I2C::Error> {
        Ok((self.acc_x().await?, self.acc_y().await?, self.acc_z().await?))
    }

    /// Returns the 3D device acceleration in Gs
    pub async fn acc_gs(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let full_scale = self.full_scale_range().await?;
        let (accx, accy, accz) = self.acc().await?;
        Ok((
            Lis2dw12::<I2C>::convert_acc_to_gs(accx, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_gs(accy, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_gs(accz, full_scale),
        ))
    }

    /// Returns the 3D device acceleration in milli-Gs
    pub async fn acc_mgs(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let full_scale = self.full_scale_range().await?;
        let (accx, accy, accz) = self.acc().await?;
        Ok((
            Lis2dw12::<I2C>::convert_acc_to_mgs(accx, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_mgs(accy, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_mgs(accz, full_scale),
        ))
    }

    /// Returns the 3D device acceleration in micro-Gs
    pub async fn acc_ugs(&mut self) -> Result<(f32, f32, f32), I2C::Error> {
        let full_scale = self.full_scale_range().await?;
        let (accx, accy, accz) = self.acc().await?;
        Ok((
            Lis2dw12::<I2C>::convert_acc_to_ugs(accx, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_ugs(accy, full_scale),
            Lis2dw12::<I2C>::convert_acc_to_ugs(accz, full_scale),
        ))
    }

    /// Reads the tap threshold value in the X axis from its register fields
    pub async fn tap_threshold_x(&mut self) -> Result<u8, I2C::Error> {
        Ok(self.read_reg(Register::TapThresholdX).await? & 0x1F)
    }

    /// Reads the tap threshold value in the Y axis from its register fields
    pub async fn tap_threshold_y(&mut self) -> Result<u8, I2C::Error> {
        Ok(self.read_reg(Register::TapThresholdY).await? & 0x1F)
    }

    /// Reads the tap threshold value in the Z axis from its register fields
    pub async fn tap_threshold_z(&mut self) -> Result<u8, I2C::Error> {
        Ok(self.read_reg(Register::TapThresholdZ).await? & 0x1F)
    }

    /// Sets the tap threshold value in the X axis
    /// Does not update the rest of the register
    pub async fn set_tap_threshold_x(&mut self, ths: u8) -> Result<(), I2C::Error> {
        self.modify_reg_field(Register::TapThresholdX, ths, 0x1F).await
    }

    /// Sets the tap threshold value in the Y axis
    /// Does not update the rest of the register
    pub async fn set_tap_threshold_y(&mut self, ths: u8) -> Result<(), I2C::Error> {
        self.modify_reg_field(Register::TapThresholdY, ths, 0x1F).await
    }

    /// Sets the tap threshold value in the Z axis
    /// Does not update the rest of the register
    pub async fn set_tap_threshold_z(&mut self, ths: u8) -> Result<(), I2C::Error> {
        self.modify_reg_field(Register::TapThresholdZ, ths, 0x1F).await
    }

    /// Reads the Status register
    pub async fn status(&mut self) -> Result<StatusReg, I2C::Error> {
        let reg: u8 = self.read_reg(Register::Status).await?;
        Ok(reg.into())
    }

    /// Reads the full scale range from Control Register 6
    pub async fn full_scale_range(&mut self) -> Result<Control6FullScale, I2C::Error> {
        let ctrl6: ControlReg6 = self.read_reg(Register::Control6).await?.into();
        Ok(ctrl6.fs())
    }

    /// Sets the full scale range in Control Register 6
    pub async fn set_full_scale_range(&mut self, new_fs: Control6FullScale) -> Result<(), I2C::Error> {
        let mut ctrl6: ControlReg6 = self.read_reg(Register::Control6).await?.into();
        ctrl6.set_fs(new_fs);
        self.write_reg(Register::Control6, ctrl6.into()).await
    }

    /// Returns free fall duration by stitching FF_DUR5 from WAKE_UP_DUR register onto FF register output
    pub async fn free_fall_duration(&mut self) -> Result<u8, I2C::Error> {
        let ff_reg: FreeFallReg = self.read_reg(Register::FreeFall).await?.into();
        let wu_reg: WakeUpDurationReg = self.read_reg(Register::WakeUpDuration).await?.into();
        let ff_dur: u8 = u8::from(ff_reg.ff_dur()) + (u8::from(wu_reg.ff_dur5()) << 5);
        Ok(ff_dur)
    }

    // -------------------------- Helper Functions --------------------------

    /// Converts i16 temperature representation to degrees Celcius
    /// For use with 8bit temp register, convert to i16 and shift data to upper (MSB) byte for input
    pub fn convert_temp_reg_to_celcius(temp_in: i16) -> f32 {
        // Convert temp int to float
        let mut temp: f32 = temp_in as f32;

        // Divide out temp offset: bit 8 (LSB of upper byte) = 1 deg C
        temp /= 256f32;

        // Add offset of 25 deg C at 0
        temp += 25f32;

        temp
    }

    /// Converts the acceleration register data to acceleration in Gs
    pub fn convert_acc_to_gs(acc_in: i16, full_scale: Control6FullScale) -> f32 {
        let acc: f32 = acc_in as f32;
        let factor: f32 = match full_scale {
            Control6FullScale::Scale2g => 1f32 / 16384f32,
            Control6FullScale::Scale4g => 1f32 / 8192f32,
            Control6FullScale::Scale8g => 1f32 / 4096f32,
            Control6FullScale::Scale16g => 1f32 / 2048f32,
        };
        acc * factor
    }

    /// Converts the acceleration register data to acceleration in milli-Gs
    pub fn convert_acc_to_mgs(acc_in: i16, full_scale: Control6FullScale) -> f32 {
        let acc: f32 = acc_in as f32;
        let factor: f32 = match full_scale {
            Control6FullScale::Scale2g => 1000f32 / 16384f32,
            Control6FullScale::Scale4g => 1000f32 / 8192f32,
            Control6FullScale::Scale8g => 1000f32 / 4096f32,
            Control6FullScale::Scale16g => 1000f32 / 2048f32,
        };
        acc * factor
    }

    /// Converts the acceleration register data to acceleration in micro-Gs
    pub fn convert_acc_to_ugs(acc_in: i16, full_scale: Control6FullScale) -> f32 {
        let acc: f32 = acc_in as f32;
        let factor: f32 = match full_scale {
            Control6FullScale::Scale2g => 1_000_000f32 / 16384f32,
            Control6FullScale::Scale4g => 1_000_000f32 / 8192f32,
            Control6FullScale::Scale8g => 1_000_000f32 / 4096f32,
            Control6FullScale::Scale16g => 1_000_000f32 / 2048f32,
        };
        acc * factor
    }
}
