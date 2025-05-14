#![cfg_attr(not(test), no_std)]

use core::fmt::Debug;

use embedded_hal_async::i2c::{Error as I2cError, I2c};

/// Represents the hardware address selection pins A1 and A2 for the FM24V10.
/// The tuple elements are expected to be 0 or 1, corresponding to the pin states.
/// Address.0 corresponds to A1 pin state.
/// Address.1 corresponds to A2 pin state.
#[derive(Debug, Clone, Copy)]
pub struct Address(pub u8, pub u8);

/// Device Type code for FM24V10 family (4-bit prefix: 1010b).
const DEVICE_TYPE_CODE: u8 = 0b1010;

impl From<Address> for u8 {
    fn from(a: Address) -> Self {
        // Constructs the base 7-bit I2C address part: 0b1010_A2A1_0
        // DEVICE_TYPE_CODE (0b1010) is shifted to become 0b1010_000.
        // a.1 (A2 pin state) is shifted to 0b000_A200.
        // a.0 (A1 pin state) is shifted to 0b000_0A10.
        (DEVICE_TYPE_CODE << 3) | (a.1 << 2) | (a.0 << 1)
    }
}

/// Number of memory address bytes (A15-A0) sent after the slave address byte.
/// The FM24V10 uses 2 bytes for the 16 lower bits of its 17-bit address.
const MEMORY_ADDRESS_BYTES: usize = 2;

/// Capacity of the FM24V10 in bytes (1Mbit = 128KB).
const CAPACITY_BYTES: usize = 128 * 1024; // 131,072 bytes

/// Custom error type for the FM24V10 driver.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error<E: Debug + I2cError> {
    /// I2C bus error
    I2c(E),
    /// Address or data length is out of bounds
    OutOfBounds,
    /// The user-provided buffer is too small for the current write operation.
    BufferTooSmall,
}

/// Driver for the FM24V10 I2C F-RAM
pub struct Fm24v10<'buf, I2C> {
    i2c: I2C,
    /// Base I2C address part (0b1010_A2A1_0), derived from device type and A2/A1 pins.
    /// The page select bit (A16) will be ORed with this to get the final 7-bit slave address.
    base_address: u8,
    /// User-provided buffer for constructing I2C write payloads.
    write_buffer: &'buf mut [u8],
}

impl<'buf, I2C, E> Fm24v10<'buf, I2C>
where
    I2C: I2c<Error = E>,
    E: Debug + I2cError,
{
    /// Creates a new FM24V10 driver instance.
    ///
    /// # Arguments
    /// * `i2c`: The I2C bus peripheral.
    /// * `address_pins`: The state of the A2 and A1 hardware address pins,
    ///                   as `Address(a1_pin_state, a2_pin_state)`.
    /// * `write_buffer`: A mutable slice provided by the user, used for assembling
    ///                   I2C write payloads. It must be large enough to hold
    ///                   `MEMORY_ADDRESS_BYTES` + the largest anticipated data write.
    pub fn new(i2c: I2C, address_pins: Address, write_buffer: &'buf mut [u8]) -> Self {
        Self {
            i2c,
            base_address: address_pins.into(),
            write_buffer,
        }
    }

    /// Constructs the final 7-bit I2C device address for a given memory operation.
    /// It combines the base address (derived from device type and A1/A2 pins)
    /// with the page select bit (A16 of the memory address).
    fn get_address_for_offset(&self, memory_offset: u32) -> Result<u8, Error<E>> {
        if memory_offset >= CAPACITY_BYTES as u32 {
            return Err(Error::OutOfBounds);
        }
        // A16 is the most significant bit of the 17-bit memory address, acts as page select.
        // It's the LSB of the OR operation with base_address (which has LSB as 0).
        let page_select_bit = (memory_offset >> 16) & 0x01;

        let final_address = self.base_address | (page_select_bit as u8);
        Ok(final_address)
    }

    /// Read a slice of data from the F-RAM.
    ///
    /// # Arguments
    /// * `offset`: The starting memory address offset to read from (0 to CAPACITY_BYTES - 1).
    /// * `bytes`: A mutable slice to store the read data.
    pub async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error<E>> {
        if bytes.is_empty() {
            return Ok(());
        }
        if offset >= CAPACITY_BYTES as u32 || bytes.len() > (CAPACITY_BYTES - offset as usize) {
            return Err(Error::OutOfBounds);
        }

        let address = self.get_address_for_offset(offset)?;
        // Memory address bytes (A15-A0) to be sent.
        let mem_addr_payload: [u8; MEMORY_ADDRESS_BYTES] =
            [((offset >> 8) & 0xFF) as u8, (offset & 0xFF) as u8];

        self.i2c
            .write_read(address, &mem_addr_payload, bytes)
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Get the total capacity of the F-RAM in bytes.
    pub async fn capacity(&self) -> Result<usize, Error<E>> {
        Ok(CAPACITY_BYTES)
    }

    /// Write a slice of data to the F-RAM.
    ///
    /// # Arguments
    /// * `offset`: The starting memory address offset to write to (0 to CAPACITY_BYTES - 1).
    /// * `data`: The slice of data to write.
    pub async fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error<E>> {
        if data.is_empty() {
            return Ok(());
        }
        if offset >= CAPACITY_BYTES as u32
            || data.len() > (CAPACITY_BYTES - offset as usize)
        {
            return Err(Error::OutOfBounds);
        }

        let required_buffer_len = MEMORY_ADDRESS_BYTES + data.len();
        if self.write_buffer.len() < required_buffer_len {
            return Err(Error::BufferTooSmall);
        }

        let i2c_7bit_address = self.get_address_for_offset(offset)?;

        // MSB of memory address (A15-A8)
        self.write_buffer[0] = ((offset >> 8) & 0xFF) as u8;
        // LSB of memory address (A7-A0)
        self.write_buffer[1] = (offset & 0xFF) as u8;
        // Data slice
        self.write_buffer[MEMORY_ADDRESS_BYTES..required_buffer_len].copy_from_slice(data);

        self.i2c
            .write(i2c_7bit_address, &self.write_buffer[..required_buffer_len])
            .await
            .map_err(Error::I2c)?;

        Ok(())
    }
}
