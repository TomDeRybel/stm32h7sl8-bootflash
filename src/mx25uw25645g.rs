// This blocking, Octo-SPI driver for the Macronix MX25UW25645G serial Flash
// memory was taken from the example of the Embassy project, liked-to below:
// <https://github.com/embassy-rs/embassy/blob/main/examples/stm32h7rs/src/bin/xspi_memory_mapped.rs>
//
// Embassy is MIT/Apache2.0 dual-licensed.
//
// The main change is a focus on the Octo-SPI mode, reducing the standard SPI
// implementation to the basics required to initialize the chip into Octo-SPI.

// TODO: Uses STR, but should this be DDR (DTR) ???
// TODO: Can I move into OPI mode sooner, with less of the SPI stuff???

use core::cmp::min;
use embassy_stm32::mode::Blocking;
use embassy_stm32::xspi::{
    AddressSize, DummyCycles, Instance, MemorySize, MemoryType, TransferConfig, Xspi, XspiWidth,
};

use crate::info;

/// Settings for the Macronix MX25UW25645G.
/// The MX25UW25645G has a program command page buffer size of 256 bytes.
/// This is different from the sector size (4K) and block size (32K or 64K).

//const CONF_OSPI_ODS:                 MX25UW25645G_CR_ODS_24   /* MX25UW25645G Output Driver Strength */
const MEMORY_TYPE: MemoryType = MemoryType::Macronix;
const DRIVE_STRENGTH: OutputDriveStrength = OutputDriveStrength::R24;
const MEMORY_FLASH_SIZE: MemorySize = MemorySize::_32MiB; // 256 megabits = 32 megabytes.
const MEMORY_BLOCK_SIZE: usize = 64 * 1024; // 512  blocks  of 64 Kbytes.
const MEMORY_SECTOR_SIZE: usize = 4 * 1024; // 8192 sectors of  4 Kbytes.
const MEMORY_PAGE_SIZE: usize = 256; // 131072 pages of 256 bytes.

const DUMMY_CYCLES_READ: DummyCycles = DummyCycles::_8;
const DUMMY_CYCLES_READ_OCTAL: DummyCycles = DummyCycles::_6;
const DUMMY_CYCLES_READ_OCTAL_DTR: DummyCycles = DummyCycles::_6;
const DUMMY_CYCLES_REG_OCTAL: DummyCycles = DummyCycles::_4;
const DUMMY_CYCLES_REG_OCTAL_DTR: DummyCycles = DummyCycles::_4;

/// SPI mode commands for the MX25UW25645G flash memory.
/// These are only used internally, to reset the chip and configure it into
/// Octo-SPI mode.
#[repr(u8)]
enum SpiCommand {
    // Device operation commands
    /// Set Write Enable Latch (WEL) bit, required before write/program/erase operations
    WriteEnable = 0x06,
    /// Enable reset operation (must precede Reset Memory command)
    ResetEnable = 0x66,
    /// Reset device to power-on state (requires prior Reset Enable)
    ResetMemory = 0x99,

    // Register Access commands
    /// Read 3-byte device identification (manufacturer ID + device ID)
    ReadIdentification = 0x9F,
    /// Read 8-bit Status Register (WIP, WEL, BP bits, etc.)
    ReadStatusRegister = 0x05,
    /// Read Configuration Register 2 from specified 4-byte address
    ReadConfigurationRegister2 = 0x71,
    /// Write Configuration Register 2 to specified 4-byte address
    WriteConfigurationRegister2 = 0x72,
}

/// Octo-SPI mode commands for the MX25UW25645G flash memory.
//#[allow(dead_code)]
#[repr(u16)]
pub enum OpiCommand {
    // Array access commands
    /// Read data using 8 I/O lines in STR mode with configurable dummy cycles (up to 200 MHz)
    OctaRead = 0xEC13,
    /// Read data using 8 I/O lines in DTR mode with configurable dummy cycles (up to 200 MHz)
    OctaDTRRead = 0xEE11,
    /// Program 1-256 bytes using 4-byte address and 8 I/O lines
    PageProgram4B = 0x12ED,
    /// Erase 4KB sector using 4-byte address
    SectorErase4B = 0x21DE,
    /// Erase 64KB block using 4-byte address
    BlockErase4B = 0xDC23,
    /// Erase entire chip (only if no blocks are protected)
    ChipErase = 0x609F,

    // Write Buffer Access commands
    /// Read data from the 256-byte page buffer using 4-byte address
    ReadBuffer = 0x25DA,
    /// Initialize interruptible write-to-buffer sequence with 4-byte address
    WriteBufferInitial = 0x22DD,
    /// Continue writing data to buffer during interruptible sequence
    WriteBufferContinue = 0x24DB,
    /// Confirm and execute write operation from buffer to flash array
    WriteBufferConfirm = 0x31CE,

    // Device operation commands
    /// Set Write Enable Latch (WEL) bit, required before write/program/erase operations
    WriteEnable = 0x06F9,
    /// Clear Write Enable Latch (WEL) bit, aborts write-to-buffer sequence
    WriteDisable = 0x04FB,
    /// Select write protection mode (BP mode or Advanced Sector Protection) - OTP bit
    WriteProtectSelection = 0x6897,
    /// Suspend ongoing program or erase operation to allow read from other banks
    ProgramEraseSuspend = 0xB04F,
    /// Resume suspended program or erase operation
    ProgramEraseResume = 0x30CF,
    /// Enter deep power-down mode for minimum power consumption
    DeepPowerDown = 0xB946,
    /// Exit deep power-down mode and return to standby
    ReleaseFromDeepPowerDown = 0xAB54,
    /// No operation, can terminate Reset Enable command
    NoOperation = 0x00FF,
    /// Enable reset operation (must precede Reset Memory command)
    ResetEnable = 0x6699,
    /// Reset device to power-on state, clears volatile settings
    ResetMemory = 0x9966,
    /// Protect all sectors using Dynamic Protection Bits (DPB)
    GangBlockLock = 0x7E81,
    /// Unprotect all sectors by clearing Dynamic Protection Bits (DPB)
    GangBlockUnlock = 0x9867,

    // Register Access commands
    /// Read 3-byte device identification with 4-byte dummy address
    ReadIdentification = 0x9F60,
    /// Read Serial Flash Discoverable Parameters (SFDP) table with 4-byte address
    ReadSFDP = 0x5AA5,
    /// Read 8-bit Status Register with 4-byte dummy address
    ReadStatusRegister = 0x05FA,
    /// Read 8-bit Configuration Register with specific address (00000001h)
    ReadConfigurationRegister = 0x15EA,
    /// Write 8-bit Status Register with specific address (00000000h) or Configuration Register with address (00000001h)
    WriteStatusConfigurationRegister = 0x01FE,
    /// Read Configuration Register 2 from specified 4-byte address
    ReadConfigurationRegister2 = 0x718E,
    /// Write Configuration Register 2 to specified 4-byte address
    WriteConfigurationRegister2 = 0x728D,
    /// Read 8-bit Security Register with 4-byte dummy address
    ReadSecurityRegister = 0x2BD4,
    /// Write Security Register to set customer lock-down bit
    WriteSecurityRegister = 0x2FD0,
    /// Set burst/wrap length for read operations with 4-byte dummy address
    SetBurstLength = 0xC03F,
    /// Read 32-bit Fast Boot Register with 4-byte dummy address
    ReadFastBootRegister = 0x16E9,
    /// Write 32-bit Fast Boot Register with 4-byte dummy address
    WriteFastBootRegister = 0x17E8,
    /// Erase Fast Boot Register (disable fast boot feature)
    EraseFastBootRegister = 0x18E7,
    /// Enter 8K-bit secured OTP mode for programming unique identifiers
    EnterSecuredOTP = 0xB14E,
    /// Exit secured OTP mode and return to main array access
    ExitSecuredOTP = 0xC13E,
    /// Write Lock Register to control SPB protection mode with 4-byte dummy address
    WriteLockRegister = 0x2CD3,
    /// Read Lock Register status with 4-byte dummy address
    ReadLockRegister = 0x2DD2,
    /// Program Solid Protection Bit (SPB) for specified 4-byte address
    WriteSPB = 0xE31C,
    /// Erase all Solid Protection Bits (SPB)
    EraseSPB = 0xE41B,
    /// Read Solid Protection Bit (SPB) status for specified 4-byte address
    ReadSPB = 0xE21D,
    /// Write Dynamic Protection Bit (DPB) for specified 4-byte address
    WriteDPB = 0xE11E,
    /// Read Dynamic Protection Bit (DPB) status for specified 4-byte address
    ReadDPB = 0xE01F,
    /// Read 64-bit password register with 4-byte dummy address and 20 dummy cycles
    ReadPassword = 0x27D8,
    /// Write 64-bit password register with 4-byte dummy address
    WritePassword = 0x28D7,
    /// Unlock SPB operations using 64-bit password with 4-byte dummy address
    PasswordUnlock = 0x29D6,
}

/// Output drive strength
/// Resistance choices listed in Ohms, for the BGA package.
#[allow(dead_code)]
#[repr(u8)]
pub enum OutputDriveStrength {
    R146 = 0x00,
    R76 = 0x01,
    R52 = 0x02,
    R41 = 0x03,
    R34 = 0x04,
    R30 = 0x05,
    R26 = 0x06,
    R24 = 0x07,
}

/// Access the Macronix MX25UW25645GXDI00 flash chip using Octo SPI.
pub struct OpiFlashMemory<I: Instance> {
    xspi: Xspi<'static, I, Blocking>,
}

impl<I: Instance> OpiFlashMemory<I> {
    pub fn new(xspi: Xspi<'static, I, Blocking>) -> Self {
        // Obtain a handle on the interface for the chip.
        let mut memory = Self { xspi };

        // Reset the memory before doing anything else.
        // This happens with the chip still in SPI mode
        memory.reset_memory_spi();

        // Set 24 Ohm drive strength.
        // TODO: config enum.
        /*
        let cr2_19 = memory.read_cr2_spi(19);
        memory.exec_command_spi(SpiCommand::WriteEnable as u8);
        memory.write_cr2_spi(19, cr2_19 | 0x07);
        */

        // Enable Octo-SPI in DTR mode.
        // Note: Do this as the last init step.
        let cr2_0 = memory.read_cr2_spi(0);
        info!("Read CR2 at 0x0: {:x}", cr2_0);
        memory.exec_command_spi(SpiCommand::WriteEnable as u8);
        memory.write_cr2_spi(0, cr2_0 | 0x02); // Set bit 1 to enable octo SPI in DTR

        // Did that work???
        let cr2_0 = memory.read_cr2(0);
        info!("Read CR2 at 0x0 DTR: {:x}", cr2_0);

        /*
        // Set 24 Ohm drive strength.
        // TODO: 19 or 0x19?????
        let cr2_19 = memory.read_cr2(0x19);
        info!("Read CR2 at 0x19 DTR: {:x}", cr2_19);
        memory.exec_command(OpiCommand::WriteEnable);
        memory.write_cr2(0x19, cr2_19 | OutputDriveStrength::R24 as u8); // WRONG: set bits must also be zeroed.... Check this reg lay-out + mask?
        let cr2_19 = memory.read_cr2(0x19);
        info!("Read CR2 at 0x19 DTR: {:x}", cr2_19);
        */

        /*
        // Bump the flash speed now DTR mode is enabled.
        // TODO: still fails miserably...
        let mut cfg = memory.xspi.get_config();
        cfg.clock_prescaler = 1; // DIV/(1+1), so 150 MHz with 300 MHz clk.
        memory.xspi.set_config(&cfg);
        */

        memory
    }

    fn reset_memory_spi(&mut self) {
        self.exec_command_spi(SpiCommand::ResetEnable as u8);
        self.exec_command_spi(SpiCommand::ResetMemory as u8);
        self.wait_write_finish_spi();
    }

    fn wait_write_finish_spi(&mut self) {
        while (self.read_register_spi(SpiCommand::ReadStatusRegister as u8) & 0x01) != 0 {}
    }

    fn exec_command_spi(&mut self, cmd: u8) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::SING,
            adwidth: XspiWidth::NONE,
            // adsize: AddressSize::_24bit,
            dwidth: XspiWidth::NONE,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.xspi.blocking_command(&transaction).unwrap();
    }

    // Note: read_register cannot be used to read the configuration register 2 since there is an
    // address required for that read.
    fn read_register_spi(&mut self, cmd: u8) -> u8 {
        let mut buffer = [0; 1];
        let transaction: TransferConfig = TransferConfig {
            iwidth: XspiWidth::SING,
            isize: AddressSize::_8bit,
            adwidth: XspiWidth::NONE,
            dwidth: XspiWidth::SING,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.xspi.blocking_read(&mut buffer, transaction).unwrap();
        buffer[0]
    }

    fn read_cr2_spi(&mut self, address: u32) -> u8 {
        let mut buffer = [0; 1];
        let transaction: TransferConfig = TransferConfig {
            iwidth: XspiWidth::SING,
            isize: AddressSize::_8bit,
            instruction: Some(SpiCommand::ReadConfigurationRegister2 as u32),
            adsize: AddressSize::_32bit,
            adwidth: XspiWidth::SING,
            dwidth: XspiWidth::SING,
            address: Some(address),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.xspi.blocking_read(&mut buffer, transaction).unwrap();
        buffer[0]
    }

    fn write_cr2_spi(&mut self, address: u32, value: u8) {
        let buffer = [value; 1];
        let transaction: TransferConfig = TransferConfig {
            iwidth: XspiWidth::SING,
            isize: AddressSize::_8bit,
            instruction: Some(SpiCommand::WriteConfigurationRegister2 as u32),
            adsize: AddressSize::_32bit,
            adwidth: XspiWidth::SING,
            dwidth: XspiWidth::SING,
            address: Some(address),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.xspi.blocking_write(&buffer, transaction).unwrap();
        self.wait_write_finish_spi();
    }

    /// Enable memory-mapped mode for OPI
    /// TODO
    pub fn enable_mm(&mut self) {
        let read_config = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit, // 2-byte command for OPI
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::OctaDTRRead as u32),
            dummy: DummyCycles::_20, // Default dummy cycles for OPI
            ..Default::default()
        };

        let write_config = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true, // DTR mode.
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true, // DTR mode.
            dwidth: XspiWidth::OCTO,
            ddtr: true, // DTR mode.
            instruction: Some(OpiCommand::PageProgram4B as u32),
            dummy: DummyCycles::_0,
            ..Default::default()
        };

        self.xspi
            .enable_memory_mapped_mode(read_config, write_config)
            .unwrap();
    }

    pub fn disable_mm(&mut self) {
        self.xspi.disable_memory_mapped_mode();
    }

    /// Execute OPI command (2-byte command)
    /// TODO
    fn exec_command(&mut self, cmd: OpiCommand) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit, // 2-byte command
            idtr: true,
            adwidth: XspiWidth::NONE,
            dwidth: XspiWidth::NONE,
            instruction: Some(cmd as u32),
            address: None,
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.xspi.blocking_command(&transaction).unwrap();
    }

    /// Enable write using OPI command
    pub fn enable_write(&mut self) {
        self.exec_command(OpiCommand::WriteEnable);
    }

    /// Read device ID in OPI mode
    /// TODO
    pub fn read_id(&mut self) -> [u8; 4] {
        let mut buffer = [0; 4];
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: false, // TODO: L1550 ST driver shows this as FALSE!!!!! Probably because we're reading 3 bytes, which is odd, and not allowed for DTR.
            instruction: Some(OpiCommand::ReadIdentification as u32),
            address: Some(0x00000000),         // Dummy address required
            dummy: DUMMY_CYCLES_REG_OCTAL_DTR, //DummyCycles::_4,    // Works better with 5???
            ..Default::default()
        };
        self.xspi.blocking_read(&mut buffer, transaction).unwrap();
        buffer
    }

    /// Read memory using OPI mode
    /// TODO ST L235
    pub fn read_memory(&mut self, addr: u32, buffer: &mut [u8]) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::OctaDTRRead as u32),
            address: Some(addr),
            dummy: DummyCycles::_20, // 20 Default for 200MHz operation
            ..Default::default()
        };
        self.xspi.blocking_read(buffer, transaction).unwrap();
    }

    /// Wait for write completion using OPI status read
    fn wait_write_finish(&mut self) {
        while (self.read_sr() & 0x01) != 0 {}
    }

    /// Perform erase operation using OPI command
    /// TODO: OK
    fn perform_erase(&mut self, addr: u32, cmd: OpiCommand) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::NONE,
            ddtr: true,
            instruction: Some(cmd as u32),
            address: Some(addr),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.enable_write();
        self.xspi.blocking_command(&transaction).unwrap();
        self.wait_write_finish();
    }

    /// Erase 4KB sector using OPI
    /// TODO: OK
    pub fn erase_sector(&mut self, addr: u32) {
        self.perform_erase(addr, OpiCommand::SectorErase4B);
    }

    /// Erase 64KB block using OPI
    /// TODO: OK
    pub fn erase_block_64k(&mut self, addr: u32) {
        self.perform_erase(addr, OpiCommand::BlockErase4B);
    }

    /// Erase entire chip using OPI
    /// TODO: OK
    pub fn erase_chip(&mut self) {
        self.enable_write();
        self.exec_command(OpiCommand::ChipErase);
        self.wait_write_finish();
    }

    /// Write single page using OPI
    /// TODO
    fn write_page(&mut self, addr: u32, buffer: &[u8], len: usize) {
        assert!(
            (len as u32 + (addr & 0x000000ff)) <= MEMORY_PAGE_SIZE as u32,
            "write_page(): page write length exceeds page boundary (len = {}, addr = {:X})",
            len,
            addr
        );

        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::PageProgram4B as u32),
            address: Some(addr),
            dummy: DummyCycles::_0,
            ..Default::default()
        };
        self.enable_write();
        self.xspi.blocking_write(buffer, transaction).unwrap();
        self.wait_write_finish();
    }

    /// Write memory using OPI (handles page boundaries)
    /// TODO
    pub fn write_memory(&mut self, addr: u32, buffer: &[u8]) {
        let mut left = buffer.len();
        let mut place = addr;
        let mut chunk_start = 0;

        while left > 0 {
            let max_chunk_size = MEMORY_PAGE_SIZE - (place & 0x000000ff) as usize;
            let chunk_size = min(max_chunk_size, left);
            let chunk = &buffer[chunk_start..(chunk_start + chunk_size)];
            self.write_page(place, chunk, chunk_size);
            place += chunk_size as u32;
            left -= chunk_size;
            chunk_start += chunk_size;
        }
    }

    /// Read register using OPI mode
    /// TODO
    fn read_register(&mut self, cmd: OpiCommand, dummy_addr: u32, dummy_cycles: DummyCycles) -> u8 {
        let mut buffer = [0; 1];
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(cmd as u32),
            address: Some(dummy_addr),
            dummy: dummy_cycles,
            ..Default::default()
        };
        self.xspi.blocking_read(&mut buffer, transaction).unwrap();
        buffer[0]
    }

    /// Read Status Register using OPI
    /// TODO
    pub fn read_sr(&mut self) -> u8 {
        self.read_register(
            OpiCommand::ReadStatusRegister,
            0x00000000, // Dummy address
            DummyCycles::_4,
        )
    }

    /// Read Configuration Register using OPI
    /// TODO
    pub fn read_cr(&mut self) -> u8 {
        self.read_register(
            OpiCommand::ReadConfigurationRegister,
            0x00000001, // Address for CR
            DummyCycles::_4,
        )
    }

    /// Write Status/Configuration Register using OPI
    /// TODO
    pub fn write_sr_cr(&mut self, sr: u8, cr: u8) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::WriteStatusConfigurationRegister as u32),
            address: Some(0x00000000),
            dummy: DummyCycles::_0,
            ..Default::default()
        };

        self.enable_write();
        self.xspi.blocking_write(&[sr, cr], transaction).unwrap();
        self.wait_write_finish();
    }

    /// Read Configuration Register 2 using OPI
    /// TODO So, we need just one BYTE, but need to R/W 2 for even length under DTR.
    ///      ST probably does something smart in HAL_XSPI_TRANSMIT and COMMAND....
    ///      ST L1311
    pub fn read_cr2(&mut self, address: u32) -> u8 {
        let mut buffer = [0; 2]; // L1353 ST (DTR mode requires an even number of bytes read.)
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::ReadConfigurationRegister2 as u32),
            address: Some(address),
            dummy: DummyCycles::_4,
            ..Default::default()
        };
        self.xspi.blocking_read(&mut buffer, transaction).unwrap();
        buffer[0]
    }

    /// Write Configuration Register 2 using OPI
    /// TODO So, we need just one BYTE, but need to R/W 2 for even length under DTR.
    ///      ST probably does something smart in HAL_XSPI_TRANSMIT and COMMAND....
    ///      ST L1244
    pub fn write_cr2(&mut self, address: u32, value: u8) {
        let transaction = TransferConfig {
            iwidth: XspiWidth::OCTO,
            isize: AddressSize::_16bit,
            idtr: true,
            adwidth: XspiWidth::OCTO,
            adsize: AddressSize::_32bit,
            addtr: true,
            dwidth: XspiWidth::OCTO,
            ddtr: true,
            instruction: Some(OpiCommand::WriteConfigurationRegister2 as u32),
            address: Some(address),
            dummy: DummyCycles::_0,
            ..Default::default()
        };

        // Need two bytes......
        let word = value as u16;

        self.enable_write();
        //self.xspi.blocking_write(&[value], transaction).unwrap();
        self.xspi.blocking_write(&[word], transaction).unwrap();
        self.wait_write_finish();
    }
}
