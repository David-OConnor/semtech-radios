use hal::gpio::Pin;

use crate::{CommandStatus, OperatingModeRead};

#[derive(Clone)]
pub struct RadioPins {
    pub cs: Pin,
    pub busy: Pin,
    pub reset: Pin,
}

/// Split a u16 address into two bytes.
pub fn split_addr(addr: u16) -> (u8, u8) {
    let result = addr.to_be_bytes();
    (result[0], result[1])
}

pub const MAX_ITERS: u32 = 400_000;

// todo: Make sure this generalizes to 1280
/// Error types associated with the radio and this library.
#[derive(Debug, defmt::Format, PartialEq)]
pub enum RadioError {
    /// An error with SPI IO.
    Spi,
    /// Invalid operating mode or command status is reported.
    // Status((OperatingModeRead, CommandStatus)),
    Status((OperatingModeRead, CommandStatus)),
    /// Invalid CRC, as reported by IRQ status,
    Crc,
    /// Device error
    Device,
    /// An unacceptable value is configured.
    Config,
    PayloadSize(usize),
    BusyTimeout,
    UnexpectedStatus(u8),
}

#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)]
#[repr(u8)]
/// Sx126x: See Table 11-1.
/// todo: I think think this table was lifted from a rust lib, and is a mix of 126x and 128x commands.
/// todo: FOr now, use for sx126x, and overrite these for sx128x below in the method.
pub enum OpCode {
    GetStatus = 0xC0,
    WriteRegister = 0x0D,
    ReadRegister = 0x1D,
    WriteBuffer = 0x0E,
    ReadBuffer = 0x1E,
    SetSleep = 0x84,
    SetStandby = 0x80,
    SetFS = 0xC1,
    SetTx = 0x83,
    SetRx = 0x82,
    SetRxDutyCycle = 0x94,
    SetCAD = 0xC5,
    SetTxContinuousWave = 0xD1,
    SetTxContinuousPremable = 0xD2,
    SetPacketType = 0x8A,
    GetPacketType = 0x11,
    SetRfFrequency = 0x86,
    SetTxParams = 0x8E,
    SetPAConfig = 0x95,
    SetCADParams = 0x88,
    SetBufferBaseAddress = 0x8F,
    SetModulationParams = 0x8B,
    SetPacketParams = 0x8C,
    GetRxBufferStatus = 0x13,
    GetPacketStatus = 0x14,
    GetRSSIInst = 0x15,
    GetStatistics = 0x10,
    ResetStats = 0x00,
    SetDioIrqParams = 0x08,
    GetIrqStatus = 0x12,
    ClearIrqStatus = 0x02,
    Calibrate = 0x89,
    CalibrateImage = 0x98,
    SetRegulatorMode = 0x96,
    GetDeviceErrors = 0x17,
    ClrErrors = 0x07,
    SetTCXOMode = 0x97,
    SetTxFallbackMode = 0x93,
    SetDIO2AsRfSwitchCtrl = 0x9d,
    SetStopRxTimerOnPreamble = 0x9F,
    SetLoRaSymbTimeout = 0xA0,
    // below: sx128x only. Ommitted for now due to conflicts.
    SetSaveContext = 0xd5,
    // SetAutoTx = 0x98,
    SetLongPreamble = 0x9b,
    // SetUartSpeed = 0x9d,
    SetRangingRole = 0xa3,
    SetAdvancedRnaging = 0x91,
}

impl OpCode {
    /// Register value for sx128x. 126x uses the u8 value for now, for backwards compat.
    /// The same spreading factors are available on both, but with reversed reg hex positions.
    /// todo: Commented-out lines are present on 126x, but absent on sx128x.
    pub fn val_8x(&self) -> u8 {
        match self {
            Self::WriteRegister => 0x18,
            Self::ReadRegister => 0x19,
            Self::WriteBuffer => 0x1a,
            Self::ReadBuffer => 0x1b,
            Self::GetPacketType => 0x03,
            // Self::SetPAConfig => 0x95,
            Self::GetRxBufferStatus => 0x17,
            Self::GetPacketStatus => 0x1d,
            Self::GetRSSIInst => 0x1f,
            // Self::GetStats => 0x10,
            // Self::ResetStats => 0x00,
            Self::SetDioIrqParams => 0x8d,
            Self::GetIrqStatus => 0x15,
            Self::ClearIrqStatus => 0x97,
            // Self::Calibrate => 0x89,
            // Self::CalibrateImage => 0x98,
            // Self::GetDeviceErrors => 0x17,
            // Self::ClrErrors => 0x07,
            // Self::SetTCXOMode => 0x97,
            // Self::SetTxFallbackMode => 0x93,
            // Self::SetDIO2AsRfSwitchCtrl => 0x9d,
            // Self::SetStopRxTimerOnPreamble => 0x9F,
            // Self::SetLoRaSymbTimeout => 0xA0,
            _ => *self as u8,
        }
    }
}

pub enum Register {
    Reg6x(Register6x),
    Reg8x(Register8x),
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
#[repr(u16)]
/// Registers, to read and write following the appropriate OpCode.
/// See DS, section 12.1: Register Table
pub enum Register6x {
    HoppingEnabled = 0x0385,
    PacketLength = 0x0386, // FSK
    NbHoppingBLocks = 0x0387,
    // todo: Sort out how these nbsymbols and freqs work.
    NbSymbols0 = 0x0388,
    NbSymbols0b = 0x0389,
    Freq0a = 0x038a,
    Freq0b = 0x038b,
    Freq0c = 0x038c,
    Freq0d = 0x038d,
    DioxOutputEnable = 0x0580,
    DioxInputEnable = 0x0583,
    DioxPullUpControl = 0x0584,
    DioxPullDownControl = 0x0585,
    WhiteningInitialValueMsb = 0x06b8,
    WhiteningInitialValueLsb = 0x06b9,
    CrcMsbInitialValue0 = 0x06bc,
    CrcMsbInitialValue1 = 0x06bd,
    CrcMsbPolynomialValue0 = 0x06be,
    CrcLsbPolynomialValue1 = 0x06bf,
    /// These are for FSK. Bytes of the sync word.
    SyncWord0 = 0x06c0,
    SyncWord1 = 0x06c1,
    SyncWord2 = 0x06c2,
    SyncWord3 = 0x06c3,
    SyncWord4 = 0x06c4,
    SyncWord5 = 0x06c5,
    SyncWord6 = 0x06c6,
    SyncWord7 = 0x06c7,
    NodeAddress = 0x06cd,
    BroadcastAddress = 0x06ce,
    IqPolaritySetup = 0x0736,
    /// These sync words must be set to the constants defined at the top of this module.
    LoraSyncWordMsb = 0x0740,
    LoraSyncWordLsb = 0x0741,
    RandomNumGen0 = 0x819,
    RandomNumGen1 = 0x81a,
    RandomNumGen2 = 0x81b,
    RandomNumGen3 = 0x81c,
    TxModulation = 0x0889,
    RxGain = 0x08ac,
    TxClampConfig = 0x08d8,
    OcpConfiguration = 0x08e7,
    RtcControl = 0x0902,
    XtaTrim = 0x0911,
    XtbTrim = 0x912,
    Dio3OutputVoltageControl = 0x0920,
    EventMask = 0x0944,
    /// These three registers aren't listed in Table 12.1, but apparently
    /// exist from the DS-included RxGain retention workaround.
    RxGainRetention0 = 0x029f,
    RxGainRetention1 = 0x02a0,
    RxGainRetention2 = 0x02a1,
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
#[repr(u16)]
/// See DS, table 13-1: Register Table
pub enum Register8x {
    RxGain = 0x891,
    ManualGainSetting = 0x895,
    LnaGainValue = 0x89e,
    LnaGainControl = 0x89f,
    SynchPeakAttenuation = 0x8c2,
    PayloadLength = 0x901,
    LoraHeaderMode = 0x903,
    RangingRequestAddressByte3 = 0x912,
    RangingRequestAddressByte2 = 0x913,
    RangingRequestAddressByte1 = 0x914,
    RangingRequestAddressByte0 = 0x915,
    RangingDeviceAddressByte3 = 0x916,
    RangingDeviceAddressByte2 = 0x917,
    RangingDeviceAddressByte1 = 0x918,
    RangingDeviceAddressByte0 = 0x919,
    RangingFilterWindowSize = 0x91e,
    ResetRangingFilter = 0x923,
    RangingResultMux = 0x924,
    SfAdditionalConfiguration = 0x925,
    // SKipping other ranging regs here for now.
    FrequencyErrorCorrection = 0x93C,
    LoraSynchWordA = 0x944,
    LoraSynchWordB = 0x945, // todo: Typo in DS:? Listed as 0x955.
    FeiByte2 = 0x954,
    FeiByte1 = 0x955,
    FeiByte0 = 0x956,
    PacketPreambleSettings = 0x9c1,
    WhiteningInitialValue = 0x9c5,
    CrcPolynomialDefinitionMsb = 0x9c6,
    CrcPolynomialDefinitionLsb = 0x9c7,
    // CrcPolynomialSeedByte2 = 0x9c7, // todo what? Duplicate
    // CrcPolynomialSeedByte1 = 0x9c8,
    // CrcPolynomialSeedByte0 = 0x9c9,
    // CrcMsbInitialValue = 0x9c8, // todo: More dupes.
    // CrcLsbInitialValue = 0x9c9,
    SynchAddressControl = 0x9cd,
    SyncAddress1Byte4 = 0x9ce,
    SyncAddress1Byte3 = 0x9cf,
    SyncAddress1Byte2 = 0x9d0,
    SyncAddress1Byte1 = 0x9d1,
    SyncAddress1Byte0 = 0x9d2,
    SyncAddress2Byte4 = 0x9c3,
    SyncAddress2Byte3 = 0x9c4,
    SyncAddress2Byte2 = 0x9d5,
    SyncAddress2Byte1 = 0x9d6,
    SyncAddress2Byte0 = 0x9d7,
    SyncAddress3Byte4 = 0x9c8,
    SyncAddress3Byte3 = 0x9c9,
    SyncAddress3Byte2 = 0x9da,
    SyncAddress3Byte1 = 0x9db,
    SyncAddress3Byte0 = 0x9dc,
}
