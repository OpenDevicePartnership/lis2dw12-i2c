use bilge::prelude::*;

pub const WHO_AM_I_EXPECTED: u8 = 0x44;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub enum Register {
    TempOutLow = 0x0D,         // OUT_T_L
    TempOutHigh = 0x0E,        // OUT_T_H
    WhoAmI = 0x0F,             // WHO_AM_I
    Control1 = 0x20,           // CTRL1
    Control2 = 0x21,           // CTRL2
    Control3 = 0x22,           // CTRL3
    Control4Interrupt1 = 0x23, // CTRL4_INT1_PAD_CTRL
    Control5Interrupt2 = 0x24, // CTRL5_INT2_PAD_CTRL
    Control6 = 0x25,           // CTRL6
    TempOut = 0x26,            // OUT_T
    Status = 0x27,             // STATUS
    XOutLow = 0x28,            // OUT_X_L
    XOutHigh = 0x29,           // OUT_X_H
    YOutLow = 0x2A,            // OUT_Y_L
    YOutHigh = 0x2B,           // OUT_Y_H
    ZOutLow = 0x2C,            // OUT_Z_L
    ZOutHigh = 0x2D,           // OUT_Z_H
    FifoControl = 0x2E,        // FIFO_CTRL
    FifoSamples = 0x2F,        // FIFO_SAMPLES
    TapThresholdX = 0x30,      // TAP_THS_X
    TapThresholdY = 0x31,      // TAP_THS_Y
    TapThresholdZ = 0x32,      // TAP_THS_Z
    InterruptDuration = 0x33,  // INT_DUR
    WakeUpThreshold = 0x34,    // WAKE_UP_THS
    WakeUpDuration = 0x35,     // WAKE_UP_DUR
    FreeFall = 0x36,           // FREE_FALL
    EventStatus = 0x37,        // STATUS_DUP
    WakeUpSource = 0x38,       // WAKE_UP_SRC
    TapSource = 0x39,          // TAP_SRC
    SixDimSource = 0x3A,       // SIXD_SRC
    AllInterruptSource = 0x3B, // ALL_INT_SRC
    XOffsetUser = 0x3C,        // X_OFS_USR
    YOffsetUser = 0x3D,        // Y_OFS_USR
    ZOffsetUser = 0x3E,        // Z_OFS_USR
    Control7 = 0x3F,           // CTRL7
}

#[bitsize(1)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Reserved0 {
    #[fallback]
    Res0 = 0b0,
}

/// Control Register 1 (0x20 R/W)
/// ODR3 ODR2 ODR1 ODR0 MODE1 MODE0 LP_MODE1 LP_MODE0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg1 {
    /// Low-power mode selection
    pub low_power_mode: Control1LowPowerMode,

    /// Mode and resolution selection
    pub mode: Control1ModeSelect,

    /// Power mode / Data Rate Configuration
    pub data_rate: Control1DataRate,
}

#[bitsize(4)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control1DataRate {
    PowerDown = 0b0000,     // Power down
    Hi12p5Lo1p6Hz = 0b0001, // High-Performance: 12.5 Hz, Low-Power: 1.6 Hz
    HiLo12p5Hz = 0b0010,    // 12.5 Hz
    HiLo25Hz = 0b0011,      // 25 Hz
    HiLo50Hz = 0b0100,      // 50 Hz
    HiLo100Hz = 0b0101,     // 100 Hz
    HiLo200Hz = 0b0110,     // 200 Hz
    Hi400Lo200Hz = 0b0111,  // High-Performance: 400 Hz, Low-Power: 200 Hz
    Hi800Lo200Hz = 0b1000,  // High-Performance: 800 Hz, Low-Power: 200 Hz
    Hi1600Lo200Hz = 0b1001, // High-Performance: 1600 Hz, Low-Power: 200 Hz
    #[fallback]
    Unused,
}

#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control1ModeSelect {
    LowPower = 0b00,        // 12/14 bit resolution
    HighPerformance = 0b01, // 14 bit resolution
    OnDemand = 0b10,        // 12/14 bit resolution
    Unused = 0b11,
}

#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control1LowPowerMode {
    LowPower1 = 0b00, // 12 bit resolution
    LowPower2 = 0b01, // 14 bit resolution
    LowPower3 = 0b10, // 14 bit resolution
    LowPower4 = 0b11, // 14 bit resolution
}

/// Control Register 2 (0x21 R/W)
/// BOOT SOFT_RESET RES0 CS_PU_DISC BDU IF_ADD_INC I2C_DISABLE SIM
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg2 {
    // SPI serial interface mode selection
    // 0: 4-wire interface; 1: 3-wire interface
    pub sim: bool,

    //  Disable I2C communication protocol
    // 0: SPI and I2C interfaces enabled; 1: I2C mode disabled)
    pub i2c_disable: bool,

    // Register address automatically incremented during multiple byte access with a serial interface
    // Default: 1
    pub if_add_inc: bool,

    // Block Data Update
    // 0: continuous update; 1: output registers not updated until MSB and LSB read
    pub bdu: bool,

    //  Disconnect CS pull-up
    pub cs_pu_disc: bool,

    // This bit must be set to 0 for the correct operation of the device
    pub res0: Reserved0,

    // Soft reset acts as reset for all control registers, then goes to 0
    pub soft_reset: bool,

    // Boot enables retrieving the correct trimming parameters from nonvolatile memory into
    // registers where trimming parameters are stored.
    // Once the operation is over, this bit automatically returns to 0
    pub boot: bool,
}

/// Control Register 3 (0x22 R/W)
/// ST1 ST0 PP_OD LIR H_LACTIVE RES0 SLP_MODE_SEL SLP_MODE_1
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg3 {
    //  Single data conversion on-demand mode enable.
    // When SLP_MODE_SEL = 1 and this bit is set to 1 logic, single data conversion on-demand mode starts.
    // When accelerometer data are available in the registers, this bit is set to 0 automatically and
    // the device is ready for another triggered session
    pub slp_mode_1: bool,

    // Single data conversion on demand mode selection
    // 0: enabled with external trigger on INT2
    // 1: enabled by I2C/SPI writing SLP_MODE_1 to 1
    pub slp_mode_sel: bool,

    // Reserved 0
    pub res0: Reserved0,

    // Interrupt active high (0) or low (1)
    pub active_hi_lo: bool,

    // Latched interrupt. Switches between latched (1 logic) and pulsed (0 logic) mode for function
    // source signals and interrupts routed to pins (wake-up, single/double-tap)
    pub lir: bool,

    // Push-Pull or Open-Drain selection on interrupt pad
    pub pp_od: bool,

    // Enables self-test
    pub self_test: Control3SelfTest,
}

#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control3SelfTest {
    NormalMode = 0b00,   // Self test disabled
    PositiveSign = 0b01, // Positive sign self test
    NegativeSign = 0b10, // Negative sign self test
    Unused = 0b11,
}

/// Control Register 4 Interrupt 1 Pad Control (0x23 R/W)
/// INT1_6D INT1_SINGLE_TAP INT1_WU INT1_FF INT1_TAP INT1_DIFF5 INT1_FTH INT1_DRDY
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg4Int1Pad {
    // Data-ready is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_drdy: bool,

    // FIFO threshold interrupt is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_fth: bool,

    // FIFO full recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_diff5: bool,

    // Double-tap recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_tap: bool,

    // Free-fall recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_ff: bool,

    // Wake-up recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_wu: bool,

    // Single-tap recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_single_tap: bool,

    // 6D recognition is routed to INT1 pad
    // 0: Disabled 1: Enabled
    pub int1_6d: bool,
}

/// Control Register 5 Interrupt 2 Pad Control (0x24 R/W)
/// INT2_SLEEP_STATE INT2_SLEEP_CHG INT2_BOOT INT2_DRDY_T INT2_OVR INT2_DIFF5 INT2_FTH INT2_DRDY
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg5Int2Pad {
    // Data-ready is routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_drdy: bool,

    // FIFO threshold interrupt is routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_fth: bool,

    // FIFO full recognition is routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_diff5: bool,

    // FIFO overrun interrupt is routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_ovr: bool,

    // Temperature data-ready is routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_drdy_t: bool,

    // Boot state routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_boot: bool,

    // Sleep change status routed to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_sleep_change: bool,

    // Enables routing SLEEP_STATE to INT2 pad
    // 0: Disabled 1: Enabled
    pub int2_sleep_state: bool,
}

/// Control Register 6 (0x25 R/W)
/// BW_FILT1 BW_FILT0 FS1 FS0 FDS LOW_NOISE 0 0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg6 {
    // Reserved 0
    pub res0: [Reserved0; 2],

    // Low-noise configuration
    // 0: Disabled 1: Enabled
    pub low_noise: bool,

    // Filtered data type selection
    // 0: Low-pass filter path selected 1: High-pass filter path selected
    pub fds: bool,

    // Full-scale selection
    pub fs: Control6FullScale,

    // Bandwidth/Digital filtering cutoff selection
    pub bw_filt: Control6BandwidthSelection,
}

#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control6BandwidthSelection {
    ODRdiv2 = 0b00,  //  ODR/2 (up to ODR = 800 Hz, 400 Hz when ODR = 1600 Hz)
    ODRdiv4 = 0b01,  //  ODR/4 (HP/LP)
    ODRdiv10 = 0b10, //  ODR/4 (HP/LP)
    ODRdiv20 = 0b11, //  ODR/4 (HP/LP)
}

#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum Control6FullScale {
    Scale2g = 0b00,  // +- 2g
    Scale4g = 0b01,  // +- 4g
    Scale8g = 0b10,  // +- 8g
    Scale16g = 0b11, // +- 16g
}

/// Status Register (0x27 R)
/// FIFO_THS WU_IA SLEEP_STATE DOUBLE_TAP SINGLE_TAP 6D_IA FF_IA DRDY
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct StatusReg {
    // Data-ready status
    // 0: not ready 1: X-, Y- and Z-axis new data available
    pub drdy: bool,

    // Free-fall event detection status
    // 0: free-fall event not detected 1: free-fall event detected
    pub ff_ia: bool,

    // Source of change in position portrait/landscape/face-up/face-down
    // 0: no event detected 1: a change in position detected
    pub sixd_ia: bool,

    // Single-tap event status
    // 0: single-tap event not detected 1: single-tap event detected
    pub single_tap: bool,

    // Double-tap event status
    // 0: double-tap event not detected 1: double-tap event detected
    pub double_tap: bool,

    // Sleep event status
    // 0: sleep event not detected 1: sleep event detected
    pub sleep_state: bool,

    // Wake-up event detection status
    // 0: wake-up event not detected 1: wake-up event detected
    pub wu_ia: bool,

    // FIFO threshold status flag
    // 0: FIFO filling is lower than threshold level
    // 1: FIFO filling is equal to or higher than the threshold level
    pub fifo_ths: bool,
}

/// FIFO Control Register (0x2E R/W)
/// FMode2 FMode1 FMode0 FTH4 FTH3 FTH2 FTH1 FTH0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct FifoControlReg {
    // FIFO threshold level setting
    pub fth: u5,

    // FIFO mode selection bits. Default: 000
    pub fmode: FifoControlModeSelect,
}

#[bitsize(3)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum FifoControlModeSelect {
    #[fallback]
    Bypass = 0b000, // Bypass mode: FIFO turned off
    Fifo = 0b001,               // FIFO mode: Stops collecting data when FIFO is full
    ContinuousToFifo = 0b011,   // Continuous-to-FIFO: stream mode until trigger is deasserted, then FIFO mode
    BypassToContinuous = 0b100, // Bypass-to-continuous: bypass mode until trigger is deasserted, then FIFO mode
    Continuous = 0b110,         // Continuous mode: If the FIFO is full, the new sample overwrites the older sample
}

/// FIFO Samples Control Register (0x2F R)
/// FIFO_FTH FIFO_OVR Diff5 Diff4 Diff3 Diff2 Diff1 Diff0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct FifoSamplesControlReg {
    // Represents the number of unread samples stored in FIFO.
    // (000000 = FIFO empty; 100000 = FIFO full, 32 unread samples)
    pub diff: u6,

    // FIFO overrun status
    // 0: FIFO is not completely filled
    // 1: FIFO is completely filled and at least one sample has been overwritten
    pub fifo_ovr: bool,

    // FIFO threshold status flag
    // 0: FIFO filling is lower than threshold level
    // 1: FIFO filling is equal to or higher than the threshold level
    pub fifo_fth: bool,
}

/// Tap Threshold X (0x30 R/W)
/// Enables 4D configuration and configures TAP threshold
/// 4D_EN 6D_THS1 6D_THS0 TAP_THSX_4 TAP_THSX_3 TAP_THSX_2 TAP_THSX_1 TAP_THSX_0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct TapThresholdXReg {
    // Threshold for TAP recognition @ +- 2g on X direction
    pub tap_thsx: u5,

    // Thresholds for 4D/6D function @ FS = +-2 g
    pub ths_6d: TapX6dThresholdDecoding,

    // Enables 4D detection portrait/landscape position
    // 0: no position detected;
    // 1: portrait/landscape detection and face-up/face-down position enabled
    pub en_4d: bool,
}

/// Threshold setting for 4D/6D function @ +- 2g
#[bitsize(2)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum TapX6dThresholdDecoding {
    Ths6Decoding80deg = 0b00,  // Threshold decoding 6: 80 degrees
    Ths11Decoding70deg = 0b01, // Threshold decoding 11: 70 degrees
    Ths16Decoding60deg = 0b10, // Threshold decoding 16: 60 degrees
    Ths21Decoding50deg = 0b11, // Threshold decoding 21: 50 degrees
}

/// Tap Threshold Y (0x31 R/W)
/// TAP_PRIOR_2 TAP_PRIOR_1 TAP_PRIOR_0 TAP_THSY_4 TAP_THSY_3 TAP_THSY_2 TAP_THSY_1 TAP_THSY_0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct TapThresholdYReg {
    // Threshold for TAP recognition @ +- 2g on Y direction
    pub tap_thsy: u5,

    //  Selection of priority axis for tap detection
    pub tap_prior: TapYTapPriority,
}

#[bitsize(3)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum TapYTapPriority {
    PriXYZ = 0b000,
    PriYXZ = 0b001,
    PriXZY = 0b010,
    PriZYX = 0b011,
    PriXYZ1 = 0b100,
    PriYZX = 0b101,
    PriZXY = 0b110,
    PriZYX1 = 0b111,
}

/// Tap Threshold Z (0x32 R/W)
/// TAP_X_EN TAP_Y_EN TAP_Z_EN TAP_THSZ_4 TAP_THSZ_3 TAP_THSZ_2 TAP_THSZ_1 TAP_THSZ_0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct TapThresholdZReg {
    // Threshold for TAP recognition @ +- 2g on Z direction
    pub tap_thsz: u5,

    // Enables Z direction in tap recognition
    // 0: disabled 1: enabled
    pub tap_z_en: bool,

    // Enables Y direction in tap recognition
    // 0: disabled 1: enabled
    pub tap_y_en: bool,

    // Enables X direction in tap recognition
    // 0: disabled 1: enabled
    pub tap_x_en: bool,
}

/// Interrupt Duration (0x33 R/W)
/// LATENCY3 LATENCY2 LATENCY1 LATENCY0 QUIET1 QUIET0 SHOCK1 SHOCK0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct InterruptDurationReg {
    // Maximum duration of overthreshold event: this register represents the maximum time of an overthreshold
    // signal detection to be recognized as a tap event.
    // Default value is SHOCK[1:0] = 00 (which is 4 * 1/ODR)
    // 1 LSB = 8 *1/ODR
    pub shock: u2,

    // Expected quiet time after a tap detection: this register represents the time after the first detected tap in which
    // there must not be any overthreshold event.
    // Default value is QUIET[1:0] = 00 (which is 2 * 1/ODR)
    // 1 LSB = 4 * 1/ODR
    pub quiet: u2,

    // Duration of maximum time gap for double-tap recognition. When double-tap recognition is enabled, this
    // register expresses the maximum time between two successive detected taps to determine a double-tap event.
    // Default value is LATENCY[3:0] = 0000 (which is 16 * 1/ODR)
    // 1 LSB = 32 * 1/ODR
    pub latency: u4,
}

/// Wake-up Threshold (0x34 R/W)
/// SINGLE_DOUBLE_TAP SLEEP_ON WK_THS5 WK_THS4 WK_THS3 WK_THS2 WK_THS1 WK_THS0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct WakeUpThresholdReg {
    // Wake-up threshold, 6-bit unsigned 1 LSB = 1/64 of FS. Default value: 000000
    pub wk_ths: u6,

    // Enables sleep (inactivity). Default value: 0
    // 0: sleep disabled; 1: sleep enabled
    pub sleep_on: bool,

    // Enables single/double-tap event. Default value: 0
    // 0: only single-tap event is enabled 1: single and double-tap events are enabled
    pub single_double_tap: bool,
}

/// Wake-up Duration (0x35 R/W)
/// Wake-up and sleep duration configuration register
/// FF_DUR5 WAKE_DUR1 WAKE_DUR0 STATIONARY SLEEP_DUR3 SLEEP_DUR2 SLEEP_DUR1 SLEEP_DUR0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct WakeUpDurationReg {
    // Duration to go in sleep mode
    // Default value is SLEEP_ DUR[3:0] = 0000 (which is 16 * 1/ODR)
    // 1 LSB = 512 * 1/ODR
    pub sleep_mode: u4,

    // Enables stationary detection / motion detection with no automatic ODR change when detecting stationary state
    // 0: disabled; 1: enabled
    pub stationary: bool,

    // Wake-up duration. 1 LSB = 1 *1/ODR
    pub wake_dur: u2,

    // Free-fall duration. In conjunction with FF_DUR [4:0] bit in the FREE_FALL (36h) register.
    // 1 LSB = 1 * 1/ODR
    pub ff_dur5: u1,
}

/// Free Fall (0x36 R/W)
/// Free-fall duration and threshold configuration register
/// FF_DUR4 FF_DUR3 FF_DUR2 FF_DUR1 FF_DUR0 FF_THS2 FF_THS1 FF_THS0
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct FreeFallReg {
    // Free-fall threshold @ FS = ±2 g
    pub ff_ths: FreeFallThresholdDecoding,

    // Free-fall duration. In conjunction with FF_DUR5 bit in the WAKE_UP_DUR (35h) register.
    // 1 LSB = 1 * 1/ODR
    pub ff_dur: u5,
}

#[bitsize(3)]
#[derive(Copy, Clone, Debug, FromBits, PartialEq, PartialOrd)]
pub enum FreeFallThresholdDecoding {
    FfLsb5 = 0b000,
    FfLsb7 = 0b001,
    FfLsb8 = 0b010,
    FfLsb10 = 0b011,
    FfLsb11 = 0b100,
    FfLsb13 = 0b101,
    FfLsb15 = 0b110,
    FfLsb16 = 0b111,
}

/// Event Detection Status Register (0x37 R)
/// OVR DRDY_T SLEEP_STATE_IA DOUBLE_TAP SINGLE_TAP 6D_IA FF_IA DRDY
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct StatusDupReg {
    // Data-ready status
    // 0: not ready 1: X-, Y- and Z-axis new data available
    pub drdy: bool,

    // Free-fall event detection status
    // 0: free-fall event not detected 1: free-fall event detected
    pub ff_ia: bool,

    // Source of change in position portrait/landscape/face-up/face-down
    // 0: no event detected 1: a change in position detected
    pub sixd_ia: bool,

    // Single-tap event status
    // 0: single-tap event not detected 1: single-tap event detected
    pub single_tap: bool,

    // Double-tap event status
    // 0: double-tap event not detected 1: double-tap event detected
    pub double_tap: bool,

    // Sleep event status
    // 0: sleep event not detected 1: sleep event detected
    pub sleep_state_ia: bool,

    // Temperature status
    // 0: data not available; 1: a new set of data is available
    pub drdy_t: bool,

    // FIFO overrun status flag
    // 0: FIFO is not completely filled
    // 1: FIFO is completely filled and at least one sample has been overwritten
    pub ovr: bool,
}

/// Wake-up Source (0x38 R)
/// 0 0 FF_IA SLEEP_STATE IA WU_IA X_WU Y_WU Z_WU
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct WakeUpSourceReg {
    // Wake-up event detection status on Z-axis
    // 0: wake-up event on Z not detected; 1: wake-up event on Z-axis is detected
    pub z_wu: bool,

    // Wake-up event detection status on Y-axis
    // 0: wake-up event on Y not detected; 1: wake-up event on Y-axis is detected
    pub y_wu: bool,

    // Wake-up event detection status on X-axis
    // 0: wake-up event on X not detected; 1: wake-up event on X-axis is detected
    pub x_wu: bool,

    // Wake-up event detection status
    // 0: wake-up event not detected; 1: wake-up event is detected
    pub wu_ia: bool,

    // Sleep event status
    // 0: sleep event not detected 1: sleep event detected
    pub sleep_state_ia: bool,

    // Free-fall event detection status
    // 0: free-fall event not detected 1: free-fall event detected
    pub ff_ia: bool,

    // Reserved 0
    pub res0: [Reserved0; 2],
}

/// Tap Source (0x39 R)
/// 0 TAP_IA SINGLE_TAP DOUBLE_TAP TAP_SIGN X_TAP Y_TAP Z_TAP
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct TapSourceReg {
    // Reserved 0
    pub res0: Reserved0,

    // Tap event detection status on Z-axis
    // 0: tap event on Z not detected 1: tap event on Z-axis is detected
    pub z_tap: bool,

    // Tap event detection status on Y-axis
    // 0: tap event on Y not detected 1: tap event on Y-axis is detected
    pub y_tap: bool,

    // Tap event detection status on X-axis
    // 0: tap event on X not detected 1: tap event on X-axis is detected
    pub x_tap: bool,

    // Sign of acceleration detected by tap event
    // 0: positive sign of acceleration detected 1: negative sign of acceleration detected
    pub tap_sign: bool,

    // Double-tap event status
    // 0: double-tap event not detected 1: double-tap event detected
    pub double_tap: bool,

    // Single-tap event status
    // 0: single-tap event not detected 1: single-tap event detected
    pub single_tap: bool,

    // Tap event status
    // 0: tap event not detected; 1: tap event detected
    pub tap_ia: bool,
}

/// 6D Source (0x3A R)
/// 0 6D_IA ZH ZL YH YL XH XL
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct SixDSourceReg {
    // XL over threshold
    // 0: XL does not exceed the threshold 1: XL is over the threshold
    pub xl: bool,

    // XH over threshold
    // 0: XH does not exceed the threshold 1: XH is over the threshold
    pub xh: bool,

    // YL over threshold
    // 0: YL does not exceed the threshold 1: YL is over the threshold
    pub yl: bool,

    // YH over threshold
    // 0: YH does not exceed the threshold 1: YH is over the threshold
    pub yh: bool,

    // ZL over threshold
    // 0: ZL does not exceed the threshold 1: ZL is over the threshold
    pub zl: bool,

    // ZH over threshold
    // 0: ZH does not exceed the threshold 1: ZH is over the threshold
    pub zh: bool,

    // Source of change in position portrait/landscape/face-up/face-down
    // 0: no event detected; 1: a change in position is detected
    pub sixd_ia: bool,

    // Reserved 0
    pub res0: Reserved0,
}

/// All Interrupt Source (0x3B R)
/// Reading this register, all related interrupt function flags routed to the INT pads are reset simultaneously.
/// 0 0 SLEEP_CHANGE_IA 6D_IA DOUBLE_TAP SINGLE_TAP WU_IA FF_IA
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct AllInterruptSourceReg {
    // Free-fall event detection status
    // 0: free-fall event not detected 1: free-fall event detected
    pub ff_ia: bool,

    // Wake-up event detection status
    // 0: wake-up event not detected 1: wake-up event detected
    pub wu_ia: bool,

    // Single-tap event status
    // 0: single-tap event not detected 1: single-tap event detected
    pub single_tap: bool,

    // Double-tap event status
    // 0: double-tap event not detected 1: double-tap event detected
    pub double_tap: bool,

    // Source of change in position portrait/landscape/face-up/face-down
    // 0: no event detected 1: a change in position is detected
    pub sixd_ia: bool,

    // Sleep change status
    // 0: sleep change not detected 1: sleep change detected
    pub sleep_change_ia: bool,

    // Reserved 0
    pub res0: [Reserved0; 2],
}

/// Control Register 7 (0x3F R/W)
/// DRDY_PULSED INT2_ON_INT1 INTERRUPTS_ENABLE USR_OFF_ON_OUT USR_OFF_ON_WU USR_OFF_W HP_REF_MODE LPASS_ON6D
#[bitsize(8)]
#[derive(DebugBits, FromBits, PartialEq, Clone, Copy)]
pub struct ControlReg7 {
    // 0: ODR/2 low-pass filtered data sent to 6D interrupt function (default)
    // 1: LPF2 output data sent to 6D interrupt function
    pub lpass_on6d: bool,

    //  Enables high-pass filter reference mode
    // 0: high-pass filter reference mode disabled (default)
    // 1: high-pass filter reference mode enabled
    pub hp_ref_mode: bool,

    // Selects the weight of the user offset words specified by
    // X_OFS_USR_[7:0], Y_OFS_USR_[7:0], and Z_OFS_USR_[7:0] bits
    // (0: 977 µg/LSB; 1: 15.6 mg/LSB)
    pub usr_off_w: bool,

    // Enables application of user offset value on accelerometer data for wake-up function only
    pub usr_off_on_wu: bool,

    // Enables application of user offset value in accelerometer output data registers
    // FDS bit in CTRL6 (25h) must be set to 0 logic (low-pass path selected)
    pub usr_off_on_out: bool,

    // Enables interrupts
    pub interrupts_enable: bool,

    // Signal routing
    // 1: all signals available only on INT2 are routed to INT1
    pub int2_on_int1: bool,

    // Switches between latched and pulsed mode for data ready interrupt
    // 0: latched mode is used; 1: pulsed mode enabled for data-ready
    pub drdy_pulsed: bool,
}
