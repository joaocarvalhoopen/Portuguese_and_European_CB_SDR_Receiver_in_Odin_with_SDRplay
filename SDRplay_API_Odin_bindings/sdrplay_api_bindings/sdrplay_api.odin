// SDRplay API v3.15 bindings for Odin
// Generated from the installed C headers at /usr/include/sdrplay_api*.h
// Supports RSP1, RSP1A, RSP1B, RSP2, RSPduo, RSPdx, RSPdx-R2

// The license for this binding for the SDRPlay API and only for this bindings
// that I have made is MIT Open Source License, please see the license of the
// API it self from SDRPlay, I beleave that you have to have a original SDRPlay
// device from SDRPlay to use the API, and I have more then one of those
// Original SDRs. And because I program in Odin, I needed to make this Bindings
// for Odin. Because more persons may have also those Original devices and
// it makes no sense to me that simple bindings have to be re maded each time
// I share my code and only my code with MIT License.
// But please see the SDRPlay API license on the site of SDRPlay.

package sdrplay_api_bindings

import "core:fmt"

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

SDRPLAY_API_VERSION           :: f32( 3.15 )
SDRPLAY_MAX_DEVICES           :: 16
SDRPLAY_MAX_TUNERS_PER_DEVICE :: 2
SDRPLAY_MAX_SER_NO_LEN        :: 64
SDRPLAY_MAX_ROOT_NM_LEN       :: 32
MAX_BB_GR                     :: 59

// Device hardware IDs
SDRPLAY_RSP1_ID    :: 1
SDRPLAY_RSP1A_ID   :: 255
SDRPLAY_RSP2_ID    :: 2
SDRPLAY_RSPduo_ID  :: 3
SDRPLAY_RSPdx_ID   :: 4
SDRPLAY_RSP1B_ID   :: 6
SDRPLAY_RSPdxR2_ID :: 7

// ---------------------------------------------------------------------------
// Sample rate constants
//
// The SDRplay API distinguishes between:
//   - BANDWIDTH (Bw_MHzT enum): the analog IF filter bandwidth.
//     The C API defines values up to 8 MHz (BW_8_000).
//   - SAMPLE RATE (fsFreq.fsHz): the ADC/output sample rate.
//     This can be set independently and goes up to ~10.66 MHz on the RSPdx.
//
// For signals wider than 8 MHz (e.g., DVB-T 8 MHz channels), set:
//   bwType = .BW_8_000   (maximum analog bandwidth available)
//   fsHz   = 10_000_000   (10 MHz sample rate → 10 MHz I/Q spectrum)
//
// The 10 MHz I/Q output covers ±5 MHz around the center frequency,
// which fully captures an 8 MHz-wide DVB-T channel with guard band.
// ---------------------------------------------------------------------------

SDRPLAY_FS_2MHZ   :: f64( 2_000_000.0 )   // 2 MHz (default, FM radio)
SDRPLAY_FS_8MHZ   :: f64( 8_000_000.0 )   // 8 MHz
SDRPLAY_FS_10MHZ  :: f64( 10_000_000.0 )  // 10 MHz (DVB-T capture)
SDRPLAY_FS_MAX    :: f64( 10_660_000.0 )  // Maximum sample rate for RSPdx

// LNA state counts
RSPIA_NUM_LNA_STATES               :: 10
RSPIA_NUM_LNA_STATES_AM            :: 7
RSPIA_NUM_LNA_STATES_LBAND         :: 9
RSPII_NUM_LNA_STATES               :: 9
RSPII_NUM_LNA_STATES_AMPORT        :: 5
RSPII_NUM_LNA_STATES_420MHZ        :: 6
RSPDUO_NUM_LNA_STATES              :: 10
RSPDUO_NUM_LNA_STATES_AMPORT       :: 5
RSPDUO_NUM_LNA_STATES_AM           :: 7
RSPDUO_NUM_LNA_STATES_LBAND        :: 9
RSPDX_NUM_LNA_STATES               :: 28
RSPDX_NUM_LNA_STATES_AMPORT2_0_12  :: 19
RSPDX_NUM_LNA_STATES_AMPORT2_12_50 :: 20
RSPDX_NUM_LNA_STATES_AMPORT2_50_60 :: 25
RSPDX_NUM_LNA_STATES_VHF_BAND3     :: 27
RSPDX_NUM_LNA_STATES_420MHZ        :: 21
RSPDX_NUM_LNA_STATES_LBAND         :: 19
RSPDX_NUM_LNA_STATES_DX            :: 22

// ---------------------------------------------------------------------------
// Enum types
// ---------------------------------------------------------------------------

// Error codes
ErrT :: enum i32 {

	Success               = 0,
	Fail                  = 1,
	InvalidParam          = 2,
	OutOfRange            = 3,
	GainUpdateError       = 4,
	RfUpdateError         = 5,
	FsUpdateError         = 6,
	HwError               = 7,
	AliasingError         = 8,
	AlreadyInitialised    = 9,
	NotInitialised        = 10,
	NotEnabled            = 11,
	HwVerError            = 12,
	OutOfMemError         = 13,
	ServiceNotResponding  = 14,
	StartPending          = 15,
	StopPending           = 16,
	InvalidMode           = 17,
	FailedVerification1   = 18,
	FailedVerification2   = 19,
	FailedVerification3   = 20,
	FailedVerification4   = 21,
	FailedVerification5   = 22,
	FailedVerification6   = 23,
	InvalidServiceVersion = 24,
}

// Reason-for-update bit flags
ReasonForUpdateT :: enum u32 {

	None                        = 0x00000000,
	// Master-only mode
	Dev_Fs                      = 0x00000001,
	Dev_Ppm                     = 0x00000002,
	Dev_SyncUpdate              = 0x00000004,
	Dev_ResetFlags              = 0x00000008,
	Rsp1a_BiasTControl          = 0x00000010,
	Rsp1a_RfNotchControl        = 0x00000020,
	Rsp1a_RfDabNotchControl     = 0x00000040,
	Rsp2_BiasTControl           = 0x00000080,
	Rsp2_AmPortSelect           = 0x00000100,
	Rsp2_AntennaControl         = 0x00000200,
	Rsp2_RfNotchControl         = 0x00000400,
	Rsp2_ExtRefControl          = 0x00000800,
	RspDuo_ExtRefControl        = 0x00001000,
	Master_Spare_1              = 0x00002000,
	Master_Spare_2              = 0x00004000,
	// Master and slave mode
	Tuner_Gr                    = 0x00008000,
	Tuner_GrLimits              = 0x00010000,
	Tuner_Frf                   = 0x00020000,
	Tuner_BwType                = 0x00040000,
	Tuner_IfType                = 0x00080000,
	Tuner_DcOffset              = 0x00100000,
	Tuner_LoMode                = 0x00200000,
	Ctrl_DCoffsetIQimbalance    = 0x00400000,
	Ctrl_Decimation             = 0x00800000,
	Ctrl_Agc                    = 0x01000000,
	Ctrl_AdsbMode               = 0x02000000,
	Ctrl_OverloadMsgAck         = 0x04000000,
	RspDuo_BiasTControl         = 0x08000000,
	RspDuo_AmPortSelect         = 0x10000000,
	RspDuo_Tuner1AmNotchControl = 0x20000000,
	RspDuo_RfNotchControl       = 0x40000000,
	RspDuo_RfDabNotchControl    = 0x80000000,
}

// Extended reason-for-update flags
ReasonForUpdateExtension1T :: enum u32 {

	None                      = 0x00000000,
	RspDx_HdrEnable           = 0x00000001,
	RspDx_BiasTControl        = 0x00000002,
	RspDx_AntennaControl      = 0x00000004,
	RspDx_RfNotchControl      = 0x00000008,
	RspDx_RfDabNotchControl   = 0x00000010,
	RspDx_HdrBw               = 0x00000020,
	RspDuo_ResetSlaveFlags    = 0x00000040,
}

// Debug levels
DbgLvl :: enum i32 {

	Disable = 0,
	Verbose = 1,
	Warning = 2,
	Error   = 3,
	Message = 4,
}

// AGC control
AgcControlT :: enum i32 {

	AGC_DISABLE = 0,
	AGC_100HZ   = 1,
	AGC_50HZ    = 2,
	AGC_5HZ     = 3,
	AGC_CTRL_EN = 4,
}

// ADSB modes
AdsbModeT :: enum i32 {

	DECIMATION                  = 0,
	NO_DECIMATION_LOWPASS       = 1,
	NO_DECIMATION_BANDPASS_2MHZ = 2,
	NO_DECIMATION_BANDPASS_3MHZ = 3,
}

// Transfer mode
TransferModeT :: enum i32 {

	ISOCH = 0,
	BULK  = 1,
}

// Bandwidth (kHz)
Bw_MHzT :: enum i32 {

	BW_Undefined = 0,
	BW_0_200     = 200,
	BW_0_300     = 300,
	BW_0_600     = 600,
	BW_1_536     = 1536,
	BW_5_000     = 5000,
	BW_6_000     = 6000,
	BW_7_000     = 7000,
	BW_8_000     = 8000,
}

// IF type (kHz)
If_kHzT :: enum i32 {

	IF_Undefined = -1,
	IF_Zero      = 0,
	IF_0_450     = 450,
	IF_1_620     = 1620,
	IF_2_048     = 2048,
}

// LO mode
LoModeT :: enum i32 {

	LO_Undefined = 0,
	LO_Auto      = 1,
	LO_120MHz    = 2,
	LO_144MHz    = 3,
	LO_168MHz    = 4,
}

// Minimum gain reduction
MinGainReductionT :: enum i32 {

	EXTENDED_MIN_GR = 0,
	NORMAL_MIN_GR   = 20,
}

// Tuner select
TunerSelectT :: enum i32 {

	Tuner_Neither = 0,
	Tuner_A       = 1,
	Tuner_B       = 2,
	Tuner_Both    = 3,
}

// RSPduo mode
RspDuoModeT :: enum i32 {

	Unknown       = 0,
	Single_Tuner  = 1,
	Dual_Tuner    = 2,
	Master        = 4,
	Slave         = 8,
}

// RSP2 antenna select
Rsp2_AntennaSelectT :: enum i32 {

	Rsp2_ANTENNA_A = 5,
	Rsp2_ANTENNA_B = 6,
}

// RSP2 AM port select
Rsp2_AmPortSelectT :: enum i32 {

	Rsp2_AMPORT_1 = 1,
	Rsp2_AMPORT_2 = 0,
}

// RSPduo AM port select
RspDuo_AmPortSelectT :: enum i32 {

	RspDuo_AMPORT_1 = 1,
	RspDuo_AMPORT_2 = 0,
}

// RSPdx antenna select
RspDx_AntennaSelectT :: enum i32 {

	RspDx_ANTENNA_A = 0,
	RspDx_ANTENNA_B = 1,
	RspDx_ANTENNA_C = 2,
}

// RSPdx HDR mode bandwidth
RspDx_HdrModeBwT :: enum i32 {

	HDRMODE_BW_0_200 = 0,
	HDRMODE_BW_0_500 = 1,
	HDRMODE_BW_1_200 = 2,
	HDRMODE_BW_1_700 = 3,
}

// Power overload callback event
PowerOverloadCbEventIdT :: enum i32 {

	Overload_Detected  = 0,
	Overload_Corrected = 1,
}

// RSPduo mode callback event
RspDuoModeCbEventIdT :: enum i32 {

	MasterInitialised    = 0,
	SlaveAttached        = 1,
	SlaveDetached        = 2,
	SlaveInitialised     = 3,
	SlaveUninitialised   = 4,
	MasterDllDisappeared = 5,
	SlaveDllDisappeared  = 6,
}

// Event type
EventT :: enum i32 {

	GainChange          = 0,
	PowerOverloadChange = 1,
	DeviceRemoved       = 2,
	RspDuoModeChange    = 3,
	DeviceFailure       = 4,
}

// ---------------------------------------------------------------------------
// Struct types
// ---------------------------------------------------------------------------

// Gain values (output parameter)
GainValuesT :: struct {

	curr: f32,
	max : f32,
	min : f32,
}

// Gain parameters
GainT :: struct {

	gRdB       : i32,
	LNAstate   : u8,
	syncUpdate : u8,
	minGr      : MinGainReductionT,
	gainVals   :  GainValuesT,
}

// RF frequency
RfFreqT :: struct {

	rfHz       : f64,
	syncUpdate : u8,
}

// DC offset for tuner
DcOffsetTunerT :: struct {

	dcCal           : u8,
	speedUp         : u8,
	trackTime       : i32,
	refreshRateTime : i32,
}

// Tuner parameters
TunerParamsT :: struct {

	bwType        : Bw_MHzT,
	ifType        : If_kHzT,
	loMode        : LoModeT,
	gain          : GainT,
	rfFreq        : RfFreqT,
	dcOffsetTuner : DcOffsetTunerT,
}

// DC offset (control)
DcOffsetT :: struct {

	DCenable : u8,
	IQenable : u8,
}

// Decimation
DecimationT :: struct {

	enable           : u8,
	decimationFactor : u8,
	wideBandSignal   : u8,
}

// AGC parameters
AgcT :: struct {

	enable             : AgcControlT,
	setPoint_dBfs      : i32,
	attack_ms          : u16,
	decay_ms           : u16,
	decay_delay_ms     : u16,
	decay_threshold_dB : u16,
	syncUpdate         : i32,
}

// Control parameters
ControlParamsT :: struct {

	dcOffset   : DcOffsetT,
	decimation : DecimationT,
	agc        : AgcT,
	adsbMode   : AdsbModeT,
}

// RSP1A device params
Rsp1aParamsT :: struct {

	rfNotchEnable    : u8,
	rfDabNotchEnable : u8,
}

// RSP1A tuner params
Rsp1aTunerParamsT :: struct {

	biasTEnable : u8,
}

// RSP2 device params
Rsp2ParamsT :: struct {

	extRefOutputEn : u8,
}

// RSP2 tuner params
Rsp2TunerParamsT :: struct {

	biasTEnable   : u8,
	amPortSel     : Rsp2_AmPortSelectT,
	antennaSel    : Rsp2_AntennaSelectT,
	rfNotchEnable : u8,
}

// RSPduo device params
RspDuoParamsT :: struct {

	extRefOutputEn : i32,
}

// RSPduo reset slave flags
RspDuo_ResetSlaveFlagsT :: struct {

	resetGainUpdate : u8,
	resetRfUpdate   : u8,
}

// RSPduo tuner params
RspDuoTunerParamsT :: struct {

	biasTEnable         : u8,
	tuner1AmPortSel     : RspDuo_AmPortSelectT,
	tuner1AmNotchEnable : u8,
	rfNotchEnable       : u8,
	rfDabNotchEnable    : u8,
	resetSlaveFlags     : RspDuo_ResetSlaveFlagsT,
}

// RSPdx device params
RspDxParamsT :: struct {

	hdrEnable        : u8,
	biasTEnable      : u8,
	antennaSel       : RspDx_AntennaSelectT,
	rfNotchEnable    : u8,
	rfDabNotchEnable : u8,
}

// RSPdx tuner params
RspDxTunerParamsT :: struct {

	hdrBw : RspDx_HdrModeBwT,
}

// Sample rate frequency
FsFreqT :: struct {

	fsHz       : f64,
	syncUpdate : u8,
	reCal      : u8,
}

// Sync update
SyncUpdateT :: struct {

	sampleNum : u32,
	period    : u32,
}

// Reset flags
ResetFlagsT :: struct {

	resetGainUpdate : u8,
	resetRfUpdate   : u8,
	resetFsUpdate   : u8,
}

// Device parameters
DevParamsT :: struct {

	ppm           : f64,
	fsFreq        : FsFreqT,
	syncUpdate    : SyncUpdateT,
	resetFlags    : ResetFlagsT,
	mode          : TransferModeT,
	samplesPerPkt : u32,
	rsp1aParams   : Rsp1aParamsT,
	rsp2Params    : Rsp2ParamsT,
	rspDuoParams  : RspDuoParamsT,
	rspDxParams   : RspDxParamsT,
}

// RX channel parameters
RxChannelParamsT :: struct {

	tunerParams       : TunerParamsT,
	ctrlParams        : ControlParamsT,
	rsp1aTunerParams  : Rsp1aTunerParamsT,
	rsp2TunerParams   : Rsp2TunerParamsT,
	rspDuoTunerParams : RspDuoTunerParamsT,
	rspDxTunerParams  : RspDxTunerParamsT,
}

// Device parameters (top-level)
DeviceParamsT :: struct {

	devParams  : ^DevParamsT,
	rxChannelA : ^RxChannelParamsT,
	rxChannelB : ^RxChannelParamsT,
}

// Device struct
DeviceT :: struct {

	SerNo            : [ SDRPLAY_MAX_SER_NO_LEN ]u8,
	hwVer            : u8,
	tuner            : TunerSelectT,
	rspDuoMode       : RspDuoModeT,
	valid            : u8,
	rspDuoSampleFreq : f64,
	dev              : rawptr,
}

// Extended error info
ErrorInfoT :: struct {

	file     : [ 256 ]u8,
	function : [ 256 ]u8,
	line     : i32,
	message  : [ 1024 ]u8,
}

// Stream callback parameters
StreamCbParamsT :: struct {

	firstSampleNum : u32,
	grChanged      : i32,
	rfChanged      : i32,
	fsChanged      : i32,
	numSamples     : u32,
}

// Gain callback parameters
GainCbParamT :: struct {

	gRdB     : u32,
	lnaGRdB  : u32,
	currGain : f64,
}

// Power overload callback parameters
PowerOverloadCbParamT :: struct {

	powerOverloadChangeType : PowerOverloadCbEventIdT,
}

// RSPduo mode callback parameters
RspDuoModeCbParamT :: struct {

	modeChangeType : RspDuoModeCbEventIdT,
}

// Event parameters (union)
EventParamsT :: struct #raw_union {

	gainParams          : GainCbParamT,
	powerOverloadParams : PowerOverloadCbParamT,
	rspDuoModeParams    : RspDuoModeCbParamT,
}

// Callback function type aliases
StreamCallbackFn :: proc "c" (

	xi         : [^]i16,
	xq         : [^]i16,
	params     : ^StreamCbParamsT,
	numSamples : u32,
	reset      : u32,
	cbContext  : rawptr,
)

EventCallbackFn :: proc "c" (

	eventId   : EventT,
	tuner     : TunerSelectT,
	params    : ^EventParamsT,
	cbContext : rawptr,
)

// Callback functions struct
CallbackFnsT :: struct {

	StreamACbFn : StreamCallbackFn,
	StreamBCbFn : StreamCallbackFn,
	EventCbFn   : EventCallbackFn,
}

// ---------------------------------------------------------------------------
// Foreign library declarations
// ---------------------------------------------------------------------------

foreign import lib "system:libsdrplay_api.so"

foreign lib {

	// Common API functions
	sdrplay_api_Open               :: proc "c" ( ) -> ErrT ---
	sdrplay_api_Close              :: proc "c" ( ) -> ErrT ---
	sdrplay_api_ApiVersion         :: proc "c" ( apiVer : ^f32 ) -> ErrT ---
	sdrplay_api_LockDeviceApi      :: proc "c" ( ) -> ErrT ---
	sdrplay_api_UnlockDeviceApi    :: proc "c" ( ) -> ErrT ---
	sdrplay_api_GetDevices         :: proc "c" ( devices : [^]DeviceT, numDevs : ^u32, maxDevs : u32 ) -> ErrT ---
	sdrplay_api_SelectDevice       :: proc "c" ( device : ^DeviceT ) -> ErrT ---
	sdrplay_api_ReleaseDevice      :: proc "c" ( device : ^DeviceT ) -> ErrT ---
	sdrplay_api_GetErrorString     :: proc "c" ( err : ErrT ) -> cstring ---
	sdrplay_api_GetLastError       :: proc "c" ( device : ^DeviceT ) -> ^ErrorInfoT ---
	sdrplay_api_GetLastErrorByType :: proc "c" ( device : ^DeviceT, type : i32, time : ^u64) -> ^ErrorInfoT ---
	sdrplay_api_DisableHeartbeat   :: proc "c" ( ) -> ErrT ---

	// Device API functions
	sdrplay_api_DebugEnable                       :: proc "c" ( dev : rawptr, enable : DbgLvl ) -> ErrT ---
	sdrplay_api_GetDeviceParams                   :: proc "c" ( dev : rawptr, deviceParams : ^^DeviceParamsT ) -> ErrT ---
	sdrplay_api_Init                              :: proc "c" ( dev : rawptr, callbackFns : ^CallbackFnsT, cbContext : rawptr ) -> ErrT ---
	sdrplay_api_Uninit                            :: proc "c" ( dev : rawptr ) -> ErrT ---
	sdrplay_api_Update                            :: proc "c" ( dev : rawptr, tuner : TunerSelectT, reasonForUpdate : ReasonForUpdateT, reasonForUpdateExt1 : ReasonForUpdateExtension1T ) -> ErrT ---
	sdrplay_api_SwapRspDuoActiveTuner             :: proc "c" ( dev : rawptr, currentTuner : ^TunerSelectT, tuner1AmPortSel : RspDuo_AmPortSelectT ) -> ErrT ---
	sdrplay_api_SwapRspDuoDualTunerModeSampleRate :: proc "c" ( dev : rawptr, currentSampleRate : ^f64, newSampleRate : f64 ) -> ErrT ---
	sdrplay_api_SwapRspDuoMode                    :: proc "c" ( currDevice : ^DeviceT, deviceParams : ^^DeviceParamsT, rspDuoMode : RspDuoModeT, sampleRate : f64, tuner : TunerSelectT, bwType : Bw_MHzT, ifType : If_kHzT, tuner1AmPortSel : RspDuo_AmPortSelectT ) -> ErrT ---
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

// Convert an error code to a human-readable string
error_string :: proc ( err: ErrT ) -> string {

	cstr := sdrplay_api_GetErrorString( err )
	if cstr == nil do return "<null>"
	return string( cstr )
}

// Convert a hardware version byte to a device name string
hw_version_name :: proc ( hwVer: u8 ) -> string {

	switch hwVer {

	case SDRPLAY_RSP1_ID    : return "RSP1"
	case SDRPLAY_RSP1A_ID   : return "RSP1A"
	case SDRPLAY_RSP1B_ID   : return "RSP1B"
	case SDRPLAY_RSP2_ID    : return "RSP2"
	case SDRPLAY_RSPduo_ID  : return "RSPduo"
	case SDRPLAY_RSPdx_ID   : return "RSPdx"
	case SDRPLAY_RSPdxR2_ID : return "RSPdx-R2"
	}
	return "Unknown"
}

// Convert a bandwidth enum to Hz.
// Note: The Bw_MHzT enum maxes out at 8 MHz (BW_8_000) per the C API.
// The sample rate (fsHz) can be set higher independently — up to ~10.66 MHz
// on the RSPdx. For 10 MHz sample rate, use BW_8_000 as the bandwidth.
bw_to_hz :: proc ( bw : Bw_MHzT ) -> f64 {

	#partial switch bw {

	case .BW_0_200 : return 200_000.0
	case .BW_0_300 : return 300_000.0
	case .BW_0_600 : return 600_000.0
	case .BW_1_536 : return 1_536_000.0
	case .BW_5_000 : return 5_000_000.0
	case .BW_6_000 : return 6_000_000.0
	case .BW_7_000 : return 7_000_000.0
	case .BW_8_000 : return 8_000_000.0
	case: return 0.0
	}
}

// Convert a sample rate in Hz to the nearest bandwidth enum.
// For sample rates >= 8 MHz (including 10 MHz), returns BW_8_000
// (the maximum available analog bandwidth). The sample rate itself
// is set separately via fsFreq.fsHz and can go up to SDRPLAY_FS_MAX.
hz_to_bw :: proc( hz : f64 ) -> Bw_MHzT {

	if hz < 300_000.0 do return .BW_0_200
	else if hz < 600_000.0 do return .BW_0_300
	else if hz < 1_536_000.0 do return .BW_0_600
	else if hz < 5_000_000.0 do return .BW_1_536
	else if hz < 6_000_000.0 do return .BW_5_000
	else if hz < 7_000_000.0 do return .BW_6_000
	else if hz < 8_000_000.0 do return .BW_7_000
	return .BW_8_000
}

// Convert a DeviceT serial number to an Odin string (null-terminated)
serial_string :: proc ( dev : ^DeviceT ) -> string {

	len := 0
	for c in dev.SerNo {

		if c == 0 do break
		len += 1
	}
	return string( dev.SerNo[ : len ] )
}

// Print an error message and return false if err != Success
check_err :: proc ( err : ErrT, ctx : string ) -> bool {

	if err != .Success {

		fmt.eprintfln( "  ERROR [%s]: %s", ctx, error_string( err ) )
		return false
	}
	return true
}
