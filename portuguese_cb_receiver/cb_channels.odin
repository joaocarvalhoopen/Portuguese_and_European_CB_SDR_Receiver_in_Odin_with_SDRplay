// Portuguese (ANACOM QNAF) & European Harmonised (CEPT ECC/DEC/(11)03 / ETSI EN 300 433)
// 40-Channel Citizens Band (CB / Banda do Cidadao - 27 MHz / 11m) Definitions & Utilities.
// All frequency and RF level calculations use 64-bit floating point (f64).

package main

import "core:fmt"
import "core:math"
import "core:strings"

CB_MIN_CHANNEL :: 1
CB_MAX_CHANNEL :: 40
CB_DEFAULT_CHANNEL :: 19

// Hardware SDRplay sample rate: 6.0 MSamples/s keeps SDRplay RSP ADCs in maximum 14-bit mode
// (<= 6.048 MSPS hardware ceiling for 14-bit resolution) while dividing cleanly by 125 (25 * 5)
// into 48,000 Hz audio/baseband processing rate.
SDR_SAMPLE_RATE_HZ   :: f64(6_000_000.0)
STAGE1_DECIM_FACTOR  :: 25
STAGE1_RATE_HZ       :: f64(240_000.0)   // 6,000,000 / 25
STAGE2_DECIM_FACTOR  :: 5
BASEBAND_RATE_HZ     :: f64(48_000.0)    // 240,000 / 5
AUDIO_SAMPLE_RATE_HZ :: f64(48_000.0)

// Hardware LO offset (-50 kHz): places the target CB channel at +50 kHz in the complex I/Q
// baseband, completely clear of Zero-IF DC offset spikes, hardware DC calibration notches,
// and mixer 1/f flicker noise, while sitting cleanly inside the 600 kHz analog IF filter.
SDR_LO_OFFSET_HZ :: f64(-50_000.0)

// Clarifier limits (fine frequency offset in Hz, standard extended CB RIT range)
CLARIFIER_MIN_HZ :: f64(-5_000.0)
CLARIFIER_MAX_HZ :: f64(+5_000.0)

// Modulation modes allowed by Portuguese ANACOM & European CEPT CB norm:
//   AM  (A3E) - 4 W RMS max TX
//   FM  (F3E) - 4 W ERP max TX
//   LSB (J3E) - 12 W PEP max TX
//   USB (J3E) - 12 W PEP max TX
Modulation_Mode :: enum i32 {
	AM  = 0,
	FM  = 1,
	LSB = 2,
	USB = 3,
}

// The 3 CB modulation families for band-specific audio noise filtering
CB_Band_Family :: enum i32 {
	AM_Band  = 0,
	FM_Band  = 1,
	SSB_Band = 2,
}

// Audio noise filter complexity levels (0 = Off, 1..4 = Simple to Ultra Deep Voice Recovery)
Filter_Level :: enum i32 {
	Off      = 0,
	Level_1  = 1, // Simple: Cascaded f64 Biquad Voice Bandpass + 50 Hz House Mains Harmonic Notch + Slew Limiter
	Level_2  = 2, // Moderate: Median Impulse Blanker + 64-tap NLMS Adaptive Tone/Noise Canceller + Voice Expander
	Level_3  = 3, // Advanced: 1024-pt f64 Overlap-Add STFT Minimum-Statistics House Noise Tracker + DD Wiener Filter
	Level_4  = 4, // Ultra: 4096-pt f64 STFT + MCRA-2 + Ephraim-Malah MMSE-LSA + SHS Pitch Comb & Formant Regenerator (-20..-30 dB SSB)
}

CB_Channel_Info :: struct {
	channel  : int,
	freq_hz  : f64,
	freq_mhz : f64,
	note_pt  : string,
}

// Official 40-channel CEPT / ANACOM (Portugal) CB frequency table.
// Includes standard R/C channel skips (26.995, 27.045, 27.095, 27.145, 27.195 MHz)
// and the historical CEPT/FCC permutation on Channels 22, 23, 24, 25:
//   Ch 22 = 27.225 MHz, Ch 23 = 27.255 MHz, Ch 24 = 27.235 MHz, Ch 25 = 27.245 MHz.
CB_CHANNELS : [40]CB_Channel_Info = {
	{  1, 26_965_000.0, 26.965, "Chamada Geral FM/AM" },
	{  2, 26_975_000.0, 26.975, "Uso Geral" },
	{  3, 26_985_000.0, 26.985, "Uso Geral" },
	{  4, 27_005_000.0, 27.005, "Uso Geral (salto R/C 26.995)" },
	{  5, 27_015_000.0, 27.015, "Uso Geral" },
	{  6, 27_025_000.0, 27.025, "Uso Geral" },
	{  7, 27_035_000.0, 27.035, "Uso Geral" },
	{  8, 27_055_000.0, 27.055, "Uso Geral (salto R/C 27.045)" },
	{  9, 27_065_000.0, 27.065, "EMERGENCIA / Seguranca Rodoviaria" },
	{ 10, 27_075_000.0, 27.075, "Uso Geral" },
	{ 11, 27_085_000.0, 27.085, "Chamada Geral" },
	{ 12, 27_105_000.0, 27.105, "Uso Geral (salto R/C 27.095)" },
	{ 13, 27_115_000.0, 27.115, "Uso Geral" },
	{ 14, 27_125_000.0, 27.125, "Uso Geral" },
	{ 15, 27_135_000.0, 27.135, "Uso Geral" },
	{ 16, 27_155_000.0, 27.155, "Todo-o-Terreno 4x4 / Maritimo (salto R/C 27.145)" },
	{ 17, 27_165_000.0, 27.165, "Uso Geral" },
	{ 18, 27_175_000.0, 27.175, "Uso Geral" },
	{ 19, 27_185_000.0, 27.185, "Camionistas / Estrada Portugal & Europa" },
	{ 20, 27_205_000.0, 27.205, "Uso Geral (salto R/C 27.195)" },
	{ 21, 27_215_000.0, 27.215, "Uso Geral" },
	{ 22, 27_225_000.0, 27.225, "Uso Geral" },
	{ 23, 27_255_000.0, 27.255, "Uso Geral (ordem historica CEPT +30 kHz)" },
	{ 24, 27_235_000.0, 27.235, "Dados / Packet / Uso Geral" },
	{ 25, 27_245_000.0, 27.245, "Dados / SSTV / Uso Geral" },
	{ 26, 27_265_000.0, 27.265, "Uso Geral" },
	{ 27, 27_275_000.0, 27.275, "Uso Geral" },
	{ 28, 27_285_000.0, 27.285, "Chamada Local" },
	{ 29, 27_295_000.0, 27.295, "Uso Geral" },
	{ 30, 27_305_000.0, 27.305, "DX / Uso Geral" },
	{ 31, 27_315_000.0, 27.315, "Chamada DX FM Europa" },
	{ 32, 27_325_000.0, 27.325, "Uso Geral SSB/FM" },
	{ 33, 27_335_000.0, 27.335, "Uso Geral SSB/FM" },
	{ 34, 27_345_000.0, 27.345, "Chamada Portuguesa SSB (LSB/USB)" },
	{ 35, 27_355_000.0, 27.355, "QSO SSB" },
	{ 36, 27_365_000.0, 27.365, "QSO SSB / Modos Digitais" },
	{ 37, 27_375_000.0, 27.375, "QSO SSB" },
	{ 38, 27_385_000.0, 27.385, "Chamada Internacional DX LSB" },
	{ 39, 27_395_000.0, 27.395, "QSO SSB" },
	{ 40, 27_405_000.0, 27.405, "QSO SSB / Canal Superior CEPT" },
}

is_valid_channel :: proc(ch: int) -> bool {
	return ch >= CB_MIN_CHANNEL && ch <= CB_MAX_CHANNEL
}

get_channel_info :: proc(ch: int) -> CB_Channel_Info {
	clamped := clamp(ch, CB_MIN_CHANNEL, CB_MAX_CHANNEL)
	return CB_CHANNELS[clamped - 1]
}

channel_to_freq_hz :: proc(ch: int) -> f64 {
	return get_channel_info(ch).freq_hz
}

effective_rx_freq_hz :: proc(ch: int, clarifier_hz: f64) -> f64 {
	return channel_to_freq_hz(ch) + clarifier_hz
}

hardware_lo_freq_hz :: proc(ch: int) -> f64 {
	return channel_to_freq_hz(ch) + SDR_LO_OFFSET_HZ
}

mode_to_band_family :: proc(mode: Modulation_Mode) -> CB_Band_Family {
	switch mode {
	case .AM:
		return .AM_Band
	case .FM:
		return .FM_Band
	case .LSB, .USB:
		return .SSB_Band
	}
	return .SSB_Band
}

mode_name :: proc(mode: Modulation_Mode) -> string {
	switch mode {
	case .AM:  return "AM"
	case .FM:  return "FM"
	case .LSB: return "LSB"
	case .USB: return "USB"
	}
	return "AM"
}

mode_norm_desc :: proc(mode: Modulation_Mode) -> string {
	switch mode {
	case .AM:  return "AM (A3E, 8.0 kHz BW, Synchronous PLL, CEPT 4W)"
	case .FM:  return "FM (F3E, 10.0 kHz BW, +-2.0 kHz dev PLL, CEPT 4W)"
	case .LSB: return "SSB-LSB (J3E, 2.68 kHz Analytic FFT, CEPT 12W PEP)"
	case .USB: return "SSB-USB (J3E, 2.68 kHz Analytic FFT, CEPT 12W PEP)"
	}
	return ""
}

parse_modulation_mode :: proc(text: string) -> (Modulation_Mode, bool) {
	t := strings.trim_space(text)
	if strings.equal_fold(t, "am")  do return .AM, true
	if strings.equal_fold(t, "fm")  do return .FM, true
	if strings.equal_fold(t, "lsb") do return .LSB, true
	if strings.equal_fold(t, "usb") do return .USB, true
	if strings.equal_fold(t, "ssb") do return .LSB, true
	return .AM, false
}

filter_level_name :: proc(lvl: Filter_Level, mode: Modulation_Mode) -> string {
	band := mode_name(mode)
	switch lvl {
	case .Off:
		return "0: OFF (Raw Demodulator Output)"
	case .Level_1:
		return fmt.tprintf("1: Simple [%s] (f64 Biquad Voice BP + 50Hz House Mains Comb + De-Clicker)", band)
	case .Level_2:
		return fmt.tprintf("2: Moderate [%s] (De-Clicker + Dual 64-Tap NLMS Adaptive + Voice Expander)", band)
	case .Level_3:
		return fmt.tprintf("3: Advanced [%s] (-15dB OM-LSA + Pitch Comb + Zero-Silence Gate)", band)
	case .Level_4:
		return fmt.tprintf("4: Ultra Voice [%s] (-20..-30dB MMSE-LSA + 3-Scale Comb + Zero-Silence)", band)
	}
	return "Unknown"
}

// Convert calibrated signal power (dBFS + gain offset -> approx dBm, IARU Region 1 HF S9 = -73 dBm, 6 dB/S-unit)
// into a standard CB S-meter string and bar fill [0..16].
format_s_meter :: proc(sig_dbfs: f64, rf_gain_reduction_db: f64, buf: []u8) -> (label: string, bar_units: int) {
	// Approximate antenna-port dBm from dBFS and total SDR gain reduction:
	// At typical GR=40 dB, 0 dBFS corresponds roughly to -20 dBm on RSP HF path, so -73 dBm (S9) ~ -53 dBFS.
	est_dbm := sig_dbfs - (60.0 - rf_gain_reduction_db) - 13.0
	s9_dbm  := -73.0
	diff    := est_dbm - s9_dbm

	if diff >= 0.0 {
		plus_db := int(math.round(diff / 5.0) * 5.0)
		if plus_db > 60 do plus_db = 60
		bar_units = clamp(9 + plus_db / 10, 9, 16)
		if plus_db == 0 {
			label = fmt.bprintf(buf, "S9 (%.1f dBm est)", est_dbm)
		} else {
			label = fmt.bprintf(buf, "S9+%ddB (%.1f dBm est)", plus_db, est_dbm)
		}
		return
	}

	// Below S9: 6 dB per S-unit down to S0 (-127 dBm)
	s_float := 9.0 + diff / 6.0
	s_val   := clamp(int(math.round(s_float)), 0, 9)
	bar_units = clamp(s_val, 0, 9)
	label = fmt.bprintf(buf, "S%d (%.1f dBm est)", s_val, est_dbm)
	return
}
