// 40-Channel × 3-Band (AM, FM, SSB [LSB & USB]) Wideband f64 Auto-Scanner,
// Sub-Hz Parabolic RF Frequency Calibrator, and Multi-Port Antenna Auto-Prober.
// Operates at 6.0 MSamples/s (14-Bit ADC) -> M=5 f64 Polyphase FIR (1.200 MSPS) ->
// 4096-pt Blackman-Harris Custom f64 FFT covering all 40 CB channels simultaneously!

package main

import "core:fmt"
import "core:math"
import "core:os"
import "core:sync"
import "core:time"

import sdr "../SDRplay_API_Odin_bindings/sdrplay_api_bindings"

// Tune LO to 26.880 MHz during wideband scan:
//   - 0 Hz DC sits out-of-band at 26.880 MHz (85 kHz below Ch 1)
//   - Ch 1  (26.965 MHz) -> +85.0 kHz
//   - Ch 19 (27.185 MHz) -> +305.0 kHz
//   - Ch 40 (27.405 MHz) -> +525.0 kHz
// All 40 channels sit cleanly inside BW_1_536 (+-768 kHz) and 1.200 MSPS Nyquist (+-600 kHz)!
SCAN_LO_FREQ_HZ   :: f64(26_880_000.0)
SCAN_DECIM_FACTOR :: 5
SCAN_RATE_HZ      :: SDR_SAMPLE_RATE_HZ / f64(SCAN_DECIM_FACTOR) // 1,200,000.0 Hz
SCAN_FIR_BRANCHES :: 16
SCAN_FIR_TAPS     :: SCAN_DECIM_FACTOR * SCAN_FIR_BRANCHES       // 80 taps
SCAN_FFT_N        :: 4096
SCAN_HOP          :: 2048
SCAN_BIN_HZ       :: SCAN_RATE_HZ / f64(SCAN_FFT_N)              // 292.96875 Hz/bin

Scan_Band_Filter :: enum i32 {
	All_Bands = 0, // AM + FM + SSB (LSB & USB) automatic classification
	AM_Only   = 1,
	FM_Only   = 2,
	SSB_Only  = 3, // LSB + USB
	LSB_Only  = 4,
	USB_Only  = 5,
}

Scan_Engine_State :: enum i32 {
	Idle        = 0,
	Settling_LO = 1,
	Capturing   = 2,
	Locked_Hold = 3,
}

Channel_Scan_Entry :: struct {
	channel          : int,
	nominal_hz       : f64,
	measured_peak_hz : f64,
	freq_offset_hz   : f64,
	best_mode        : Modulation_Mode,
	sig_dbfs         : f64,
	noise_dbfs       : f64,
	snr_db           : f64,
	snr_am_db        : f64,
	snr_fm_db        : f64,
	snr_lsb_db       : f64,
	snr_usb_db       : f64,
	env_mod_idx      : f64,
	carr_constancy   : f64,
	is_modulated     : bool,
	active           : bool,
}

Spectrum_Scan_Report :: struct {
	entries           : [40]Channel_Scan_Entry,
	band_noise_dbfs   : f64, // Noise floor in 2.68 kHz SSB reference BW
	global_peak_hz    : f64,
	global_peak_dbfs  : f64,
	nearest_channel   : int,
	nearest_offset_hz : f64,
	active_count      : int,
	frames_integrated : int,
}

Antenna_Port_Result :: struct {
	port_index      : int,
	port_label      : string,
	band_floor_dbfs : f64,
	peak_sig_dbfs   : f64,
	best_snr_db     : f64,
	best_channel    : int,
	score           : f64,
}

CB_Auto_Scanner :: struct {
	mutex            : sync.Mutex,

	// M=5 f64 Polyphase Lowpass Decimator (6.0 MSPS -> 1.200 MSPS, fc = 555 kHz)
	poly_taps        : [SCAN_DECIM_FACTOR][SCAN_FIR_BRANCHES]f64,
	acc_i            : [SCAN_FIR_BRANCHES]f64,
	acc_q            : [SCAN_FIR_BRANCHES]f64,
	poly_phase       : int,

	// 4096-pt Custom f64 FFT & Blackman-Harris Window
	fft_plan         : FFT_Plan,
	window           : [SCAN_FFT_N]f64,
	win_pow_norm     : f64,
	ring_iq          : [SCAN_FFT_N]complex128,
	work_fft         : [SCAN_FFT_N]complex128,
	hop_fill         : int,
	ring_primed      : bool,

	// Spectral & Envelope Accumulators across capture window
	capture_active   : bool,
	capture_lo_hz    : f64,
	target_frames    : int,
	captured_frames  : int,
	bin_pow_sum      : [SCAN_FFT_N]f64,
	bin_mag_sum      : [SCAN_FFT_N]f64,
	ch_amp_sum       : [40]f64,
	ch_amp_sq_sum    : [40]f64,
	ch_am_fm_corr    : [40]f64,

	// Background Continuous Auto-Scanner State Machine
	enabled          : bool,
	state            : Scan_Engine_State,
	band_filter      : Scan_Band_Filter,
	threshold_db     : f64,
	search_start_ch  : int,
	locked_ch        : int,
	locked_mode      : Modulation_Mode,
	settle_ms_left   : int,
	hang_ms_left     : int,
	quiet_sweeps     : int,
	saved_channel    : int,
	saved_mode       : Modulation_Mode,
	last_report      : Spectrum_Scan_Report,
}

scan_band_name :: proc(bf: Scan_Band_Filter) -> string {
	switch bf {
	case .All_Bands: return "ALL (AM + FM + SSB [LSB/USB])"
	case .AM_Only:   return "AM Only"
	case .FM_Only:   return "FM Only"
	case .SSB_Only:  return "SSB (LSB + USB)"
	case .LSB_Only:  return "SSB-LSB Only"
	case .USB_Only:  return "SSB-USB Only"
	}
	return "ALL"
}

scanner_init :: proc(sc: ^CB_Auto_Scanner) {
	sc^ = {}
	fft_plan_init(&sc.fft_plan, SCAN_FFT_N)

	// Design 80-tap Blackman-Harris lowpass FIR (fc = 555 kHz at 6.0 MHz; Nyquist after M=5 is 600 kHz)
	lin_taps : [SCAN_FIR_TAPS]f64
	design_lowpass_fir_f64(lin_taps[:], 555_000.0, SDR_SAMPLE_RATE_HZ)
	for j in 0 ..< SCAN_DECIM_FACTOR {
		r := (SCAN_DECIM_FACTOR - 1) - j
		for p in 0 ..< SCAN_FIR_BRANCHES {
			sc.poly_taps[j][p] = lin_taps[p * SCAN_DECIM_FACTOR + r]
		}
	}

	win_sq_sum := 0.0
	for n in 0 ..< SCAN_FFT_N {
		w := blackman_harris_f64(n, SCAN_FFT_N)
		sc.window[n] = w
		win_sq_sum  += w * w
	}
	// Exact Parseval power calibration for windowed N-point unscaled FFT
	sc.win_pow_norm    = 1.0 / (f64(SCAN_FFT_N) * win_sq_sum)
	sc.capture_lo_hz   = SCAN_LO_FREQ_HZ
	sc.threshold_db    = 4.5
	sc.search_start_ch = 1
	sc.band_filter     = .All_Bands
}

scanner_begin_capture :: proc(sc: ^CB_Auto_Scanner, frames_to_collect: int = 110) {
	sync.mutex_lock(&sc.mutex)
	sc.acc_i           = {}
	sc.acc_q           = {}
	sc.poly_phase      = 0
	sc.ring_iq         = {}
	sc.hop_fill        = 0
	sc.ring_primed     = false
	sc.bin_pow_sum     = {}
	sc.bin_mag_sum     = {}
	sc.ch_amp_sum      = {}
	sc.ch_amp_sq_sum   = {}
	sc.ch_am_fm_corr   = {}
	sc.captured_frames = 0
	sc.target_frames   = max(frames_to_collect, 16)
	sc.capture_active  = true
	sync.mutex_unlock(&sc.mutex)
}

scanner_is_capture_done :: proc(sc: ^CB_Auto_Scanner) -> (active: bool, done: bool, frames: int) {
	sync.mutex_lock(&sc.mutex)
	active = sc.capture_active
	frames = sc.captured_frames
	done   = sc.captured_frames >= sc.target_frames
	sync.mutex_unlock(&sc.mutex)
	return
}

// Feed raw 6.0 MSamples/s 14-bit I/Q samples directly into the wideband 40-channel f64 analyzer
scanner_process_iq_6m :: proc(sc: ^CB_Auto_Scanner, xi: [^]i16, xq: [^]i16, num_samples: int, lo_hz: f64 = SCAN_LO_FREQ_HZ) {
	sync.mutex_lock(&sc.mutex)
	defer sync.mutex_unlock(&sc.mutex)

	if !sc.capture_active || sc.captured_frames >= sc.target_frames {
		sc.capture_active = false
		return
	}
	sc.capture_lo_hz = lo_hz if lo_hz > 0.0 else SCAN_LO_FREQ_HZ

	SCALE_I16 :: f64(1.0 / 32768.0)
	j := sc.poly_phase

	for n in 0 ..< num_samples {
		ri := f64(xi[n]) * SCALE_I16
		rq := f64(xq[n]) * SCALE_I16

		taps := &sc.poly_taps[j]
		#unroll for p in 0 ..< SCAN_FIR_BRANCHES {
			w := taps[p]
			sc.acc_i[p] += w * ri
			sc.acc_q[p] += w * rq
		}

		j += 1
		if j == SCAN_DECIM_FACTOR {
			j = 0
			z_1m2 := complex(sc.acc_i[0], sc.acc_q[0])
			#unroll for p in 0 ..< (SCAN_FIR_BRANCHES - 1) {
				sc.acc_i[p] = sc.acc_i[p + 1]
				sc.acc_q[p] = sc.acc_q[p + 1]
			}
			sc.acc_i[SCAN_FIR_BRANCHES - 1] = 0.0
			sc.acc_q[SCAN_FIR_BRANCHES - 1] = 0.0

			if !sc.ring_primed {
				sc.ring_iq[sc.hop_fill] = z_1m2
				sc.hop_fill += 1
				if sc.hop_fill == SCAN_FFT_N {
					sc.ring_primed = true
					sc.hop_fill    = 0
					scanner_accumulate_fft_frame(sc)
					if sc.captured_frames >= sc.target_frames {
						sc.capture_active = false
						sc.poly_phase     = j
						return
					}
				}
			} else {
				sc.ring_iq[SCAN_FFT_N - SCAN_HOP + sc.hop_fill] = z_1m2
				sc.hop_fill += 1
				if sc.hop_fill == SCAN_HOP {
					sc.hop_fill = 0
					scanner_accumulate_fft_frame(sc)
					// Shift 50% overlap for next hop
					copy(sc.ring_iq[0 : SCAN_FFT_N - SCAN_HOP], sc.ring_iq[SCAN_HOP : SCAN_FFT_N])
					if sc.captured_frames >= sc.target_frames {
						sc.capture_active = false
						sc.poly_phase     = j
						return
					}
				}
			}
		}
	}

	sc.poly_phase = j
}

@(private)
scanner_accumulate_fft_frame :: proc(sc: ^CB_Auto_Scanner) {
	for i in 0 ..< SCAN_FFT_N {
		sc.work_fft[i] = sc.ring_iq[i] * complex(sc.window[i], 0.0)
	}
	fft_forward(&sc.fft_plan, sc.work_fft[:])

	norm := sc.win_pow_norm
	for k in 0 ..< SCAN_FFT_N {
		p := c64_abs2(sc.work_fft[k]) * norm
		sc.bin_pow_sum[k] += p
		sc.bin_mag_sum[k] += math.sqrt(p)
	}

	eff_lo := sc.capture_lo_hz if sc.capture_lo_hz > 0.0 else SCAN_LO_FREQ_HZ

	// Track per-channel instantaneous RF envelope amplitude across +-4.4 kHz (+-15 bins)
	// so AM/SSB voice modulation is cleanly separated from constant-envelope FM & unmodulated birdies!
	for ch_idx in 0 ..< 40 {
		f_off := CB_CHANNELS[ch_idx].freq_hz - eff_lo
		k_c   := int(math.round(f_off / SCAN_BIN_HZ))
		k_a   := clamp(k_c - 15, -(SCAN_FFT_N / 2 - 2), SCAN_FFT_N / 2 - 2)
		k_b   := clamp(k_c + 15, -(SCAN_FFT_N / 2 - 2), SCAN_FFT_N / 2 - 2)
		p_ch  := 0.0
		for ks in k_a ..= k_b {
			if ks >= -1 && ks <= 1 do continue
			ku := (ks + SCAN_FFT_N) & (SCAN_FFT_N - 1)
			p_ch += c64_abs2(sc.work_fft[ku]) * norm
		}
		amp := math.sqrt(math.max(p_ch, 1e-24))
		sc.ch_amp_sum[ch_idx]    += amp
		sc.ch_amp_sq_sum[ch_idx] += amp * amp

		// Exact AM (+1) vs FM (-1) conjugate-sideband phase discriminator around carrier peak bin:
		// Because Blackman-Harris W_0(u) > 0 across (-2, +2) bins, Re{X[k_pk+m] * X[k_pk-m] * conj(X[k_pk])^2}
		// is strictly > 0 for in-phase AM sidebands and < 0 for quadrature FM Bessel J_{-1} = -J_1 sidebands!
		k_pk  := k_c
		ku_pk := (k_pk + SCAN_FFT_N) & (SCAN_FFT_N - 1)
		ku_p1 := (k_c + 1 + SCAN_FFT_N) & (SCAN_FFT_N - 1)
		ku_m1 := (k_c - 1 + SCAN_FFT_N) & (SCAN_FFT_N - 1)
		if c64_abs2(sc.work_fft[ku_p1]) > c64_abs2(sc.work_fft[ku_pk]) {
			k_pk  = k_c + 1
			ku_pk = ku_p1
		}
		if c64_abs2(sc.work_fft[ku_m1]) > c64_abs2(sc.work_fft[ku_pk]) {
			k_pk  = k_c - 1
			ku_pk = ku_m1
		}
		c0      := sc.work_fft[ku_pk]
		c0_norm := c0 / complex(math.max(c64_abs(c0), 1e-15), 0.0)
		c0_sq_c := conj(c0_norm * c0_norm)
		corr    := 0.0
		for m in 2 ..= 9 {
			up_k := (k_pk + m + SCAN_FFT_N) & (SCAN_FFT_N - 1)
			dn_k := (k_pk - m + SCAN_FFT_N) & (SCAN_FFT_N - 1)
			prod := sc.work_fft[up_k] * sc.work_fft[dn_k] * c0_sq_c
			corr += real(prod)
		}
		sc.ch_am_fm_corr[ch_idx] += corr * norm
	}

	sc.captured_frames += 1
}

// 3-point Gaussian log-magnitude parabolic peak interpolation on signed FFT bin k_signed (achieves < 2 Hz precision)
@(private)
interpolate_peak_freq_hz :: proc(avg_pow: []f64, k_signed: int, lo_hz: f64) -> f64 {
	if k_signed <= -(SCAN_FFT_N / 2 - 2) || k_signed >= (SCAN_FFT_N / 2 - 2) {
		return lo_hz + f64(k_signed) * SCAN_BIN_HZ
	}
	km := (k_signed - 1 + SCAN_FFT_N) & (SCAN_FFT_N - 1)
	k0 := (k_signed     + SCAN_FFT_N) & (SCAN_FFT_N - 1)
	kp := (k_signed + 1 + SCAN_FFT_N) & (SCAN_FFT_N - 1)
	l_m   := 0.5 * math.ln(math.max(avg_pow[km], 1e-30))
	l_0   := 0.5 * math.ln(math.max(avg_pow[k0], 1e-30))
	l_p   := 0.5 * math.ln(math.max(avg_pow[kp], 1e-30))
	denom := l_m - 2.0 * l_0 + l_p
	delta := 0.0
	if denom < -1e-12 {
		delta = clamp(0.5 * (l_m - l_p) / denom, -0.5, +0.5)
	}
	return lo_hz + (f64(k_signed) + delta) * SCAN_BIN_HZ
}

// Default Portuguese / European CB mode tie-breaker when carrier is unmodulated
@(private)
preferred_cb_channel_mode :: proc(ch: int) -> Modulation_Mode {
	if ch == 34 do return .USB
	if ch >= 35 && ch <= 40 do return .LSB
	if ch == 1 || ch == 31 do return .FM
	return .AM
}

// Analyze accumulated 4096-pt f64 spectrum into a complete 40-channel 3-band classification & frequency report
scanner_build_report :: proc(
	sc          : ^CB_Auto_Scanner,
	band_filter : Scan_Band_Filter,
	thresh_db   : f64,
) -> Spectrum_Scan_Report {
	sync.mutex_lock(&sc.mutex)
	sc.capture_active = false
	n_frames := max(sc.captured_frames, 1)
	inv_f    := 1.0 / f64(n_frames)

	avg_pow : [SCAN_FFT_N]f64
	avg_mag : [SCAN_FFT_N]f64
	for k in 0 ..< SCAN_FFT_N {
		avg_pow[k] = sc.bin_pow_sum[k] * inv_f
		avg_mag[k] = sc.bin_mag_sum[k] * inv_f
	}
	amp_sum    := sc.ch_amp_sum
	amp_sq_sum := sc.ch_amp_sq_sum
	am_fm_corr := sc.ch_am_fm_corr
	eff_lo     := sc.capture_lo_hz if sc.capture_lo_hz > 0.0 else SCAN_LO_FREQ_HZ
	sync.mutex_unlock(&sc.mutex)

	rep : Spectrum_Scan_Report
	rep.frames_integrated = n_frames

	// 1. Estimate unbiased CB-band noise power per bin across 26.950 .. 27.420 MHz
	// using 25th-percentile sub-band order statistics (supports both 26.880 MHz scan LO and any IQ file LO!)
	k_band_lo := clamp(int(math.floor((26_950_000.0 - eff_lo) / SCAN_BIN_HZ)), -(SCAN_FFT_N / 2 - 4), SCAN_FFT_N / 2 - 28)
	k_band_hi := clamp(int(math.ceil((27_420_000.0  - eff_lo) / SCAN_BIN_HZ)), k_band_lo + 24,       SCAN_FFT_N / 2 - 4)
	sb_arr : [96]f64
	sb_cnt := 0
	for k := k_band_lo; k + 24 <= k_band_hi && sb_cnt < len(sb_arr); k += 24 {
		s     := 0.0
		valid := 0
		for b in 0 ..< 24 {
			ks := k + b
			if ks >= -2 && ks <= 2 do continue
			ku := (ks + SCAN_FFT_N) & (SCAN_FFT_N - 1)
			s     += avg_pow[ku]
			valid += 1
		}
		if valid >= 16 {
			sb_arr[sb_cnt] = s / f64(valid)
			sb_cnt += 1
		}
	}
	for i in 1 ..< sb_cnt {
		key := sb_arr[i]
		j   := i - 1
		for j >= 0 && sb_arr[j] > key {
			sb_arr[j + 1] = sb_arr[j]
			j -= 1
		}
		sb_arr[j + 1] = key
	}
	// Note: Blackman-Harris ENBW = 2.004 bins, so summing raw bin powers across B bins integrates B bins of noise;
	// noise_bin_pow is the exact average noise contribution per FFT bin in any B-bin sum!
	noise_bin_pow := 1e-18
	if sb_cnt > 0 {
		noise_bin_pow = math.max(sb_arr[(sb_cnt * 25) / 100] * 1.06, 1e-18)
	}
	ssb_bins_equiv      := 2680.0 / SCAN_BIN_HZ
	rep.band_noise_dbfs  = 10.0 * math.log10(noise_bin_pow * ssb_bins_equiv)

	// 2. Find global peak RF frequency across 26.950 .. 27.420 MHz for exact crystal/frequency verification
	best_glob_k   := k_band_lo
	best_glob_pow := 0.0
	for ks in k_band_lo ..= k_band_hi {
		if ks >= -2 && ks <= 2 do continue
		ku := (ks + SCAN_FFT_N) & (SCAN_FFT_N - 1)
		if avg_pow[ku] > best_glob_pow {
			best_glob_pow = avg_pow[ku]
			best_glob_k   = ks
		}
	}
	rep.global_peak_hz   = interpolate_peak_freq_hz(avg_pow[:], best_glob_k, eff_lo)
	rep.global_peak_dbfs = 10.0 * math.log10(math.max(best_glob_pow * 2.004, 1e-18))

	best_ch_dist := 1.0e12
	for idx in 0 ..< 40 {
		d := math.abs(rep.global_peak_hz - CB_CHANNELS[idx].freq_hz)
		if d < best_ch_dist {
			best_ch_dist          = d
			rep.nearest_channel   = CB_CHANNELS[idx].channel
			rep.nearest_offset_hz = rep.global_peak_hz - CB_CHANNELS[idx].freq_hz
		}
	}

	// 3. Classify each of the 40 CB channels across AM, FM, LSB, and USB
	for idx in 0 ..< 40 {
		ch_info := CB_CHANNELS[idx]
		f_off   := ch_info.freq_hz - eff_lo
		k_mid   := int(math.round(f_off / SCAN_BIN_HZ))

		p_carr     := 0.0; n_carr     := 0
		p_carr_500 := 0.0; n_carr_500 := 0
		p_sb_500   := 0.0; n_sb_500   := 0
		p_lsb      := 0.0; n_lsb      := 0
		p_usb      := 0.0; n_usb      := 0
		p_outer    := 0.0; n_outer    := 0

		pk_k   := k_mid
		pk_pow := -1.0

		for ks in max(k_mid - 18, -(SCAN_FFT_N / 2 - 2)) ..= min(k_mid + 18, SCAN_FFT_N / 2 - 2) {
			if ks >= -1 && ks <= 1 do continue
			ku := (ks + SCAN_FFT_N) & (SCAN_FFT_N - 1)
			df := f64(ks) * SCAN_BIN_HZ - f_off
			pw := avg_pow[ku]

			if math.abs(df) <= 4500.0 && pw > pk_pow {
				pk_pow = pw
				pk_k   = ks
			}

			if math.abs(df) <= 480.0 {
				p_carr_500 += pw
				n_carr_500 += 1
			} else if math.abs(df) <= 4800.0 {
				p_sb_500 += pw
				n_sb_500 += 1
			}

			// Partition channel into Carrier Core [-240..+240 Hz], Full LSB [-2950..-180 Hz],
			// Full USB [+180..+2950 Hz], and Outer FM Bessel skirt [2950..4800 Hz]
			if math.abs(df) <= 240.0 {
				p_carr += pw
				n_carr += 1
			}
			if df >= -2950.0 && df <= -180.0 {
				p_lsb += pw
				n_lsb += 1
			} else if df >= +180.0 && df <= +2950.0 {
				p_usb += pw
				n_usb += 1
			} else if math.abs(df) > 2950.0 && math.abs(df) <= 4800.0 {
				p_outer += pw
				n_outer += 1
			}
		}

		meas_hz := interpolate_peak_freq_hz(avg_pow[:], pk_k, eff_lo)
		pk_ku   := (pk_k + SCAN_FFT_N) & (SCAN_FFT_N - 1)

		// Compute passband SNRs for each of the 4 modes
		p_am_tot  := p_carr + p_lsb + p_usb + 0.5 * p_outer
		n_am_tot  := noise_bin_pow * (f64(n_carr + n_lsb + n_usb) + 0.5 * f64(n_outer))
		snr_am    := 10.0 * math.log10(math.max(p_am_tot, 1e-20) / math.max(n_am_tot, 1e-20))

		p_fm_tot  := p_carr + p_lsb + p_usb + p_outer
		n_fm_tot  := noise_bin_pow * f64(max(n_carr + n_lsb + n_usb + n_outer, 1))
		snr_fm    := 10.0 * math.log10(math.max(p_fm_tot, 1e-20) / math.max(n_fm_tot, 1e-20))

		n_lsb_flr := noise_bin_pow * f64(max(n_lsb, 1))
		snr_lsb   := 10.0 * math.log10(math.max(p_lsb, 1e-20) / math.max(n_lsb_flr, 1e-20))

		n_usb_flr := noise_bin_pow * f64(max(n_usb, 1))
		snr_usb   := 10.0 * math.log10(math.max(p_usb, 1e-20) / math.max(n_usb_flr, 1e-20))

		// Envelope modulation index std(A)/mean(A) across FFT hops
		mean_a := amp_sum[idx] * inv_f
		var_a  := math.max(amp_sq_sum[idx] * inv_f - mean_a * mean_a, 0.0)
		env_mi := math.sqrt(var_a) / math.max(mean_a, 1e-15)

		// Peak bin phase/amplitude constancy: ~1.0 for pure unmodulated CW/carrier, <0.92 for voice bins
		constancy := (avg_mag[pk_ku] * avg_mag[pk_ku]) / math.max(avg_pow[pk_ku], 1e-24)

		// Excess carrier vs sideband energies above noise floor
		exc_carr     := math.max(p_carr     - noise_bin_pow * f64(max(n_carr, 1)),     0.0)
		exc_carr_500 := math.max(p_carr_500 - noise_bin_pow * f64(max(n_carr_500, 1)), 0.0)
		exc_sb_500   := math.max(p_sb_500   - noise_bin_pow * f64(max(n_sb_500, 1)),   0.0)
		exc_lsb      := math.max(p_lsb      - n_lsb_flr, 0.0)
		exc_usb      := math.max(p_usb      - n_usb_flr, 0.0)

		// Determine optimal modulation mode according to requested Scan_Band_Filter
		chosen_mode := Modulation_Mode.AM
		switch band_filter {
		case .AM_Only:
			chosen_mode = .AM
		case .FM_Only:
			chosen_mode = .FM
		case .LSB_Only:
			chosen_mode = .LSB
		case .USB_Only:
			chosen_mode = .USB
		case .SSB_Only:
			if snr_usb > snr_lsb + 1.0 {
				chosen_mode = .USB
			} else if snr_lsb > snr_usb + 1.0 {
				chosen_mode = .LSB
			} else {
				chosen_mode = .USB if ch_info.channel == 34 else .LSB
			}
		case .All_Bands:
			// Physical 3-Band (AM / FM / SSB [LSB, USB]) Classifier:
			// 1. Suppressed-carrier SSB has strong sideband asymmetry (> 3.4 dB) between [-2950..-180] and [+180..+2950]
			if snr_usb >= (thresh_db - 1.0) && p_usb > 2.2 * p_lsb && exc_carr < 1.2 * exc_usb {
				chosen_mode = .USB
			} else if snr_lsb >= (thresh_db - 1.0) && p_lsb > 2.2 * p_usb && exc_carr < 1.2 * exc_lsb {
				chosen_mode = .LSB
			} else if max(snr_am, snr_fm) >= thresh_db && max(snr_lsb, snr_usb) >= 4.5 {
				// Symmetric sidebands are actively modulated:
				// Conjugate-sideband phase correlation > 0 -> AM (in-phase sidebands);
				// Conjugate-sideband phase correlation < 0 -> FM (quadrature Bessel J_{-1} = -J_1 sidebands)!
				_ = exc_carr_500
				_ = exc_sb_500
				if am_fm_corr[idx] < 0.0 {
					chosen_mode = .FM
				} else {
					chosen_mode = .AM
				}
			} else {
				// Unmodulated carrier or momentary pause between words -> European/Portuguese CB bandplan
				chosen_mode = preferred_cb_channel_mode(ch_info.channel)
			}
		}

		best_snr  := snr_am
		best_pow  := p_am_tot
		best_nflr := n_am_tot
		switch chosen_mode {
		case .AM:
			best_snr  = snr_am
			best_pow  = p_am_tot
			best_nflr = n_am_tot
		case .FM:
			best_snr  = snr_fm
			best_pow  = p_fm_tot
			best_nflr = n_fm_tot
		case .LSB:
			best_snr  = snr_lsb
			best_pow  = p_lsb
			best_nflr = n_lsb_flr
		case .USB:
			best_snr  = snr_usb
			best_pow  = p_usb
			best_nflr = n_usb_flr
		}

		// Distinguish genuine modulated voice/FM/AM signals from narrow unmodulated crystal spurs:
		// Either sidebands have excess energy (> 3 dB over noise) OR envelope modulates!
		sb_snr_max   := max(snr_lsb, snr_usb)
		is_modulated := sb_snr_max >= 3.0 || env_mi >= 0.08

		// In SSB-only scan, reject pure centered unmodulated dead birdies that have zero sideband energy
		is_active := best_snr >= thresh_db
		if (band_filter == .SSB_Only || band_filter == .LSB_Only || band_filter == .USB_Only) && sb_snr_max < thresh_db {
			is_active = false
		}

		entry := Channel_Scan_Entry{
			channel          = ch_info.channel,
			nominal_hz       = ch_info.freq_hz,
			measured_peak_hz = meas_hz,
			freq_offset_hz   = meas_hz - ch_info.freq_hz,
			best_mode        = chosen_mode,
			sig_dbfs         = 10.0 * math.log10(math.max(best_pow, 1e-18)),
			noise_dbfs       = 10.0 * math.log10(math.max(best_nflr, 1e-18)),
			snr_db           = best_snr,
			snr_am_db        = snr_am,
			snr_fm_db        = snr_fm,
			snr_lsb_db       = snr_lsb,
			snr_usb_db       = snr_usb,
			env_mod_idx      = env_mi,
			carr_constancy   = constancy,
			is_modulated     = is_modulated,
			active           = is_active,
		}
		rep.entries[idx] = entry
		if is_active {
			rep.active_count += 1
		}
	}

	return rep
}

// Run a synchronous wideband 40-channel sweep on live hardware, simulator, or recorded IQ file (~240 ms)
scanner_run_oneshot_sweep :: proc(
	rx          : ^Receiver_Context,
	eng         : ^SDR_Engine,
	band_filter : Scan_Band_Filter = .All_Bands,
	frames      : int = 110,
) -> Spectrum_Scan_Report {
	in_file := iq_is_file_mode(&rx.iq_eng)
	sync.mutex_lock(&rx.cfg_mutex)
	if !in_file {
		rx.cfg.lo_override_hz = SCAN_LO_FREQ_HZ
		rx.cfg.hw_dirty_rf    = true
	}
	rx.cfg.mute_for_scan = true
	thresh               := rx.scanner.threshold_db
	sync.mutex_unlock(&rx.cfg_mutex)

	if !in_file {
		sdr_engine_apply_pending_updates(eng)
		time.sleep(55 * time.Millisecond)
	}

	scanner_begin_capture(&rx.scanner, frames)

	// If paused on a loaded IQ file, read blocks directly from disk at current file position!
	if in_file && !iq_is_playing(&rx.iq_eng) {
		sync.mutex_lock(&rx.iq_eng.ctrl_mutex)
		if rx.iq_eng.play_file != nil && rx.iq_eng.play_preview_slot != nil {
			start_b := sync.atomic_load_explicit(&rx.iq_eng.play_cur_block, .Acquire)
			tot_b   := rx.iq_eng.play_info.total_blocks
			for b := start_b; b < tot_b; b += 1 {
				off := i64(IQ_HEADER_BYTES) + b * i64(IQ_BLOCK_DISK_BYTES)
				nr, rerr := os.read_at(rx.iq_eng.play_file, rx.iq_eng.play_preview_slot.raw[:], off)
				if rerr != nil || nr != IQ_BLOCK_DISK_BYTES do break
				meta, mok := iq_slot_unpack_header(rx.iq_eng.play_preview_slot)
				if !mok do continue
				xi_s := iq_slot_xi_slice(rx.iq_eng.play_preview_slot)
				xq_s := iq_slot_xq_slice(rx.iq_eng.play_preview_slot)
				scanner_process_iq_6m(&rx.scanner, &xi_s[0], &xq_s[0], meta.valid_samples, meta.lo_freq_hz)
				_, done_p, _ := scanner_is_capture_done(&rx.scanner)
				if done_p do break
			}
		}
		sync.mutex_unlock(&rx.iq_eng.ctrl_mutex)
	} else {
		waited_ms := 0
		for rx.running && waited_ms < 1200 {
			_, done, _ := scanner_is_capture_done(&rx.scanner)
			if done do break
			time.sleep(15 * time.Millisecond)
			waited_ms += 15
		}
	}

	rep := scanner_build_report(&rx.scanner, band_filter, thresh)

	// Restore normal -50 kHz offset LO for user's channel and unmute audio
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.lo_override_hz = 0.0
	rx.cfg.mute_for_scan  = false
	if !in_file do rx.cfg.hw_dirty_rf = true
	sync.mutex_unlock(&rx.cfg_mutex)
	if !in_file {
		sdr_engine_apply_pending_updates(eng)
	}

	sync.mutex_lock(&rx.scanner.mutex)
	rx.scanner.last_report = rep
	sync.mutex_unlock(&rx.scanner.mutex)

	return rep
}

// Print plain-text 40-channel spectrum & RF frequency calibration verification report
print_spectrum_scan_report :: proc(rx: ^Receiver_Context, rep: ^Spectrum_Scan_Report, band_filter: Scan_Band_Filter) {
	t := receiver_get_telemetry(rx)
	gr_eff := f64(t.rf_gr_db) + f64(t.lna_state) * 3.0

	frame : Screen_Frame
	frame_println(&frame, "================================================================================")
	frame_printfln(&frame, "  40-CHANNEL CB SPECTRUM & RF FREQUENCY CHECK  |  Filter: %s", scan_band_name(band_filter))
	frame_printfln(&frame, "  Wideband 6.0 MSPS (14-Bit) -> 1.200 MSPS -> 4096-pt f64 FFT (%.2f Hz/bin, %d frames)",
		SCAN_BIN_HZ, rep.frames_integrated)
	frame_printfln(&frame, "  CB Band Noise Floor (2.7 kHz BW): %+.1f dBFS  |  RF Gain: gRdB=%d dB, LNA=%d",
		rep.band_noise_dbfs, t.rf_gr_db, t.lna_state)
	frame_printfln(&frame, "  Strongest RF Peak in Band       : %.6f MHz (%.0f Hz, %+.1f dBFS) -> Near Ch %02d (%+.0f Hz)",
		rep.global_peak_hz / 1e6, rep.global_peak_hz, rep.global_peak_dbfs,
		rep.nearest_channel, rep.nearest_offset_hz)
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_println(&frame, "   Ch   Nominal MHz   Measured Peak MHz   Off(Hz)  Mode    Sig dBFS   SNR dB   S-Meter    Status")
	frame_println(&frame, "--------------------------------------------------------------------------------")

	printed := 0
	// Print all active channels first; if fewer than 6 active, also show top strongest channels for diagnosis!
	shown : [40]bool
	for idx in 0 ..< 40 {
		e := rep.entries[idx]
		if e.active {
			print_scan_entry_row(&frame, &e, gr_eff)
			shown[idx] = true
			printed   += 1
		}
	}

	if printed < 8 {
		// Show top remaining channels ranked by SNR so the user always sees live band floor & peak frequencies
		need := 8 - printed
		for _ in 0 ..< need {
			best_i   := -1
			best_snr := -999.0
			for idx in 0 ..< 40 {
				if !shown[idx] && rep.entries[idx].snr_db > best_snr {
					best_snr = rep.entries[idx].snr_db
					best_i   = idx
				}
			}
			if best_i < 0 do break
			shown[best_i] = true
			e := rep.entries[best_i]
			print_scan_entry_row(&frame, &e, gr_eff)
		}
	}

	frame_println(&frame, "--------------------------------------------------------------------------------")
	if rep.active_count == 0 {
		frame_println(&frame, "  [INFO] 0 channels exceeded threshold. Check antenna port ('ant auto') or try 'scan thresh 3' / 'f 4'.")
	} else {
		frame_printfln(&frame, "  [OK] Found %d active CB channel(s) above threshold.", rep.active_count)
	}
	frame_println(&frame, "================================================================================")
	screen_submit_frame(&frame)
}

@(private)
print_scan_entry_row :: proc(frame: ^Screen_Frame, e: ^Channel_Scan_Entry, gr_eff: f64) {
	s_buf : [48]u8
	s_lbl, _ := format_s_meter(e.sig_dbfs, gr_eff, s_buf[:])
	short_s  := s_lbl
	if sp := index_byte(s_lbl, ' '); sp > 0 {
		short_s = s_lbl[:sp]
	}

	status := "Quiet"
	if e.active {
		status = "ACTIVE VOICE/MOD" if e.is_modulated else "ACTIVE CARRIER"
	} else if e.snr_db >= 2.5 {
		status = "Weak trace"
	}

	b1, b2, b3, b4, b5 : [24]u8
	s_nom  := fmt.bprintf(b1[:], "%.6f", e.nominal_hz / 1e6)
	s_meas := fmt.bprintf(b2[:], "%.6f", e.measured_peak_hz / 1e6)
	s_off  := fmt.bprintf(b3[:], "%+.0f", e.freq_offset_hz)
	s_sig  := fmt.bprintf(b4[:], "%+.1f", e.sig_dbfs)
	s_snr  := fmt.bprintf(b5[:], "%+.1f", e.snr_db)

	frame_printfln(
		frame,
		"   %02d   %-10s    %-13s     %7s    %-4s   %7s    %6s   %-8s   %s",
		e.channel,
		s_nom,
		s_meas,
		s_off,
		mode_name(e.best_mode),
		s_sig,
		s_snr,
		short_s,
		status,
	)
}

@(private)
index_byte :: proc(s: string, b: u8) -> int {
	for i in 0 ..< len(s) {
		if s[i] == b do return i
	}
	return -1
}

// Probe all physical antenna ports (A, B, C on RSPdx / RSPdx-R2; A, B on RSP2) to find where the 27 MHz antenna is connected!
scanner_probe_antennas :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, auto_select: bool) {
	num_ports := 1
	hw_ver    := eng.dev.hwVer
	if rx.sim_mode {
		num_ports = 3
	} else if hw_ver == sdr.SDRPLAY_RSPdx_ID || hw_ver == sdr.SDRPLAY_RSPdxR2_ID {
		num_ports = 3
	} else if hw_ver == sdr.SDRPLAY_RSP2_ID {
		num_ports = 2
	}

	if num_ports <= 1 {
		screen_println("  [ANT] Hardware has a single RF antenna input (Port A).")
		return
	}

	port_names := [3]string{"Port A (SMA)", "Port B (SMA)", "Port C (BNC <=200MHz)"}
	results    : [3]Antenna_Port_Result

	sync.mutex_lock(&rx.cfg_mutex)
	orig_port := rx.cfg.antenna_port
	sync.mutex_unlock(&rx.cfg_mutex)

	screen_println("--------------------------------------------------------------------------------")
	screen_println("  PROBING ANTENNA PORTS ON 27 MHz CB BAND (26.965 - 27.405 MHz)...")

	best_idx   := 0
	best_score := -1.0e9

	for p in 0 ..< num_ports {
		sync.mutex_lock(&rx.cfg_mutex)
		rx.cfg.antenna_port = p
		rx.cfg.hw_dirty_ant = true
		sync.mutex_unlock(&rx.cfg_mutex)
		sdr_engine_apply_pending_updates(eng)
		time.sleep(80 * time.Millisecond)

		rep := scanner_run_oneshot_sweep(rx, eng, .All_Bands, 64)
		max_snr  := -99.0
		max_sig  := -140.0
		best_ch  := 19
		for idx in 0 ..< 40 {
			e := rep.entries[idx]
			if e.snr_db > max_snr {
				max_snr = e.snr_db
				best_ch = e.channel
			}
			if e.sig_dbfs > max_sig {
				max_sig = e.sig_dbfs
			}
		}

		// Connected 27 MHz HF antenna exhibits higher atmospheric noise floor AND/OR stronger CB signals
		// than an open/unterminated port!
		score := rep.band_noise_dbfs + 1.8 * math.max(max_snr, 0.0)
		results[p] = Antenna_Port_Result{
			port_index      = p,
			port_label      = port_names[p],
			band_floor_dbfs = rep.band_noise_dbfs,
			peak_sig_dbfs   = max_sig,
			best_snr_db     = max_snr,
			best_channel    = best_ch,
			score           = score,
		}
		if score > best_score {
			best_score = score
			best_idx   = p
		}
	}

	chosen_port := best_idx if auto_select else orig_port
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.antenna_port = chosen_port
	rx.cfg.hw_dirty_ant = true
	sync.mutex_unlock(&rx.cfg_mutex)
	sdr_engine_apply_pending_updates(eng)

	ant_frame : Screen_Frame
	for p in 0 ..< num_ports {
		r   := results[p]
		tag := " <-- SELECTED (Best 27 MHz RF input)" if p == chosen_port else ""
		bf1, bf2, bf3 : [24]u8
		flr_s := fmt.bprintf(bf1[:], "%+.1f", r.band_floor_dbfs)
		pk_s  := fmt.bprintf(bf2[:], "%+.1f", r.peak_sig_dbfs)
		snr_s := fmt.bprintf(bf3[:], "%+.1f", r.best_snr_db)
		frame_printfln(
			&ant_frame,
			"    %-21s : Floor %6s dBFS | Peak Ch %02d: %6s dBFS (SNR %5s dB)%s",
			r.port_label, flr_s, r.best_channel, pk_s, snr_s, tag,
		)
	}
	frame_println(&ant_frame, "--------------------------------------------------------------------------------")
	screen_submit_frame(&ant_frame)
}

// Start continuous 40-channel background auto-scanner
scanner_start_continuous :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, bf: Scan_Band_Filter) {
	in_file := iq_is_file_mode(&rx.iq_eng)
	if in_file && !iq_is_playing(&rx.iq_eng) {
		iq_play_cmd(&rx.iq_eng, "", false)
	}

	sync.mutex_lock(&rx.cfg_mutex)
	cur_ch   := rx.cfg.channel
	cur_mode := rx.cfg.mode
	if !in_file {
		rx.cfg.lo_override_hz = SCAN_LO_FREQ_HZ
		rx.cfg.hw_dirty_rf    = true
	} else {
		rx.cfg.lo_override_hz = 0.0
	}
	rx.cfg.mute_for_scan = true
	sync.mutex_unlock(&rx.cfg_mutex)

	sync.mutex_lock(&rx.scanner.mutex)
	rx.scanner.enabled         = true
	rx.scanner.band_filter     = bf
	rx.scanner.state           = .Settling_LO
	rx.scanner.settle_ms_left  = 10 if in_file else 60
	rx.scanner.search_start_ch = (cur_ch % CB_MAX_CHANNEL) + 1
	rx.scanner.saved_channel   = cur_ch
	rx.scanner.saved_mode      = cur_mode
	rx.scanner.quiet_sweeps    = 0
	thresh := rx.scanner.threshold_db
	sync.mutex_unlock(&rx.scanner.mutex)

	if !in_file {
		sdr_engine_apply_pending_updates(eng)
	}
	f_start : Screen_Frame
	frame_printfln(
		&f_start,
		"  [SCAN] Auto-Scanner STARTED across 40 CB Channels | Bands: %s | Threshold: %+.1f dB",
		scan_band_name(bf), thresh,
	)
	frame_println(&f_start, "         ('scan stop' to lock on current channel, 'scan next' to skip, 'scan thresh <dB>' to adjust)")
	screen_submit_frame(&f_start)
}

// Stop continuous auto-scanner and remain cleanly tuned on current active channel
scanner_stop_continuous :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, announce: bool = true) {
	was_enabled := false
	sync.mutex_lock(&rx.scanner.mutex)
	was_enabled               = rx.scanner.enabled
	rx.scanner.enabled        = false
	rx.scanner.capture_active = false
	rx.scanner.state          = .Idle
	sync.mutex_unlock(&rx.scanner.mutex)

	if was_enabled {
		in_file := iq_is_file_mode(&rx.iq_eng)
		sync.mutex_lock(&rx.cfg_mutex)
		rx.cfg.lo_override_hz = 0.0
		rx.cfg.mute_for_scan  = false
		if !in_file {
			rx.cfg.hw_dirty_rf = true
		}
		ch := rx.cfg.channel
		m  := rx.cfg.mode
		sync.mutex_unlock(&rx.cfg_mutex)
		if !in_file {
			sdr_engine_apply_pending_updates(eng)
		}
		if announce {
			screen_printfln("  [SCAN] Auto-Scanner STOPPED -> Locked on Ch %02d (%.3f MHz) %s",
				ch, channel_to_freq_hz(ch) / 1e6, mode_name(m))
		}
	}
}

// Advance continuous auto-scanner immediately to search the next channel
scanner_skip_to_next :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine) {
	in_file := iq_is_file_mode(&rx.iq_eng)
	sync.mutex_lock(&rx.cfg_mutex)
	cur_ch := rx.cfg.channel
	if !in_file {
		rx.cfg.lo_override_hz = SCAN_LO_FREQ_HZ
		rx.cfg.hw_dirty_rf    = true
	} else {
		rx.cfg.lo_override_hz = 0.0
	}
	rx.cfg.mute_for_scan = true
	sync.mutex_unlock(&rx.cfg_mutex)

	sync.mutex_lock(&rx.scanner.mutex)
	rx.scanner.enabled         = true
	rx.scanner.state           = .Settling_LO
	rx.scanner.settle_ms_left  = 10 if in_file else 60
	rx.scanner.search_start_ch = (cur_ch % CB_MAX_CHANNEL) + 1
	sync.mutex_unlock(&rx.scanner.mutex)

	if !in_file {
		sdr_engine_apply_pending_updates(eng)
	}
	screen_printfln("  [SCAN] Skipping Ch %02d -> Resuming scan from Ch %02d...", cur_ch, (cur_ch % CB_MAX_CHANNEL) + 1)
}

// Periodic 100 ms tick driving the non-blocking 40-channel auto-scanner state machine
scanner_tick_state_machine :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, dt_ms: int) {
	sync.mutex_lock(&rx.scanner.mutex)
	if !rx.scanner.enabled {
		sync.mutex_unlock(&rx.scanner.mutex)
		return
	}
	st     := rx.scanner.state
	bf     := rx.scanner.band_filter
	thresh := rx.scanner.threshold_db
	sync.mutex_unlock(&rx.scanner.mutex)

	switch st {
	case .Idle:
		return

	case .Settling_LO:
		sync.mutex_lock(&rx.scanner.mutex)
		rx.scanner.settle_ms_left -= dt_ms
		ready := rx.scanner.settle_ms_left <= 0
		if ready {
			rx.scanner.state = .Capturing
		}
		sync.mutex_unlock(&rx.scanner.mutex)
		if ready {
			scanner_begin_capture(&rx.scanner, 96) // ~165 ms wideband 4096-pt FFT integration
		}

	case .Capturing:
		_, done, _ := scanner_is_capture_done(&rx.scanner)
		if !done do return

		rep := scanner_build_report(&rx.scanner, bf, thresh)

		sync.mutex_lock(&rx.scanner.mutex)
		rx.scanner.last_report = rep
		start_ch := rx.scanner.search_start_ch
		sync.mutex_unlock(&rx.scanner.mutex)

		// Search circularly from start_ch..40..start_ch-1; prioritize modulated voice channels first!
		hit_idx := -1
		for pass in 0 ..< 2 {
			for step in 0 ..< 40 {
				ch_num := ((start_ch - 1 + step) % 40) + 1
				e      := rep.entries[ch_num - 1]
				if !e.active do continue
				if pass == 0 && !e.is_modulated do continue
				hit_idx = ch_num - 1
				break
			}
			if hit_idx >= 0 do break
		}

		if hit_idx >= 0 {
			hit     := rep.entries[hit_idx]
			in_file := iq_is_file_mode(&rx.iq_eng)
			// Lock onto found channel & modulation and unmute audio immediately!
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.channel        = hit.channel
			rx.cfg.mode           = hit.best_mode
			if rx.cfg.auto_clarifier do rx.cfg.clarifier_hz = 0.0
			rx.cfg.lo_override_hz = 0.0
			rx.cfg.mute_for_scan  = false
			if !in_file {
				rx.cfg.hw_dirty_rf = true
			}
			sync.mutex_unlock(&rx.cfg_mutex)
			if !in_file {
				sdr_engine_apply_pending_updates(eng)
			}

			sync.mutex_lock(&rx.scanner.mutex)
			rx.scanner.state           = .Locked_Hold
			rx.scanner.locked_ch       = hit.channel
			rx.scanner.locked_mode     = hit.best_mode
			rx.scanner.hang_ms_left    = 4000 // 4.0 sec QSO reply hang timer
			rx.scanner.search_start_ch = (hit.channel % 40) + 1
			rx.scanner.quiet_sweeps    = 0
			sync.mutex_unlock(&rx.scanner.mutex)

			screen_printfln(
				"  [SCAN LOCK] Ch %02d (%.6f MHz | Peak %.6f MHz, %+.0f Hz) %-3s | Sig %+.1f dBFS | SNR %+.1f dB (%s)",
				hit.channel, hit.nominal_hz / 1e6, hit.measured_peak_hz / 1e6, hit.freq_offset_hz,
				mode_name(hit.best_mode), hit.sig_dbfs, hit.snr_db,
				"Voice/Modulated" if hit.is_modulated else "Carrier",
			)
		} else {
			// No channel above threshold: report floor periodically and sweep again after brief pause
			sync.mutex_lock(&rx.scanner.mutex)
			rx.scanner.quiet_sweeps += 1
			qs := rx.scanner.quiet_sweeps
			rx.scanner.state          = .Settling_LO
			rx.scanner.settle_ms_left = 120
			sync.mutex_unlock(&rx.scanner.mutex)

			if qs == 1 || (qs % 25) == 0 {
				best_e := rep.entries[0]
				for idx in 1 ..< 40 {
					if rep.entries[idx].snr_db > best_e.snr_db {
						best_e = rep.entries[idx]
					}
				}
				screen_printfln(
					"  [SCANNING...] 40 ch swept (%s) | Band Floor %+.1f dBFS | Strongest: Ch %02d %-3s (SNR %+.1f dB < %+.1f dB)",
					scan_band_name(bf), rep.band_noise_dbfs, best_e.channel, mode_name(best_e.best_mode),
					best_e.snr_db, thresh,
				)
			}
		}

	case .Locked_Hold:
		t := receiver_get_telemetry(rx)
		// Keep holding while SNR stays above (threshold - 2.5 dB) OR Filter 4 detects human voice pitch!
		still_active := t.snr_db >= (thresh - 2.5) || t.voice_conf_pct >= 25.0

		sync.mutex_lock(&rx.scanner.mutex)
		if still_active {
			rx.scanner.hang_ms_left = 4000
			sync.mutex_unlock(&rx.scanner.mutex)
			return
		}
		rx.scanner.hang_ms_left -= dt_ms
		expired := rx.scanner.hang_ms_left <= 0
		if expired {
			rx.scanner.state          = .Settling_LO
			rx.scanner.settle_ms_left = 60
		}
		l_ch := rx.scanner.locked_ch
		sync.mutex_unlock(&rx.scanner.mutex)

		if expired {
			in_file := iq_is_file_mode(&rx.iq_eng)
			sync.mutex_lock(&rx.cfg_mutex)
			if !in_file {
				rx.cfg.lo_override_hz = SCAN_LO_FREQ_HZ
				rx.cfg.hw_dirty_rf    = true
			} else {
				rx.cfg.lo_override_hz = 0.0
			}
			rx.cfg.mute_for_scan = true
			sync.mutex_unlock(&rx.cfg_mutex)
			if !in_file {
				sdr_engine_apply_pending_updates(eng)
			}
			screen_printfln("  [SCAN] Ch %02d transmission ended -> Resuming 40-channel scan...", l_ch)
		}
	}
}

// ---------------------------------------------------------------------------
// Deep Live Channel & 4-Filter Diagnostic Benchmark (e.g. Channel 27 @ 27.275 MHz)
// ---------------------------------------------------------------------------

@(private)
Diag_Peak_Info :: struct {
	bin        : int,
	freq_hz    : f64,
	pwr_dbfs   : f64,
	prom_db    : f64,
	constancy  : f64,
}

@(private)
Diag_Eval_Workspace :: struct {
	chan_f   : Channel_Selectivity_Filter,
	demod    : Demodulator_State,
	fbank    : Audio_Filter_Bank,
	fft_4k   : FFT_Plan,
	win_4k   : [4096]f64,
	fft_buf  : [4096]complex128,
	psd_mean : [4096]f64,
	psd_sq   : [4096]f64,
}

// Capture live 6.0 MSPS -> 48 kHz I/Q on target_ch, analyze RF spectrum/birdies/hum,
// and benchmark all 4 modulations (AM, FM, LSB, USB) x Filter Levels 0..4 on the exact live signal.
run_channel_deep_diagnostic :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, target_ch: int) {
	ch := clamp(target_ch, CB_MIN_CHANNEL, CB_MAX_CHANNEL)
	nom_hz := channel_to_freq_hz(ch)
	info   := get_channel_info(ch)

	scanner_stop_continuous(rx, eng, false)

	f_hdr : Screen_Frame
	frame_println(&f_hdr, "========================================================================================")
	frame_printfln(&f_hdr, "  DEEP HARDWARE CHANNEL & 4-FILTER DIAGNOSTIC: CB Ch %02d (%.6f MHz - %s)",
		ch, nom_hz / 1e6, info.note_pt)
	frame_println(&f_hdr, "========================================================================================")
	frame_println(&f_hdr, "  [1/3] Sweeping 40 CB channels (26.965 - 27.405 MHz) for band context & RF check...")
	screen_submit_frame(&f_hdr)

	rep := scanner_run_oneshot_sweep(rx, eng, .All_Bands, 110)
	print_spectrum_scan_report(rx, &rep, .All_Bands)
	ce := rep.entries[ch - 1]
	screen_printfln(
		"  [Ch %02d Wideband Summary] Best=%-3s | Sig=%+.1f dBFS | Floor=%+.1f dBFS | SNR: AM=%+.1f FM=%+.1f LSB=%+.1f USB=%+.1f dB | Peak=%+.1f Hz",
		ch, mode_name(ce.best_mode), ce.sig_dbfs, ce.noise_dbfs,
		ce.snr_am_db, ce.snr_fm_db, ce.snr_lsb_db, ce.snr_usb_db, ce.freq_offset_hz,
	)

	// 2. Tune directly to target_ch (LO = f_ch - 50 kHz) and let Auto-LNA / Auto-RF-Gain settle
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.channel        = ch
	rx.cfg.lo_override_hz = 0.0
	rx.cfg.mute_for_scan  = false
	rx.cfg.hw_dirty_rf    = true
	sync.mutex_unlock(&rx.cfg_mutex)
	sdr_engine_apply_pending_updates(eng)

	screen_printfln("  [2/3] Settling Auto-LNA & Auto-RF-Gain on Ch %02d (%.6f MHz, LO %.6f MHz)...",
		ch, nom_hz / 1e6, hardware_lo_freq_hz(ch) / 1e6)
	for _ in 0 ..< 4 {
		time.sleep(260 * time.Millisecond)
		sdr_engine_auto_gain_tick(eng)
	}

	// Capture 122,880 complex128 samples at 48 kHz (120 blocks of 1024 = 2.560 seconds)
	CAPTURE_N :: 120 * CHAN_HOP_B // 122,880 samples (2.56 s @ 48 kHz)
	iq_cap := make([]complex128, CAPTURE_N)
	defer delete(iq_cap)

	t_before := receiver_get_telemetry(rx)

	sync.mutex_lock(&rx.diag_mutex)
	rx.diag_iq_buf    = iq_cap
	rx.diag_iq_write  = 0
	rx.diag_capturing = true
	sync.mutex_unlock(&rx.diag_mutex)

	screen_println("        Capturing 2.56 s of live 14-bit 6.0 MSPS -> 48.0 kHz f64 baseband I/Q...")
	wait_ms := 0
	for wait_ms < 5000 {
		sync.mutex_lock(&rx.diag_mutex)
		done := !rx.diag_capturing
		got  := rx.diag_iq_write
		sync.mutex_unlock(&rx.diag_mutex)
		if done || got >= CAPTURE_N do break
		time.sleep(25 * time.Millisecond)
		wait_ms += 25
	}

	sync.mutex_lock(&rx.diag_mutex)
	n_captured        := rx.diag_iq_write
	rx.diag_capturing = false
	rx.diag_iq_buf    = nil
	sync.mutex_unlock(&rx.diag_mutex)

	t_after := receiver_get_telemetry(rx)
	new_blanked := t_after.blanked_impulses - t_before.blanked_impulses

	ant_str := "Ant A (SMA)"
	if t_after.antenna_port == 1 do ant_str = "Ant B (SMA)"
	if t_after.antenna_port == 2 do ant_str = "Ant C (BNC)"

	screen_printfln(
		"        Captured %d samples (%.2f s) | %s | gRdB=%d dB | LNA=%d/%d | ADC=%+.1f dBFS (Pk %.1f%%) | Blanked Impulses=%d",
		n_captured, f64(n_captured) / 48000.0, ant_str,
		t_after.rf_gr_db, t_after.lna_state, t_after.max_lna_state,
		t_after.adc_dbfs, t_after.adc_peak_pct, new_blanked,
	)

	if n_captured < 16_384 {
		screen_println("  [!] Insufficient I/Q samples captured for deep spectral analysis.")
		return
	}

	ws := new(Diag_Eval_Workspace)
	defer free(ws)
	fft_plan_init(&ws.fft_4k, 4096)

	win_sum_sq := 0.0
	for i in 0 ..< 4096 {
		w := blackman_harris_f64(i, 4096)
		ws.win_4k[i] = w
		win_sum_sq  += w * w
	}
	norm_4k := 1.0 / (4096.0 * math.max(win_sum_sq, 1e-18))

	// High-resolution 4096-pt (11.71875 Hz/bin) complex baseband PSD around 27.275 MHz
	n_frames := 0
	for start := 0; start + 4096 <= n_captured; start += 2048 {
		for i in 0 ..< 4096 {
			ws.fft_buf[i] = iq_cap[start + i] * complex(ws.win_4k[i], 0.0)
		}
		fft_forward(&ws.fft_4k, ws.fft_buf[:])
		for b in 0 ..< 4096 {
			z := ws.fft_buf[b]
			p := (real(z) * real(z) + imag(z) * imag(z)) * norm_4k
			ws.psd_mean[b] += p
			ws.psd_sq[b]   += p * p
		}
		n_frames += 1
	}

	inv_f := 1.0 / f64(max(n_frames, 1))
	for b in 0 ..< 4096 {
		ws.psd_mean[b] *= inv_f
		ws.psd_sq[b]   *= inv_f
	}

	bin_hz : f64 = 48000.0 / 4096.0 // 11.71875 Hz/bin
	band_power_hz :: proc(psd: []f64, f_lo, f_hi, bin_hz: f64) -> f64 {
		sum := 0.0
		for b in 0 ..< 4096 {
			sb := b if b < 2048 else b - 4096
			fh := f64(sb) * bin_hz
			if fh >= f_lo && fh <= f_hi {
				sum += psd[b]
			}
		}
		return 10.0 * math.log10(math.max(sum, 1e-20))
	}

	p_carr_db  := band_power_hz(ws.psd_mean[:], -125.0, +125.0, bin_hz)
	p_lsb_db   := band_power_hz(ws.psd_mean[:], -2800.0, -125.0, bin_hz)
	p_usb_db   := band_power_hz(ws.psd_mean[:], +125.0, +2800.0, bin_hz)
	p_am_db    := band_power_hz(ws.psd_mean[:], -4000.0, +4000.0, bin_hz)
	p_fm_db    := band_power_hz(ws.psd_mean[:], -5000.0, +5000.0, bin_hz)

	// Estimate per-bin median-like quiet floor across [-12 kHz..-6.5 kHz] U [+6.5 kHz..+12 kHz]
	guard_sum := 0.0
	guard_cnt := 0
	for b in 0 ..< 4096 {
		sb := b if b < 2048 else b - 4096
		af := math.abs(f64(sb) * bin_hz)
		if af >= 6500.0 && af <= 12000.0 {
			guard_sum += ws.psd_mean[b]
			guard_cnt += 1
		}
	}
	bin_floor_lin := math.max(guard_sum / f64(max(guard_cnt, 1)), 1e-20)
	floor_2k7_db  := 10.0 * math.log10(bin_floor_lin * (2675.0 / bin_hz))

	diag_frame : Screen_Frame
	frame_println(&diag_frame, "----------------------------------------------------------------------------------------")
	frame_printfln(&diag_frame, "  [Ch %02d High-Res 11.7 Hz/bin Baseband RF Analysis around %.6f MHz]", ch, nom_hz / 1e6)
	frame_printfln(&diag_frame, "    Carrier [-125..+125 Hz] : %+.2f dBFS  |  2.7 kHz Thermal/House Floor : %+.2f dBFS", p_carr_db, floor_2k7_db)
	frame_printfln(&diag_frame, "    LSB [-2800..-125 Hz]    : %+.2f dBFS  |  USB [+125..+2800 Hz]        : %+.2f dBFS", p_lsb_db, p_usb_db)
	frame_printfln(&diag_frame, "    AM  [-4000..+4000 Hz]   : %+.2f dBFS  |  FM  [-5000..+5000 Hz]       : %+.2f dBFS", p_am_db, p_fm_db)

	// Find top 6 narrowband peaks/birdies/carriers inside [-5500..+5500 Hz]
	peaks : [6]Diag_Peak_Info
	n_peaks := 0
	for _ in 0 ..< 6 {
		best_b := -1
		best_p := 0.0
		for b in 0 ..< 4096 {
			sb := b if b < 2048 else b - 4096
			fh := f64(sb) * bin_hz
			if fh < -5500.0 || fh > 5500.0 do continue
			p_cur := ws.psd_mean[b]
			p_l   := ws.psd_mean[(b - 1 + 4096) & 4095]
			p_r   := ws.psd_mean[(b + 1) & 4095]
			if p_cur <= p_l || p_cur < p_r do continue
			already := false
			for k in 0 ..< n_peaks {
				diff := math.abs(f64(sb - peaks[k].bin))
				if diff < 3.0 {
					already = true
					break
				}
			}
			if !already && p_cur > best_p {
				best_p = p_cur
				best_b = b
			}
		}
		if best_b < 0 do break
		sb := best_b if best_b < 2048 else best_b - 4096
		p_l := math.ln(math.max(ws.psd_mean[(best_b - 1 + 4096) & 4095], 1e-30))
		p_c := math.ln(math.max(ws.psd_mean[best_b], 1e-30))
		p_r := math.ln(math.max(ws.psd_mean[(best_b + 1) & 4095], 1e-30))
		den := p_l - 2.0 * p_c + p_r
		delta := 0.0
		if math.abs(den) > 1e-12 {
			delta = clamp(0.5 * (p_l - p_r) / den, -0.5, 0.5)
		}
		f_exact := (f64(sb) + delta) * bin_hz
		c_ratio := clamp((best_p * best_p) / math.max(ws.psd_sq[best_b], 1e-30), 0.0, 1.0)
		peaks[n_peaks] = Diag_Peak_Info{
			bin       = sb,
			freq_hz   = f_exact,
			pwr_dbfs  = 10.0 * math.log10(math.max(best_p, 1e-20)),
			prom_db   = 10.0 * math.log10(math.max(best_p / bin_floor_lin, 1e-12)),
			constancy = c_ratio * 100.0,
		}
		n_peaks += 1
	}

	if n_peaks > 0 {
		frame_println(&diag_frame, "    Dominant Narrowband Peaks / Carriers / House Birdies in Ch Passband:")
		for k in 0 ..< n_peaks {
			pk := peaks[k]
			kind := "Transient/Voice"
			if pk.constancy >= 82.0 {
				kind = "Steady Carrier/Birdie"
			} else if pk.constancy >= 60.0 {
				kind = "Modulated/Semi-Steady"
			}
			pb1, pb2, pb3 : [24]u8
			sf_hz  := fmt.bprintf(pb1[:], "%+.2f", pk.freq_hz)
			sp_db  := fmt.bprintf(pb2[:], "%+.1f", pk.pwr_dbfs)
			spr_db := fmt.bprintf(pb3[:], "%+.1f", pk.prom_db)
			frame_printfln(&diag_frame, "      #%d: %9s Hz (%.6f MHz) | %6s dBFS (%6s dB vs floor) | Constancy %.1f%% [%s]",
				k + 1, sf_hz, (nom_hz + pk.freq_hz) / 1e6, sp_db, spr_db, pk.constancy, kind)
		}
	}

	// 3. Offline matrix evaluation: 4 Modes (AM, FM, LSB, USB) x 5 Filter Levels (Off, L1, L2, L3, L4)
	frame_println(&diag_frame, "----------------------------------------------------------------------------------------")
	frame_println(&diag_frame, "  [3/3] Evaluating All 4 Audio Noise Filters on Live Captured Ch I/Q (AM / FM / LSB / USB)")
	frame_println(&diag_frame, "  (Skip first 0.42 s warmup; measure pre-AGC physical noise suppression + post-AGC audio)")
	frame_println(&diag_frame, "----------------------------------------------------------------------------------------")
	frame_println(&diag_frame, "  Mode | Flt | ChSNR  | FltNR | TotCut | Hum<130Hz | SpeechBand | Hiss>2.7k | Sp/Hiss | VConf | F0(Hz) | OutRMS")
	frame_println(&diag_frame, "-------+-----+--------+-------+--------+-----------+------------+-----------+---------+-------+--------+--------")
	screen_submit_frame(&diag_frame)

	modes   := [4]Modulation_Mode{.AM, .FM, .LSB, .USB}
	levels  := [5]Filter_Level{.Off, .Level_1, .Level_2, .Level_3, .Level_4}
	aud_blk : [CHAN_HOP_B]f64
	aud_cap := make([]f64, n_captured)
	defer delete(aud_cap)

	for m in modes {
		raw_rms_ref := 1e-12
		for fl in levels {
			channel_filter_init(&ws.chan_f)
			demod_init(&ws.demod)
			audio_filter_bank_init(&ws.fbank)

			out_len  := 0
			vc_acc   := 0.0
			f0_acc   := 0.0
			nr_acc   := 0.0
			stat_cnt := 0
			post_sq  := 0.0
			post_cnt := 0

			for s_idx in 0 ..< n_captured {
				if channel_filter_push(&ws.chan_f, iq_cap[s_idx], m) {
					demodulate_block_f64(&ws.demod, ws.chan_f.out_ready[:], m, aud_blk[:], ws.chan_f.noise_power_lin)
					vc, f0, nr := audio_filter_bank_process(&ws.fbank, aud_blk[:], fl, m, rx.cfg.rf_notch)

					if out_len + CHAN_HOP_B <= len(aud_cap) {
						copy(aud_cap[out_len : out_len + CHAN_HOP_B], aud_blk[:])
						out_len += CHAN_HOP_B
					}

					// Accumulate steady-state metrics after 20 blocks (~0.42 s) warmup
					if out_len > 20 * CHAN_HOP_B {
						vc_acc   += vc
						f0_acc   += f0
						nr_acc   += nr
						stat_cnt += 1
					}

					post_process_audio_block_f64(
						&ws.demod, aud_blk[:],
						ws.chan_f.sig_dbfs, ws.chan_f.snr_db, vc,
						false, -120.0, 0.90, fl,
					)
					if out_len > 20 * CHAN_HOP_B {
						for v in aud_blk {
							post_sq  += v * v
							post_cnt += 1
						}
					}
				}
			}

			warmup_idx := min(20 * CHAN_HOP_B, out_len / 2)
			eval_slice := aud_cap[warmup_idx : out_len]

			// Compute total pre-AGC RMS & Welch 4096-pt audio spectrum (Hum, Speech, Hiss)
			sq_sum := 0.0
			for v in eval_slice do sq_sum += v * v
			pre_rms := math.sqrt(sq_sum / f64(max(len(eval_slice), 1)))
			if fl == .Off {
				raw_rms_ref = math.max(pre_rms, 1e-12)
			}
			tot_cut_db := 20.0 * math.log10(math.max(raw_rms_ref / math.max(pre_rms, 1e-12), 1e-6))

			for b in 0 ..< 4096 do ws.psd_mean[b] = 0.0
			af_cnt := 0
			for st := 0; st + 4096 <= len(eval_slice); st += 2048 {
				for i in 0 ..< 4096 {
					ws.fft_buf[i] = complex(eval_slice[st + i] * ws.win_4k[i], 0.0)
				}
				fft_forward(&ws.fft_4k, ws.fft_buf[:])
				for b in 0 ..= 2048 {
					z := ws.fft_buf[b]
					w_mult := 2.0 if (b > 0 && b < 2048) else 1.0
					ws.psd_mean[b] += w_mult * (real(z) * real(z) + imag(z) * imag(z)) * norm_4k
				}
				af_cnt += 1
			}
			inv_af := 1.0 / f64(max(af_cnt, 1))
			hum_p    := 0.0
			speech_p := 0.0
			hiss_p   := 0.0
			for b in 1 ..= 2048 {
				p  := ws.psd_mean[b] * inv_af
				fh := f64(b) * bin_hz
				if fh <= 130.0 {
					hum_p += p
				} else if fh >= 165.0 && fh <= 2550.0 {
					speech_p += p
				} else if fh >= 2700.0 && fh <= 8000.0 {
					hiss_p += p
				}
			}
			hum_db    := 10.0 * math.log10(math.max(hum_p, 1e-20))
			speech_db := 10.0 * math.log10(math.max(speech_p, 1e-20))
			hiss_db   := 10.0 * math.log10(math.max(hiss_p, 1e-20))
			sp_hiss   := speech_db - hiss_db
			post_rms_db := 20.0 * math.log10(math.max(math.sqrt(post_sq / f64(max(post_cnt, 1))), 1e-9))

			inv_st := 1.0 / f64(max(stat_cnt, 1))
			tb1, tb2, tb3, tb4, tb5, tb6, tb7, tb8, tb9, tb10 : [20]u8
			s_csnr := fmt.bprintf(tb1[:], "%+.1f", ws.chan_f.snr_db)
			s_fnr  := fmt.bprintf(tb2[:], "%.1f", nr_acc * inv_st)
			s_tcut := fmt.bprintf(tb3[:], "%+.1f", -tot_cut_db)
			s_hum  := fmt.bprintf(tb4[:], "%+.1f", hum_db)
			s_sp   := fmt.bprintf(tb5[:], "%+.1f", speech_db)
			s_hi   := fmt.bprintf(tb6[:], "%+.1f", hiss_db)
			s_sh   := fmt.bprintf(tb7[:], "%+.1f", sp_hiss)
			s_vc   := fmt.bprintf(tb8[:], "%.0f%%", (vc_acc * inv_st) * 100.0)
			s_f0   := fmt.bprintf(tb9[:], "%.1f", f0_acc * inv_st)
			s_out  := fmt.bprintf(tb10[:], "%+.1f", post_rms_db)
			frame_printfln(
				&diag_frame,
				"  %-4s | F%-2d | %6sdB| %4sdB| %6sdB| %7sdB | %8sdB | %7sdB | %6sdB| %5s | %6s | %6sdB",
				mode_name(m), int(fl), s_csnr, s_fnr, s_tcut,
				s_hum, s_sp, s_hi, s_sh, s_vc, s_f0, s_out,
			)
		}
		frame_println(&diag_frame, "-------+-----+--------+-------+--------+-----------+------------+-----------+---------+-------+--------+--------")
		screen_submit_frame(&diag_frame)
	}
	screen_println("========================================================================================")
}

// ===========================================================================
// Real-Time 40-Channel Braille Spectrum & 60-Second Waterfall ('sp' / 'spetrum')
//   - Zero-interruption wideband tap at 6.0 MSamples/s (BW_1_536 covers all 40 CB
//     channels 26.965..27.405 MHz simultaneously while demodulating any channel!)
//   - Upper Part : Always displays all 40 CB channels (2 digits/channel: 01..40) +
//                  live instantaneous 40 dB Braille spectrum (4 sub-bands/channel)
//   - Lower Part : Last 60 seconds Waterfall (1s circular ring, 20x3s peak-hold rows),
//                  2 Braille characters per channel with dB intensity mapped to the
//                  8-dot Unicode Braille matrix (U+2800..U+28FF)
// ===========================================================================

WF_HISTORY_SECONDS :: 60
WF_DISPLAY_ROWS    :: 20 // 20 rows x 3 seconds/row = 60 seconds
WF_SEC_SAMPLES_6M  :: 6_000_000
WF_BURST_PERIOD_6M :: 375_000 // 16 evenly-spaced 4096-pt 1.2 MSPS FFTs per second (~5.5% duty)
WF_WARMUP_DECIM    :: SCAN_FIR_BRANCHES

Waterfall_Sec_Slice :: struct {
	valid           : bool,
	timestamp_ns    : i64,        // Exact UTC nanosecond timestamp (live clock or recorded IQ_file_xxxx.samples!)
	sub_dbfs        : [40][4]f64, // 4x 2.5 kHz sub-bands across [-5..+5 kHz] per channel
	half_dbfs       : [40][2]f64, // Left [-5..0 kHz] & Right [0..+5 kHz] per channel (2 Braille chars/ch)
	chan_dbfs       : [40]f64,
	floor_half_dbfs : f64,        // Self-calibrated 25th-percentile half-channel floor in dBFS
}

Braille_Waterfall_State :: struct {
	mutex          : sync.Mutex,

	// M=5 f64 Polyphase Lowpass Decimator (6.0 MSPS -> 1.200 MSPS, fc = 555 kHz)
	poly_taps      : [SCAN_DECIM_FACTOR][SCAN_FIR_BRANCHES]f64,
	acc_i          : [SCAN_FIR_BRANCHES]f64,
	acc_q          : [SCAN_FIR_BRANCHES]f64,
	poly_phase     : int,

	// 4096-pt Custom f64 FFT & Blackman-Harris Window
	fft_plan       : FFT_Plan,
	window         : [SCAN_FFT_N]f64,
	win_pow_norm   : f64,
	ring_iq        : [SCAN_FFT_N]complex128,
	work_fft       : [SCAN_FFT_N]complex128,

	// Burst & 1-second hardware-clocked scheduler
	burst_active   : bool,
	warmup_left    : int,
	fft_fill       : int,
	burst_lo_hz    : f64,
	since_burst_6m : int,
	sec_samples_6m : int,

	// Active 1-second accumulators (mapped immediately to true RF channels 1..40)
	cur_sub_pow    : [40][4]f64,
	cur_frames     : int,
	cur_ts_ns      : i64,
	live_slice     : Waterfall_Sec_Slice,

	// 60-second circular history buffer (head points to next write slot; newest is (head-1+60)%60)
	history        : [WF_HISTORY_SECONDS]Waterfall_Sec_Slice,
	head           : int,
	history_count  : int,
}

// Encode Unicode Braille codepoint U+2800 + mask (0x00..0xFF) into 3-byte UTF-8
encode_braille_utf8 :: #force_inline proc(dst: []u8, pos: int, mask: u8) -> int {
	cp := u32(0x2800) + u32(mask)
	dst[pos + 0] = u8(0xE0 | ((cp >> 12) & 0x0F))
	dst[pos + 1] = u8(0x80 | ((cp >> 6)  & 0x3F))
	dst[pos + 2] = u8(0x80 | ( cp        & 0x3F))
	return pos + 3
}

// Map dB excess above band noise floor monotonically onto 0..8 lit dots in the 2x4 Braille matrix
braille_db_to_mask :: #force_inline proc(excess_db: f64) -> (mask: u8, n_dots: int) {
	if excess_db <  1.8 do return 0x00, 0 // ⠀ (0/8 dots)
	if excess_db <  4.5 do return 0x40, 1 // ⡀ (1/8 dots)
	if excess_db <  7.5 do return 0xC0, 2 // ⣀ (2/8 dots)
	if excess_db < 11.0 do return 0xC4, 3 // ⣄ (3/8 dots)
	if excess_db < 15.0 do return 0xE4, 4 // ⣤ (4/8 dots)
	if excess_db < 19.5 do return 0xE6, 5 // ⣥ (5/8 dots)
	if excess_db < 24.5 do return 0xF6, 6 // ⣶ (6/8 dots)
	if excess_db < 30.0 do return 0xF7, 7 // ⣷ (7/8 dots)
	return 0xFF, 8                        // ⣿ (8/8 dots)
}

waterfall_init :: proc(wf: ^Braille_Waterfall_State) {
	wf^ = {}
	fft_plan_init(&wf.fft_plan, SCAN_FFT_N)

	lin_taps : [SCAN_FIR_TAPS]f64
	design_lowpass_fir_f64(lin_taps[:], 555_000.0, SDR_SAMPLE_RATE_HZ)
	for j in 0 ..< SCAN_DECIM_FACTOR {
		r := (SCAN_DECIM_FACTOR - 1) - j
		for p in 0 ..< SCAN_FIR_BRANCHES {
			wf.poly_taps[j][p] = lin_taps[p * SCAN_DECIM_FACTOR + r]
		}
	}

	win_sq_sum := 0.0
	for n in 0 ..< SCAN_FFT_N {
		w := blackman_harris_f64(n, SCAN_FFT_N)
		wf.window[n] = w
		win_sq_sum  += w * w
	}
	wf.win_pow_norm   = 1.0 / (f64(SCAN_FFT_N) * win_sq_sum)
	// Trigger first FFT burst immediately on startup
	wf.since_burst_6m = WF_BURST_PERIOD_6M
}

@(private)
waterfall_build_slice :: proc(sub_pow: ^[40][4]f64, frames: int) -> Waterfall_Sec_Slice {
	sl : Waterfall_Sec_Slice
	if frames <= 0 {
		sl.floor_half_dbfs = -110.0
		return sl
	}
	sl.valid = true
	inv_f := 1.0 / f64(frames)

	half_sorted : [80]f64
	hs_n := 0

	for c in 0 ..< 40 {
		s0 := math.max(sub_pow[c][0] * inv_f, 1e-18)
		s1 := math.max(sub_pow[c][1] * inv_f, 1e-18)
		s2 := math.max(sub_pow[c][2] * inv_f, 1e-18)
		s3 := math.max(sub_pow[c][3] * inv_f, 1e-18)

		h0 := s0 + s1
		h1 := s2 + s3

		sl.sub_dbfs[c][0]  = 10.0 * math.log10(s0)
		sl.sub_dbfs[c][1]  = 10.0 * math.log10(s1)
		sl.sub_dbfs[c][2]  = 10.0 * math.log10(s2)
		sl.sub_dbfs[c][3]  = 10.0 * math.log10(s3)
		sl.half_dbfs[c][0] = 10.0 * math.log10(h0)
		sl.half_dbfs[c][1] = 10.0 * math.log10(h1)
		sl.chan_dbfs[c]    = 10.0 * math.log10(h0 + h1)

		half_sorted[hs_n + 0] = sl.half_dbfs[c][0]
		half_sorted[hs_n + 1] = sl.half_dbfs[c][1]
		hs_n += 2
	}

	// Insertion sort 80 half-channel dBFS values -> 25th percentile unbiased band floor
	for i in 1 ..< hs_n {
		key := half_sorted[i]
		j   := i - 1
		for j >= 0 && half_sorted[j] > key {
			half_sorted[j + 1] = half_sorted[j]
			j -= 1
		}
		half_sorted[j + 1] = key
	}
	sl.floor_half_dbfs = half_sorted[20] + 0.35
	return sl
}

@(private)
waterfall_accumulate_fft_frame :: proc(wf: ^Braille_Waterfall_State, lo_hz: f64) {
	for i in 0 ..< SCAN_FFT_N {
		wf.work_fft[i] = wf.ring_iq[i] * complex(wf.window[i], 0.0)
	}
	fft_forward(&wf.fft_plan, wf.work_fft[:])

	norm := wf.win_pow_norm
	for c in 0 ..< 40 {
		f_if := CB_CHANNELS[c].freq_hz - lo_hz
		if math.abs(f_if) > 565_000.0 do continue

		kc := int(math.round(f_if / SCAN_BIN_HZ))
		// 4 symmetric sub-bands across [-5.0 kHz .. +5.0 kHz]; center bin kc splits 50/50 between q=1 & q=2
		ranges := [4][2]int{
			{kc - 17, kc - 9},
			{kc - 8,  kc},
			{kc,      kc + 8},
			{kc + 9,  kc + 17},
		}

		for q in 0 ..< 4 {
			b_lo := ranges[q][0]
			b_hi := ranges[q][1]
			psum  := 0.0
			w_tot := 0.0
			w_val := 0.0
			for b in b_lo ..= b_hi {
				w := 0.5 if b == kc else 1.0
				w_tot += w
				idx := b & (SCAN_FFT_N - 1)
				// Skip Zero-IF hardware DC notch bins (-1, 0, +1) and compensate weight
				if idx == 0 || idx == 1 || idx == (SCAN_FFT_N - 1) {
					continue
				}
				psum  += w * c64_abs2(wf.work_fft[idx]) * norm
				w_val += w
			}
			if w_val > 0.0 {
				wf.cur_sub_pow[c][q] += psum * (w_tot / w_val)
			}
		}
	}

	wf.cur_frames += 1
	sl := waterfall_build_slice(&wf.cur_sub_pow, wf.cur_frames)
	sl.timestamp_ns = wf.cur_ts_ns
	sync.mutex_lock(&wf.mutex)
	wf.live_slice = sl
	sync.mutex_unlock(&wf.mutex)
}

// Feed raw 6.0 MSamples/s 14-bit I/Q stream continuously (works during live RX, wideband scan, and IQ file playback!)
// Lock-minimal: DSP & 4096-pt FFT run with ZERO locks; wf.mutex is held for < 100 ns ONLY on slice commit!
waterfall_process_iq_6m :: proc(
	wf           : ^Braille_Waterfall_State,
	xi           : [^]i16,
	xq           : [^]i16,
	num_samples  : int,
	lo_hz        : f64,
	timestamp_ns : i64 = 0,
) {
	if num_samples <= 0 || lo_hz <= 0.0 do return
	base_ts_ns := timestamp_ns if timestamp_ns > 0 else time.now()._nsec

	// If LO jumped mid-burst (e.g. channel change), restart only the 3.4 ms burst; channel accumulators remain valid!
	if wf.burst_active && math.abs(lo_hz - wf.burst_lo_hz) > 1.0 {
		wf.burst_active   = false
		wf.since_burst_6m = WF_BURST_PERIOD_6M
	}

	SCALE_I16 :: f64(1.0 / 32768.0)
	idx := 0
	for idx < num_samples {
		wf.cur_ts_ns = base_ts_ns + i64(math.round((f64(idx) / SDR_SAMPLE_RATE_HZ) * 1e9))

		if !wf.burst_active {
			rem_to_burst := WF_BURST_PERIOD_6M - wf.since_burst_6m
			rem_to_sec   := WF_SEC_SAMPLES_6M  - wf.sec_samples_6m
			step         := min(num_samples - idx, min(max(rem_to_burst, 0), max(rem_to_sec, 1)))
			if step > 0 {
				idx               += step
				wf.since_burst_6m += step
				wf.sec_samples_6m += step
				wf.cur_ts_ns       = base_ts_ns + i64(math.round((f64(idx) / SDR_SAMPLE_RATE_HZ) * 1e9))
			}
			if wf.sec_samples_6m >= WF_SEC_SAMPLES_6M {
				if wf.cur_frames > 0 {
					sl := waterfall_build_slice(&wf.cur_sub_pow, wf.cur_frames)
					sl.timestamp_ns = wf.cur_ts_ns
					sync.mutex_lock(&wf.mutex)
					wf.history[wf.head] = sl
					wf.head             = (wf.head + 1) % WF_HISTORY_SECONDS
					if wf.history_count < WF_HISTORY_SECONDS {
						wf.history_count += 1
					}
					wf.live_slice = sl
					sync.mutex_unlock(&wf.mutex)
				}
				wf.cur_sub_pow    = {}
				wf.cur_frames     = 0
				wf.sec_samples_6m = 0
			}
			if wf.since_burst_6m >= WF_BURST_PERIOD_6M && idx < num_samples {
				wf.burst_active   = true
				wf.since_burst_6m = 0
				wf.acc_i          = {}
				wf.acc_q          = {}
				wf.poly_phase     = 0
				wf.warmup_left    = WF_WARMUP_DECIM
				wf.fft_fill       = 0
				wf.burst_lo_hz    = lo_hz
			}
			continue
		}

		// Active 3.43 ms M=5 polyphase decimation burst
		ri := f64(xi[idx]) * SCALE_I16
		rq := f64(xq[idx]) * SCALE_I16
		idx               += 1
		wf.since_burst_6m += 1
		wf.sec_samples_6m += 1

		j    := wf.poly_phase
		taps := &wf.poly_taps[j]
		#unroll for p in 0 ..< SCAN_FIR_BRANCHES {
			w := taps[p]
			wf.acc_i[p] += w * ri
			wf.acc_q[p] += w * rq
		}
		j += 1
		if j == SCAN_DECIM_FACTOR {
			j = 0
			out_i := wf.acc_i[0]
			out_q := wf.acc_q[0]
			#unroll for p in 0 ..< SCAN_FIR_BRANCHES - 1 {
				wf.acc_i[p] = wf.acc_i[p + 1]
				wf.acc_q[p] = wf.acc_q[p + 1]
			}
			wf.acc_i[SCAN_FIR_BRANCHES - 1] = 0.0
			wf.acc_q[SCAN_FIR_BRANCHES - 1] = 0.0

			if wf.warmup_left > 0 {
				wf.warmup_left -= 1
			} else {
				wf.ring_iq[wf.fft_fill] = complex(out_i, out_q)
				wf.fft_fill += 1
				if wf.fft_fill == SCAN_FFT_N {
					wf.cur_ts_ns = base_ts_ns + i64(math.round((f64(idx) / SDR_SAMPLE_RATE_HZ) * 1e9))
					waterfall_accumulate_fft_frame(wf, wf.burst_lo_hz)
					wf.burst_active = false
				}
			}
		}
		wf.poly_phase = j

		if wf.sec_samples_6m >= WF_SEC_SAMPLES_6M {
			if wf.cur_frames > 0 {
				wf.cur_ts_ns = base_ts_ns + i64(math.round((f64(idx) / SDR_SAMPLE_RATE_HZ) * 1e9))
				sl := waterfall_build_slice(&wf.cur_sub_pow, wf.cur_frames)
				sl.timestamp_ns = wf.cur_ts_ns
				sync.mutex_lock(&wf.mutex)
				wf.history[wf.head] = sl
				wf.head             = (wf.head + 1) % WF_HISTORY_SECONDS
				if wf.history_count < WF_HISTORY_SECONDS {
					wf.history_count += 1
				}
				wf.live_slice = sl
				sync.mutex_unlock(&wf.mutex)
			}
			wf.cur_sub_pow    = {}
			wf.cur_frames     = 0
			wf.sec_samples_6m = 0
		}
	}
}

// Render Upper 40-Channel Header + Instantaneous Braille Spectrum, ALL Current Settings & Modulation,
// Exact Date & Time (Live or Recorded IQ File!), and Lower 60-Second Braille Waterfall.
// Double-buffered: builds the entire Unicode Braille matrix in RAM and commits atomically in one frame!
print_braille_waterfall :: proc(rx: ^Receiver_Context) {
	// Wait briefly (up to 200 ms) on immediate startup if first FFT frame has not completed yet
	for _ in 0 ..< 4 {
		sync.mutex_lock(&rx.waterfall.mutex)
		ready := rx.waterfall.live_slice.valid || rx.waterfall.history_count > 0
		sync.mutex_unlock(&rx.waterfall.mutex)
		if ready do break
		time.sleep(50 * time.Millisecond)
	}

	// Thread-safe snapshot of live slice + 60-second history
	live      : Waterfall_Sec_Slice
	hist_ord  : [WF_HISTORY_SECONDS]Waterfall_Sec_Slice // [0] = newest (-1s), [59] = oldest (-60s)
	hist_cnt  := 0

	sync.mutex_lock(&rx.waterfall.mutex)
	live     = rx.waterfall.live_slice
	hist_cnt = rx.waterfall.history_count
	for age in 0 ..< hist_cnt {
		slot := (rx.waterfall.head - 1 - age + WF_HISTORY_SECONDS * 2) % WF_HISTORY_SECONDS
		hist_ord[age] = rx.waterfall.history[slot]
	}
	sync.mutex_unlock(&rx.waterfall.mutex)

	t := receiver_get_telemetry(rx)
	if !live.valid && hist_cnt > 0 {
		live = hist_ord[0]
	}

	// Authoritative stream timestamp (recorded timestamp when playing/paused on IQ file; live UTC when LIVE)
	active_ts_ns := t.stream_timestamp_ns
	if live.valid && live.timestamp_ns > 0 && !t.iq_file_mode {
		active_ts_ns = live.timestamp_ns
	}
	if active_ts_ns <= 0 {
		active_ts_ns = time.now()._nsec
	}

	floor_half := live.floor_half_dbfs if live.valid else -68.0
	floor_sub  := floor_half - 3.01 // 2.5 kHz sub-band floor = half-channel floor - 3.01 dB

	dt_buf : [48]u8
	dt_str := format_datetime_utc(active_ts_ns, dt_buf[:], true)

	live_tag := "LIVE REFRESH ON" if t.waterfall_enabled else "SNAPSHOT ('sp on' for live)"

	// Format complete transport / recording status
	src_buf : [160]u8
	src_str : string
	if t.iq_file_mode {
		p_buf, d_buf : [32]u8
		nm := string(t.iq_play_name_buf[:t.iq_play_name_len])
		st := "PLAYING" if t.iq_playing else "STOPPED/PAUSED"
		src_str = fmt.bprintf(
			src_buf[:],
			"IQ FILE [%s] %s @ %s / %s",
			st, nm,
			format_duration_mmss(t.iq_play_pos_sec, p_buf[:]),
			format_duration_mmss(t.iq_play_tot_sec, d_buf[:]),
		)
	} else if t.iq_saving {
		s_buf : [32]u8
		nm := string(t.iq_save_name_buf[:t.iq_save_name_len])
		src_str = fmt.bprintf(
			src_buf[:],
			"LIVE + SAVING -> %s (%s, %.1f MiB, Drops:%d)",
			nm, format_duration_mmss(t.iq_save_sec, s_buf[:]), t.iq_save_mb, t.iq_save_drops,
		)
	} else {
		src_str = fmt.bprintf(src_buf[:], "LIVE STREAM (%s)", t.device_name)
	}

	// Format ALL current receiver settings & current modulation
	sq_state := "OPEN" if t.squelch_open else "MUTED"
	sq_buf   : [56]u8
	sq_desc  : string
	if !t.squelch_enabled {
		sq_desc = fmt.bprintf(sq_buf[:], "OFF (%s)", sq_state)
	} else if t.auto_squelch {
		sq_desc = fmt.bprintf(sq_buf[:], "AUTO %+.1f dBFS (%s)", t.squelch_dbfs, sq_state)
	} else {
		sq_desc = fmt.bprintf(sq_buf[:], "MAN %+.1f dBFS (%s)", t.squelch_dbfs, sq_state)
	}

	clar_mode := "MANUAL"
	if t.auto_clarifier {
		clar_mode = "AUTO-LOCKED" if t.clar_locked else "AUTO-TRACK"
	}
	gain_mode := "AUTO" if t.auto_rf_gain else ("HW-AGC" if t.hw_agc_enabled else "MANUAL")
	lna_mode  := "AUTO" if t.auto_lna else "MANUAL"
	ant_lbl   := "A"
	if t.antenna_port == 1 do ant_lbl = "B"
	if t.antenna_port == 2 do ant_lbl = "C"

	s_meter_buf : [48]u8
	gr_eff      := f64(t.rf_gr_db) + f64(t.lna_state) * 3.0
	s_lbl, _    := format_s_meter(t.sig_dbfs, gr_eff, s_meter_buf[:])

	flt_short := "L0 OFF (Raw)"
	switch t.filter_level {
	case .Off:     flt_short = "L0 OFF (Raw)"
	case .Level_1: flt_short = "L1 Simple (Biquad+HumComb)"
	case .Level_2: flt_short = "L2 Moderate (DeClick+NLMS+Exp)"
	case .Level_3: flt_short = "L3 Adv (-15dB OM-LSA+Comb)"
	case .Level_4: flt_short = "L4 Ultra (-20..-30dB MMSE+Comb)"
	}

	frame : Screen_Frame
	frame_println(&frame, "==============================================================================================")
	frame_printfln(&frame, "  40-CH CB BRAILLE SPECTRUM & 60s WATERFALL  |  %s", live_tag)
	frame_printfln(&frame, "  Date & Time : %-28s |  Source: %s", dt_str, src_str)
	frame_println(&frame, "----------------------------------------------------------------------------------------------")
	frame_printfln(
		&frame,
		"  Modulation  : %-4s (%s) | Ch %02d: %.6f MHz -> Eff: %.6f MHz",
		mode_name(t.mode), mode_norm_desc(t.mode), t.channel, t.nominal_freq_hz / 1e6, t.effective_hz / 1e6,
	)
	frame_printfln(
		&frame,
		"  Settings 1  : Clar: %s (%+.1f Hz) | Filter: %s (NR %.1f dB) | Vol: %.0f%%",
		clar_mode, t.clarifier_hz, flt_short, t.filter_nr_db, t.volume_pct,
	)
	frame_printfln(
		&frame,
		"  Settings 2  : Sq: %s | Gain: %s (gR=%ddB) | LNA: %s (%d/%d) | Ant:%s | Notch:%s | PPM:%+.1f | Scan:%s",
		sq_desc, gain_mode, t.rf_gr_db, lna_mode, t.lna_state, t.max_lna_state,
		ant_lbl, "ON" if t.rf_notch else "OFF", t.ppm_cal, t.scan_band_label,
	)
	frame_printfln(
		&frame,
		"  Ch Signal   : Sig %+.1f dBFS (%s) | Noise %+.1f dBFS | SNR %+.1f dB | SpeechAGC %+.1f dB | LO %.6f MHz",
		t.sig_dbfs, s_lbl, t.noise_dbfs, t.snr_db, t.agc_gain_db, t.lo_freq_hz / 1e6,
	)
	frame_println(&frame, "----------------------------------------------------------------------------------------------")

	// --- UPPER PART: Always shows all 40 Channels (2 digits/ch: 01..40) + Live SP Amplitude Plot ---
	mhz_line : [80]u8
	for i in 0 ..< 80 do mhz_line[i] = ' '
	copy(mhz_line[0:6],   "26.965") // Ch 01
	copy(mhz_line[16:22], "27.065") // Ch 09
	copy(mhz_line[36:42], "27.185") // Ch 19
	copy(mhz_line[52:58], "27.275") // Ch 27
	copy(mhz_line[74:80], "27.405") // Ch 40
	frame_printfln(&frame, "  Freq (MHz)  |%s|", string(mhz_line[:]))

	odd_line : [80]u8
	evn_line : [80]u8
	ptr_line : [80]u8
	for i in 0 ..< 80 {
		odd_line[i] = ' '
		evn_line[i] = ' '
		ptr_line[i] = ' '
	}
	for ch in 1 ..= 40 {
		col := (ch - 1) * 2
		d1  := u8('0' + (ch / 10))
		d0  := u8('0' + (ch % 10))
		if (ch & 1) == 1 {
			odd_line[col + 0] = d1
			odd_line[col + 1] = d0
		} else {
			evn_line[col + 0] = d1
			evn_line[col + 1] = d0
		}
		if ch == t.channel {
			ptr_line[col + 0] = '^'
			ptr_line[col + 1] = '^'
		}
	}
	frame_printfln(&frame, "  Ch (Odd)    |%s|", string(odd_line[:]))
	frame_printfln(&frame, "  Ch (Even)   |%s|", string(evn_line[:]))
	frame_printfln(&frame, "  Tuned Ch    |%s|", string(ptr_line[:]))
	frame_printfln(&frame, "  SP Plot Time| %s  (Instantaneous 40-Channel dB Amplitude Plot)", dt_str)
	frame_println(&frame, "----------------------------------------------------------------------------------------------")

	// 4 character rows = 16 vertical Braille dots (2.5 dB/dot = 10.0 dB/row, 40 dB span above floor)
	LEFT_DOT_BIT  := [4]u8{0x01, 0x02, 0x04, 0x40}
	RIGHT_DOT_BIT := [4]u8{0x08, 0x10, 0x20, 0x80}
	line_utf8 : [80 * 3]u8

	for r_ch in 0 ..< 4 {
		row_top_db := int(math.round(floor_half + f64(4 - r_ch) * 10.0 - 5.0))
		pos := 0
		for col in 0 ..< 80 {
			c_idx := col / 2
			h_idx := col & 1
			ex_l  := (live.sub_dbfs[c_idx][h_idx * 2 + 0] - floor_sub) if live.valid else 0.0
			ex_r  := (live.sub_dbfs[c_idx][h_idx * 2 + 1] - floor_sub) if live.valid else 0.0

			mask : u8 = 0
			for sr in 0 ..< 4 {
				g_dot := (3 - r_ch) * 4 + (3 - sr) // 0 (bottom baseline) .. 15 (top +37.5 dB)
				thr   := -0.8 if g_dot == 0 else f64(g_dot) * 2.5
				if ex_l >= thr do mask |= LEFT_DOT_BIT[sr]
				if ex_r >= thr do mask |= RIGHT_DOT_BIT[sr]
			}
			pos = encode_braille_utf8(line_utf8[:], pos, mask)
		}
		b_amp : [16]u8
		amp_s := fmt.bprintf(b_amp[:], "%+d", row_top_db)
		frame_printfln(&frame, "  Amp %5sdB |%s|", amp_s, string(line_utf8[:pos]))
	}
	frame_println(&frame, "----------------------------------------------------------------------------------------------")

	// --- LOWER PART: Last 60 Seconds Spectrum Waterfall (20 rows x 3s = 60s, UTC HH:MM:SS + dB Braille) ---
	newest_row_ns : i64 = 0
	oldest_row_ns : i64 = 0

	for wf_r in 0 ..< WF_DISPLAY_ROWS {
		sec_start := wf_r * 3 + 1
		pos       := 0
		has_any   := false
		row_ts_ns : i64 = 0

		for ds in 0 ..< 3 {
			age := wf_r * 3 + ds
			if age < hist_cnt && hist_ord[age].valid && hist_ord[age].timestamp_ns > 0 {
				if row_ts_ns == 0 {
					row_ts_ns = hist_ord[age].timestamp_ns
				}
				oldest_row_ns = hist_ord[age].timestamp_ns
			}
		}
		if row_ts_ns == 0 && wf_r == 0 && hist_cnt == 0 && live.valid {
			row_ts_ns = active_ts_ns
		}
		if row_ts_ns > 0 && newest_row_ns == 0 {
			newest_row_ns = row_ts_ns
		}
		// Synthesize expected row clock tick for not-yet-filled history rows relative to active_ts_ns
		disp_row_ns := row_ts_ns if row_ts_ns > 0 else (active_ts_ns - i64(sec_start) * 1_000_000_000)
		hms_buf     : [16]u8
		hms_str     := format_hms_utc(disp_row_ns, hms_buf[:])

		for col in 0 ..< 80 {
			c_idx := col / 2
			h_idx := col & 1
			peak_ex := -99.0

			for ds in 0 ..< 3 {
				age := wf_r * 3 + ds
				if age < hist_cnt && hist_ord[age].valid {
					ex := hist_ord[age].half_dbfs[c_idx][h_idx] - hist_ord[age].floor_half_dbfs
					if ex > peak_ex do peak_ex = ex
					has_any = true
				} else if wf_r == 0 && ds == 0 && hist_cnt == 0 && live.valid {
					ex := live.half_dbfs[c_idx][h_idx] - live.floor_half_dbfs
					if ex > peak_ex do peak_ex = ex
					has_any = true
				}
			}

			mask : u8 = 0
			if peak_ex > -90.0 {
				mask, _ = braille_db_to_mask(peak_ex)
			}
			pos = encode_braille_utf8(line_utf8[:], pos, mask)
		}

		sep := "|" if has_any else ":"
		frame_printfln(&frame, " %s -%02ds%s%s%s", hms_str, sec_start, sep, string(line_utf8[:pos]), sep)
	}

	frame_println(&frame, "----------------------------------------------------------------------------------------------")
	// Build exact dBFS Braille dot-matrix legend calibrated to current band floor + Waterfall Date/Time span
	leg_masks := [9]u8{0x00, 0x40, 0xC0, 0xC4, 0xE4, 0xE6, 0xF6, 0xF7, 0xFF}
	leg_offs  := [9]f64{0.0, 1.8, 4.5, 7.5, 11.0, 15.0, 19.5, 24.5, 30.0}
	g_buf : [9][3]u8
	for i in 0 ..< 9 {
		encode_braille_utf8(g_buf[i][:], 0, leg_masks[i])
	}
	frame_printfln(
		&frame,
		"  dB Map      | [%s]<%+.0f [%s]%+.0f [%s]%+.0f [%s]%+.0f [%s]%+.0f [%s]%+.0f [%s]%+.0f [%s]%+.0f [%s]>=%+.0f dBFS (Hist: %02d/60s)",
		string(g_buf[0][:]), floor_half + 1.8,
		string(g_buf[1][:]), floor_half + leg_offs[1],
		string(g_buf[2][:]), floor_half + leg_offs[2],
		string(g_buf[3][:]), floor_half + leg_offs[3],
		string(g_buf[4][:]), floor_half + leg_offs[4],
		string(g_buf[5][:]), floor_half + leg_offs[5],
		string(g_buf[6][:]), floor_half + leg_offs[6],
		string(g_buf[7][:]), floor_half + leg_offs[7],
		string(g_buf[8][:]), floor_half + leg_offs[8],
		min(hist_cnt, 60),
	)
	if newest_row_ns > 0 && oldest_row_ns > 0 {
		nw_b, ol_b : [48]u8
		frame_printfln(
			&frame,
			"  WF Time Span| Newest: %s  ->  Oldest: %s",
			format_datetime_utc(newest_row_ns, nw_b[:], false),
			format_datetime_utc(oldest_row_ns, ol_b[:], false),
		)
	}
	frame_println(&frame, "==============================================================================================")
	screen_submit_frame(&frame)
}


