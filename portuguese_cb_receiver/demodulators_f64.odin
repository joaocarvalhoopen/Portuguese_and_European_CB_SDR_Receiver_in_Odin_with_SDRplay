// High-Precision f64 Demodulators for AM (Synchronous Type-II PLL), FM (Threshold-Extension
// PLL + Conjugate-Product Arctan Discriminator), and SSB - LSB & USB (Analytic Coherent
// Product Detector), plus Post-Filter Speech AGC, Soft-Knee Limiter, and Hysteresis Squelch.

package main

import "core:math"

Demodulator_State :: struct {
	// --- Synchronous AM (SAM) Type-II 2nd-Order Carrier PLL ---
	sam_phase       : f64,
	sam_freq_rad    : f64, // radians/sample carrier offset
	sam_carr_i_lpf  : f64,
	sam_carr_q_lpf  : f64,
	sam_lock_metric : f64,
	sam_dc_x1       : f64,
	sam_dc_y1       : f64,
	sam_env_lpf1    : f64,
	sam_env_avg     : f64,

	// --- FM Arctan Discriminator + Threshold-Extension PLL ---
	fm_prev_z       : complex128,
	fm_pll_phase    : f64,
	fm_pll_freq_rad : f64,
	fm_env_rms      : f64,
	fm_deemph_state : f64,
	fm_hp_x1        : f64,
	fm_hp_y1        : f64,
	fm_dc_avg       : f64,

	// --- SSB DC & Pre-Filter Channel IF AGC Conditioning ---
	ssb_dc_x1       : f64,
	ssb_dc_y1       : f64,
	ssb_if_env      : f64,
	if_agc_init     : bool,

	// --- Post-Filter Adaptive Speech AGC (f64) ---
	post_agc_init   : bool,
	post_agc_blks   : int,
	agc_env_fast    : f64,
	agc_env_slow    : f64,
	agc_noise_floor : f64,
	agc_gain        : f64,

	// --- Hysteresis & Adaptive Auto-Squelch State ---
	sq_is_open         : bool,
	sq_hold_samples    : int,
	sq_env_gain        : f64,
	eff_sq_thresh_dbfs : f64,

	// --- Output Audio Metrics ---
	audio_rms          : f64,
	audio_peak         : f64,
	carrier_off_hz     : f64,
}

demod_init :: proc(dm: ^Demodulator_State) {
	dm^ = {}
	dm.fm_prev_z          = complex(1.0, 0.0)
	dm.fm_env_rms         = 1e-6
	dm.sam_env_lpf1       = 1e-6
	dm.sam_env_avg        = 1e-6
	dm.ssb_if_env         = 1e-6
	dm.if_agc_init        = false
	dm.post_agc_init      = false
	dm.agc_env_slow       = 1e-4
	dm.agc_noise_floor    = 1e-4
	dm.agc_gain           = 1.0
	dm.sq_is_open         = false
	dm.sq_env_gain        = 1.0
	dm.eff_sq_thresh_dbfs = -98.0
}

demod_reset_mode :: proc(dm: ^Demodulator_State) {
	dm.sam_phase       = 0.0
	dm.sam_freq_rad    = 0.0
	dm.sam_carr_i_lpf  = 0.0
	dm.sam_carr_q_lpf  = 0.0
	dm.sam_lock_metric = 0.0
	dm.sam_dc_x1       = 0.0
	dm.sam_dc_y1       = 0.0
	dm.sam_env_lpf1    = 0.0

	dm.fm_prev_z       = complex(1.0, 0.0)
	dm.fm_pll_phase    = 0.0
	dm.fm_pll_freq_rad = 0.0
	dm.fm_deemph_state = 0.0
	dm.fm_hp_x1        = 0.0
	dm.fm_hp_y1        = 0.0
	dm.fm_dc_avg       = 0.0

	dm.ssb_dc_x1       = 0.0
	dm.ssb_dc_y1       = 0.0
	dm.if_agc_init     = false
}

// Wrap phase into [-pi, +pi] in f64
@(private)
wrap_pi_f64 :: #force_inline proc(angle: f64) -> f64 {
	a := math.mod(angle + math.PI, 2.0 * math.PI)
	if a < 0.0 do a += 2.0 * math.PI
	return a - math.PI
}

// Synchronous AM (SAM) Demodulator in f64 at 48 kHz with Zero-Lag Wideband Type-II PLL & Unpumped Carrier IF AGC
@(private)
demod_sample_am :: #force_inline proc(dm: ^Demodulator_State, z: complex128, noise_rms: f64) -> f64 {
	env := c64_abs(z)

	// Rotate incoming signal by -sam_phase
	c  := math.cos(dm.sam_phase)
	s  := math.sin(dm.sam_phase)
	ri :=  real(z) * c + imag(z) * s
	rq := -real(z) * s + imag(z) * c

	if !dm.if_agc_init && env > 1e-12 {
		init_c          := math.max(env, 1e-8)
		dm.sam_env_lpf1  = init_c
		dm.sam_env_avg   = init_c
		dm.sam_carr_i_lpf = ri
		dm.sam_carr_q_lpf = rq
		dm.if_agc_init   = true
	}

	// 1. Wideband zero-lag 4-quadrant carrier phase detector:
	// For AM z(t) = A_c * (1 + m(t)) * exp(j*theta_e), whenever env >= 0.26 * A_c we are guaranteed
	// (1 + m(t)) > 0 (even on 125% overmodulated transmitters!), so (1 + m(t)) cancels identically
	// in atan2(rq, ri) == theta_e across (-pi, +pi] with ZERO voice-modulation phase jitter!
	// During deep negative overmodulation pinch-offs (env < 0.26 * A_c), phase_err = 0 coasts cleanly.
	phase_err := 0.0
	if env >= 0.26 * dm.sam_env_avg && env > 1e-12 {
		phase_err = math.atan2(rq, ri)
	}

	// 2nd-order Type-II PLL (f_n ~ 75 Hz, zeta = 0.85, lock range +-1200 Hz > 2x CEPT +-600 Hz tolerance)
	PLL_ALPHA :: 0.0167
	PLL_BETA  :: 0.0000964
	MAX_RAD   :: (2.0 * math.PI * 1200.0) / BASEBAND_RATE_HZ

	dm.sam_freq_rad = clamp(dm.sam_freq_rad + PLL_BETA * phase_err, -MAX_RAD, +MAX_RAD)
	dm.sam_phase    = wrap_pi_f64(dm.sam_phase + dm.sam_freq_rad + PLL_ALPHA * phase_err)

	// 2. Out-of-loop 50 Hz coherent carrier LPF (alpha = 1 - exp(-2*pi*50/48000) = 0.00652)
	// Strictly outside the PLL feedback loop so it adds zero loop phase lag while accurately
	// measuring coherent carrier lock and unmodulated carrier amplitude A_c!
	CARR_ALPHA :: 0.00652
	dm.sam_carr_i_lpf += CARR_ALPHA * (ri - dm.sam_carr_i_lpf)
	dm.sam_carr_q_lpf += CARR_ALPHA * (rq - dm.sam_carr_q_lpf)
	carr_mag          := math.hypot(dm.sam_carr_i_lpf, dm.sam_carr_q_lpf)

	inst_lock := 0.0
	if carr_mag > 1e-12 {
		inst_lock = clamp(dm.sam_carr_i_lpf / carr_mag, 0.0, 1.0)
	}
	dm.sam_lock_metric = 0.992 * dm.sam_lock_metric + 0.008 * inst_lock
	dm.carrier_off_hz  = (dm.sam_freq_rad * BASEBAND_RATE_HZ) / (2.0 * math.PI)

	// 3. Beat-whistle-free coherent lock gate:
	// Pure envelope |z| when unlocked (< 0.70, zero heterodyne whistle!), pure linear synchronous ri once locked (> 0.92).
	w_sam := clamp((dm.sam_lock_metric - 0.70) / 0.22, 0.0, 1.0)
	raw_a := w_sam * ri + (1.0 - w_sam) * env

	// 4. Unpumped 2-pole 15 Hz carrier amplitude tracker (attenuates >140 Hz voice modulation ripple by >45..70 dB
	// so positive voice peaks never compress or distort the recovered AM waveform, while snapping fast on key-up!)
	carr_est := w_sam * math.max(dm.sam_carr_i_lpf, 0.25 * env) + (1.0 - w_sam) * env
	ENV_ALPHA :: 0.0020
	dm.sam_env_lpf1 += ENV_ALPHA * (carr_est - dm.sam_env_lpf1)
	if dm.sam_env_lpf1 > 2.2 * dm.sam_env_avg {
		// Fast key-up acquisition when a strong carrier suddenly appears on an idle channel
		dm.sam_env_avg = 0.96 * dm.sam_env_avg + 0.04 * dm.sam_env_lpf1
	} else {
		dm.sam_env_avg += ENV_ALPHA * (dm.sam_env_lpf1 - dm.sam_env_avg)
	}

	// High-precision 40 Hz DC blocker (y[n] = x[n] - x[n-1] + R * y[n-1], R = 1 - 2*pi*40/48000 = 0.99476)
	R_DC :: 0.99476
	audio := raw_a - dm.sam_dc_x1 + R_DC * dm.sam_dc_y1
	dm.sam_dc_x1 = raw_a
	dm.sam_dc_y1 = audio

	// Normalize AC audio by true carrier amplitude A_c (floored by channel noise RMS so empty-channel hiss stays calm)
	denom := math.max(dm.sam_env_avg, math.max(noise_rms * 2.4, 1e-8))
	return clamp((audio / denom) * 0.45, -1.5, +1.5)
}

// Narrowband CEPT CB FM Demodulator (+-2.0 kHz ETSI EN 300 433 deviation) with Exact Linear
// Conjugate-Product Discriminator, Deep-Fade Threshold-Extension PLL, and 530 us (300 Hz) De-Emphasis
@(private)
demod_sample_fm :: #force_inline proc(dm: ^Demodulator_State, z: complex128) -> f64 {
	mag := c64_abs(z)
	dm.fm_env_rms = 0.995 * dm.fm_env_rms + 0.005 * math.max(mag, 1e-9)

	// 1. Exact linear conjugate-product phase discriminator: arg(z[n] * conj(z[n-1]))
	prod := z * conj(dm.fm_prev_z)
	dm.fm_prev_z = z

	disc_rad := 0.0
	if c64_abs2(prod) > 1e-24 {
		disc_rad = math.atan2(imag(prod), real(prod))
	}

	// 2. 2nd-Order Threshold-Extension FM-PLL (tracks +-6.5 kHz to accommodate off-center rigs + full deviation)
	// Engaged ONLY during deep Rayleigh envelope fades (<28% RMS) or >5.2 kHz phase-slip threshold clicks,
	// preserving 100% pure zero-THD linear discriminator fidelity on normal signals!
	z_vco := complex(math.cos(dm.fm_pll_phase), -math.sin(dm.fm_pll_phase))
	z_err := z * z_vco
	pll_err := 0.0
	if mag > 1e-10 {
		pll_err = math.atan2(imag(z_err), real(z_err))
	}

	null_weight   := clamp(mag / math.max(dm.fm_env_rms * 0.45, 1e-10), 0.08, 1.0)
	FM_ALPHA      :: 0.36
	FM_BETA       :: 0.045
	MAX_DEV_RAD   :: (2.0 * math.PI * 6500.0) / BASEBAND_RATE_HZ
	CLICK_DEV_RAD :: (2.0 * math.PI * 5200.0) / BASEBAND_RATE_HZ

	dm.fm_pll_freq_rad = clamp(dm.fm_pll_freq_rad + (FM_BETA * null_weight) * pll_err, -MAX_DEV_RAD, +MAX_DEV_RAD)
	inst_pll_rad      := clamp(dm.fm_pll_freq_rad + (FM_ALPHA * null_weight) * pll_err, -MAX_DEV_RAD, +MAX_DEV_RAD)
	dm.fm_pll_phase    = wrap_pi_f64(dm.fm_pll_phase + inst_pll_rad)

	selected_rad := disc_rad
	if mag < 0.28 * dm.fm_env_rms || math.abs(disc_rad - dm.fm_pll_freq_rad) > CLICK_DEV_RAD {
		selected_rad = inst_pll_rad
	}

	// Slow DC frequency-offset tracker (~3.8 Hz corner) for carrier offset readout & automatic centering
	dm.fm_dc_avg      = 0.9995 * dm.fm_dc_avg + 0.0005 * selected_rad
	dm.carrier_off_hz = (dm.fm_dc_avg * BASEBAND_RATE_HZ) / (2.0 * math.PI)
	centered_rad     := selected_rad - dm.fm_dc_avg

	// Convert radians/sample so +-2.0 kHz CEPT deviation maps to nominal speech level with mid-band makeup gain
	NOM_DEV_RAD :: (2.0 * math.PI * 2000.0) / BASEBAND_RATE_HZ
	demod_norm  := (centered_rad / NOM_DEV_RAD) * (0.50 * 1.90)

	// True CEPT / ETSI EN 300 433 narrowband CB FM voice de-emphasis:
	// tau = 530.5 us (f_c = 300.0 Hz, -6 dB/oct across 300..3000 Hz, alpha = 1 - exp(-2*pi*300/48000) = 0.038513)
	// Flattens +6 dB/oct transmitter pre-emphasis and suppresses f^2 FM discriminator hiss by ~10 dB!
	DEEMPH_ALPHA :: 0.038513
	dm.fm_deemph_state += DEEMPH_ALPHA * (demod_norm - dm.fm_deemph_state)

	// 110 Hz high-pass filter strips sub-audible CTCSS/DCS squelch tones while preserving warm male chest voice
	// R = 1 - 2*pi*110/48000 = 0.9856
	R_HP :: 0.9856
	hp_out := dm.fm_deemph_state - dm.fm_hp_x1 + R_HP * dm.fm_hp_y1
	dm.fm_hp_x1 = dm.fm_deemph_state
	dm.fm_hp_y1 = hp_out

	return clamp(hp_out, -1.5, +1.5)
}

// Coherent Analytic SSB Demodulator (USB & LSB) in f64 at 48 kHz with Syllabic Channel IF AGC.
// Opposite sideband (>95 dB) and DC carrier have already been rejected by the 2048-pt
// complex analytic Overlap-Save filter, preserving exact upright voice pitch & phase.
@(private)
demod_sample_ssb :: #force_inline proc(dm: ^Demodulator_State, z: complex128, noise_rms: f64) -> f64 {
	mag := c64_abs(z)
	if !dm.if_agc_init && mag > 1e-12 {
		dm.ssb_if_env  = math.max(mag, 1e-8)
		dm.if_agc_init = true
	} else if mag > dm.ssb_if_env {
		dm.ssb_if_env = 0.9994 * dm.ssb_if_env + 0.0006 * mag // ~35 ms smooth IF ceiling (prevents syllabic noise pumping)
	} else {
		dm.ssb_if_env = 0.999975 * dm.ssb_if_env + 0.000025 * mag // ~830 ms slow propagation tracking
	}

	// Anchor IF scaling to 2.2x channel noise RMS so weak/buried distant SSB signals (< +6 dB SNR)
	// deliver a 100% stationary thermal noise floor into Filter 3 & Filter 4's spectral estimators!
	denom := math.max(dm.ssb_if_env, math.max(noise_rms * 2.2, 1e-8))
	raw   := (real(z) * math.SQRT_TWO / denom) * 0.28

	// Gentle 55 Hz DC/sub-sonic blocker
	R_SSB_DC :: 0.9928
	y := raw - dm.ssb_dc_x1 + R_SSB_DC * dm.ssb_dc_y1
	dm.ssb_dc_x1 = raw
	dm.ssb_dc_y1 = y
	dm.carrier_off_hz = 0.0
	return clamp(y, -2.5, +2.5)
}

// Demodulate a block of 48 kHz complex analytic/bandpass samples into normalized f64 audio
demodulate_block_f64 :: proc(
	dm              : ^Demodulator_State,
	iq_block        : []complex128,
	mode            : Modulation_Mode,
	dst_aud         : []f64,
	noise_power_lin : f64 = 0.0,
) {
	n := min(len(iq_block), len(dst_aud))
	noise_rms := math.sqrt(math.max(noise_power_lin, 0.0))
	switch mode {
	case .AM:
		for i in 0 ..< n {
			dst_aud[i] = demod_sample_am(dm, iq_block[i], noise_rms)
		}
	case .FM:
		for i in 0 ..< n {
			dst_aud[i] = demod_sample_fm(dm, iq_block[i])
		}
	case .LSB, .USB:
		for i in 0 ..< n {
			dst_aud[i] = demod_sample_ssb(dm, iq_block[i], noise_rms)
		}
	}
}

// Post-Filter Adaptive Speech AGC, Soft-Knee Limiter, Manual/Auto Hysteresis Squelch & User Volume (all f64)
post_process_audio_block_f64 :: proc(
	dm              : ^Demodulator_State,
	audio_buf       : []f64,
	sig_dbfs        : f64,
	snr_db          : f64,
	voice_conf      : f64,
	squelch_enabled : bool,
	squelch_dbfs    : f64,
	volume_linear   : f64,
	filter_level    : Filter_Level,
	auto_squelch    : bool            = false,
	noise_dbfs      : f64             = -110.0,
	mode            : Modulation_Mode = .AM,
	clar_locked     : bool            = false,
) {
	_ = snr_db
	// 1. Update Squelch Gate Decision (Manual dBFS or Adaptive Multi-Mode f64 Auto-Squelch)
	if !squelch_enabled {
		dm.eff_sq_thresh_dbfs = squelch_dbfs
		dm.sq_is_open         = true
		dm.sq_hold_samples    = int(0.20 * AUDIO_SAMPLE_RATE_HZ)
	} else {
		open_thresh  := squelch_dbfs
		close_thresh := squelch_dbfs - 2.5
		hold_sec     := 0.24
		eff_sig      := sig_dbfs

		if auto_squelch {
			// Adaptive noise-floor-referenced threshold immune to RF gain / LNA / band propagation swings
			req_excess_db := 4.6
			switch mode {
			case .FM:
				req_excess_db = 4.8
				hold_sec      = 0.26
			case .AM:
				req_excess_db = 4.5
				hold_sec      = 0.26
				if dm.sam_lock_metric > 0.58 {
					eff_sig += 2.4 * (dm.sam_lock_metric - 0.50)
				}
			case .LSB, .USB:
				req_excess_db = 3.8 if filter_level == .Level_4 else 4.4
				hold_sec      = 0.48 // Longer syllabic bridge prevents chopping between SSB words
				if voice_conf > 0.38 {
					v_scale := 8.5 if filter_level == .Level_4 else 4.5
					eff_sig += v_scale * (voice_conf - 0.30)
				}
				if clar_locked {
					eff_sig += 2.5
				}
			}
			open_thresh  = noise_dbfs + req_excess_db
			close_thresh = open_thresh - 2.6
		} else {
			// Manual threshold: in Filter Level 4 deep-weak-signal mode, allow pitch-coherence assist
			if filter_level == .Level_4 && voice_conf > 0.45 {
				eff_sig += 6.0 * voice_conf
			}
		}
		dm.eff_sq_thresh_dbfs = open_thresh

		if eff_sig >= open_thresh {
			dm.sq_is_open      = true
			dm.sq_hold_samples = int(hold_sec * AUDIO_SAMPLE_RATE_HZ)
		} else if eff_sig < close_thresh {
			if dm.sq_hold_samples > len(audio_buf) {
				dm.sq_hold_samples -= len(audio_buf)
			} else {
				dm.sq_hold_samples = 0
				dm.sq_is_open      = false
			}
		}
	}

	TARGET_RMS :: 0.27
	MAX_GAIN   := 95.0 if filter_level == .Level_4 else (60.0 if filter_level == .Level_3 else 45.0)
	MIN_GAIN   :: 0.05
	is_deep_flt := filter_level == .Level_3 || filter_level == .Level_4

	idle_ceil_rms := 0.155
	snr_open_thr  := 1.42
	switch filter_level {
	case .Off:
		idle_ceil_rms = 0.155 // -16.2 dBFS classic open-channel monitor
		snr_open_thr  = 1.40
	case .Level_1:
		idle_ceil_rms = 0.110 // -19.2 dBFS
		snr_open_thr  = 1.42
	case .Level_2:
		idle_ceil_rms = 0.065 // -23.7 dBFS
		snr_open_thr  = 1.48
	case .Level_3:
		idle_ceil_rms = 0.004 // Zero-silence VAD gated; keeps any transition tail ultra-quiet
		snr_open_thr  = 1.75
	case .Level_4:
		idle_ceil_rms = 0.003 // Zero-silence VAD gated; keeps any transition tail ultra-quiet
		snr_open_thr  = 1.85
	}

	blk_rms := math.sqrt(block_power_f64(audio_buf))
	if blk_rms > 1e-7 {
		dm.post_agc_blks += 1
		if !dm.post_agc_init {
			dm.agc_env_fast    = blk_rms * 0.95
			dm.agc_env_slow    = blk_rms * 1.15 if (is_deep_flt && voice_conf >= 0.18) else blk_rms * 1.45
			dm.agc_noise_floor = blk_rms * 0.35 if (is_deep_flt && voice_conf >= 0.18) else blk_rms * 1.45
			init_ceil          := TARGET_RMS / math.max(blk_rms, 1e-6) if (is_deep_flt && voice_conf >= 0.18) else idle_ceil_rms / math.max(blk_rms, 1e-8)
			dm.agc_gain        = clamp(math.min(TARGET_RMS / math.max(blk_rms, 1e-6), init_ceil), MIN_GAIN, 18.0)
			dm.post_agc_init   = true
		} else if dm.post_agc_blks <= 16 && voice_conf < 0.12 && !is_deep_flt {
			// Allow noise-floor tracker to converge rapidly onto steady-state output during first ~0.34 s warmup
			target_flr := blk_rms * 1.52
			if target_flr > dm.agc_noise_floor {
				dm.agc_noise_floor = 0.72 * dm.agc_noise_floor + 0.28 * target_flr
			}
		}
	}

	sum_sq := 0.0
	peak   := 0.0
	act_vc := clamp((voice_conf - 0.16) * 2.15, 0.0, 1.0) if is_deep_flt else clamp((voice_conf - 0.44) * 2.5, 0.0, 1.0)

	for i in 0 ..< len(audio_buf) {
		x  := audio_buf[i]
		ax := math.abs(x)

		// Exact digital zero-volume preservation during Filter 3 & Filter 4 gated non-speech moments:
		// Freeze AGC noise-floor & slow envelope so gain never winds up to 100x during silent pauses!
		if is_deep_flt && ax <= 1e-11 {
			target_sq := 1.0 if dm.sq_is_open else 0.0
			if target_sq > dm.sq_env_gain {
				dm.sq_env_gain = 0.985 * dm.sq_env_gain + 0.015 * target_sq
			} else {
				dm.sq_env_gain = 0.996 * dm.sq_env_gain + 0.004 * target_sq
			}
			// Gently relax extreme speech gain toward safe nominal ceiling during long pauses to prevent onset pops
			if dm.agc_gain > 28.0 {
				dm.agc_gain = 0.99990 * dm.agc_gain + 0.00010 * 28.0
			}
			audio_buf[i] = 0.0
			continue
		}

		// Ignore exact digital zero-fill during initial STFT OLA pipeline warmup (1024..3072 samples)
		// so dm.agc_noise_floor never collapses toward zero before real filtered audio arrives!
		if ax > 1e-7 || (!is_deep_flt && dm.agc_env_slow > 1e-6) {
			if dm.agc_env_slow <= 1e-6 && ax > 1e-7 {
				dm.agc_env_fast    = ax
				dm.agc_env_slow    = ax
				dm.agc_noise_floor = ax * (0.35 if (is_deep_flt && voice_conf >= 0.18) else 1.0)
			} else {
				// Short-time (~6 ms on deep filters, ~8 ms otherwise) smoothed amplitude envelope
				if ax > dm.agc_env_fast {
					a_att := 0.0080 if is_deep_flt else 0.0040
					dm.agc_env_fast = (1.0 - a_att) * dm.agc_env_fast + a_att * ax
				} else {
					dm.agc_env_fast = 0.9988 * dm.agc_env_fast + 0.0012 * ax
				}

				// Syllabic (~14 ms attack / ~320 ms hang-decay) speech envelope prevents inter-word gain pumping
				if ax > dm.agc_env_slow {
					s_att := 0.0022 if is_deep_flt else 0.0012
					dm.agc_env_slow = (1.0 - s_att) * dm.agc_env_slow + s_att * ax
				} else {
					dm.agc_env_slow = 0.99993 * dm.agc_env_slow + 0.00007 * ax
				}

				// Track minimum of the SAME syllabic detector (dm.agc_env_slow); on deep VAD-gated filters,
				// never collapse agc_noise_floor during low-level gate fade-outs!
				if !is_deep_flt || voice_conf < 0.12 {
					if dm.agc_env_slow < dm.agc_noise_floor && (!is_deep_flt || ax > 1e-4) {
						dm.agc_noise_floor = 0.9940 * dm.agc_noise_floor + 0.0060 * dm.agc_env_slow
					} else if !is_deep_flt {
						dm.agc_noise_floor = 0.99990 * dm.agc_noise_floor + 0.00010 * math.min(dm.agc_env_slow, dm.agc_noise_floor * 1.75 + 1e-7)
					}
				}
			}
		}

		ref_env     := math.max(dm.agc_env_fast * 1.25, dm.agc_env_slow)
		speech_gain := TARGET_RMS / math.max(ref_env * 0.62, 1e-8)
		snr_env     := dm.agc_env_slow / math.max(dm.agc_noise_floor, 1e-8)
		snr_fast    := (dm.agc_env_fast * 1.45) / math.max(dm.agc_noise_floor, 1e-8)
		act_env     := clamp((snr_env  - snr_open_thr) / 0.85, 0.0, 1.0)
		act_fast    := clamp((snr_fast - snr_open_thr) / 0.85, 0.0, 1.0)
		speech_act  := act_vc if is_deep_flt else math.max(act_env,  act_vc)
		inst_speech := act_vc if is_deep_flt else math.max(act_fast, act_vc)

		est_noise_rms      := math.max(dm.agc_noise_floor * 0.62, 1e-8)
		noise_ceiling_gain := idle_ceil_rms / est_noise_rms
		if filter_level == .Level_3 {
			noise_ceiling_gain = math.min(noise_ceiling_gain, 6.0)
		} else if filter_level == .Level_4 {
			noise_ceiling_gain = math.min(noise_ceiling_gain, 5.0)
		}
		eff_ceil     := noise_ceiling_gain + speech_act * math.max(speech_gain - noise_ceiling_gain, 0.0)
		desired_gain := clamp(math.min(speech_gain, eff_ceil), MIN_GAIN, MAX_GAIN)

		// Instant peak-overload guard when coming out of digital zero silence on strong syllable onset
		if is_deep_flt && ax * dm.agc_gain > 0.72 {
			dm.agc_gain = 0.85 * dm.agc_gain + 0.15 * (0.58 / math.max(ax, 1e-6))
		}

		if desired_gain < dm.agc_gain {
			// True SSB Syllabic Hang-AGC: fast anti-overload attack when signal rises,
			// gentle hang-hold when instantaneous speech pauses toward noise_ceiling_gain
			if inst_speech > 0.25 || desired_gain > noise_ceiling_gain {
				dm.agc_gain = 0.986 * dm.agc_gain + 0.014 * desired_gain
			} else {
				dm.agc_gain = 0.99985 * dm.agc_gain + 0.00015 * desired_gain
			}
		} else if inst_speech > 0.20 || (!is_deep_flt && dm.agc_gain < noise_ceiling_gain) {
			// Only ramp gain upward when speech is actively present (zero inter-word noise pumping!)
			up_a := 0.0014 if is_deep_flt else 0.0008
			dm.agc_gain = (1.0 - up_a) * dm.agc_gain + up_a * desired_gain
		}

		// Smooth clickless squelch envelope
		target_sq := 1.0 if dm.sq_is_open else 0.0
		if target_sq > dm.sq_env_gain {
			dm.sq_env_gain = 0.985 * dm.sq_env_gain + 0.015 * target_sq
		} else {
			dm.sq_env_gain = 0.996 * dm.sq_env_gain + 0.004 * target_sq
		}

		y := x * dm.agc_gain * dm.sq_env_gain * volume_linear

		// Smooth hyperbolic tangent soft-knee limiter prevents harsh clipping on static bursts
		if math.abs(y) > 0.65 {
			sign := 1.0 if y >= 0.0 else -1.0
			ay   := math.abs(y)
			y     = sign * (0.65 + 0.34 * math.tanh((ay - 0.65) / 0.34))
		}
		y = clamp(y, -0.995, +0.995)

		audio_buf[i] = y
		sum_sq += y * y
		if math.abs(y) > peak do peak = math.abs(y)
	}

	if len(audio_buf) > 0 {
		rms := math.sqrt(sum_sq / f64(len(audio_buf)))
		if is_deep_flt && peak == 0.0 {
			dm.audio_rms  = 0.0
			dm.audio_peak = 0.0
		} else {
			dm.audio_rms  = 0.80 * dm.audio_rms + 0.20 * rms
			dm.audio_peak = peak
		}
	}
}

// ---------------------------------------------------------------------------
// Closed-Loop Multi-Mode f64 Auto-Clarifier (AM, FM, SSB-LSB & SSB-USB)
//   - AM : Coarse 2048-pt spectral carrier pull-in (+-2.5 kHz) + Type-II SAM PLL zeroing (<0.4 Hz)
//   - FM : Coarse spectral pull-in + modulated carrier DC offset centering (<2.0 Hz)
//   - SSB: 4096-pt Blackman-Harris Welch Harmonic Inharmonicity / Subharmonic Summation
//          estimator tracking human vocal harmonics h*F0 + df_audio at 0.5 Hz resolution
//          with exact sideband polarity (+df for USB, -df for LSB) & birdie immunity!
// ---------------------------------------------------------------------------

AC_FFT_N :: 4096
AC_BINS  :: (AC_FFT_N / 2) + 1

Auto_Clarifier_State :: struct {
	fft_plan       : FFT_Plan,
	window         : [AC_FFT_N]f64,
	audio_ring     : [AC_FFT_N]f64,
	work_fft       : [AC_FFT_N]complex128,

	welch_psd      : [AC_BINS]f64,
	noise_psd      : [AC_BINS]f64,
	tonal_mag      : [AC_BINS]f64,
	tonal_pow      : [AC_BINS]f64,

	welch_frames   : int,
	settle_blocks  : int,
	blk_counter    : int,
	fm_off_ema     : f64,
	fm_valid_cnt   : int,
	prev_ssb_err   : f64,
	ssb_consist    : int,

	est_off_hz     : f64,
	ssb_no_h12_adj : f64,
	locked         : bool,
	hold_ticks     : int,
}

auto_clarifier_init :: proc(ac: ^Auto_Clarifier_State) {
	ac^ = {}
	fft_plan_init(&ac.fft_plan, AC_FFT_N)
	for n in 0 ..< AC_FFT_N {
		ac.window[n] = blackman_harris_f64(n, AC_FFT_N)
	}
	ac.settle_blocks = 3
}

auto_clarifier_reset :: proc(ac: ^Auto_Clarifier_State) {
	ac.audio_ring     = {}
	ac.welch_psd      = {}
	ac.noise_psd      = {}
	ac.tonal_mag      = {}
	ac.tonal_pow      = {}
	ac.welch_frames   = 0
	ac.settle_blocks  = 3
	ac.blk_counter    = 0
	ac.fm_off_ema     = 0.0
	ac.fm_valid_cnt   = 0
	ac.prev_ssb_err   = 0.0
	ac.ssb_consist    = 0
	ac.est_off_hz     = 0.0
	ac.ssb_no_h12_adj = 0.0
	ac.locked         = false
	ac.hold_ticks     = 0
}

@(private)
ac_eval_harmonic_comb :: #force_inline proc(
	exc        : []f64,
	cand_f0    : f64,
	df         : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_bin_hz : f64,
) -> (score: f64, active_harms: int, has_h12: bool) {
	max_h        := clamp(int(math.floor(2450.0 / cand_f0)), 5, 16)
	sum          := 0.0
	w_tot        := 0.0
	act          := 0
	low_act      := 0
	low3_act     := 0
	low_band_act := 0
	consec_pairs := 0
	prev_h_act   := false
	h12_strong   := false
	for h in 1 ..= max_h {
		fh := f64(h) * cand_f0 + df
		if fh < f_lo || fh > f_hi {
			prev_h_act = false
			continue
		}

		bf  := fh * inv_bin_hz
		b0  := int(math.floor(bf))
		frc := bf - f64(b0)
		pk  := 0.0
		if b0 >= k_lo && b0 + 1 <= k_hi {
			pk = (1.0 - frc) * exc[b0] + frc * exc[b0 + 1]
		}

		fv_lo := (f64(h) - 0.5) * cand_f0 + df
		fv_hi := (f64(h) + 0.5) * cand_f0 + df
		val   := 0.0
		vcnt  := 0.0
		if fv_lo >= f_lo && fv_lo <= f_hi {
			vb  := fv_lo * inv_bin_hz
			vb0 := int(math.floor(vb))
			vfr := vb - f64(vb0)
			if vb0 >= k_lo && vb0 + 1 <= k_hi {
				val  += (1.0 - vfr) * exc[vb0] + vfr * exc[vb0 + 1]
				vcnt += 1.0
			}
		}
		if fv_hi >= f_lo && fv_hi <= f_hi {
			vb  := fv_hi * inv_bin_hz
			vb0 := int(math.floor(vb))
			vfr := vb - f64(vb0)
			if vb0 >= k_lo && vb0 + 1 <= k_hi {
				val  += (1.0 - vfr) * exc[vb0] + vfr * exc[vb0 + 1]
				vcnt += 1.0
			}
		}
		if vcnt > 0.0 {
			val /= vcnt
		}

		is_act := pk > 0.14 && pk > val + 0.11
		if is_act {
			act += 1
			if h <= 6 do low_act += 1
			if h <= 3 do low3_act += 1
			if fh <= 1180.0 do low_band_act += 1
			if h <= 2 && pk > 0.26 && pk > val + 0.19 do h12_strong = true
			if prev_h_act do consec_pairs += 1
		}
		prev_h_act = is_act
		wh    := 1.0 / math.pow(f64(h), 0.30)
		sum   += wh * (pk - 0.85 * val)
		w_tot += wh * wh
	}
	h12_ok := h12_strong && low3_act >= 2
	// Accept EITHER wideband voice (h=1 or h=2 chest fundamental + >=3 low harmonics) OR
	// real CB SSB crystal/DSP TX-filtered voice (<350 Hz cut off -> h=3..12 consecutive harmonic comb with |df| <= 0.36*F0)!
	cb_tx_cut_ok := cand_f0 <= 165.0 && math.abs(df) <= math.min(42.0, 0.36 * cand_f0) &&
	                low_band_act >= 4 && consec_pairs >= 3 && act >= 5
	if !(h12_ok && low_act >= 3) && !cb_tx_cut_ok {
		return -0.25, 0, false
	}
	prior := 1.0 - 0.03 * ((df / 75.0) * (df / 75.0))
	return (sum / math.sqrt(w_tot + 1.5)) * prior, act, h12_ok
}

// Process one 1024-sample 48 kHz demodulated block and return (delta_clar_hz, should_adjust)
auto_clarifier_step :: proc(
	ac      : ^Auto_Clarifier_State,
	dm      : ^Demodulator_State,
	cf      : ^Channel_Selectivity_Filter,
	aud_blk : []f64,
	mode    : Modulation_Mode,
	enabled : bool,
) -> (delta_hz: f64, adjust: bool) {
	// Always keep SSB sliding audio ring warm for instant response when toggled on
	n_in := len(aud_blk)
	if n_in > 0 && n_in <= AC_FFT_N {
		copy(ac.audio_ring[0 : AC_FFT_N - n_in], ac.audio_ring[n_in : AC_FFT_N])
		copy(ac.audio_ring[AC_FFT_N - n_in : AC_FFT_N], aud_blk)
	}

	if !enabled {
		ac.locked     = false
		ac.hold_ticks = 0
		return 0.0, false
	}

	if ac.hold_ticks > 0 {
		ac.hold_ticks -= 1
		if ac.hold_ticks == 0 do ac.locked = false
	}

	// Wait out decimator + 2048-pt overlap-save pipeline delay (~42-64 ms) after an NCO step
	if ac.settle_blocks > 0 {
		ac.settle_blocks -= 1
		return 0.0, false
	}

	ac.blk_counter += 1

	switch mode {
	case .AM:
		if (ac.blk_counter & 3) != 0 do return 0.0, false

		// 1. Fine Type-II Synchronous AM PLL lock
		if dm.sam_lock_metric >= 0.55 && cf.snr_db >= 1.5 {
			err_hz := dm.carrier_off_hz
			ac.est_off_hz = err_hz
			if math.abs(err_hz) <= 0.40 {
				ac.locked     = true
				ac.hold_ticks = 30
				return 0.0, false
			}
			step_hz := clamp(0.78 * err_hz, -500.0, +500.0)
			dm.sam_freq_rad -= (2.0 * math.PI * step_hz) / BASEBAND_RATE_HZ
			ac.settle_blocks = 2
			ac.locked        = math.abs(err_hz) < 5.0
			ac.hold_ticks    = 25
			return step_hz, true
		}

		// 2. Wide coarse spectral carrier pull-in (+-2484 Hz) when carrier is outside SAM PLL lock range
		if cf.coarse_peak_valid && math.abs(cf.coarse_peak_hz) > 900.0 {
			err_hz := cf.coarse_peak_hz
			ac.est_off_hz    = err_hz
			step_hz         := clamp(0.88 * err_hz, -2400.0, +2400.0)
			dm.sam_freq_rad  = 0.0
			ac.settle_blocks = 3
			return step_hz, true
		}
		return 0.0, false

	case .FM:
		if cf.snr_db < 4.0 {
			ac.fm_valid_cnt = 0
			return 0.0, false
		}
		// Coarse pull-in if FM station is far off-center (> 900 Hz)
		if cf.coarse_peak_valid && math.abs(cf.coarse_peak_hz) > 900.0 && ac.fm_valid_cnt < 3 {
			step_hz             := clamp(0.82 * cf.coarse_peak_hz, -2400.0, +2400.0)
			dm.fm_dc_avg         = 0.0
			dm.fm_pll_freq_rad   = 0.0
			ac.fm_off_ema        = 0.0
			ac.fm_valid_cnt      = 0
			ac.settle_blocks     = 3
			return step_hz, true
		}

		ac.fm_off_ema    = (0.82 * ac.fm_off_ema + 0.18 * dm.carrier_off_hz) if ac.fm_valid_cnt > 0 else dm.carrier_off_hz
		ac.fm_valid_cnt += 1
		if ac.fm_valid_cnt < 8 do return 0.0, false

		err_hz := ac.fm_off_ema
		ac.est_off_hz   = err_hz
		ac.fm_valid_cnt = 4
		if math.abs(err_hz) <= 2.0 {
			ac.locked     = true
			ac.hold_ticks = 30
			return 0.0, false
		}
		step_hz := clamp(0.68 * err_hz, -450.0, +450.0)
		rad_adj := (2.0 * math.PI * step_hz) / BASEBAND_RATE_HZ
		dm.fm_dc_avg       -= rad_adj
		dm.fm_pll_freq_rad -= rad_adj
		ac.fm_off_ema      -= step_hz
		ac.fm_valid_cnt     = 0
		ac.settle_blocks    = 3
		ac.locked           = math.abs(err_hz) < 8.0
		ac.hold_ticks       = 25
		return step_hz, true

	case .LSB, .USB:
		// Spectral accumulation on every block; harmonic mistune search once >= 3 pure 4096-pt frames accumulate
		for i in 0 ..< AC_FFT_N {
			ac.work_fft[i] = complex(ac.audio_ring[i] * ac.window[i], 0.0)
		}
		fft_forward(&ac.fft_plan, ac.work_fft[:])
		ac.welch_frames += 1

		for k in 0 ..< AC_BINS {
			p   := math.max(c64_abs2(ac.work_fft[k]), 1e-24)
			mag := math.sqrt(p)
			if ac.welch_frames == 1 || (ac.welch_frames == 4 && ac.blk_counter <= 6) {
				ac.welch_psd[k] = p
				ac.tonal_mag[k] = mag
				ac.tonal_pow[k] = p
			} else {
				w_steady := 0.22 if cf.snr_db < 0.0 else 0.26
				w_a      := 0.38 if ac.welch_frames <= 6 else w_steady
				t_a      := 0.22 if ac.welch_frames <= 7 else 0.12
				ac.welch_psd[k] = (1.0 - w_a) * ac.welch_psd[k] + w_a * p
				if p > 12.0 * ac.tonal_pow[k] {
					ac.tonal_mag[k] = mag
					ac.tonal_pow[k] = p
				} else {
					ac.tonal_mag[k] = (1.0 - t_a) * ac.tonal_mag[k] + t_a * mag
					ac.tonal_pow[k] = (1.0 - t_a) * ac.tonal_pow[k] + t_a * p
				}
			}
		}

		if ac.welch_frames < 6 || (ac.locked && (ac.blk_counter & 1) != 0) {
			dm.carrier_off_hz = ac.est_off_hz
			return 0.0, false
		}

		bin_hz     := AUDIO_SAMPLE_RATE_HZ / f64(AC_FFT_N) // 11.71875 Hz/bin
		inv_bin_hz := 1.0 / bin_hz
		f_lo       := 122.0
		f_hi       := 2650.0
		k_lo       := clamp(int(math.floor(f_lo * inv_bin_hz)), 4, AC_BINS - 12)
		k_hi       := clamp(int(math.ceil(f_hi * inv_bin_hz)), k_lo + 20, AC_BINS - 12)

		nf_k_lo := clamp(int(math.ceil(175.0 * inv_bin_hz)), k_lo, k_hi - 16)
		nf_k_hi := clamp(int(math.floor(2520.0 * inv_bin_hz)), nf_k_lo + 16, k_hi)
		for k in k_lo ..= k_hi {
			ac.noise_psd[k] = math.max(local_quantile_noise_floor(ac.welch_psd[:], k, 28, nf_k_lo, nf_k_hi, 44, 1.095, bin_hz), 1e-24)
		}

		// Mask isolated constant-amplitude heterodyne whistles / SMPS birdies before comb & peak extraction
		// so strong or weak unmodulated carriers (e.g. Ch 34 1007.8 Hz) never skew sub-noise SSB zero-beating!
		eff_psd : [AC_BINS]f64
		for k in k_lo ..= k_hi {
			eff_psd[k] = ac.welch_psd[k]
		}
		if ac.welch_frames >= 5 {
			for k in (k_lo + 1) ..= (k_hi - 1) {
				n_flr     := ac.noise_psd[k]
				constancy := (ac.tonal_mag[k] * ac.tonal_mag[k]) / math.max(ac.tonal_pow[k], 1e-24)
				is_pk     := ac.welch_psd[k] >= ac.welch_psd[k - 1] && ac.welch_psd[k] >= ac.welch_psd[k + 1]
				sharp_spur := false
				if is_pk && constancy > 0.82 && ac.welch_psd[k] > 1.80 * n_flr {
					nb_out := 0.0
					nb_cnt := 0
					for dk in -5 ..= 5 {
						if math.abs(dk) <= 2 do continue
						kk := k + dk
						if kk >= k_lo && kk <= k_hi {
							nb_out += ac.welch_psd[kk]
							nb_cnt += 1
						}
					}
					if nb_cnt > 0 && ac.welch_psd[k] > 1.90 * (nb_out / f64(nb_cnt)) {
						sharp_spur = true
					}
				}
				if is_pk && ((constancy > 0.90 && ac.welch_psd[k] > 6.5 * n_flr) || sharp_spur) {
					if !is_vocal_harmonic_member(ac.welch_psd[:], ac.welch_psd[:], ac.noise_psd[:], k, bin_hz, k_lo, k_hi) {
						rad := 2 if ac.welch_psd[k] > 4.5 * n_flr else 1
						for dk in -rad ..= rad {
							kk := k + dk
							if kk >= k_lo && kk <= k_hi {
								eff_psd[kk] = min(eff_psd[kk], ac.noise_psd[kk])
							}
						}
					}
				}
			}
		}

		sum_sig    := 0.0
		sum_flr    := 0.0
		voiced_sig := 0.0
		voiced_flr := 0.0
		inst_v_sig := 0.0
		exc        : [AC_BINS]f64
		for k in k_lo ..= k_hi {
			n_flr    := ac.noise_psd[k]
			eff_p    := eff_psd[k]
			fk       := f64(k) * bin_hz
			if k >= nf_k_lo && k <= nf_k_hi {
				sum_sig += eff_p
				sum_flr += n_flr
			}
			if fk >= 175.0 && fk <= 1250.0 {
				voiced_sig += eff_p
				voiced_flr += n_flr
				inst_p     := math.max(c64_abs2(ac.work_fft[k]), 1e-24)
				if eff_p < ac.welch_psd[k] do inst_p = min(inst_p, n_flr)
				inst_v_sig += inst_p
			}
			exc[k] = math.ln(math.max(eff_p / n_flr, 0.55))
		}

		sum_ratio    := sum_sig    / math.max(sum_flr,    1e-24)
		voiced_ratio := voiced_sig / math.max(voiced_flr, 1e-24)
		inst_v_ratio := inst_v_sig / math.max(voiced_flr, 1e-24)
		if (sum_ratio < 1.035 && voiced_ratio < 1.055) || inst_v_ratio < 1.035 {
			if ac.ssb_consist > 0 && (ac.blk_counter & 7) == 0 {
				ac.ssb_consist -= 1
			}
			dm.carrier_off_hz = ac.est_off_hz
			return 0.0, false
		}

		// Extract sub-bin log-parabolic spectral peaks across [122 .. 2650 Hz] for high-SNR WLS Stage C
		MAX_AC_PEAKS :: 64
		pk_freq  : [MAX_AC_PEAKS]f64
		pk_wt    : [MAX_AC_PEAKS]f64
		n_peaks  := 0
		pk_min_s := 1.62 if cf.snr_db < 2.0 else 2.55
		for k in (k_lo + 1) ..= (k_hi - 1) {
			p0 := eff_psd[k]
			if p0 > eff_psd[k - 1] && p0 >= eff_psd[k + 1] {
				snr_k := p0 / ac.noise_psd[k]
				if snr_k >= pk_min_s && n_peaks < MAX_AC_PEAKS {
					lm    := 0.5 * math.ln(math.max(eff_psd[k - 1], 1e-30))
					l0    := 0.5 * math.ln(math.max(p0,             1e-30))
					lp    := 0.5 * math.ln(math.max(eff_psd[k + 1], 1e-30))
					denom := lm - 2.0 * l0 + lp
					delta := clamp(0.5 * (lm - lp) / denom, -0.5, 0.5) if denom < -1e-12 else 0.0
					pk_freq[n_peaks] = (f64(k) + delta) * bin_hz
					pk_wt[n_peaks]   = math.ln(snr_k)
					n_peaks         += 1
				}
			}
		}

		// Stage A: Coarse 2D log-comb (F0, df_audio) search across F0 = 86..252 Hz (step 2 Hz), df = -78..+78 Hz (step 6 Hz)
		// Coherently sums up to 16 harmonics so both -10..-14 dB sub-noise-floor voices & <350 Hz CB TX-cut voices lock cleanly!
		N_F0_GRID :: 84
		f0_prof   : [N_F0_GRID]f64
		best_sc   := -1.0e9
		best_f0   := 140.0
		best_df   := 0.0
		best_act  := 0
		best_h12  := false

		for i_f in 0 ..< N_F0_GRID {
			cand_f0 := 86.0 + f64(i_f) * 2.0
			max_df  := math.min(78.0, 0.45 * cand_f0)
			row_max := -1.0e9
			row_df  := 0.0
			row_act := 0
			row_h12 := false
			for df_c := -78.0; df_c <= 78.0; df_c += 6.0 {
				if math.abs(df_c) > max_df do continue
				sc, act, h12 := ac_eval_harmonic_comb(exc[:], cand_f0, df_c, f_lo, f_hi, k_lo, k_hi, inv_bin_hz)
				if sc > row_max {
					row_max = sc
					row_df  = df_c
					row_act = act
					row_h12 = h12
				}
			}
			f0_prof[i_f] = row_max
			if row_max > best_sc {
				best_sc  = row_max
				best_f0  = cand_f0
				best_df  = row_df
				best_act = row_act
				best_h12 = row_h12
			}
		}

		mean_p := 0.0
		for v in f0_prof do mean_p += v
		mean_p /= f64(N_F0_GRID)

		var_p := 1e-9
		for v in f0_prof {
			d := v - mean_p
			var_p += d * d
		}
		std_p   := math.sqrt(var_p / f64(N_F0_GRID))
		z_score := (best_sc - mean_p) / math.max(std_p, 1e-6)

		// Octave (0.5 * F0, checking BOTH even-harmonic df & odd-harmonic df +- half_f0!) & 3/2 disambiguation!
		if best_f0 >= 170.0 {
			half_f0 := 0.5 * best_f0
			h_dfs   := [3]f64{best_df, best_df + half_f0, best_df - half_f0}
			for hdf in h_dfs {
				if math.abs(hdf) <= math.min(78.0, 0.45 * half_f0) {
					sc_half, act_h, h12_h := ac_eval_harmonic_comb(exc[:], half_f0, hdf, f_lo, f_hi, k_lo, k_hi, inv_bin_hz)
					if sc_half > mean_p + 0.45 * std_p && sc_half >= 0.44 * best_sc && act_h >= 5 {
						best_f0  = half_f0
						best_df  = hdf
						best_act = act_h
						best_h12 = h12_h
						best_sc  = max(best_sc, sc_half)
						break
					}
				}
			}
		}
		if best_f0 >= 180.0 {
			f_23 := (2.0 / 3.0) * best_f0
			df_opts := [2]f64{best_df - 0.5 * f_23, best_df + 0.5 * f_23}
			for df_cand in df_opts {
				if math.abs(df_cand) <= math.min(78.0, 0.46 * f_23) {
					sc_23, act_23, h12_23 := ac_eval_harmonic_comb(exc[:], f_23, df_cand, f_lo, f_hi, k_lo, k_hi, inv_bin_hz)
					if sc_23 > 1.06 * best_sc && act_23 >= 5 {
						best_f0  = f_23
						best_df  = df_cand
						best_act = act_23
						best_sc  = sc_23
						best_h12 = h12_23
						break
					}
				}
			}
		}

		if best_act < 5 || z_score < 2.38 || best_sc < 0.118 {
			dm.carrier_off_hz = ac.est_off_hz
			return 0.0, false
		}

		// Stage B: Fine 0.5 Hz 2D local refinement around (best_f0, best_df)
		fine_sc  := -1.0e9
		fine_f0  := best_f0
		fine_df  := best_df
		fine_act := best_act
		fine_h12 := best_h12
		for ff := best_f0 - 2.5; ff <= best_f0 + 2.5; ff += 0.5 {
			lim_df := math.min(82.0, 0.46 * ff)
			for fd := best_df - 6.5; fd <= best_df + 6.5; fd += 0.5 {
				if math.abs(fd) > lim_df do continue
				sc, act_f, h12_f := ac_eval_harmonic_comb(exc[:], ff, fd, f_lo, f_hi, k_lo, k_hi, inv_bin_hz)
				if sc > fine_sc {
					fine_sc  = sc
					fine_f0  = ff
					fine_df  = fd
					fine_act = act_f
					fine_h12 = h12_f
				}
			}
		}

		// Stage C: Exact f64 Weighted Least-Squares Harmonic Regression f_h = h * F0 + df_audio
		// Decouples F0 slope from h=0 intercept df_audio to sub-Hz precision when >= 5 peaks stand out,
		// supporting BOTH full-band speech (mn_h <= 2) AND real CB <350 Hz TX-filtered speech (mn_h <= 5)!
		reg_f0      := fine_f0
		reg_df      := fine_df
		wls_ok      := false
		wls_has_h12 := fine_h12
		req_cons    := 2

		if n_peaks >= 5 && (voiced_ratio >= 1.18 || sum_ratio >= 1.14) && z_score >= 2.72 && fine_sc >= 0.135 {
			MAX_H_IDX :: 24
			h_used : [MAX_H_IDX + 1]bool
			h_freq : [MAX_H_IDX + 1]f64
			h_wgt  : [MAX_H_IDX + 1]f64

			tol_hz := math.min(13.5, 0.13 * fine_f0)
			for m in 0 ..< n_peaks {
				hm := int(math.round((pk_freq[m] - fine_df) / fine_f0))
				if hm < 1 || hm > MAX_H_IDX do continue
				pred := f64(hm) * fine_f0 + fine_df
				if math.abs(pk_freq[m] - pred) <= tol_hz {
					if !h_used[hm] || pk_wt[m] > h_wgt[hm] {
						h_used[hm] = true
						h_freq[hm] = pk_freq[m]
						h_wgt[hm]  = pk_wt[m]
					}
				}
			}

			sw_fin     := 0.0
			fit_ok     := true
			mn_h_final := MAX_H_IDX + 1
			consec_wls := 0
			for pass in 0 ..< 2 {
				sw, sh, shh, sf, shf := 0.0, 0.0, 0.0, 0.0, 0.0
				cnt      := 0
				mn_h     := MAX_H_IDX + 1
				mx_h     := 0
				cons_cnt := 0
				prev_u   := false
				for hm in 1 ..= MAX_H_IDX {
					if !h_used[hm] {
						prev_u = false
						continue
					}
					if pass == 1 && math.abs(h_freq[hm] - (f64(hm) * reg_f0 + reg_df)) > 4.8 {
						h_used[hm] = false
						prev_u = false
						continue
					}
					if prev_u do cons_cnt += 1
					prev_u = true
					w   := h_wgt[hm]
					fh  := f64(hm)
					sw  += w
					sh  += w * fh
					shh += w * fh * fh
					sf  += w * h_freq[hm]
					shf += w * fh * h_freq[hm]
					cnt += 1
					if hm < mn_h do mn_h = hm
					if hm > mx_h do mx_h = hm
				}
				det := sw * shh - sh * sh
				anchor_ok := mn_h <= 2 || (mn_h <= 5 && cons_cnt >= 3 && fine_f0 <= 168.0)
				if cnt < 5 || !anchor_ok || (mx_h - mn_h) < 4 || det <= 1e-9 {
					fit_ok = false
					break
				}
				reg_f0     = (sw * shf - sh * sf) / det
				reg_df     = (shh * sf - sh * shf) / det
				sw_fin     = sw
				mn_h_final = mn_h
				consec_wls = cons_cnt
			}

			if fit_ok {
				w_err2 := 0.0
				for hm in 1 ..= MAX_H_IDX {
					if !h_used[hm] do continue
					e := h_freq[hm] - (f64(hm) * reg_f0 + reg_df)
					w_err2 += h_wgt[hm] * e * e
				}
				rms_resid := math.sqrt(w_err2 / math.max(sw_fin, 1e-12))
				has_chest_anchor := mn_h_final <= 2 && h_freq[mn_h_final] <= 315.0
				if has_chest_anchor {
					if voiced_ratio >= 1.12 && inst_v_ratio >= 1.05 && reg_f0 >= 85.0 && reg_f0 <= 260.0 &&
					   math.abs(reg_df) <= 0.47 * reg_f0 && rms_resid <= 4.5 {
						wls_ok      = true
						wls_has_h12 = true
					}
				} else {
					// CB <350 Hz TX-filtered speech: lowest peak > 315 Hz -> enforce F0 <= 165 Hz, |df| <= 0.36*F0 & voiced excess!
					if voiced_ratio >= 1.22 && inst_v_ratio >= 1.10 && reg_f0 >= 85.0 && reg_f0 <= 165.0 && consec_wls >= 3 &&
					   math.abs(reg_df) <= math.min(42.0, 0.36 * reg_f0) && rms_resid <= 3.0 {
						wls_ok      = true
						wls_has_h12 = false
						req_cons    = 3
					}
				}
			}
		}

		if !wls_ok {
			// Sub-noise-floor coherent 2D comb lock gate: requires strong h=1/h=2 anchor + real voiced band excess
			// so empty static / -15..-20 dB noise NEVER triggers false clarifier steps while -10..-12 dB speech locks!
			sc_zero, _, _ := ac_eval_harmonic_comb(exc[:], fine_f0, 0.0, f_lo, f_hi, k_lo, k_hi, inv_bin_hz)
			if fine_h12 && voiced_ratio >= 1.085 && inst_v_ratio >= 1.045 && cf.snr_db >= -11.5 && cf.snr_db < 3.0 &&
			   fine_act >= 6 && z_score >= 3.25 && fine_sc >= 0.165 &&
			   (math.abs(fine_df) <= 2.5 || fine_sc > 1.24 * math.max(sc_zero, 0.02)) {
				reg_f0      = fine_f0
				reg_df      = fine_df
				wls_has_h12 = true
				req_cons    = 3
			} else {
				dm.carrier_off_hz = ac.est_off_hz
				return 0.0, false
			}
		}

		// Convert demodulated audio harmonic shift to RF frequency error:
		//   USB: f_audio = h*F0 + (f_TX - f_RX) -> rf_err = +reg_df
		//   LSB: f_audio = h*F0 - (f_TX - f_RX) -> rf_err = -reg_df
		rf_err_hz := reg_df if mode == .USB else -reg_df
		ac.est_off_hz     = rf_err_hz
		dm.carrier_off_hz = rf_err_hz

		min_deadband := 0.9 if wls_has_h12 else 2.2
		if math.abs(rf_err_hz) <= min_deadband {
			ac.locked       = true
			ac.hold_ticks   = 40
			ac.prev_ssb_err = rf_err_hz
			ac.ssb_consist  = req_cons
			return 0.0, false
		}

		cons_tol := 5.5 if req_cons >= 3 else 8.0
		if ac.ssb_consist == 0 || math.abs(rf_err_hz - ac.prev_ssb_err) <= cons_tol {
			ac.ssb_consist += 1
		} else {
			ac.ssb_consist = 1
		}
		ac.prev_ssb_err = rf_err_hz

		if ac.ssb_consist < req_cons {
			return 0.0, false
		}

		step_gain := 0.90 if wls_has_h12 else 0.78
		step_hz   := clamp(step_gain * rf_err_hz, -85.0, +85.0)
		if !wls_has_h12 {
			// Bound cumulative un-anchored (CB <350 Hz TX-filtered) offset within +-45 Hz crystal tolerance!
			clamped_tot := clamp(ac.ssb_no_h12_adj + step_hz, -45.0, +45.0)
			step_hz      = clamped_tot - ac.ssb_no_h12_adj
			if math.abs(step_hz) < 2.0 {
				ac.locked     = true
				ac.hold_ticks = 45
				return 0.0, false
			}
			ac.ssb_no_h12_adj = clamped_tot
		} else {
			ac.ssb_no_h12_adj = 0.0
		}

		if math.abs(step_hz) >= 1.5 {
			// Flush 100% of scratch + overlap-save (2 blocks) + 4096-sample audio_ring (4 blocks) = 6 blocks!
			ac.welch_frames  = 0
			ac.settle_blocks = 6
			ac.ssb_consist   = 0
		}
		ac.locked     = math.abs(rf_err_hz) < 4.5
		ac.hold_ticks = 40
		return step_hz, true
	}

	return 0.0, false
}

