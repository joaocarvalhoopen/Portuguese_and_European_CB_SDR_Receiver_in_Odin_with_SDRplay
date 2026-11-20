// Audio Noise Filters Level 1 (Simple) and Level 2 (Moderate) in pure 64-bit float (f64).
// Tailored specifically to each of the 3 CB bands (AM, FM, SSB) to remove indoor household
// EMI (50 Hz mains harmonics, SMPS switching birdies, appliance clicks) and channel noise.

package main

import "core:math"

// ---------------------------------------------------------------------------
// Direct-Form II Transposed f64 Biquad Filter
// ---------------------------------------------------------------------------

Biquad_f64 :: struct {
	b0, b1, b2 : f64,
	a1, a2     : f64,
	z1, z2     : f64,
}

biquad_reset :: proc(bq: ^Biquad_f64) {
	bq.z1 = 0.0
	bq.z2 = 0.0
}

biquad_step :: #force_inline proc(bq: ^Biquad_f64, x: f64) -> f64 {
	y     := bq.b0 * x + bq.z1
	bq.z1  = bq.b1 * x - bq.a1 * y + bq.z2
	bq.z2  = bq.b2 * x - bq.a2 * y
	return y
}

biquad_design_lowpass :: proc(bq: ^Biquad_f64, fc_hz, q, fs_hz: f64) {
	w0     := (2.0 * math.PI * fc_hz) / fs_hz
	cosw0  := math.cos(w0)
	alpha  := math.sin(w0) / (2.0 * q)
	inv_a0 := 1.0 / (1.0 + alpha)

	bq.b0 = ((1.0 - cosw0) * 0.5) * inv_a0
	bq.b1 = (1.0 - cosw0) * inv_a0
	bq.b2 = bq.b0
	bq.a1 = (-2.0 * cosw0) * inv_a0
	bq.a2 = (1.0 - alpha) * inv_a0
}

biquad_design_highpass :: proc(bq: ^Biquad_f64, fc_hz, q, fs_hz: f64) {
	w0     := (2.0 * math.PI * fc_hz) / fs_hz
	cosw0  := math.cos(w0)
	alpha  := math.sin(w0) / (2.0 * q)
	inv_a0 := 1.0 / (1.0 + alpha)

	bq.b0 = ((1.0 + cosw0) * 0.5) * inv_a0
	bq.b1 = -(1.0 + cosw0) * inv_a0
	bq.b2 = bq.b0
	bq.a1 = (-2.0 * cosw0) * inv_a0
	bq.a2 = (1.0 - alpha) * inv_a0
}

biquad_design_notch :: proc(bq: ^Biquad_f64, fc_hz, q, fs_hz: f64) {
	w0     := (2.0 * math.PI * fc_hz) / fs_hz
	cosw0  := math.cos(w0)
	alpha  := math.sin(w0) / (2.0 * q)
	inv_a0 := 1.0 / (1.0 + alpha)

	bq.b0 = 1.0 * inv_a0
	bq.b1 = (-2.0 * cosw0) * inv_a0
	bq.b2 = 1.0 * inv_a0
	bq.a1 = (-2.0 * cosw0) * inv_a0
	bq.a2 = (1.0 - alpha) * inv_a0
}

biquad_design_peaking :: proc(bq: ^Biquad_f64, fc_hz, q, gain_db, fs_hz: f64) {
	amp    := math.pow(10.0, gain_db / 40.0)
	w0     := (2.0 * math.PI * fc_hz) / fs_hz
	cosw0  := math.cos(w0)
	alpha  := math.sin(w0) / (2.0 * q)
	inv_a0 := 1.0 / (1.0 + alpha / amp)

	bq.b0 = (1.0 + alpha * amp) * inv_a0
	bq.b1 = (-2.0 * cosw0) * inv_a0
	bq.b2 = (1.0 - alpha * amp) * inv_a0
	bq.a1 = (-2.0 * cosw0) * inv_a0
	bq.a2 = (1.0 - alpha / amp) * inv_a0
}

// ---------------------------------------------------------------------------
// Reusable Pre-Conditioning: Lookahead Audio Impulse De-Clicker + 50 Hz Mains Comb
// ---------------------------------------------------------------------------

Pre_Impulse_Hum_Cleaner :: struct {
	ring        : [16]f64,
	pos         : int,
	curv_floor  : f64,
	initialized : bool,
	hum_comb    : [5]Biquad_f64, // 50, 100, 150, 200, 250 Hz European power-grid harmonic notches
}

pre_cleaner_init :: proc(pc: ^Pre_Impulse_Hum_Cleaner) {
	pc^ = {}
	pc.curv_floor = 1e-4
	freqs := [5]f64{50.0, 100.0, 150.0, 200.0, 250.0}
	qs    := [5]f64{44.0, 46.0,  48.0,  44.0,  44.0}
	for i in 0 ..< 5 {
		biquad_design_notch(&pc.hum_comb[i], freqs[i], qs[i], AUDIO_SAMPLE_RATE_HZ)
	}
}

pre_cleaner_step :: #force_inline proc(pc: ^Pre_Impulse_Hum_Cleaner, raw: f64, thresh_mult: f64, is_ssb: bool = false) -> f64 {
	_ = is_ssb
	// 1. Curvature-based lookahead impulse blanker MUST run BEFORE any IIR notch filters so sharp
	// indoor triac/LED/switch spikes are interpolated cleanly without causing IIR filter ringing!
	// Second-difference |x[n] - 0.5*(x[n-2] + x[n+2])| is naturally blind to 50/100 Hz mains hum.
	p := pc.pos
	pc.ring[p] = raw
	pc.pos     = (p + 1) & 15

	c_idx  := (p - 4) & 15
	l_idx  := (p - 5) & 15
	r_idx  := (p - 3) & 15
	center := pc.ring[c_idx]
	left   := pc.ring[l_idx]
	right  := pc.ring[r_idx]
	interp := 0.5 * (left + right)
	dl     := math.abs(center - left)
	dr     := math.abs(center - right)
	avg_d  := 0.5 * (dl + dr)

	if !pc.initialized && avg_d > 0.0 {
		pc.curv_floor  = avg_d
		pc.initialized = true
	} else {
		thresh_est := math.max(pc.curv_floor, 1e-9) * thresh_mult
		// Only update background slope floor on non-spike samples!
		if !(dl > thresh_est && dr > thresh_est) {
			pc.curv_floor = 0.992 * pc.curv_floor + 0.008 * avg_d
		}
	}

	cleaned := center
	thresh  := math.max(pc.curv_floor, 1e-9) * (thresh_mult * 0.78)
	if pc.initialized && dl > thresh && dr > thresh {
		cleaned = interp
		pc.ring[c_idx] = interp
	}

	// 2. Remove 50/100/150/200/250 Hz European power-line & bridge-rectifier harmonics cleanly (high Q=44..48)
	#unroll for k in 0 ..< 5 {
		cleaned = biquad_step(&pc.hum_comb[k], cleaned)
	}
	return cleaned
}

// ---------------------------------------------------------------------------
// FILTER LEVEL 1: Simple 6th-Order Butterworth Voice Conditioning + 50-300 Hz Mains Comb
// ---------------------------------------------------------------------------

Simple_Band_Chain :: struct {
	cleaner      : Pre_Impulse_Hum_Cleaner,
	notch300     : Biquad_f64,
	hp1, hp2, hp3: Biquad_f64, // 6th-order Butterworth high-pass (-36 dB/oct)
	lp1, lp2, lp3: Biquad_f64, // 6th-order Butterworth low-pass  (-36 dB/oct)
	presence     : Biquad_f64, // Band-specific articulation / FM triangular de-hiss EQ
	gain_scale   : f64,        // Calibrated passband unity-speech / net-noise-cut trim
}

Simple_Filter_State :: struct {
	chains : [3]Simple_Band_Chain, // AM_Band, FM_Band, SSB_Band
}

@(private)
init_simple_chain :: proc(ch: ^Simple_Band_Chain, hp_hz, lp_hz, pres_hz, pres_q, pres_db, gain_scale: f64) {
	ch^ = {}
	pre_cleaner_init(&ch.cleaner)
	biquad_design_notch(&ch.notch300, 300.0, 34.0, AUDIO_SAMPLE_RATE_HZ)
	// Exact 6th-order Butterworth cascade Q factors: 1 / (2 * cos((2k-1)*pi/12))
	biquad_design_highpass(&ch.hp1, hp_hz, 0.5176380902050415, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_highpass(&ch.hp2, hp_hz, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_highpass(&ch.hp3, hp_hz, 1.9318516525781366, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&ch.lp1,  lp_hz, 0.5176380902050415, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&ch.lp2,  lp_hz, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&ch.lp3,  lp_hz, 1.9318516525781366, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_peaking(&ch.presence, pres_hz, pres_q, pres_db, AUDIO_SAMPLE_RATE_HZ)
	ch.gain_scale = gain_scale
}

simple_filter_init :: proc(sf: ^Simple_Filter_State) {
	// AM: 135 - 2850 Hz 6th-order voice band (cuts >2.85 kHz wideband AM hiss by -36 dB/oct + gentle high-hiss trim)
	init_simple_chain(&sf.chains[int(CB_Band_Family.AM_Band)],  135.0, 2850.0, 2550.0, 1.10, -2.0, 0.90)
	// FM: 155 - 2650 Hz 6th-order voice band + strong f^2 discriminator hiss de-emphasis above 2.1 kHz
	init_simple_chain(&sf.chains[int(CB_Band_Family.FM_Band)],  155.0, 2650.0, 2350.0, 0.95, -4.5, 0.90)
	// SSB (LSB/USB): Tight 138 - 2380 Hz DX communications voice passband + formant clarity without broadband boost
	init_simple_chain(&sf.chains[int(CB_Band_Family.SSB_Band)], 138.0, 2380.0, 1480.0, 1.45, +1.2, 0.84)
}

simple_filter_process :: proc(sf: ^Simple_Filter_State, buf: []f64, mode: Modulation_Mode) {
	band := mode_to_band_family(mode)
	ch   := &sf.chains[int(band)]

	spike_thresh := 3.6
	if band == .SSB_Band do spike_thresh = 3.3

	g := ch.gain_scale
	for i in 0 ..< len(buf) {
		x := pre_cleaner_step(&ch.cleaner, buf[i], spike_thresh)
		x  = biquad_step(&ch.notch300, x)
		x  = biquad_step(&ch.hp1, x)
		x  = biquad_step(&ch.hp2, x)
		x  = biquad_step(&ch.hp3, x)
		x  = biquad_step(&ch.lp1, x)
		x  = biquad_step(&ch.lp2, x)
		x  = biquad_step(&ch.lp3, x)
		x  = biquad_step(&ch.presence, x)
		buf[i] = x * g
	}
}

// ---------------------------------------------------------------------------
// FILTER LEVEL 2: Moderate Time-Domain Adaptive DSP (NLMS + Impulse Blanker + Expander)
// ---------------------------------------------------------------------------

NLMS_TAPS     :: 64
NLMS_HIST_LEN :: 256
NLMS_HIST_MSK :: NLMS_HIST_LEN - 1

Moderate_Filter_State :: struct {
	pre_simple   : Simple_Filter_State,

	// Stage A: Long-delay NLMS Adaptive Narrowband Tone / Heterodyne / SMPS Canceller
	anc_hist     : [NLMS_HIST_LEN]f64,
	anc_weights  : [NLMS_TAPS]f64,
	anc_pos      : int,

	// Stage B: Short-delay NLMS Quasi-Periodic Speech Line Enhancer (ALE)
	ale_hist     : [NLMS_HIST_LEN]f64,
	ale_weights  : [NLMS_TAPS]f64,
	ale_pos      : int,

	// Soft-knee downward voice expander state (smoothed to prevent <130 Hz envelope intermodulation)
	exp_env_fast : f64,
	exp_floor    : f64,
	exp_gain_sm  : f64,

	// Post-NLMS / Post-Expander sub-vocal rumble & high-hiss clean-up filters per band
	post_hp      : [3]Biquad_f64,
	post_lp      : [3]Biquad_f64,
}

moderate_filter_init :: proc(mf: ^Moderate_Filter_State) {
	mf^ = {}
	simple_filter_init(&mf.pre_simple)
	mf.exp_floor   = 1e-4
	mf.exp_gain_sm = 0.50

	biquad_design_highpass(&mf.post_hp[int(CB_Band_Family.AM_Band)],  135.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&mf.post_lp[int(CB_Band_Family.AM_Band)],  2800.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)

	biquad_design_highpass(&mf.post_hp[int(CB_Band_Family.FM_Band)],  155.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&mf.post_lp[int(CB_Band_Family.FM_Band)],  2580.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)

	biquad_design_highpass(&mf.post_hp[int(CB_Band_Family.SSB_Band)], 140.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
	biquad_design_lowpass(&mf.post_lp[int(CB_Band_Family.SSB_Band)], 2320.0, 0.7071067811865476, AUDIO_SAMPLE_RATE_HZ)
}

nlms_step_f64 :: #force_inline proc(
	hist    : ^[NLMS_HIST_LEN]f64,
	weights : ^[NLMS_TAPS]f64,
	pos_ptr : ^int,
	x       : f64,
	delay   : int,
	mu      : f64,
	leak    : f64,
) -> (y_pred: f64, err: f64) {
	pos := pos_ptr^
	hist[pos] = x

	pred   := 0.0
	norm_x := 1e-10
	base   := (pos - delay) & NLMS_HIST_MSK

	for k in 0 ..< NLMS_TAPS {
		xk := hist[(base - k) & NLMS_HIST_MSK]
		pred   += weights[k] * xk
		norm_x += xk * xk
	}

	e    := x - pred
	step := (mu * e) / norm_x

	for k in 0 ..< NLMS_TAPS {
		xk := hist[(base - k) & NLMS_HIST_MSK]
		weights[k] = weights[k] * leak + step * xk
	}

	pos_ptr^ = (pos + 1) & NLMS_HIST_MSK
	return pred, e
}

moderate_filter_process :: proc(mf: ^Moderate_Filter_State, buf: []f64, mode: Modulation_Mode) {
	// Pre-clean through Level 1 (impulse blanker + 50 Hz house hum comb + 4th-order voice bandpass)
	simple_filter_process(&mf.pre_simple, buf, mode)

	band := mode_to_band_family(mode)

	// Stage A uses long delay (176 samples = 3.67 ms) with slow mu so ONLY unmodulated narrowband
	// continuous SMPS/heterodyne tones remain phase-predictable while speech formants pass cleanly.
	// Stage B uses short delay (24 samples = 0.50 ms) to reinforce voiced vocal-cord periodicity.
	anc_delay := 176
	anc_mu    := 0.025
	ale_delay := 24
	ale_mu    := 0.065
	ale_mix   := 0.55

	switch band {
	case .AM_Band:
		anc_delay = 168
		anc_mu    = 0.035 // Cancels continuous AM heterodyne carrier whistles & SMPS birdies
		ale_delay = 22
		ale_mu    = 0.060
		ale_mix   = 0.52
	case .FM_Band:
		anc_delay = 160
		anc_mu    = 0.025
		ale_delay = 20
		ale_mu    = 0.070 // Suppresses uncorrelated high-freq FM discriminator hiss
		ale_mix   = 0.58
	case .SSB_Band:
		anc_delay = 180
		anc_mu    = 0.030
		ale_delay = 26
		ale_mu    = 0.080 // Reinforces weak SSB vocal harmonics
		ale_mix   = 0.68
	}

	for i in 0 ..< len(buf) {
		clean_x := buf[i]

		// 1. Cancel continuous narrowband birdies/whistles
		_, tone_cancelled := nlms_step_f64(&mf.anc_hist, &mf.anc_weights, &mf.anc_pos, clean_x, anc_delay, anc_mu, 0.9997)

		// 2. Reinforce quasi-periodic human voice harmonics over uncorrelated broadband noise
		speech_pred, _ := nlms_step_f64(&mf.ale_hist, &mf.ale_weights, &mf.ale_pos, tone_cancelled, ale_delay, ale_mu, 0.9994)

		combined := (1.0 - ale_mix) * tone_cancelled + ale_mix * speech_pred

		// 3. Soft-knee syllabic downward expander
		ac := math.abs(combined)
		if ac > mf.exp_env_fast {
			mf.exp_env_fast = 0.985 * mf.exp_env_fast + 0.015 * ac
		} else {
			mf.exp_env_fast = 0.9992 * mf.exp_env_fast + 0.0008 * ac
		}
		if ac < mf.exp_floor {
			mf.exp_floor = 0.995 * mf.exp_floor + 0.005 * ac
		} else {
			mf.exp_floor = 0.99992 * mf.exp_floor + 0.00008 * math.min(ac, mf.exp_floor * 1.6 + 1e-9)
		}

		snr_local   := mf.exp_env_fast / math.max(mf.exp_floor * 1.85, 1e-10)
		target_gain := clamp((snr_local / (1.0 + snr_local)) * 1.35, 0.22, 1.0)
		if target_gain > mf.exp_gain_sm {
			mf.exp_gain_sm = 0.985 * mf.exp_gain_sm + 0.015 * target_gain
		} else {
			mf.exp_gain_sm = 0.996 * mf.exp_gain_sm + 0.004 * target_gain
		}

		y := combined * mf.exp_gain_sm
		y  = biquad_step(&mf.post_hp[int(band)], y)
		y  = biquad_step(&mf.post_lp[int(band)], y)
		buf[i] = y
	}
}
