// Pure Odin 64-bit Floating-Point (f64 / complex128) Radix-2 FFT, Window Functions,
// FIR Kernel Synthesis, and 2048-point Overlap-Save Complex Analytic Channel Filter.
// Implemented 100% from scratch in Odin without FFTW or external libraries.

package main

import "core:math"

FFT_MAX_N :: 4096

// Precomputed Radix-2 FFT plan for zero-allocation real-time f64 execution
FFT_Plan :: struct {
	n       : int,
	log2_n  : int,
	bit_rev : [FFT_MAX_N]u16,
	// Forward twiddles: exp(-j * 2*pi * k / N) for k in 0 ..< N/2
	twiddle : [FFT_MAX_N / 2]complex128,
}

fft_plan_init :: proc(plan: ^FFT_Plan, n: int) {
	assert(n >= 2 && n <= FFT_MAX_N && (n & (n - 1)) == 0, "FFT size must be power of two <= 4096")
	plan.n = n

	bits := 0
	tmp  := n
	for tmp > 1 {
		tmp >>= 1
		bits += 1
	}
	plan.log2_n = bits

	// Precompute bit-reversal table
	for i in 0 ..< n {
		rev := 0
		v   := i
		for _ in 0 ..< bits {
			rev = (rev << 1) | (v & 1)
			v >>= 1
		}
		plan.bit_rev[i] = u16(rev)
	}

	// Precompute high-precision f64 twiddle factors
	half := n / 2
	inv_n := (2.0 * math.PI) / f64(n)
	for k in 0 ..< half {
		angle := -inv_n * f64(k)
		plan.twiddle[k] = complex(math.cos(angle), math.sin(angle))
	}
}

// In-place forward Radix-2 Cooley-Tukey FFT in f64 (complex128)
fft_forward :: proc(plan: ^FFT_Plan, buf: []complex128) {
	n := plan.n
	assert(len(buf) >= n)

	// Bit-reversal permutation
	for i in 0 ..< n {
		j := int(plan.bit_rev[i])
		if i < j {
			buf[i], buf[j] = buf[j], buf[i]
		}
	}

	// Iterative butterfly stages
	half_size := 1
	step      := n / 2
	for _ in 0 ..< plan.log2_n {
		span := half_size * 2
		for base := 0; base < n; base += span {
			tw_idx := 0
			for k in 0 ..< half_size {
				w := plan.twiddle[tw_idx]
				u := buf[base + k]
				v := buf[base + k + half_size] * w
				buf[base + k]             = u + v
				buf[base + k + half_size] = u - v
				tw_idx += step
			}
		}
		half_size = span
		step >>= 1
	}
}

// In-place inverse Radix-2 IFFT in f64 (complex128), normalized by 1/N
fft_inverse :: proc(plan: ^FFT_Plan, buf: []complex128) {
	n := plan.n
	assert(len(buf) >= n)

	// Conjugate input
	for i in 0 ..< n {
		buf[i] = conj(buf[i])
	}

	fft_forward(plan, buf)

	// Conjugate and scale by 1/N in f64
	scale := 1.0 / f64(n)
	for i in 0 ..< n {
		buf[i] = complex(real(buf[i]) * scale, -imag(buf[i]) * scale)
	}
}

// ---------------------------------------------------------------------------
// Precision f64 DSP Window & Math Functions
// ---------------------------------------------------------------------------

// Normalized sinc(x) = sin(pi * x) / (pi * x) in f64
sinc_f64 :: #force_inline proc(x: f64) -> f64 {
	if math.abs(x) < 1e-12 {
		return 1.0
	}
	px := math.PI * x
	return math.sin(px) / px
}

// 4-term Blackman-Harris window (-92 dB sidelobes) in f64
blackman_harris_f64 :: #force_inline proc(n, length: int) -> f64 {
	if length <= 1 do return 1.0
	a0 :: 0.35875
	a1 :: 0.48829
	a2 :: 0.14128
	a3 :: 0.01168
	phase := (2.0 * math.PI * f64(n)) / f64(length - 1)
	return a0 - a1 * math.cos(phase) + a2 * math.cos(2.0 * phase) - a3 * math.cos(3.0 * phase)
}

// Root-Hann (Sine) window: satisfies exact COLA (sum w^2 = 2.0) at 75% overlap (hop = N/4)
// when applied on both STFT analysis and ISTFT synthesis.
root_hann_f64 :: #force_inline proc(n, length: int) -> f64 {
	return math.sin(math.PI * (f64(n) + 0.5) / f64(length))
}

c64_abs2 :: #force_inline proc(z: complex128) -> f64 {
	return real(z) * real(z) + imag(z) * imag(z)
}

c64_abs :: #force_inline proc(z: complex128) -> f64 {
	return math.hypot(real(z), imag(z))
}

// Synthesize normalized real low-pass FIR kernel in f64 using 4-term Blackman-Harris window
design_lowpass_fir_f64 :: proc(taps: []f64, cutoff_hz: f64, sample_rate_hz: f64) {
	length := len(taps)
	center := f64(length - 1) * 0.5
	norm_fc := (2.0 * cutoff_hz) / sample_rate_hz

	sum := 0.0
	for i in 0 ..< length {
		t := f64(i) - center
		w := blackman_harris_f64(i, length)
		v := norm_fc * sinc_f64(norm_fc * t) * w
		taps[i] = v
		sum += v
	}
	if math.abs(sum) > 1e-18 {
		inv_sum := 1.0 / sum
		for i in 0 ..< length {
			taps[i] *= inv_sum
		}
	}
}

// ---------------------------------------------------------------------------
// 2048-point Overlap-Save Complex Analytic / Bandpass Channel Filter (48 kHz)
//   FIR length L = 1025 taps (4-term Blackman-Harris, >95 dB stopband/image rejection)
//   Block hop  B = 1024 samples (exact linear convolution, zero circular aliasing)
// ---------------------------------------------------------------------------

CHAN_FFT_N   :: 2048
CHAN_HOP_B   :: 1024
CHAN_FIR_LEN :: 1025

Channel_Selectivity_Filter :: struct {
	fft_plan     : FFT_Plan,
	// Precomputed frequency-domain transfer functions H[k] for AM, FM, LSB, USB
	h_spec       : [4][CHAN_FFT_N]complex128,
	pass_bin_lo  : [4]int,
	pass_bin_hi  : [4]int,
	pass_bw_hz   : [4]f64,

	// Streaming Overlap-Save state
	prev_overlap : [CHAN_HOP_B]complex128,
	in_staging   : [CHAN_HOP_B]complex128,
	out_ready    : [CHAN_HOP_B]complex128,
	work_fft     : [CHAN_FFT_N]complex128,
	stage_idx    : int,
	has_output   : bool,
	iq_env_ema   : f64,

	// Spectral telemetry (in f64 linear power and dBFS)
	sig_power_lin     : f64,
	noise_power_lin   : f64,
	sig_dbfs          : f64,
	noise_dbfs        : f64,
	snr_db            : f64,
	coarse_peak_hz    : f64,
	coarse_peak_valid : bool,
}

// Build complex bandpass kernel for passband [f_low_hz, f_high_hz] at Fs = 48,000 Hz
@(private)
build_complex_bandpass_spec :: proc(
	plan      : ^FFT_Plan,
	dst_spec  : ^[CHAN_FFT_N]complex128,
	f_low_hz  : f64,
	f_high_hz : f64,
	fs_hz     : f64,
) {
	f_mid   := 0.5 * (f_low_hz + f_high_hz)
	bw_half := 0.5 * (f_high_hz - f_low_hz)

	lp_taps : [CHAN_FIR_LEN]f64
	design_lowpass_fir_f64(lp_taps[:], bw_half, fs_hz)

	kern : [CHAN_FFT_N]complex128
	center := f64(CHAN_FIR_LEN - 1) * 0.5
	for i in 0 ..< CHAN_FIR_LEN {
		t   := f64(i) - center
		phi := (2.0 * math.PI * f_mid * t) / fs_hz
		kern[i] = complex(lp_taps[i] * math.cos(phi), lp_taps[i] * math.sin(phi))
	}
	fft_forward(plan, kern[:])
	dst_spec^ = kern
}

channel_filter_init :: proc(cf: ^Channel_Selectivity_Filter) {
	cf^ = {}
	fft_plan_init(&cf.fft_plan, CHAN_FFT_N)

	// 1. AM: Symmetric 8.0 kHz RF channel [-4000 Hz, +4000 Hz]
	build_complex_bandpass_spec(&cf.fft_plan, &cf.h_spec[int(Modulation_Mode.AM)], -4000.0, +4000.0, BASEBAND_RATE_HZ)
	cf.pass_bw_hz[int(Modulation_Mode.AM)] = 8000.0

	// 2. FM: Symmetric 11.2 kHz CEPT CB FM Carson channel [-5600 Hz, +5600 Hz] (+-2.5 kHz dev + 3.0 kHz audio)
	build_complex_bandpass_spec(&cf.fft_plan, &cf.h_spec[int(Modulation_Mode.FM)], -5600.0, +5600.0, BASEBAND_RATE_HZ)
	cf.pass_bw_hz[int(Modulation_Mode.FM)] = 11200.0

	// 3. LSB: Analytic negative sideband [-2800 Hz, -125 Hz] (>95 dB USB & carrier rejection, full male F0 + F2/F3 consonants)
	build_complex_bandpass_spec(&cf.fft_plan, &cf.h_spec[int(Modulation_Mode.LSB)], -2800.0, -125.0, BASEBAND_RATE_HZ)
	cf.pass_bw_hz[int(Modulation_Mode.LSB)] = 2675.0

	// 4. USB: Analytic positive sideband [+125 Hz, +2800 Hz] (>95 dB LSB & carrier rejection, full male F0 + F2/F3 consonants)
	build_complex_bandpass_spec(&cf.fft_plan, &cf.h_spec[int(Modulation_Mode.USB)], +125.0, +2800.0, BASEBAND_RATE_HZ)
	cf.pass_bw_hz[int(Modulation_Mode.USB)] = 2675.0

	cf.sig_dbfs   = -120.0
	cf.noise_dbfs = -125.0
	cf.snr_db     = 0.0
}

channel_filter_reset :: proc(cf: ^Channel_Selectivity_Filter) {
	cf.prev_overlap = {}
	cf.in_staging   = {}
	cf.out_ready    = {}
	cf.stage_idx    = 0
	cf.has_output   = false
	cf.iq_env_ema   = 0.0
}

// Estimate true passband signal power vs broadband noise floor density from the 2048-pt FFT
@(private)
update_channel_spectral_metrics :: proc(cf: ^Channel_Selectivity_Filter, spec: []complex128, mode: Modulation_Mode) {
	// Parseval normalization for unscaled forward DFT of length N:
	// Time-domain average power (1/N) * sum |x[n]|^2 == (1/N^2) * sum |X[k]|^2.
	inv_n2 := 1.0 / (f64(CHAN_FFT_N) * f64(CHAN_FFT_N))
	bin_hz := BASEBAND_RATE_HZ / f64(CHAN_FFT_N) // 23.4375 Hz/bin

	pass_power := 0.0
	pass_bins  := 0

	// Sub-band power array across [-18 kHz .. +18 kHz] (48 sub-bands of 32 bins = 750 Hz each)
	// Using order statistics (20th percentile sub-band PSD) yields an unbiased noise-floor
	// density estimate even when multiple CB channels are active simultaneously.
	NUM_SUBBANDS :: 48
	BINS_PER_SB  :: 32
	sb_psd : [NUM_SUBBANDS]f64
	sb_cnt := 0

	for k in 0 ..< CHAN_FFT_N {
		// Signed frequency in Hz for FFT bin k
		signed_k := k if k < (CHAN_FFT_N / 2) else (k - CHAN_FFT_N)
		freq_hz  := f64(signed_k) * bin_hz
		p_bin    := c64_abs2(spec[k]) * inv_n2

		in_pass := false
		switch mode {
		case .AM:  in_pass = freq_hz >= -4000.0 && freq_hz <= +4000.0
		case .FM:  in_pass = freq_hz >= -5600.0 && freq_hz <= +5600.0
		case .LSB: in_pass = freq_hz >= -2800.0 && freq_hz <= -125.0
		case .USB: in_pass = freq_hz >= +125.0  && freq_hz <= +2800.0
		}
		if in_pass {
			pass_power += p_bin
			pass_bins  += 1
		}
	}

	// Collect 32-bin sub-bands within [-18 kHz, +18 kHz] (excluding DC +-2 bins)
	for sb in 0 ..< NUM_SUBBANDS {
		signed_start := -768 + sb * BINS_PER_SB
		if signed_start >= -32 && signed_start < 32 {
			continue // skip immediate 0 Hz region
		}
		sum_sb := 0.0
		for b in 0 ..< BINS_PER_SB {
			sk  := signed_start + b
			idx := sk if sk >= 0 else (CHAN_FFT_N + sk)
			sum_sb += c64_abs2(spec[idx]) * inv_n2
		}
		sb_psd[sb_cnt] = sum_sb / f64(BINS_PER_SB) // power per bin
		sb_cnt += 1
	}

	// Insertion sort on small array (<= 46 elements) to extract 20th-percentile noise power/bin
	for i in 1 ..< sb_cnt {
		key := sb_psd[i]
		j   := i - 1
		for j >= 0 && sb_psd[j] > key {
			sb_psd[j + 1] = sb_psd[j]
			j -= 1
		}
		sb_psd[j + 1] = key
	}

	noise_per_bin := 1e-16
	if sb_cnt > 0 {
		q_idx := clamp(sb_cnt / 5, 0, sb_cnt - 1)
		// Median/20th-percentile bias correction (~1.18 for chi-square averaged over 32 bins)
		noise_per_bin = math.max(sb_psd[q_idx] * 1.12, 1e-16)
	}

	equiv_pass_noise := noise_per_bin * f64(math.max(pass_bins, 1))

	// Smooth telemetry in linear power domain (time constant ~120 ms; match alpha on wideband RF gain jumps)
	alpha := 0.18
	if cf.sig_power_lin <= 0.0 {
		cf.sig_power_lin   = math.max(pass_power, 1e-16)
		cf.noise_power_lin = math.max(equiv_pass_noise, 1e-16)
	} else {
		alpha_n := alpha if equiv_pass_noise > 1.8 * cf.noise_power_lin else 0.08
		cf.sig_power_lin   = (1.0 - alpha) * cf.sig_power_lin + alpha * math.max(pass_power, 1e-16)
		cf.noise_power_lin = (1.0 - alpha_n) * cf.noise_power_lin + alpha_n * math.max(equiv_pass_noise, 1e-16)
	}

	cf.sig_dbfs   = 10.0 * math.log10(math.max(cf.sig_power_lin, 1e-16))
	cf.noise_dbfs = 10.0 * math.log10(math.max(cf.noise_power_lin, 1e-16))

	// Net signal-above-noise SNR estimate: subtract noise floor contribution when > floor
	pure_sig := math.max(cf.sig_power_lin - cf.noise_power_lin * 0.92, cf.noise_power_lin * 0.001)
	cf.snr_db = 10.0 * math.log10(pure_sig / math.max(cf.noise_power_lin, 1e-16))

	// Coarse carrier peak tracker across [-2484 .. +2484 Hz] for wide-pull-in AM/FM Auto-Clarifier
	best_sk := 0
	best_p  := 0.0
	for sk in -106 ..= 106 {
		idx := sk if sk >= 0 else (CHAN_FFT_N + sk)
		p   := c64_abs2(spec[idx]) * inv_n2
		if p > best_p {
			best_p  = p
			best_sk = sk
		}
	}
	idx_l := (best_sk - 1) if (best_sk - 1) >= 0 else (CHAN_FFT_N + best_sk - 1)
	idx_c := best_sk if best_sk >= 0 else (CHAN_FFT_N + best_sk)
	idx_r := (best_sk + 1) if (best_sk + 1) >= 0 else (CHAN_FFT_N + best_sk + 1)
	pl    := math.max(c64_abs2(spec[idx_l]) * inv_n2, 1e-30)
	pc    := math.max(c64_abs2(spec[idx_c]) * inv_n2, 1e-30)
	pr    := math.max(c64_abs2(spec[idx_r]) * inv_n2, 1e-30)
	lnl   := math.ln(pl)
	lnc   := math.ln(pc)
	lnr   := math.ln(pr)
	den   := lnl - 2.0 * lnc + lnr
	delta := 0.0
	if math.abs(den) > 1e-12 {
		delta = clamp(0.5 * (lnl - lnr) / den, -0.5, +0.5)
	}
	cf.coarse_peak_hz    = (f64(best_sk) + delta) * bin_hz
	cf.coarse_peak_valid = (best_p > 18.0 * noise_per_bin) && ((pl + pc + pr) > 0.24 * math.max(pass_power, 1e-20))
}

// Push one 48 kHz complex sample; returns true whenever a filtered block of CHAN_HOP_B (1024)
// linear-convolution complex samples is ready in cf.out_ready[:].
channel_filter_push :: proc(cf: ^Channel_Selectivity_Filter, sample: complex128, mode: Modulation_Mode) -> bool {
	cf.in_staging[cf.stage_idx] = sample
	cf.stage_idx += 1
	if cf.stage_idx < CHAN_HOP_B {
		return false
	}
	cf.stage_idx = 0

	// Zero-delay 48 kHz baseband IQ impulse blanker: surgically interpolates >13 dB wideband switching/triac spikes
	// BEFORE the 1025-tap narrowband channel FIR can disperse them into 21 ms ringing bursts (0% Rayleigh/AM/FM/SSB touch!)
	sum_mag := 0.0
	for i in 0 ..< CHAN_HOP_B {
		sum_mag += c64_abs(cf.in_staging[i])
	}
	mean_mag := sum_mag / f64(CHAN_HOP_B)
	clean_sum := 0.0
	clean_cnt := 0
	for i in 0 ..< CHAN_HOP_B {
		m := c64_abs(cf.in_staging[i])
		if m <= 3.0 * mean_mag {
			clean_sum += m
			clean_cnt += 1
		}
	}
	clean_mag := (clean_sum / f64(clean_cnt)) if clean_cnt > 0 else mean_mag
	if cf.iq_env_ema <= 0.0 {
		cf.iq_env_ema = clean_mag
	} else {
		cf.iq_env_ema = 0.78 * cf.iq_env_ema + 0.22 * clean_mag
	}
	ib_thresh := 4.45 * math.max(cf.iq_env_ema, clean_mag)
	if ib_thresh > 1e-15 {
		for i in 0 ..< CHAN_HOP_B {
			if c64_abs(cf.in_staging[i]) > ib_thresh {
				z_l := cf.prev_overlap[CHAN_HOP_B - 1] if i == 0 else cf.in_staging[i - 1]
				z_r := complex(0.0, 0.0)
				if i + 1 < CHAN_HOP_B && c64_abs(cf.in_staging[i + 1]) <= ib_thresh {
					z_r = cf.in_staging[i + 1]
				} else {
					z_r = z_l
				}
				cf.in_staging[i] = 0.5 * (z_l + z_r)
			}
		}
	}

	// Assemble 2048-sample Overlap-Save input vector: [prev_overlap (1024) | in_staging (1024)]
	copy(cf.work_fft[0:CHAN_HOP_B], cf.prev_overlap[:])
	copy(cf.work_fft[CHAN_HOP_B:CHAN_FFT_N], cf.in_staging[:])
	copy(cf.prev_overlap[:], cf.in_staging[:])

	// Forward 2048-pt f64 FFT
	fft_forward(&cf.fft_plan, cf.work_fft[:])

	// Update signal power (dBFS), noise floor (dBFS), and SNR (dB)
	update_channel_spectral_metrics(cf, cf.work_fft[:], mode)

	// Complex frequency-domain multiplication with mode-specific analytic/bandpass transfer function
	h := &cf.h_spec[int(mode)]
	for k in 0 ..< CHAN_FFT_N {
		cf.work_fft[k] *= h[k]
	}

	// Inverse 2048-pt f64 IFFT
	fft_inverse(&cf.fft_plan, cf.work_fft[:])

	// Discard first 1024 circular-wrap samples; keep exact linear convolution samples [1024..2047]
	copy(cf.out_ready[:], cf.work_fft[CHAN_HOP_B:CHAN_FFT_N])
	cf.has_output = true
	return true
}
