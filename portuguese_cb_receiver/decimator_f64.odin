// 6.0 MSamples/s (14-bit SDRplay ADC) to 48 kHz Complex Baseband Multi-Stage Polyphase
// Decimator, Drift-Free Quadrature f64 NCO, and Pre-Filter Wideband RF Impulse Blanker.

package main

import "core:math"

STAGE1_BRANCHES :: 8
STAGE1_TAPS     :: STAGE1_DECIM_FACTOR * STAGE1_BRANCHES // 25 * 8 = 200 taps

STAGE2_BRANCHES :: 24
STAGE2_TAPS     :: STAGE2_DECIM_FACTOR * STAGE2_BRANCHES // 5 * 24 = 120 taps

WB_BLANKER_DELAY :: 16
WB_BLANKER_MASK  :: WB_BLANKER_DELAY - 1

FrontEnd_Decimator :: struct {
	// Drift-free f64 quadrature NCO at 6.0 MHz
	nco_cos         : f64,
	nco_sin         : f64,
	nco_step_cos    : f64,
	nco_step_sin    : f64,
	nco_freq_hz     : f64,
	nco_renorm_cnt  : u32,

	// Stage 1 Transpose Polyphase FIR (6.0 MHz -> 240 kHz, M=25, 200 taps)
	// Indexed [phase_j][branch_p] for contiguous L1 cache SIMD access
	s1_poly_taps    : [STAGE1_DECIM_FACTOR][STAGE1_BRANCHES]f64,
	s1_acc_i        : [STAGE1_BRANCHES]f64,
	s1_acc_q        : [STAGE1_BRANCHES]f64,
	s1_phase        : int,

	// Wideband Complex Impulse Blanker at 240 kHz (pre-channel-filter)
	nb_ring         : [WB_BLANKER_DELAY]complex128,
	nb_mag_ring     : [WB_BLANKER_DELAY]f64,
	nb_pos          : int,
	nb_env_floor    : f64,
	nb_hold_left    : int,
	nb_warmup       : int,
	nb_high_streak  : int,
	nb_last_clean   : complex128,
	nb_blanked_cnt  : u64,

	// Stage 2 Transpose Polyphase FIR (240 kHz -> 48 kHz, M=5, 120 taps)
	s2_poly_taps    : [STAGE2_DECIM_FACTOR][STAGE2_BRANCHES]f64,
	s2_acc_i        : [STAGE2_BRANCHES]f64,
	s2_acc_q        : [STAGE2_BRANCHES]f64,
	s2_phase        : int,

	// Wideband ADC dBFS tracker (tracks raw 14-bit ADC headroom / overload margin)
	adc_power_ema   : f64,
	adc_peak        : f64,
}

frontend_set_nco_freq :: proc(fe: ^FrontEnd_Decimator, shift_hz: f64) {
	if math.abs(shift_hz - fe.nco_freq_hz) < 1e-6 && (fe.nco_step_cos != 0.0 || fe.nco_step_sin != 0.0) {
		return
	}
	fe.nco_freq_hz = shift_hz
	// Multiply by exp(-j * 2*pi * shift_hz / Fs) to mix +shift_hz down to 0 Hz
	dphi := (-2.0 * math.PI * shift_hz) / SDR_SAMPLE_RATE_HZ
	fe.nco_step_cos = math.cos(dphi)
	fe.nco_step_sin = math.sin(dphi)
}

frontend_init :: proc(fe: ^FrontEnd_Decimator, clarifier_hz: f64) {
	fe^ = {}
	fe.nco_cos = 1.0
	fe.nco_sin = 0.0
	frontend_set_nco_freq(fe, -SDR_LO_OFFSET_HZ + clarifier_hz)

	// Design Stage 1 200-tap Blackman-Harris lowpass (fc = 90 kHz at 6.0 MHz; Nyquist after M=25 is 120 kHz)
	s1_linear : [STAGE1_TAPS]f64
	design_lowpass_fir_f64(s1_linear[:], 90_000.0, SDR_SAMPLE_RATE_HZ)
	for j in 0 ..< STAGE1_DECIM_FACTOR {
		r := (STAGE1_DECIM_FACTOR - 1) - j
		for p in 0 ..< STAGE1_BRANCHES {
			fe.s1_poly_taps[j][p] = s1_linear[p * STAGE1_DECIM_FACTOR + r]
		}
	}

	// Design Stage 2 120-tap Blackman-Harris lowpass (fc = 19 kHz at 240 kHz; Nyquist after M=5 is 24 kHz)
	s2_linear : [STAGE2_TAPS]f64
	design_lowpass_fir_f64(s2_linear[:], 19_000.0, STAGE1_RATE_HZ)
	for j in 0 ..< STAGE2_DECIM_FACTOR {
		r := (STAGE2_DECIM_FACTOR - 1) - j
		for p in 0 ..< STAGE2_BRANCHES {
			fe.s2_poly_taps[j][p] = s2_linear[p * STAGE2_DECIM_FACTOR + r]
		}
	}

	fe.nb_env_floor  = 0.0
	fe.nb_warmup     = 0
	fe.nb_high_streak = 0
	fe.adc_power_ema = 1e-12
}

// Wideband 240 kHz complex I/Q impulse blanker for indoor triac/SMPS/motor/static spikes
@(private)
wideband_impulse_blank :: #force_inline proc(
	fe           : ^FrontEnd_Decimator,
	z_in         : complex128,
	filter_level : Filter_Level,
) -> complex128 {
	if filter_level == .Off {
		return z_in
	}

	mag := c64_abs(z_in)

	// Wait 12 samples (50 us) for Stage-1 200-tap polyphase FIR startup delay before seeding Rayleigh floor,
	// and fast-track genuine >=100 us RF level steps (Auto-LNA steps / adjacent-channel key-ups) while keeping
	// <=50 us microsecond triac/ignition impulses 100% clamped!
	if fe.nb_warmup < 12 {
		fe.nb_warmup    += 1
		fe.nb_ring[fe.nb_pos]     = z_in
		fe.nb_mag_ring[fe.nb_pos] = mag
		fe.nb_pos                 = (fe.nb_pos + 1) & WB_BLANKER_MASK
		fe.nb_last_clean          = z_in
		if fe.nb_warmup == 12 {
			sum_m := 0.0
			for i in 4 ..< 12 do sum_m += fe.nb_mag_ring[i]
			fe.nb_env_floor = math.max(sum_m / 8.0, 1e-7)
		}
		return z_in
	}

	if fe.nb_env_floor <= 1e-7 {
		fe.nb_env_floor   = math.max(mag, 1e-7)
		fe.nb_high_streak = 0
	} else if mag <= fe.nb_env_floor * 3.8 {
		fe.nb_env_floor   = 0.9985 * fe.nb_env_floor + 0.0015 * mag
		fe.nb_high_streak = 0
	} else {
		fe.nb_high_streak += 1
		if fe.nb_high_streak >= 24 {
			// Sustained RF envelope step (>=100 us, e.g. Auto-LNA change or strong adjacent carrier key-up)
			fe.nb_env_floor = 0.85 * fe.nb_env_floor + 0.15 * mag
		} else {
			fe.nb_env_floor = 0.9999 * fe.nb_env_floor + 0.0001 * (fe.nb_env_floor * 3.0)
		}
	}
	fe.nb_env_floor = math.max(fe.nb_env_floor, 1e-8)

	// For Rayleigh complex Gaussian envelope, P(|z| > k * mean) = exp(-pi/4 * k^2):
	//   k >= 4.4 -> < 6e-7 false alarm (<0.2 samples/sec on pure thermal noise, zero IMD splatter!)
	//   while genuine indoor triac/SMPS/motor clicks exceed 5x..40x mean at 240 kHz BW.
	thresh_mult := 9.0
	switch filter_level {
	case .Off:     thresh_mult = 100.0
	case .Level_1: thresh_mult = 5.6
	case .Level_2: thresh_mult = 4.9
	case .Level_3: thresh_mult = 4.5
	case .Level_4: thresh_mult = 4.3
	}

	// Lookahead ring buffer so we catch the leading edge of fast impulses
	idx := fe.nb_pos
	out_sample := fe.nb_ring[idx]
	center_idx := (idx + (WB_BLANKER_DELAY / 2)) & WB_BLANKER_MASK
	center_mag := fe.nb_mag_ring[center_idx]

	fe.nb_ring[idx]     = z_in
	fe.nb_mag_ring[idx] = mag
	fe.nb_pos           = (idx + 1) & WB_BLANKER_MASK

	if center_mag > fe.nb_env_floor * thresh_mult {
		fe.nb_hold_left = 3 // Blank +-1 sample around impulse peak (~12.5 us at 240 kHz)
		fe.nb_blanked_cnt += 1
	}

	if fe.nb_hold_left > 0 {
		fe.nb_hold_left -= 1
		// Smoothly interpolate toward incoming post-spike sample rather than hard zeroing
		ahead := fe.nb_ring[(idx + 4) & WB_BLANKER_MASK]
		if c64_abs(ahead) <= fe.nb_env_floor * thresh_mult {
			return 0.65 * fe.nb_last_clean + 0.35 * ahead
		}
		return fe.nb_last_clean * 0.85
	}

	fe.nb_last_clean = out_sample
	return out_sample
}

// Process a chunk of raw 6.0 MSamples/s 14-bit signed I/Q samples into 48 kHz complex128 output.
// Returns the number of 48 kHz complex baseband samples written to dst_48k.
frontend_process_iq_block :: proc(
	fe             : ^FrontEnd_Decimator,
	xi             : []i16,
	xq             : []i16,
	clarifier_hz   : f64,
	filter_level   : Filter_Level,
	dst_48k        : []complex128,
	base_offset_hz : f64 = -SDR_LO_OFFSET_HZ,
) -> int {
	frontend_set_nco_freq(fe, base_offset_hz + clarifier_hz)

	// Scale factor: SDRplay delivers 14-bit ADC samples in 16-bit signed container.
	// Using 1/32768.0 maps full-scale i16 container to [-1.0, +1.0] in f64 without clipping.
	SCALE_I16 :: f64(1.0 / 32768.0)

	nco_c  := fe.nco_cos
	nco_s  := fe.nco_sin
	step_c := fe.nco_step_cos
	step_s := fe.nco_step_sin
	rcnt   := fe.nco_renorm_cnt

	out_count := 0
	num_in    := min(len(xi), len(xq))
	p_accum   := 0.0
	peak_blk  := 0.0

	for n in 0 ..< num_in {
		raw_i := f64(xi[n]) * SCALE_I16
		raw_q := f64(xq[n]) * SCALE_I16

		// Track ADC wideband power every 16th sample for ultra-low overhead
		if (n & 15) == 0 {
			mag2 := raw_i * raw_i + raw_q * raw_q
			p_accum += mag2
			if mag2 > peak_blk do peak_blk = mag2
		}

		// Complex mix: (raw_i + j*raw_q) * (nco_c + j*nco_s)
		mix_i := raw_i * nco_c - raw_q * nco_s
		mix_q := raw_i * nco_s + raw_q * nco_c

		// Advance f64 quadrature NCO
		next_c := nco_c * step_c - nco_s * step_s
		next_s := nco_s * step_c + nco_c * step_s
		nco_c = next_c
		nco_s = next_s

		rcnt += 1
		if (rcnt & 1023) == 0 {
			// Exact unit-circle renormalization: 1st-order Newton step 0.5*(3 - r^2)
			r2  := nco_c * nco_c + nco_s * nco_s
			inv := 0.5 * (3.0 - r2)
			nco_c *= inv
			nco_s *= inv
		}

		// Stage 1 Transpose Polyphase FIR accumulation (6.0 MHz -> 240 kHz)
		j1   := fe.s1_phase
		taps := &fe.s1_poly_taps[j1]
		#unroll for p in 0 ..< STAGE1_BRANCHES {
			w := taps[p]
			fe.s1_acc_i[p] += w * mix_i
			fe.s1_acc_q[p] += w * mix_q
		}

		j1 += 1
		if j1 == STAGE1_DECIM_FACTOR {
			j1 = 0
			y240 := complex(fe.s1_acc_i[0], fe.s1_acc_q[0])
			#unroll for p in 0 ..< (STAGE1_BRANCHES - 1) {
				fe.s1_acc_i[p] = fe.s1_acc_i[p + 1]
				fe.s1_acc_q[p] = fe.s1_acc_q[p + 1]
			}
			fe.s1_acc_i[STAGE1_BRANCHES - 1] = 0.0
			fe.s1_acc_q[STAGE1_BRANCHES - 1] = 0.0

			// Wideband 240 kHz impulse blanker
			y240_clean := wideband_impulse_blank(fe, y240, filter_level)
			yi2 := real(y240_clean)
			yq2 := imag(y240_clean)

			// Stage 2 Transpose Polyphase FIR accumulation (240 kHz -> 48 kHz)
			j2    := fe.s2_phase
			taps2 := &fe.s2_poly_taps[j2]
			#unroll for p in 0 ..< STAGE2_BRANCHES {
				w2 := taps2[p]
				fe.s2_acc_i[p] += w2 * yi2
				fe.s2_acc_q[p] += w2 * yq2
			}

			j2 += 1
			if j2 == STAGE2_DECIM_FACTOR {
				j2 = 0
				if out_count < len(dst_48k) {
					dst_48k[out_count] = complex(fe.s2_acc_i[0], fe.s2_acc_q[0])
					out_count += 1
				}
				#unroll for p in 0 ..< (STAGE2_BRANCHES - 1) {
					fe.s2_acc_i[p] = fe.s2_acc_i[p + 1]
					fe.s2_acc_q[p] = fe.s2_acc_q[p + 1]
				}
				fe.s2_acc_i[STAGE2_BRANCHES - 1] = 0.0
				fe.s2_acc_q[STAGE2_BRANCHES - 1] = 0.0
			}
			fe.s2_phase = j2
		}
		fe.s1_phase = j1
	}

	fe.nco_cos        = nco_c
	fe.nco_sin        = nco_s
	fe.nco_renorm_cnt = rcnt

	sampled_cnt := math.max(num_in / 16, 1)
	avg_p       := p_accum / f64(sampled_cnt)
	fe.adc_power_ema = 0.90 * fe.adc_power_ema + 0.10 * math.max(avg_p, 1e-16)
	fe.adc_peak      = math.sqrt(peak_blk)

	return out_count
}
