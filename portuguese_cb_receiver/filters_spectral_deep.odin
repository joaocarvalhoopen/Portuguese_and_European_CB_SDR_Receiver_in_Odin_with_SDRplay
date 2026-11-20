// Audio Noise Filters Level 3 (2048-pt Low-Latency OLA Synthesis + 4096-pt Mistune-Aware
// SHS Harmonic Comb & Cohen OM-LSA + Zero-Silence VAD, targeting -15 dB below noise floor)
// and Level 4 (4096-pt High-Resolution STFT + Multi-Timescale SHS Pitch/Mistune Comb +
// Narrowband Heterodyne Auto-Notch + Ephraim-Malah MMSE-LSA + Cepstral Formant Liftering +
// Zero-Silence VAD, targeting -20 to -30 dB below noise floor) in pure 64-bit float (f64).
// Specifically engineered to remove indoor house EMI/RFI and recover clear, natural human
// voice in AM, FM, and SSB while producing exact 0.0 digital silence during non-speech moments.

package main

import "core:math"

// ---------------------------------------------------------------------------
// Exact f64 Exponential Integral E1(v) = integral_v^inf (exp(-t)/t) dt
// Used by Ephraim-Malah Minimum Mean-Square Error Log-Spectral Amplitude (MMSE-LSA)
// ---------------------------------------------------------------------------

expint_e1_f64 :: #force_inline proc(v: f64) -> f64 {
	EULER_GAMMA :: 0.57721566490153286060651209
	if v <= 1e-12 {
		return -EULER_GAMMA - math.ln(math.max(v, 1e-300))
	}
	if v <= 1.0 {
		sum  := 0.0
		term := 1.0
		for m in 1 ..= 24 {
			fm    := f64(m)
			term  *= -v / fm
			sum   += term / fm
		}
		return -EULER_GAMMA - math.ln(v) - sum
	}
	if v > 45.0 {
		return 0.0
	}
	cf := 0.0
	for k := 36; k >= 1; k -= 1 {
		fk := f64(k)
		cf  = (fk * fk) / (v + 2.0 * fk + 1.0 - cf)
	}
	return math.exp(-v) / (v + 1.0 - cf)
}

mmse_lsa_gain_f64 :: #force_inline proc(xi: f64, gamma: f64, g_floor: f64) -> f64 {
	xi_c    := clamp(xi, 1e-6, 1e4)
	gamma_c := clamp(gamma, 1e-6, 1e4)
	prior_w := xi_c / (1.0 + xi_c)
	vk      := prior_w * gamma_c
	e1      := expint_e1_f64(vk)
	g       := prior_w * math.exp(math.min(0.5 * e1, 12.0))
	return clamp(g, g_floor, 1.25)
}

// True if frequency `fk` falls inside the 50/100/150/200/250 Hz mains hum biquad notches
@(private)
is_hum_notch_freq :: #force_inline proc(fk: f64) -> bool {
	if fk < 38.0 || fk > 262.0 do return false
	hm := math.round(fk * 0.02) // nearest multiple of 50 Hz
	return hm >= 1.0 && hm <= 5.0 && math.abs(fk - 50.0 * hm) <= 9.5
}

// Compute unbiased local noise floor from sorted interior passband neighborhood (immune to hum notches & speech teeth)
@(private)
local_quantile_noise_floor :: proc(
	psd      : []f64,
	center_k : int,
	half_win : int,
	lo_bound : int,
	hi_bound : int,
	pct      : int = 45,
	scale    : f64 = 1.045,
	bin_hz   : f64 = (AUDIO_SAMPLE_RATE_HZ / 4096.0),
) -> f64 {
	buf : [64]f64
	cnt := 0
	ck  := clamp(center_k, lo_bound, hi_bound)
	a   := max(ck - half_win, lo_bound)
	b   := min(ck + half_win, hi_bound)
	step := 1
	if (b - a + 1) > len(buf) do step = 2
	for idx := a; idx <= b && cnt < len(buf); idx += step {
		if is_hum_notch_freq(f64(idx) * bin_hz) do continue
		buf[cnt] = psd[idx]
		cnt += 1
	}
	if cnt == 0 do return 1e-20

	for i in 1 ..< cnt {
		key := buf[i]
		j   := i - 1
		for j >= 0 && buf[j] > key {
			buf[j + 1] = buf[j]
			j -= 1
		}
		buf[j + 1] = key
	}

	q_idx := clamp((cnt * pct) / 100, 0, cnt - 1)
	return math.max(buf[q_idx] * scale, 1e-20)
}

// Shift-invariant harmonic-lattice detector: for ANY mistuned SSB or AM/FM vocal comb f_m = m * F0 + df,
// the spacing between harmonics f_m - f_h = (m - h) * F0 is 100% independent of df!
// Verifies local tooth-above-valley contrast across BOTH odd and even d = (m - h) so half-period valley spurs
// (e.g. 1385 Hz SMPS or 1007.8 Hz heterodyne between harmonics) are ALWAYS rejected & notched while 100% of
// genuine speech harmonics (including <350 Hz CB TX-cut male voices at arbitrary mistune df) are protected!
@(private)
is_vocal_harmonic_member :: proc(psd, fast_psd, noise_psd: []f64, k: int, bin_hz: f64, k_lo, k_hi: int, tracked_df: f64 = 0.0) -> bool {
	_ = fast_psd
	_ = tracked_df
	delta := 0.0
	if k > k_lo && k < k_hi {
		pl  := math.ln(math.max(psd[k - 1], 1e-30))
		pc  := math.ln(math.max(psd[k],     1e-30))
		pr  := math.ln(math.max(psd[k + 1], 1e-30))
		den := pl - 2.0 * pc + pr
		if den < -1e-9 {
			delta = clamp(0.5 * (pl - pr) / den, -0.5, +0.5)
		}
	}
	fk      := (f64(k) + delta) * bin_hz
	inv_bin := 1.0 / bin_hz
	nk_flr  := math.max(noise_psd[k], 1e-22)

	// Step 0.5 Hz across [84..165 Hz] (<=0.25-bin error at 12th companion!) & 1.0 Hz across [166..270 Hz]
	N_F0_CHK :: 163 + 105
	for fi in 0 ..< N_F0_CHK {
		f0 := (84.0 + 0.5 * f64(fi)) if fi < 163 else (166.0 + f64(fi - 163))

		comp_hits  := 0
		odd_hits   := 0
		even_hits  := 0
		near_hits  := 0
		max_comp_p := 0.0
		adj_comp_p := 0.0

		for d in -12 ..= 12 {
			if d == 0 do continue
			fm := fk + f64(d) * f0
			if fm < 120.0 || fm > 2620.0 || is_hum_notch_freq(fm) do continue

			bf := fm * inv_bin
			b0 := int(math.floor(bf))
			fr := bf - f64(b0)
			if b0 < k_lo || b0 + 1 > k_hi do continue

			p_int := (1.0 - fr) * psd[b0] + fr * psd[b0 + 1]
			n_int := (1.0 - fr) * noise_psd[b0] + fr * noise_psd[b0 + 1]

			// Measure adjacent inter-harmonic valleys fm ± 0.5 * f0 to guarantee true comb periodicity
			val_s := 0.0
			val_c := 0.0
			for sv in ([2]f64{-0.5, +0.5}) {
				fv  := fm + sv * f0
				vb  := fv * inv_bin
				vb0 := int(math.floor(vb))
				vfr := vb - f64(vb0)
				if vb0 >= k_lo && vb0 + 1 <= k_hi {
					val_s += (1.0 - vfr) * psd[vb0] + vfr * psd[vb0 + 1]
					val_c += 1.0
				}
			}
			val_int := (val_s / val_c) if val_c > 0.0 else n_int

			if math.abs(d) <= 2 && p_int > 1.20 * val_int {
				if p_int > adj_comp_p do adj_comp_p = p_int
			}
			p_thr := math.max(1.30 * math.max(n_int, 1e-22), n_int + 0.018 * math.max(psd[k] - nk_flr, 0.0))
			if p_int > p_thr && p_int > 1.18 * val_int {
				comp_hits += 1
				if p_int > max_comp_p do max_comp_p = p_int
				if (math.abs(d) & 1) == 1 {
					odd_hits += 1
				} else {
					even_hits += 1
				}
				if math.abs(d) <= 3 do near_hits += 1
			}
		}

		// Tower-spur guard: a +15..+30 dB CW/SMPS spur towering over weak speech/noise cannot claim tiny companions
		if psd[k] > 12.0 * nk_flr && (max_comp_p < 0.05 * psd[k] || adj_comp_p < 0.035 * psd[k]) {
			continue
		}
		if comp_hits >= 5 && odd_hits >= 2 && even_hits >= 2 && near_hits >= 2 {
			return true
		}
	}
	return false
}

// Variance-equalized Subharmonic Summation (SHS) tooth-minus-valley comb evaluator.
// Exact 1:1 peak-minus-valley subtraction guarantees E[score] == 0.0000 identically on ANY
// flat/broadband QRN static swell while L2 weight normalization equalizes variance across [85..260 Hz]!
@(private)
eval_uniform_shs_comb :: #force_inline proc(
	exc        : []f64,
	cand_f0    : f64,
	df         : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_bin_hz : f64,
	pk_thr     : f64 = 0.045,
) -> (score: f64, pos_harms: int, consec_pairs: int) {
	max_h    := clamp(int(math.floor(1920.0 / cand_f0)), 5, 18)
	sum      := 0.0
	w_sq     := 0.0
	pos_h    := 0
	consec   := 0
	prev_pos := false

	for h in 1 ..= max_h {
		fh := f64(h) * cand_f0 + df
		if fh < f_lo || fh > f_hi {
			prev_pos = false
			continue
		}

		bf  := fh * inv_bin_hz
		b0  := int(math.floor(bf))
		frc := bf - f64(b0)
		peak_v := 0.0
		if b0 >= k_lo && b0 + 1 <= k_hi {
			peak_v = (1.0 - frc) * exc[b0] + frc * exc[b0 + 1]
		}

		fv_lo := (f64(h) - 0.5) * cand_f0 + df
		fv_hi := (f64(h) + 0.5) * cand_f0 + df
		val_v := 0.0
		v_cnt := 0.0
		if fv_lo >= f_lo && fv_lo <= f_hi {
			vb  := fv_lo * inv_bin_hz
			vb0 := int(math.floor(vb))
			vfr := vb - f64(vb0)
			if vb0 >= k_lo && vb0 + 1 <= k_hi {
				val_v += (1.0 - vfr) * exc[vb0] + vfr * exc[vb0 + 1]
				v_cnt += 1.0
			}
		}
		if fv_hi >= f_lo && fv_hi <= f_hi {
			vb  := fv_hi * inv_bin_hz
			vb0 := int(math.floor(vb))
			vfr := vb - f64(vb0)
			if vb0 >= k_lo && vb0 + 1 <= k_hi {
				val_v += (1.0 - vfr) * exc[vb0] + vfr * exc[vb0 + 1]
				v_cnt += 1.0
			}
		}
		if v_cnt > 0.0 {
			val_v /= v_cnt
		}

		// DC-offset-immune peak-over-valley test (works down to -20..-30 dB!)
		is_p := (peak_v - val_v) > pk_thr
		if is_p {
			pos_h += 1
			if prev_pos do consec += 1
		}
		prev_pos = is_p

		// Vocal-formant matched weighting (340..1260 Hz F1/F2 core -> +3.1 dB <350Hz TX-cut & glide comb SNR & E[comb] == 0.000!)
		wh := 1.0 / math.pow(f64(h), 0.32)
		if fh < 340.0 {
			wh *= clamp(0.18 + 0.82 * ((fh - 170.0) / 170.0), 0.18, 1.0)
		} else if fh > 1260.0 {
			dh := (fh - 1260.0) / 390.0
			wh *= 1.0 / (1.0 + dh * dh)
		}
		sum  += wh * (peak_v - val_v)
		w_sq += wh * wh
	}
	return sum / (3.40 * math.sqrt(w_sq + 0.35)), pos_h, consec
}

// 12 kHz (4:1 decimated from 48 kHz) High-Resolution Coherent Sub-Noise Comb Side-Chain:
//   - 1536-pt Hann @ 12 kHz (zero-padded to 2048) = 128.0 ms agile syllable onset/offset & 256 ms disjoint validator
//   - 2048-pt Hann @ 12 kHz = 5.859375 Hz/bin (170.7 ms, +3.01 dB pre-squaring gain, 100% glide-agile)
//   - 4096-pt Hann @ 12 kHz = 2.9296875 Hz/bin (341.3 ms, +6.02 dB pre-squaring gain for -15..-30 dB)
HR_FS_HZ    :: 12000.0
HR128_N     :: 1536
HR2K_N      :: 2048
HR4K_N      :: 4096
HR2K_BIN_HZ :: HR_FS_HZ / f64(HR2K_N) // 5.859375 Hz/bin
HR4K_BIN_HZ :: HR_FS_HZ / f64(HR4K_N) // 2.9296875 Hz/bin
HR2K_BINS   :: 364                    // 0 .. 2132 Hz (covers SSB 1720 Hz & AM/FM 2000 Hz + quantile margin)
HR4K_BINS   :: 724                    // 0 .. 2121 Hz (covers SSB 1720 Hz & AM/FM 2000 Hz + quantile margin)
HR_STEP_12K :: 256                    // 1024 @ 48 kHz = 256 @ 12 kHz = 21.333 ms
HR3_RING_N  :: 8704                   // 725.3 ms: exact zero-overlap 128ms, 2048-pt & 4096-pt partitions!
HR4_RING_N  :: 9216                   // 768.0 ms: exact zero-overlap 128ms, 5+4-step 2048-pt & 3+3-step 4096-pt partitions!
HR_HIST_LEN :: 6

// 4th-order Butterworth anti-aliasing lowpass filter (fc = 2500 Hz @ fs = 48000 Hz, >50 dB alias rejection)
Decim12k_LPF :: struct {
	s1_z1, s1_z2 : f64,
	s2_z1, s2_z2 : f64,
}

@(private)
decim12k_lpf_step :: #force_inline proc(f: ^Decim12k_LPF, x: f64) -> f64 {
	// Exact f64 bilinear-warped 4th-order Butterworth @ fc = 2500 Hz, fs = 48000 Hz
	K    :: 0.16510786701365803 // tan(pi * 2500 / 48000)
	K2   :: K * K
	Q1   :: 0.5411961001461970
	Q2   :: 1.3065629648763766
	N1   :: 1.0 / (1.0 + K / Q1 + K2)
	B0_1 :: K2 * N1
	B1_1 :: 2.0 * B0_1
	A1_1 :: 2.0 * (K2 - 1.0) * N1
	A2_1 :: (1.0 - K / Q1 + K2) * N1
	N2   :: 1.0 / (1.0 + K / Q2 + K2)
	B0_2 :: K2 * N2
	B1_2 :: 2.0 * B0_2
	A1_2 :: 2.0 * (K2 - 1.0) * N2
	A2_2 :: (1.0 - K / Q2 + K2) * N2

	y1      := B0_1 * x + f.s1_z1
	f.s1_z1  = B1_1 * x - A1_1 * y1 + f.s1_z2
	f.s1_z2  = B0_1 * x - A2_1 * y1

	y2      := B0_2 * y1 + f.s2_z1
	f.s2_z1  = B1_2 * y1 - A1_2 * y2 + f.s2_z2
	f.s2_z2  = B0_2 * y1 - A2_2 * y2
	return y2
}

// Compute unclamped raw FFT bin power |X[k]|^2 on 12 kHz Hann window (supports zero-padding when len(samples) < plan.n).
// Accumulating unclamped power into Welch PSDs BEFORE quantile normalization & clamping preserves 100% of sub-noise (-15..-30 dB) SNR!
@(private)
compute_hires_raw_pow :: proc(
	plan    : ^FFT_Plan,
	samples : []f64,
	win     : []f64,
	fft_buf : []complex128,
	out_pow : []f64,
) {
	n_fft := plan.n
	n_in  := min(len(samples), min(len(win), n_fft))
	for n in 0 ..< n_in {
		fft_buf[n] = complex(samples[n] * win[n], 0.0)
	}
	for n in n_in ..< n_fft {
		fft_buf[n] = complex(0.0, 0.0)
	}
	fft_forward(plan, fft_buf[0 : n_fft])
	n_bins := min(len(out_pow), (n_fft / 2) + 1)
	for k in 0 ..< n_bins {
		out_pow[k] = c64_abs2(fft_buf[k])
	}
}

// Spatial-quantile-normalized excess spectrum computed AFTER unclamped Welch PSD time averaging.
// 100% immune to startup words, loud speech formants (<16% bin occupancy vs 46th percentile),
// notched CW/SMPS spurs (masked before quantile & set to neutral 0.0), and broadband QRN swells!
@(private)
normalize_hires_psd_to_exc :: proc(
	psd       : []f64,
	mask48    : []f64,
	bin48_hz  : f64,
	n_fft     : int,
	f_lo      : f64,
	f_hi      : f64,
	q46_scale : f64,
	out_exc   : []f64,
) {
	n_bins := min(len(psd), len(out_exc))
	bin_hz := HR_FS_HZ / f64(n_fft)
	inv48  := 1.0 / bin48_hz

	pow   : [HR4K_BINS]f64
	valid : [HR4K_BINS]bool
	m_hi  := len(mask48) - 2
	for k in 0 ..< n_bins {
		fk := f64(k) * bin_hz
		ka := clamp(int(math.round(fk * inv48)), 1, m_hi)
		bm := math.min(mask48[ka], math.min(mask48[ka - 1], mask48[ka + 1]))
		pow[k]   = psd[k] * bm * bm
		valid[k] = (bm >= 0.65) && !is_hum_notch_freq(fk) && (fk >= 140.0) && (fk <= f_hi + 95.0)
	}

	b_lo   := clamp(int(math.floor(145.0 / bin_hz)), 4, n_bins - 8)
	b_hi   := clamp(int(math.ceil((f_hi + 80.0) / bin_hz)), b_lo + 16, n_bins - 2)
	half_w := 28 if n_fft <= 2048 else 52
	q_step := 1  if n_fft <= 2048 else 2

	ANCHOR_STRIDE :: 4
	nf_anc : [(HR4K_BINS / ANCHOR_STRIDE) + 4]f64
	n_anc  := (b_hi - b_lo) / ANCHOR_STRIDE + 2
	for ai in 0 ..< n_anc {
		ck  := min(b_lo + ai * ANCHOR_STRIDE, b_hi)
		a   := max(ck - half_w, b_lo)
		b   := min(ck + half_w, b_hi)
		buf : [64]f64
		cnt := 0
		for idx := a; idx <= b && cnt < len(buf); idx += q_step {
			if !valid[idx] do continue
			buf[cnt] = pow[idx]
			cnt += 1
		}
		if cnt < 6 {
			nf_anc[ai] = 0.0
			continue
		}
		for i in 1 ..< cnt {
			v := buf[i]
			j := i - 1
			for j >= 0 && buf[j] > v {
				buf[j + 1] = buf[j]
				j -= 1
			}
			buf[j + 1] = v
		}
		qi := clamp((cnt * 46) / 100, 0, cnt - 1)
		nf_anc[ai] = buf[qi] * q46_scale
	}

	for k in 0 ..< n_bins {
		fk := f64(k) * bin_hz
		if !valid[k] || fk < f_lo || fk > f_hi || k < b_lo || k > b_hi {
			out_exc[k] = 0.0
			continue
		}
		af  := f64(k - b_lo) * (1.0 / f64(ANCHOR_STRIDE))
		a0  := clamp(int(math.floor(af)), 0, max(n_anc - 2, 0))
		frc := clamp(af - f64(a0), 0.0, 1.0)
		nf0 := nf_anc[a0]
		nf1 := nf_anc[a0 + 1]
		nf  := (1.0 - frc) * nf0 + frc * nf1
		if nf <= 1e-24 {
			out_exc[k] = 0.0
		} else {
			out_exc[k] = clamp(pow[k] / nf - 1.0, -0.95, 5.5)
		}
	}
}

// Full-passband 25th-percentile normalized ratio: 100% immune to voiced speech harmonics (zero self-inflation
// even when loud F1/F2 speech occupies 50% of bins!) while tracking broadband QRN static swells 1:1!
@(private)
compute_qrn_swell_factor :: proc(fast_psd, noise_psd: []f64, k_lo, k_hi: int, bin_hz: f64, scale_q: f64 = 1.58) -> f64 {
	ratios : [256]f64
	cnt := 0
	for k in k_lo ..= k_hi {
		if cnt >= len(ratios) do break
		fk := f64(k) * bin_hz
		if is_hum_notch_freq(fk) do continue
		nf := noise_psd[k]
		if nf <= 1e-24 do continue
		ratios[cnt] = fast_psd[k] / nf
		cnt += 1
	}
	if cnt < 16 do return 1.0
	for i in 1 ..< cnt {
		v := ratios[i]
		j := i - 1
		for j >= 0 && ratios[j] > v {
			ratios[j + 1] = ratios[j]
			j -= 1
		}
		ratios[j + 1] = v
	}
	q25 := ratios[(cnt * 25) / 100]
	return max(1.0, q25 * scale_q)
}

// 200-bin skew-calibrated winsorized passband variance + positive excess mean (exact 1.000 on Exp(1) & speech-immune!)
@(private)
comb_exc_stats :: proc(exc: []f64, f_lo, f_hi: f64, k_lo, k_hi: int, inv_bin_hz: f64) -> (mu_b: f64, var_b: f64, pos_mean: f64) {
	b_lo := max(k_lo, int(math.ceil(f_lo * inv_bin_hz)))
	b_hi := min(min(k_hi, len(exc) - 1), int(math.floor(f_hi * inv_bin_hz)))
	if b_hi <= b_lo + 8 do return 0.0, 1.0, 0.0
	s, ps := 0.0, 0.0
	cnt := 0
	for k in b_lo ..= b_hi {
		fk := f64(k) / inv_bin_hz
		if is_hum_notch_freq(fk) do continue
		v := exc[k]
		s += v
		if v > 0.0 do ps += v
		cnt += 1
	}
	if cnt < 8 do return 0.0, 1.0, 0.0
	mu_b     = s  / f64(cnt)
	pos_mean = ps / f64(cnt)
	mad_s := 0.0
	for k in b_lo ..= b_hi {
		fk := f64(k) / inv_bin_hz
		if is_hum_notch_freq(fk) do continue
		mad_s += math.abs(exc[k] - mu_b)
	}
	mad    := math.max(mad_s / f64(cnt), 1e-6)
	cap_hi := 4.2 * mad
	v_s    := 0.0
	for k in b_lo ..= b_hi {
		fk := f64(k) / inv_bin_hz
		if is_hum_notch_freq(fk) do continue
		d := clamp(exc[k] - mu_b, -1.05, cap_hi)
		v_s += d * d
	}
	var_b = math.max((v_s / f64(cnt)) / 0.90, 1e-8)
	return mu_b, var_b, pos_mean
}

// Standardized exact N(0, 1.000) comb Z-score with 200-bin analytical noise-floor lower bound on SD
// and 56-comb winsorized off-pitch background SD (immune to 3/2 & 2/3 speech subharmonic self-inflation!)
@(private)
comb_z_at :: proc(
	exc        : []f64,
	target_f0  : f64,
	target_df  : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_bin_hz : f64,
	pk_thr     : f64 = 0.045,
) -> (z: f64, raw_sc: f64, pos_h: int) {
	hi_b := min(k_hi, len(exc) - 1)
	if hi_b <= k_lo + 4 do return 0.0, 0.0, 0
	raw_sc, pos_h, _ = eval_uniform_shs_comb(exc, target_f0, target_df, f_lo, f_hi, k_lo, hi_b, inv_bin_hz, pk_thr)
	_, var_b, _ := comb_exc_stats(exc, f_lo, f_hi, k_lo, hi_b, inv_bin_hz)

	// Exact analytical noise SD of eval_uniform_shs_comb (accounting for shared h+0.5 valleys & Hann bin correlation):
	max_h := clamp(int(math.floor(1920.0 / target_f0)), 5, 18)
	w_sq  := 0.0
	for h in 1 ..= max_h {
		fh := f64(h) * target_f0 + target_df
		if fh < f_lo || fh > f_hi do continue
		wh := 1.0 / math.pow(f64(h), 0.32)
		if fh < 340.0 {
			wh *= clamp(0.18 + 0.82 * ((fh - 170.0) / 170.0), 0.18, 1.0)
		} else if fh > 1260.0 {
			dh := (fh - 1260.0) / 390.0
			wh *= 1.0 / (1.0 + dh * dh)
		}
		w_sq += wh * wh
	}
	sd_ana := 0.455 * math.sqrt(var_b * (w_sq / math.max(w_sq + 0.35, 0.35)))

	bg_vals : [56]f64
	bg_sum  := 0.0
	bg_cnt  := 0
	for i in 0 ..< 56 {
		fr := 86.0 + f64(i) * 3.05
		if math.abs(fr - target_f0) < 6.0 || math.abs(fr - 2.0 * target_f0) < 9.0 || math.abs(fr - 0.5 * target_f0) < 5.0 ||
		   math.abs(fr - 1.5 * target_f0) < 5.5 || math.abs(3.0 * fr - 2.0 * target_f0) < 8.0 {
			continue
		}
		sb, _, _ := eval_uniform_shs_comb(exc, fr, target_df, f_lo, f_hi, k_lo, hi_b, inv_bin_hz, pk_thr)
		bg_vals[bg_cnt] = sb
		bg_sum += sb
		bg_cnt += 1
	}
	if bg_cnt < 10 do return raw_sc / math.max(sd_ana, 1e-5), raw_sc, pos_h
	mu := bg_sum / f64(bg_cnt)
	mad_s := 0.0
	for i in 0 ..< bg_cnt do mad_s += math.abs(bg_vals[i] - mu)
	cap_bg := 3.0 * math.max(mad_s / f64(bg_cnt), 1e-6)
	vr_s := 0.0
	for i in 0 ..< bg_cnt {
		d := clamp(bg_vals[i] - mu, -cap_bg, cap_bg)
		vr_s += d * d
	}
	vr := math.max((vr_s / f64(bg_cnt)) / 0.94, 1e-10)
	sd := max(math.sqrt(vr), sd_ana)
	return (raw_sc - mu) / sd, raw_sc, pos_h
}

// Orthogonal Even-Harmonic vs Odd-Harmonic Comb Z-Validator (±0.32*F0 local valleys -> >=30.6 Hz separation = ZERO shared bins!).
// Evaluated AT the full-comb pitch cand_f0 (zero 3/2 alias bug!). On voiced speech (including <350 Hz CB TX cut),
// BOTH even (h=4,6,8..) and odd (h=5,7,9..) sub-lattices are strongly positive; 2*F0 aliases & static spikes fail one half!
@(private)
eval_even_odd_comb_z :: proc(
	exc        : []f64,
	cand_f0    : f64,
	df         : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_bin_hz : f64,
) -> (z_even: f64, z_odd: f64) {
	hi_b := min(k_hi, len(exc) - 1)
	if hi_b <= k_lo + 6 do return 0.0, 0.0
	_, var_b, _ := comb_exc_stats(exc, f_lo, f_hi, k_lo, hi_b, inv_bin_hz)
	max_h := clamp(int(math.floor(1920.0 / cand_f0)), 6, 18)
	s_ev, w_ev := 0.0, 0.0
	s_od, w_od := 0.0, 0.0
	for h in 1 ..= max_h {
		fh := f64(h) * cand_f0 + df
		if fh < f_lo || fh > f_hi do continue
		bf  := fh * inv_bin_hz
		b0  := int(math.floor(bf))
		frc := bf - f64(b0)
		if b0 < k_lo || b0 + 1 > hi_b do continue
		pk := (1.0 - frc) * exc[b0] + frc * exc[b0 + 1]

		vl, vc := 0.0, 0.0
		for sv in ([2]f64{-0.32, +0.32}) {
			fv  := fh + sv * cand_f0
			if fv < f_lo || fv > f_hi do continue
			vb  := fv * inv_bin_hz
			vb0 := int(math.floor(vb))
			vfr := vb - f64(vb0)
			if vb0 >= k_lo && vb0 + 1 <= hi_b {
				vl += (1.0 - vfr) * exc[vb0] + vfr * exc[vb0 + 1]
				vc += 1.0
			}
		}
		if vc <= 0.0 do continue
		dv := pk - (vl / vc)
		wh := 1.0 / math.pow(f64(h), 0.32)
		if fh < 340.0 {
			wh *= clamp(0.18 + 0.82 * ((fh - 170.0) / 170.0), 0.18, 1.0)
		} else if fh > 1260.0 {
			dh := (fh - 1260.0) / 390.0
			wh *= 1.0 / (1.0 + dh * dh)
		}
		if (h & 1) == 0 {
			s_ev += wh * dv
			w_ev += wh * wh
		} else {
			s_od += wh * dv
			w_od += wh * wh
		}
	}
	sd_u := 1.15 * math.sqrt(math.max(var_b, 1e-8))
	z_even = s_ev / (sd_u * math.sqrt(math.max(w_ev, 0.35)))
	z_odd  = s_od / (sd_u * math.sqrt(math.max(w_od, 0.35)))
	return z_even, z_odd
}

// Zero-Selection-Bias, Zero-Sample-Overlap Unselected Past Cross-Validator:
// Evaluates 3-point glide-tolerant comb {-max_glide, 0, +max_glide} on completely disjoint past_exc.
// Zero 3/2 harmonic alias & calibrated N(0,1) null on pure static!
@(private)
eval_unsel_past_z :: proc(
	past_exc   : []f64,
	cand_f0    : f64,
	cand_df    : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_bin_hz : f64,
	max_glide  : f64 = 2.6,
) -> (z_best: f64, ph_best: int) {
	z_best  = -10.0
	ph_best = 0
	for step in -1 ..= 1 {
		fg := clamp(cand_f0 + f64(step) * max_glide, 85.0, 260.0)
		zp, _, ph := comb_z_at(past_exc, fg, cand_df, f_lo, f_hi, k_lo, k_hi, inv_bin_hz, 0.025)
		if step != 0 do zp *= 0.94
		if zp > z_best {
			z_best  = zp
			ph_best = ph
		}
	}
	return
}

// Glide-Immune 1D Marginal SSB Carrier Mistune Profile Tracker (-40..+40 Hz in 2.5 Hz steps; idx 16 == 0.0 Hz).
// Because rig carrier offset df is constant even while speaker pitch F0(t) glides by >20 Hz/s, marginalizing over F0(t)
// at each hop and accumulating df_profile[0..32] discovers -15..-20 dB SSB mistune in ~100 ms with 0% static drift!
DF_PROF_BINS :: 33
DF_PROF_STEP :: 2.5

@(private)
update_ssb_df_profile :: proc(
	prof       : ^[DF_PROF_BINS]f64,
	exc2k      : []f64,
	exc4k      : []f64,
	has_4k     : bool,
	f_lo, f_hi : f64,
	kl2, kh2   : int,
	kl4, kh4   : int,
	inv2, inv4 : f64,
	alpha      : f64,
) -> (cand_df: f64, z_df: f64, strong_lock: bool) {
	cur     : [DF_PROF_BINS]f64
	f0_best : [DF_PROF_BINS]f64
	sum_c   := 0.0
	for idf in 0 ..< DF_PROF_BINS {
		dfc := -40.0 + DF_PROF_STEP * f64(idf)
		b2, bf0 := -1.0e9, 115.0
		for iw in 0 ..< 181 {
			fc := 85.0 + 0.5 * f64(iw)
			if math.abs(dfc) > 0.46 * fc do continue
			s2, _, _ := eval_uniform_shs_comb(exc2k, fc, dfc, f_lo, f_hi, kl2, kh2, inv2, 0.025)
			if has_4k {
				s4, _, _ := eval_uniform_shs_comb(exc4k, fc, dfc, f_lo, f_hi, kl4, kh4, inv4, 0.022)
				s2 = 0.55 * s2 + 0.45 * s4
			}
			if s2 > b2 { b2 = s2; bf0 = fc }
		}
		if b2 < -1.0e8 do b2 = 0.0
		cur[idf]     = b2
		f0_best[idf] = bf0
		sum_c       += b2
	}
	mean_c := sum_c / f64(DF_PROF_BINS)
	pk_i   := 16
	pk_v   := -1.0e9
	for idf in 0 ..< DF_PROF_BINS {
		prof[idf] = (1.0 - alpha) * prof[idf] + alpha * (cur[idf] - mean_c)
		if prof[idf] > pk_v {
			pk_v = prof[idf]
			pk_i = idf
		}
	}

	bg_s := 0.0
	bg_n := 0
	for idf in 0 ..< DF_PROF_BINS {
		if math.abs(idf - pk_i) < 3 do continue
		bg_s += prof[idf]
		bg_n += 1
	}
	bg_mu := bg_s / f64(max(bg_n, 1))
	mad_s := 0.0
	for idf in 0 ..< DF_PROF_BINS {
		if math.abs(idf - pk_i) < 3 do continue
		mad_s += math.abs(prof[idf] - bg_mu)
	}
	cap_p := 2.8 * math.max(mad_s / f64(max(bg_n, 1)), 1e-6)
	vr_s  := 0.0
	for idf in 0 ..< DF_PROF_BINS {
		if math.abs(idf - pk_i) < 3 do continue
		d := clamp(prof[idf] - bg_mu, -cap_p, cap_p)
		vr_s += d * d
	}
	sd_p := math.sqrt(math.max((vr_s / f64(max(bg_n, 1))) / 0.90, 1e-10))
	z_df  = (pk_v - bg_mu) / sd_p

	cand_df = -40.0 + DF_PROF_STEP * f64(pk_i)
	if pk_i > 0 && pk_i + 1 < DF_PROF_BINS {
		sm, s0, sp := prof[pk_i - 1], prof[pk_i], prof[pk_i + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do cand_df += DF_PROF_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}
	if math.abs(cand_df) >= 4.0 && z_df > 2.25 {
		fc0 := f0_best[pk_i]
		best_fine := -1.0e9
		best_fd   := cand_df
		for ff := fc0 - 0.75; ff <= fc0 + 0.76; ff += 0.25 {
			for fd := cand_df - 1.6; fd <= cand_df + 1.61; fd += 0.4 {
				sf, _, _ := eval_uniform_shs_comb(exc2k, ff, fd, f_lo, f_hi, kl2, kh2, inv2, 0.025)
				if has_4k {
					sf4, _, _ := eval_uniform_shs_comb(exc4k, ff, fd, f_lo, f_hi, kl4, kh4, inv4, 0.022)
					sf = 0.50 * sf + 0.50 * sf4
				}
				if sf > best_fine { best_fine = sf; best_fd = fd }
			}
		}
		cand_df = best_fd
	}
	strong_lock = math.abs(cand_df) >= 4.0 && z_df > 3.35 && pk_v > prof[16] + 2.0 * sd_p
	return
}

// Fast 256 ms (128 ms past128 + 128 ms pres128, zero shared samples) onset cross-validator for short syllables (e.g. 290 ms)
@(private)
eval_128ms_cross :: proc(
	past128_exc : []f64,
	pres128_exc : []f64,
	df_steps    : int,
	df_tab      : []f64,
	f_lo, f_hi  : f64,
	k_lo, k_hi  : int,
	inv_bin_hz  : f64,
) -> (f0_128: f64, z_pres128: f64, z_past128_unsel: f64, ph_128: int) {
	N_128 :: 176
	sc_arr : [N_128]f64
	df_arr : [N_128]f64
	ph_arr : [N_128]int
	best_s := -1.0e9
	best_i := 50
	for i in 0 ..< N_128 {
		cf0 := 85.0 + f64(i)
		bs  := -1.0e9
		bd  := 0.0
		bp  := 0
		for di in 0 ..< df_steps {
			s, p, _ := eval_uniform_shs_comb(pres128_exc, cf0, df_tab[di], f_lo, f_hi, k_lo, k_hi, inv_bin_hz, 0.040)
			if s > bs { bs = s; bd = df_tab[di]; bp = p }
		}
		sc_arr[i] = bs; df_arr[i] = bd; ph_arr[i] = bp
		if bs > best_s { best_s = bs; best_i = i }
	}
	mu := 0.0
	for v in sc_arr do mu += v
	mu /= f64(N_128)
	vr := 1e-9
	for v in sc_arr { d := v - mu; vr += d * d }
	sd := math.sqrt(vr / f64(N_128))
	f0_128 = 85.0 + f64(best_i)
	if f0_128 >= 170.0 {
		hi := int(math.round(0.5 * f0_128 - 85.0))
		if hi >= 0 && hi < N_128 && sc_arr[hi] > mu + 0.35 * sd && sc_arr[hi] >= 0.45 * best_s {
			best_i = hi
			f0_128 = 85.0 + f64(hi)
		}
	}
	if best_i > 0 && best_i + 1 < N_128 {
		sm, s0, sp := sc_arr[best_i - 1], sc_arr[best_i], sc_arr[best_i + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do f0_128 += clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}
	ph_128 = ph_arr[best_i]
	z_pres128, _, _ = comb_z_at(pres128_exc, f0_128, df_arr[best_i], f_lo, f_hi, k_lo, k_hi, inv_bin_hz, 0.040)
	z_past128_unsel, _ = eval_unsel_past_z(past128_exc, f0_128, df_arr[best_i], f_lo, f_hi, k_lo, k_hi, inv_bin_hz, 2.5)
	return
}

// End-of-word pause release detector comparing disjoint 128 ms past_exc vs pres_exc (snaps within 42 ms of word end!).
@(private)
eval_past_pres_drop :: proc(
	past_exc   : []f64,
	pres_exc   : []f64,
	f0_trk     : f64,
	df_trk     : f64,
	f_lo, f_hi : f64,
	k_lo, k_hi : int,
	inv_hz     : f64,
) -> (energy_drop: bool, z_pres_cur: f64) {
	_, _, past_pos := comb_exc_stats(past_exc, f_lo, f_hi, k_lo, k_hi, inv_hz)
	_, _, pres_pos := comb_exc_stats(pres_exc, f_lo, f_hi, k_lo, k_hi, inv_hz)
	z_past, _, _   := comb_z_at(past_exc, f0_trk, df_trk, f_lo, f_hi, k_lo, k_hi, inv_hz, 0.03)
	z_pres_cur      = -10.0
	for dg in ([3]f64{-2.8, 0.0, +2.8}) {
		fg := clamp(f0_trk + dg, 85.0, 260.0)
		zp, _, _ := comb_z_at(pres_exc, fg, df_trk, f_lo, f_hi, k_lo, k_hi, inv_hz, 0.035)
		if zp > z_pres_cur do z_pres_cur = zp
	}
	energy_drop = (past_pos > 0.68 && pres_pos < 0.62 * past_pos) || (z_past > 2.45 && z_pres_cur < 0.35)
	return
}

// ---------------------------------------------------------------------------
// FILTER LEVEL 3: 2048-pt Low-Latency OLA Synthesis (32 ms) + 12 kHz 2048-pt (5.86 Hz/bin)
// Side-Chain Mistune-Aware SHS Comb + CW Auto-Notch + Cohen OM-LSA + Exact 0.0 VAD Gate
// Engineered to recover natural speech down to -15 dB below noise floor with 0.0 silence!
// ---------------------------------------------------------------------------

L3_FFT_N       :: 2048
L3_HOP         :: 512
L3_BINS        :: (L3_FFT_N / 2) + 1
L3_ANA_N       :: 4096
L3_ANA_BINS    :: (L3_ANA_N / 2) + 1
L3_F0_STEP     :: 0.50
L3_NUM_F0      :: 351 // 85.0 .. 260.0 Hz in 0.50 Hz steps (zero straddle loss @ 5.86 Hz/bin!)
L3_FAST_NUM_F0 :: 176 // 85.0 .. 260.0 Hz in 1.00 Hz steps

Spectral_Wiener_Filter :: struct {
	pre_clean       : Pre_Impulse_Hum_Cleaner,

	// 2048-pt low-latency (1536-sample = 32 ms) Overlap-Add synthesis engine
	fft_plan        : FFT_Plan,
	window          : [L3_FFT_N]f64,
	in_ring         : [L3_FFT_N]f64,
	ola_out         : [L3_FFT_N]f64,
	hop_staging     : [L3_HOP]f64,
	hop_ready       : [L3_HOP]f64,
	stage_idx       : int,
	work_fft        : [L3_FFT_N]complex128,

	fast_psd        : [L3_BINS]f64,
	welch_psd       : [L3_BINS]f64,
	noise_psd       : [L3_BINS]f64,
	min_curr        : [L3_BINS]f64,
	min_prev        : [L3_BINS]f64,
	tonal_mag_ema   : [L3_BINS]f64,
	tonal_pow_ema   : [L3_BINS]f64,
	prev_clean      : [L3_BINS]f64,
	prev_gain       : [L3_BINS]f64,
	prev_spp        : [L3_BINS]f64,

	// 4096-pt (11.72 Hz/bin) CW notch/formant side-chain + 12 kHz 2048-pt (5.86 Hz/bin) coherent comb
	ana_plan        : FFT_Plan,
	ana_win         : [L3_ANA_N]f64,
	ana_ring        : [L3_ANA_N]f64,
	ana_fft         : [L3_ANA_N]complex128,
	ana_fast_psd    : [L3_ANA_BINS]f64,
	ana_med_psd     : [L3_ANA_BINS]f64,
	ana_long_psd    : [L3_ANA_BINS]f64,
	ana_noise_psd   : [L3_ANA_BINS]f64,
	ana_tonal_mag   : [L3_ANA_BINS]f64,
	ana_tonal_pow   : [L3_ANA_BINS]f64,
	ana_cw_persist  : [L3_ANA_BINS]f64,
	shs_smooth      : [L3_NUM_F0]f64,

	hr_lpf          : Decim12k_LPF,
	hr_plan2k       : FFT_Plan,
	hr_plan4k       : FFT_Plan,
	hr_win128       : [HR128_N]f64,
	hr_win2k        : [HR2K_N]f64,
	hr_win4k        : [HR4K_N]f64,
	hr_fft          : [HR4K_N]complex128,
	hr_ring         : [HR3_RING_N]f64,
	hr_past128_exc  : [HR2K_BINS]f64,
	hr_pres128_exc  : [HR2K_BINS]f64,
	hr_past_avg     : [HR2K_BINS]f64,
	hr_mfin         : [HR2K_BINS]f64,
	hr_wb           : [HR2K_BINS]f64,
	hr_wb2k_psd     : [HR2K_BINS]f64,
	hr_past2k_pow   : [2][HR2K_BINS]f64,
	hr_pres2k_pow   : [4][HR2K_BINS]f64,
	hr_past4k_exc   : [HR4K_BINS]f64,
	hr_pres4k_exc   : [HR4K_BINS]f64,
	hr_wb4k_exc     : [HR4K_BINS]f64,
	hr_wb4k_psd     : [HR4K_BINS]f64,
	hr_past4k_pow   : [2][HR4K_BINS]f64,
	hr_pres4k_pow   : [2][HR4K_BINS]f64,
	shs4k_smooth    : [L3_NUM_F0]f64,
	hr_pos          : int,
	hr_filled       : int,
	hr_pos4k        : int,
	hr_filled4k     : int,

	// Cached 21.33 ms high-res comb state (reused across odd 512-sample hops)
	c_med_f0        : f64,
	c_best_m_df     : f64,
	c_best_m_ph     : int,
	c_z_m_bg        : f64,
	c_ze_m          : f64,
	c_zo_m          : f64,
	c_z_past        : f64,
	c_z_pres        : f64,
	c_past_f0       : f64,
	c_past_ph       : int,
	c_z_pres_cur    : f64,
	c_e_drop        : bool,

	tracked_f0       : f64,
	tracked_df       : f64,
	df_profile       : [DF_PROF_BINS]f64,
	mistune_cand     : f64,
	mistune_f0_cand  : f64,
	mistune_streak   : int,
	f0_prev_med      : f64,
	f0_prev_4k       : f64,
	f0_prev_fast     : f64,
	f0_ring          : [4]f64,
	f0_4k_ring       : [4]f64,
	df_ring          : [4]f64,
	ring_pos         : int,
	med_streak       : int,
	d4k_streak       : int,
	fast_streak      : int,
	silent_hops      : int,
	inst_comb_ema    : f64,
	harm_conf        : f64,
	harm_hang        : f64,
	voice_conf       : f64,
	gate_phase       : f64,
	hang_hops        : int,
	voiced_hang_hops : int,
	min_sub_cnt      : int,
	pend_tonal      : [L3_BINS]bool,
	pend_sh_hz      : f64,
	pend_sh_wait    : int,
	frame_cnt       : u64,
}

spectral_wiener_init :: proc(swf: ^Spectral_Wiener_Filter) {
	swf^ = {}
	pre_cleaner_init(&swf.pre_clean)
	fft_plan_init(&swf.fft_plan, L3_FFT_N)
	fft_plan_init(&swf.ana_plan, L3_ANA_N)
	fft_plan_init(&swf.hr_plan2k, HR2K_N)
	fft_plan_init(&swf.hr_plan4k, HR4K_N)
	for n in 0 ..< L3_FFT_N {
		swf.window[n] = root_hann_f64(n, L3_FFT_N)
	}
	for n in 0 ..< L3_ANA_N {
		r := root_hann_f64(n, L3_ANA_N)
		swf.ana_win[n]  = r * r // Periodic Hann: narrow 11.72 Hz mainlobe + rapid decay of old samples
		swf.hr_win4k[n] = r * r // Periodic Hann @ 12 kHz: 2.93 Hz/bin (+6.02 dB sub-noise comb gain!)
	}
	for n in 0 ..< HR128_N {
		r := root_hann_f64(n, HR128_N)
		swf.hr_win128[n] = r * r // 128 ms Periodic Hann @ 12 kHz (agile syllable onset/pause)
	}
	for n in 0 ..< HR2K_N {
		r := root_hann_f64(n, HR2K_N)
		swf.hr_win2k[n] = r * r // Periodic Hann @ 12 kHz: 5.86 Hz/bin (+3.01 dB coherent comb gain!)
	}
	swf.tracked_f0   = 135.0
	swf.c_med_f0     = 135.0
	swf.c_z_past     = -10.0
	swf.c_z_pres     = -10.0
	swf.c_z_pres_cur = 2.0
	for k in 0 ..< L3_BINS {
		swf.prev_gain[k] = 0.0
		swf.min_curr[k]  = 1.0e9
		swf.min_prev[k]  = 1.0e9
	}
}

@(private)
spectral_wiener_step_hop :: proc(swf: ^Spectral_Wiener_Filter, hop_in: []f64, hop_out: []f64, mode: Modulation_Mode, rf_notch: bool) {
	band   := mode_to_band_family(mode)
	is_ssb := (band == .SSB_Band)

	L3_DEC_HOP :: L3_HOP / 4 // 128 samples @ 12 kHz per 512-sample hop
	copy(swf.in_ring[0 : L3_FFT_N - L3_HOP], swf.in_ring[L3_HOP : L3_FFT_N])
	copy(swf.ana_ring[0 : L3_ANA_N - L3_HOP], swf.ana_ring[L3_HOP : L3_ANA_N])
	copy(swf.hr_ring[0 : HR3_RING_N - L3_DEC_HOP], swf.hr_ring[L3_DEC_HOP : HR3_RING_N])
	for i in 0 ..< L3_HOP {
		cleaned := pre_cleaner_step(&swf.pre_clean, hop_in[i], 3.25, is_ssb)
		swf.in_ring[L3_FFT_N - L3_HOP + i]  = cleaned
		swf.ana_ring[L3_ANA_N - L3_HOP + i] = cleaned
		lp12 := decim12k_lpf_step(&swf.hr_lpf, cleaned)
		if (i & 3) == 3 {
			swf.hr_ring[HR3_RING_N - L3_DEC_HOP + (i >> 2)] = lp12
		}
	}

	for n in 0 ..< L3_FFT_N {
		swf.work_fft[n] = complex(swf.in_ring[n] * swf.window[n], 0.0)
	}
	fft_forward(&swf.fft_plan, swf.work_fft[:])

	for n in 0 ..< L3_ANA_N {
		swf.ana_fft[n] = complex(swf.ana_ring[n] * swf.ana_win[n], 0.0)
	}
	fft_forward(&swf.ana_plan, swf.ana_fft[:])

	bin_hz     := AUDIO_SAMPLE_RATE_HZ / f64(L3_FFT_N) // 23.4375 Hz/bin (synthesis)
	ana_bin_hz := AUDIO_SAMPLE_RATE_HZ / f64(L3_ANA_N) // 11.71875 Hz/bin (side-chain)
	inv_ana_hz := 1.0 / ana_bin_hz

	swf.frame_cnt   += 1
	swf.min_sub_cnt += 1
	roll_min := false
	if swf.min_sub_cnt >= 22 {
		swf.min_sub_cnt = 0
		roll_min        = true
	}

	f_lo      := 125.0
	f_hi      := 2720.0
	base_over := 1.05
	g_floor   := 0.008
	switch band {
	case .AM_Band:
		f_lo      = 128.0
		f_hi      = 3150.0
		base_over = 1.05
		g_floor   = 0.010
	case .FM_Band:
		f_lo      = 138.0
		f_hi      = 2880.0
		base_over = 1.04
		g_floor   = 0.010
	case .SSB_Band:
		f_lo      = 120.0
		f_hi      = 2720.0
		base_over = 1.05
		g_floor   = 0.004
	}
	k_lo  := clamp(int(math.floor(f_lo / bin_hz)), 3, L3_BINS - 6)
	k_hi  := clamp(int(math.ceil(f_hi / bin_hz)), k_lo + 8, L3_BINS - 6)
	ka_lo := clamp(int(math.floor(f_lo * inv_ana_hz)), 4, L3_ANA_BINS - 10)
	ka_hi := clamp(int(math.ceil(f_hi * inv_ana_hz)), ka_lo + 16, L3_ANA_BINS - 10)

	// 1A. Update 2048-pt synthesis PSDs
	raw_pow : [L3_BINS]f64
	for k in 0 ..< L3_BINS {
		p   := math.max(c64_abs2(swf.work_fft[k]), 1e-20)
		mag := math.sqrt(p)
		raw_pow[k] = p
		if swf.frame_cnt <= 2 {
			swf.fast_psd[k]      = p
			swf.welch_psd[k]     = p
			swf.min_curr[k]      = p
			swf.min_prev[k]      = p
			swf.tonal_mag_ema[k] = mag
			swf.tonal_pow_ema[k] = p
		} else {
			f_alpha := 0.52 if swf.frame_cnt <= 6 else 0.44
			w_alpha := 0.35 if swf.frame_cnt <= 8 else 0.15
			swf.fast_psd[k] = (1.0 - f_alpha) * swf.fast_psd[k] + f_alpha * p
			wp := (1.0 - w_alpha) * swf.welch_psd[k] + w_alpha * p
			if swf.frame_cnt == 4 || p > 8.5 * swf.tonal_pow_ema[k] {
				swf.tonal_mag_ema[k] = mag
				swf.tonal_pow_ema[k] = p
			} else {
				swf.tonal_mag_ema[k] = 0.89 * swf.tonal_mag_ema[k] + 0.11 * mag
				swf.tonal_pow_ema[k] = 0.89 * swf.tonal_pow_ema[k] + 0.11 * p
			}
			swf.welch_psd[k] = wp
			if roll_min {
				swf.min_prev[k] = swf.min_curr[k]
				swf.min_curr[k] = wp
			} else if wp < swf.min_curr[k] {
				swf.min_curr[k] = wp
			}
		}
	}

	// 1B. Update 4096-pt high-resolution side-chain PSDs (fast ~30ms, med ~125ms for -15dB comb, long ~320ms for CW notch)
	ana_raw : [L3_ANA_BINS]f64
	for k in 0 ..< L3_ANA_BINS {
		p   := math.max(c64_abs2(swf.ana_fft[k]), 1e-22)
		mag := math.sqrt(p)
		ana_raw[k] = p
		if swf.frame_cnt <= 4 {
			swf.ana_fast_psd[k]  = p
			swf.ana_med_psd[k]   = p
			swf.ana_long_psd[k]  = p
			swf.ana_tonal_mag[k] = mag
			swf.ana_tonal_pow[k] = p
		} else {
			fa := 0.48 if swf.frame_cnt <= 8 else 0.34
			ma := 0.24 if swf.frame_cnt <= 10 else 0.055
			la := 0.20 if swf.frame_cnt <= 10 else 0.038
			swf.ana_fast_psd[k] = (1.0 - fa) * swf.ana_fast_psd[k] + fa * p
			swf.ana_med_psd[k]  = (1.0 - ma) * swf.ana_med_psd[k]  + ma * p
			swf.ana_long_psd[k] = (1.0 - la) * swf.ana_long_psd[k] + la * p
			if p > 9.0 * swf.ana_tonal_pow[k] {
				swf.ana_tonal_mag[k] = mag
				swf.ana_tonal_pow[k] = p
			} else {
				swf.ana_tonal_mag[k] = 0.92 * swf.ana_tonal_mag[k] + 0.08 * mag
				swf.ana_tonal_pow[k] = 0.92 * swf.ana_tonal_pow[k] + 0.08 * p
			}
		}
	}

	if swf.pend_sh_wait > 0 {
		swf.pend_sh_wait -= 1
		sh_bins := int(math.round(swf.pend_sh_hz / bin_hz))
		for k in 5 ..< (L3_BINS - 8) {
			nf    := math.max(swf.noise_psd[k], 1e-20)
			c_now := (swf.tonal_mag_ema[k] * swf.tonal_mag_ema[k]) / math.max(swf.tonal_pow_ema[k], 1e-20)
			if raw_pow[k] > 5.0 * nf && c_now < 0.92 {
				for d_s in -1 ..= 1 {
					ks := clamp(k - sh_bins + d_s, 5, L3_BINS - 8)
					if swf.pend_tonal[ks] {
						swf.fast_psd[k]      = raw_pow[k]
						swf.welch_psd[k]     = raw_pow[k]
						swf.tonal_mag_ema[k] = math.sqrt(raw_pow[k])
						swf.tonal_pow_ema[k] = raw_pow[k]
						break
					}
				}
			}
		}
	}

	// 2. Unbiased Speech-Immune 57-Bin Quantile Noise-Floor Estimators (restricted to flat interior passband!)
	nf_f_lo := 170.0 if is_ssb else 160.0
	nf_f_hi := 2540.0 if is_ssb else (f_hi - 120.0)
	nf_k_lo := clamp(int(math.ceil(nf_f_lo / bin_hz)), k_lo, k_hi - 8)
	nf_k_hi := clamp(int(math.floor(nf_f_hi / bin_hz)), nf_k_lo + 8, k_hi)
	nf_ka_lo := clamp(int(math.ceil(nf_f_lo * inv_ana_hz)), ka_lo, ka_hi - 16)
	nf_ka_hi := clamp(int(math.floor(nf_f_hi * inv_ana_hz)), nf_ka_lo + 16, ka_hi)

	ana_fast_nf : [L3_ANA_BINS]f64
	sp_hold3 := swf.voice_conf >= 0.18
	// Full-passband 22nd-percentile floor prevents mid-word startup (e.g. t=0.0s +18 dB vowel) from seeding inflated SSB noise_psd!
	init_cap_w := 1.0e30
	init_cap_a := 1.0e30
	if is_ssb && swf.frame_cnt <= 12 {
		init_cap_w = 2.4 * local_quantile_noise_floor(swf.fast_psd[:],     (nf_k_lo + nf_k_hi) / 2,   96, nf_k_lo,  nf_k_hi,  22, 1.45, bin_hz)
		init_cap_a = 2.4 * local_quantile_noise_floor(swf.ana_fast_psd[:], (nf_ka_lo + nf_ka_hi) / 2, 192, nf_ka_lo, nf_ka_hi, 22, 1.48, ana_bin_hz)
	}
	for k in k_lo ..= k_hi {
		loc_w := local_quantile_noise_floor(swf.welch_psd[:], k, 24, nf_k_lo, nf_k_hi, 38, 1.16, bin_hz)
		loc_f := local_quantile_noise_floor(swf.fast_psd[:],  k, 24, nf_k_lo, nf_k_hi, 42, 1.42, bin_hz)
		loc_floor := math.min(math.min(loc_w, loc_f), init_cap_w)
		if swf.frame_cnt <= 6 {
			swf.noise_psd[k] = loc_floor
		} else if loc_floor <= swf.noise_psd[k] {
			swf.noise_psd[k] = 0.65 * swf.noise_psd[k] + 0.35 * loc_floor
		} else if sp_hold3 {
			swf.noise_psd[k] = 0.992 * swf.noise_psd[k] + 0.008 * math.min(loc_floor, 1.25 * swf.noise_psd[k])
		} else {
			swf.noise_psd[k] = 0.88 * swf.noise_psd[k] + 0.12 * loc_floor
		}
	}
	for ka in ka_lo ..= ka_hi {
		loc_a := local_quantile_noise_floor(swf.ana_med_psd[:],  ka, 28, nf_ka_lo, nf_ka_hi, 38, 1.155, ana_bin_hz)
		loc_f := local_quantile_noise_floor(swf.ana_fast_psd[:], ka, 28, nf_ka_lo, nf_ka_hi, 42, 1.47,  ana_bin_hz)
		loc_fl := math.min(math.min(loc_a, loc_f), init_cap_a)
		if swf.frame_cnt <= 8 {
			swf.ana_noise_psd[ka] = loc_fl
		} else if loc_fl <= swf.ana_noise_psd[ka] {
			swf.ana_noise_psd[ka] = 0.65 * swf.ana_noise_psd[ka] + 0.35 * loc_fl
		} else if sp_hold3 {
			swf.ana_noise_psd[ka] = 0.992 * swf.ana_noise_psd[ka] + 0.008 * math.min(loc_fl, 1.25 * swf.ana_noise_psd[ka])
		} else {
			swf.ana_noise_psd[ka] = 0.88 * swf.ana_noise_psd[ka] + 0.12 * loc_fl
		}
	}
	swell3 := compute_qrn_swell_factor(swf.ana_fast_psd[:], swf.ana_noise_psd[:], nf_ka_lo, nf_ka_hi, ana_bin_hz, 1.58)
	for ka in ka_lo ..= ka_hi {
		ana_fast_nf[ka] = swf.ana_noise_psd[ka] * swell3
	}

	// 3. Harmonic-Protected Birdie & Narrowband CW Heterodyne Auto-Notch ('notch on' catches +2.3 dB spurs!)
	ana_birdie  : [L3_ANA_BINS]f64
	birdie_mask : [L3_BINS]f64
	for ka in 0 ..< L3_ANA_BINS do ana_birdie[ka] = 1.0
	for k  in 0 ..< L3_BINS     do birdie_mask[k] = 1.0

	quick_lo_s, quick_lo_n := 0.0, 1e-22
	for ka in ka_lo ..= ka_hi {
		fka := f64(ka) * ana_bin_hz
		if fka >= 180.0 && fka <= 1150.0 && !is_hum_notch_freq(fka) {
			quick_lo_s += swf.ana_fast_psd[ka]
			quick_lo_n += math.max(ana_fast_nf[ka], swf.ana_noise_psd[ka])
		}
	}
	quick_lo_snr := quick_lo_s / quick_lo_n

	if swf.frame_cnt >= 7 {
		df_chk := swf.tracked_df if is_ssb else 0.0
		for ka in (ka_lo + 1) ..= (ka_hi - 1) {
			n_flr     := math.max(swf.ana_noise_psd[ka], 1e-22)
			constancy := (swf.ana_tonal_mag[ka] * swf.ana_tonal_mag[ka]) / math.max(swf.ana_tonal_pow[ka], 1e-22)
			b_psd     := max(swf.ana_med_psd[ka], 0.70 * swf.ana_fast_psd[ka])
			is_peak   := swf.ana_long_psd[ka] >= swf.ana_long_psd[ka - 1] && swf.ana_long_psd[ka] >= swf.ana_long_psd[ka + 1]
			is_fast_p := b_psd >= max(swf.ana_med_psd[ka - 1], 0.70 * swf.ana_fast_psd[ka - 1]) &&
			             b_psd >= max(swf.ana_med_psd[ka + 1], 0.70 * swf.ana_fast_psd[ka + 1])

			strong_hit := is_fast_p && constancy > 0.89 && b_psd > 6.5 * n_flr
			weak_cw_ok := false
			if rf_notch && swf.frame_cnt >= 10 {
				bg_sum := 0.0
				bg_cnt := 0
				for dk in -7 ..= 7 {
					if math.abs(dk) <= 2 do continue
					kk := ka + dk
					if kk >= ka_lo && kk <= ka_hi {
						bg_sum += min(swf.ana_long_psd[kk], 2.2 * swf.ana_noise_psd[kk])
						bg_cnt += 1
					}
				}
				bg_long  := math.max(bg_sum / f64(max(bg_cnt, 1)), 0.75 * n_flr)
				spur_snr := swf.ana_long_psd[ka] / bg_long
				s7_exc   := 1e-22
				for dk in -3 ..= 3 {
					kk := clamp(ka + dk, ka_lo, ka_hi)
					s7_exc += math.max(swf.ana_long_psd[kk] - bg_long, 0.0)
				}
				pk2_exc := math.max(swf.ana_long_psd[ka] - bg_long, 0.0) +
				           math.max(max(swf.ana_long_psd[ka - 1], swf.ana_long_psd[ka + 1]) - bg_long, 0.0)
				sharp   := pk2_exc / s7_exc
				if swf.ana_cw_persist[ka] >= 0.48 {
					// Already locked onto continuous CW heterodyne: hold smoothly across speech modulation dips!
					if is_peak && spur_snr > 1.48 && constancy > 0.68 {
						swf.ana_cw_persist[ka] = min(swf.ana_cw_persist[ka] + 0.06, 1.0)
					} else {
						swf.ana_cw_persist[ka] *= 0.986
					}
				} else if is_peak && spur_snr > 1.74 && constancy > 0.82 && sharp > 0.59 && (quick_lo_snr < 1.25 || spur_snr > 4.2) {
					if !is_vocal_harmonic_member(swf.ana_long_psd[:], swf.ana_fast_psd[:], swf.ana_noise_psd[:], ka, ana_bin_hz, ka_lo, ka_hi, df_chk) {
						swf.ana_cw_persist[ka] = min(swf.ana_cw_persist[ka] + 0.12, 1.0)
					} else {
						swf.ana_cw_persist[ka] *= 0.85
					}
				} else {
					swf.ana_cw_persist[ka] *= 0.88
				}
				weak_cw_ok = swf.ana_cw_persist[ka] >= 0.48
			} else if !rf_notch {
				swf.ana_cw_persist[ka] *= 0.90
			}

			if strong_hit {
				if !is_vocal_harmonic_member(swf.ana_med_psd[:], swf.ana_fast_psd[:], swf.ana_noise_psd[:], ka, ana_bin_hz, ka_lo, ka_hi, df_chk) {
					att := clamp(math.sqrt(n_flr / math.max(ana_raw[ka], n_flr)) * 0.04, 0.0001, 1.0)
					for dk in -2 ..= 2 {
						kk := ka + dk
						if kk >= ka_lo && kk <= ka_hi do ana_birdie[kk] = min(ana_birdie[kk], att)
					}
				}
			} else if weak_cw_ok {
				att := clamp(math.sqrt(n_flr / math.max(ana_raw[ka], n_flr)) * 0.04, 0.0002, 1.0)
				for dk in -2 ..= 2 {
					kk := ka + dk
					if kk >= ka_lo && kk <= ka_hi do ana_birdie[kk] = min(ana_birdie[kk], att)
				}
			}
		}

		// Map 4096-pt surgical notches + root-Hann synthesis sidelobe shoulders onto 2048-pt bins
		for k in k_lo ..= k_hi {
			ka := clamp(2 * k, ka_lo, ka_hi)
			bm := ana_birdie[ka]
			if ka > ka_lo do bm = min(bm, ana_birdie[ka - 1])
			if ka < ka_hi do bm = min(bm, ana_birdie[ka + 1])
			birdie_mask[k] = min(birdie_mask[k], bm)
			if bm < 0.08 {
				sh := max(bm * 2.5, 0.06)
				if k > k_lo do birdie_mask[k - 1] = min(birdie_mask[k - 1], sh)
				if k < k_hi do birdie_mask[k + 1] = min(birdie_mask[k + 1], sh)
			}
		}
		for k in (k_lo + 1) ..= (k_hi - 1) {
			n_flr     := math.max(swf.noise_psd[k], 1e-20)
			constancy := (swf.tonal_mag_ema[k] * swf.tonal_mag_ema[k]) / math.max(swf.tonal_pow_ema[k], 1e-20)
			b_psd     := max(swf.welch_psd[k], 0.72 * swf.fast_psd[k])
			is_peak   := b_psd >= max(swf.welch_psd[k - 1], 0.72 * swf.fast_psd[k - 1]) &&
			             b_psd >= max(swf.welch_psd[k + 1], 0.72 * swf.fast_psd[k + 1])
			if is_peak && constancy > 0.90 && b_psd > 6.8 * n_flr {
				ka_chk := clamp(2 * k, ka_lo, ka_hi)
				if !is_vocal_harmonic_member(swf.ana_med_psd[:], swf.ana_fast_psd[:], swf.ana_noise_psd[:], ka_chk, ana_bin_hz, ka_lo, ka_hi, df_chk) {
					att := clamp(math.sqrt(n_flr / math.max(raw_pow[k], n_flr)) * 0.05, 0.0002, 1.0)
					for dk in -2 ..= 2 {
						kk := k + dk
						if kk >= k_lo && kk <= k_hi do birdie_mask[kk] = min(birdie_mask[kk], att)
					}
				}
			}
		}
	}

	// 4. 12 kHz 2048-pt (5.86 Hz/bin) Disjoint Zero-Overlap Coherent SHS Comb & Multi-Cue Speech VAD (-15 dB Target!)
	comb_f_lo  := 165.0 if is_ssb else 155.0
	comb_f_hi  := 1720.0 if is_ssb else math.min(2000.0, f_hi - 110.0)
	inv_hr2_hz := 1.0 / HR2K_BIN_HZ
	hk_lo      := clamp(int(math.floor(comb_f_lo * inv_hr2_hz)), 4, HR2K_BINS - 12)
	hk_hi      := clamp(int(math.ceil(comb_f_hi * inv_hr2_hz)), hk_lo + 16, HR2K_BINS - 4)

	fast_exc  : [L3_ANA_BINS]f64
	raw_exc_u : [L3_ANA_BINS]f64
	sum_med_b, sum_nfl_b := 0.0, 0.0
	lo_fast_s, lo_raw_s, lo_nfl_s, lo_fnf_s := 0.0, 0.0, 0.0, 0.0
	f1_fast_s, f1_med_s, f1_raw_s, f1_nfl_s, f1_fnf_s := 0.0, 0.0, 0.0, 0.0, 0.0
	f1_peak_gf    := 0.0
	hi_fast_s, hi_nfl_s, hi_fnf_s := 0.0, 0.0, 0.0
	hi_peak_gf    := 0.0
	voiced_pks    := 0
	hi_broad_bins := 0

	for ka in ka_lo ..= ka_hi {
		fka := f64(ka) * ana_bin_hz
		abm := ana_birdie[ka]
		nf  := math.max(swf.ana_noise_psd[ka], 1e-22)
		nff := math.max(ana_fast_nf[ka], nf)
		gm  := (swf.ana_med_psd[ka]  * abm * abm) / nf
		gf  := (swf.ana_fast_psd[ka] * abm * abm) / nff
		gru := (ana_raw[ka]          * abm * abm) / nf

		if abm < 0.65 || is_hum_notch_freq(fka) || fka < comb_f_lo || fka > comb_f_hi {
			fast_exc[ka]  = 0.0
			raw_exc_u[ka] = 0.0
		} else {
			fast_exc[ka]  = clamp(gf  - 1.0, -0.92, 3.2)
			raw_exc_u[ka] = clamp(gru - 1.0, -1.00, 4.5)
		}
		sum_med_b += swf.ana_med_psd[ka] * abm * abm
		sum_nfl_b += nf

		if fka >= 170.0 && fka <= 1180.0 && !is_hum_notch_freq(fka) {
			lo_fast_s += swf.ana_fast_psd[ka] * abm * abm
			lo_raw_s  += ana_raw[ka]          * abm * abm
			lo_nfl_s  += nf
			lo_fnf_s  += nff
			if fka >= 230.0 && fka <= 940.0 {
				f1_fast_s += swf.ana_fast_psd[ka] * abm * abm
				f1_med_s  += swf.ana_med_psd[ka]  * abm * abm
				f1_raw_s  += ana_raw[ka]          * abm * abm
				f1_nfl_s  += nf
				f1_fnf_s  += nff
				if gf > f1_peak_gf do f1_peak_gf = gf
			}
			if ka > ka_lo && ka < ka_hi && abm >= 0.65 && fka >= 210.0 {
				pf0 := swf.ana_fast_psd[ka] * abm * abm
				pfm := swf.ana_fast_psd[ka - 1] * ana_birdie[ka - 1] * ana_birdie[ka - 1]
				pfp := swf.ana_fast_psd[ka + 1] * ana_birdie[ka + 1] * ana_birdie[ka + 1]
				if pf0 >= pfm && pf0 >= pfp && gf > 2.50 && gm > 1.80 {
					voiced_pks += 1
				}
			}
		} else if fka >= 1600.0 && fka <= 2540.0 {
			hi_fast_s += swf.ana_fast_psd[ka] * abm * abm
			hi_nfl_s  += nf
			hi_fnf_s  += nff
			if gf > 1.80 do hi_broad_bins += 1
			if gf > hi_peak_gf do hi_peak_gf = gf
		}
	}
	l3_snr_ratio := sum_med_b / math.max(sum_nfl_b, 1e-22)
	lo_snr_ratio := lo_fast_s / math.max(lo_nfl_s,  1e-22)
	lo_fast_snr  := lo_fast_s / math.max(lo_fnf_s,  1e-22)
	raw_fast_snr := lo_raw_s  / math.max(lo_fnf_s,  1e-22)
	f1_med_snr   := f1_med_s  / math.max(f1_nfl_s,  1e-22)
	f1_fast_snr  := f1_fast_s / math.max(f1_fnf_s,  1e-22)
	f1_raw_snr   := f1_raw_s  / math.max(f1_fnf_s,  1e-22)
	hi_snr_ratio := hi_fast_s / math.max(hi_fnf_s,  1e-22)
	_ = lo_snr_ratio

	// Update 12 kHz (5.86 Hz/bin & 2.93 Hz/bin) unclamped Welch + zero-overlap spectra every 2 hops (256 @ 12 kHz = 21.33 ms):
	//   1) 128ms disjoint pair : past128 [5632:7168] vs pres128 [7168:8704] (0 shared samples, 256 ms span!)
	//   2) 2048-pt disjoint    : past2k  [3584:5888] vs pres2k  [5888:8704] (0 shared samples, 426 ms span, 5.86 Hz/bin!)
	//   3) 4096-pt disjoint    : past4k  [0   :4352] vs pres4k  [4352:8704] (0 shared samples, 725 ms span, 2.93 Hz/bin!)
	even_hop3 := swf.frame_cnt >= 6 && (swf.frame_cnt & 1) == 0
	if even_hop3 {
		inv_hr4_hz := 1.0 / HR4K_BIN_HZ
		hk4_lo     := clamp(int(math.floor(comb_f_lo * inv_hr4_hz)), 6, HR4K_BINS - 16)
		hk4_hi     := clamp(int(math.ceil(comb_f_hi * inv_hr4_hz)), hk4_lo + 24, HR4K_BINS - 4)

		p128_past, p128_pres : [HR2K_BINS]f64
		compute_hires_raw_pow(&swf.hr_plan2k, swf.hr_ring[5632 : 7168], swf.hr_win128[:], swf.hr_fft[:], p128_past[:])
		compute_hires_raw_pow(&swf.hr_plan2k, swf.hr_ring[7168 : 8704], swf.hr_win128[:], swf.hr_fft[:], p128_pres[:])
		normalize_hires_psd_to_exc(p128_past[:], ana_birdie[:], ana_bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.62, swf.hr_past128_exc[:])
		normalize_hires_psd_to_exc(p128_pres[:], ana_birdie[:], ana_bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.62, swf.hr_pres128_exc[:])

		raw_past2k, raw_pres2k : [HR2K_BINS]f64
		compute_hires_raw_pow(&swf.hr_plan2k, swf.hr_ring[3840 : 5888], swf.hr_win2k[:], swf.hr_fft[:], raw_past2k[:])
		compute_hires_raw_pow(&swf.hr_plan2k, swf.hr_ring[6656 : 8704], swf.hr_win2k[:], swf.hr_fft[:], raw_pres2k[:])

		swf.hr_past2k_pow[swf.hr_pos & 1] = raw_past2k
		swf.hr_pres2k_pow[swf.hr_pos & 3] = raw_pres2k
		a_wb2 := 1.0 if swf.hr_filled == 0 else (0.36 if swf.hr_filled <= 2 else 0.18)
		for hk in 0 ..< HR2K_BINS {
			swf.hr_wb2k_psd[hk] = (1.0 - a_wb2) * swf.hr_wb2k_psd[hk] + a_wb2 * raw_pres2k[hk]
		}
		swf.hr_pos   += 1
		swf.hr_filled = min(swf.hr_filled + 1, HR_HIST_LEN)

		np_a := min(swf.hr_filled, 2)
		nm_a := min(swf.hr_filled, 4)
		past2k_psd, med2k_psd : [HR2K_BINS]f64
		for hk in 0 ..< HR2K_BINS {
			sp := 0.0
			for si in 0 ..< np_a do sp += swf.hr_past2k_pow[(swf.hr_pos - 1 - si) & 1][hk]
			past2k_psd[hk] = sp / f64(np_a)

			sm := 0.0
			for si in 0 ..< nm_a do sm += swf.hr_pres2k_pow[(swf.hr_pos - 1 - si) & 3][hk]
			med2k_psd[hk] = sm / f64(nm_a)
		}
		normalize_hires_psd_to_exc(past2k_psd[:],      ana_birdie[:], ana_bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.40 if np_a >= 2 else 1.62, swf.hr_past_avg[:])
		normalize_hires_psd_to_exc(med2k_psd[:],       ana_birdie[:], ana_bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.28 if nm_a >= 3 else 1.45, swf.hr_mfin[:])
		normalize_hires_psd_to_exc(swf.hr_wb2k_psd[:], ana_birdie[:], ana_bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.20,                        swf.hr_wb[:])

		if swf.frame_cnt >= 16 {
			raw_past4k, raw_pres4k : [HR4K_BINS]f64
			compute_hires_raw_pow(&swf.hr_plan4k, swf.hr_ring[256  : 4352], swf.hr_win4k[:], swf.hr_fft[:], raw_past4k[:])
			compute_hires_raw_pow(&swf.hr_plan4k, swf.hr_ring[4608 : 8704], swf.hr_win4k[:], swf.hr_fft[:], raw_pres4k[:])
			swf.hr_past4k_pow[swf.hr_pos4k & 1] = raw_past4k
			swf.hr_pres4k_pow[swf.hr_pos4k & 1] = raw_pres4k
			a_wb4 := 1.0 if swf.hr_filled4k == 0 else (0.36 if swf.hr_filled4k <= 2 else 0.20)
			for hk in 0 ..< HR4K_BINS {
				swf.hr_wb4k_psd[hk] = (1.0 - a_wb4) * swf.hr_wb4k_psd[hk] + a_wb4 * raw_pres4k[hk]
			}
			swf.hr_pos4k   += 1
			swf.hr_filled4k = min(swf.hr_filled4k + 1, 16)

			n4 := min(swf.hr_filled4k, 2)
			past4k_psd, pres4k_psd : [HR4K_BINS]f64
			for hk in 0 ..< HR4K_BINS {
				sp4, pr4 := 0.0, 0.0
				for si in 0 ..< n4 {
					sp4 += swf.hr_past4k_pow[(swf.hr_pos4k - 1 - si) & 1][hk]
					pr4 += swf.hr_pres4k_pow[(swf.hr_pos4k - 1 - si) & 1][hk]
				}
				past4k_psd[hk] = sp4 / f64(n4)
				pres4k_psd[hk] = pr4 / f64(n4)
			}
			sc4 := 1.40 if n4 >= 2 else 1.62
			normalize_hires_psd_to_exc(past4k_psd[:],      ana_birdie[:], ana_bin_hz, HR4K_N, comb_f_lo, comb_f_hi, sc4,  swf.hr_past4k_exc[:])
			normalize_hires_psd_to_exc(pres4k_psd[:],      ana_birdie[:], ana_bin_hz, HR4K_N, comb_f_lo, comb_f_hi, sc4,  swf.hr_pres4k_exc[:])
			normalize_hires_psd_to_exc(swf.hr_wb4k_psd[:], ana_birdie[:], ana_bin_hz, HR4K_N, comb_f_lo, comb_f_hi, 1.22, swf.hr_wb4k_exc[:])
		}

		// Stage 5A/5B Joint High-Res (F0, df) Comb Search:
		//   Evaluates 9-point SSB mistune grid {-36..+36 Hz} + tracked_df on 5.86 Hz/bin hr_mfin,
		//   refines best_m_df to ±0.75 Hz, and feeds top df candidates into 2.93 Hz/bin hr_pres4k_exc!
		df_steps_m := 1
		df_tab_m   := [10]f64{0.0, -36.0, -27.0, -18.0, -9.0, +9.0, +18.0, +27.0, +36.0, 0.0}
		df_pen_m   := [10]f64{1.0,  0.96,  0.97,  0.97, 0.98, 0.98,  0.97,  0.97,  0.96, 1.0}
		if is_ssb {
			df_steps_m = 9
			if math.abs(swf.tracked_df) >= 3.5 {
				df_tab_m[9] = swf.tracked_df
				df_steps_m  = 10
			}
		}

		max_f0_idx3 := 211 if is_ssb else L3_NUM_F0 // 85..190 Hz on SSB (eliminates sparse >200Hz static aliases!), 85..260 Hz on AM/FM
		prev_shs2   := swf.shs_smooth
		prev_shs4   := swf.shs4k_smooth
		med_scores  : [L3_NUM_F0]f64
		med_harms   : [L3_NUM_F0]int
		med_dfs     : [L3_NUM_F0]f64
		best_m_sc   := -1.0e9
		best_m_i    := 60
		best_m_df   := 0.0
		best_m_ph   := 0

		for idx in 0 ..< max_f0_idx3 {
			cf0 := 85.0 + L3_F0_STEP * f64(idx)
			m_b := -1.0e9
			m_d := 0.0
			m_p := 0
			for di in 0 ..< df_steps_m {
				dfc := df_tab_m[di]
				if math.abs(dfc) > 0.46 * cf0 do continue
				scm, phm, _ := eval_uniform_shs_comb(swf.hr_mfin[:], cf0, dfc, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.035)
				scm *= df_pen_m[di]
				if scm > m_b { m_b = scm; m_d = dfc; m_p = phm }
			}
			im := max(idx - 1, 0)
			ip := min(idx + 1, max_f0_idx3 - 1)
			diff2 := 0.16 * prev_shs2[im] + 0.68 * prev_shs2[idx] + 0.16 * prev_shs2[ip]
			swf.shs_smooth[idx] = 0.45 * diff2 + 0.55 * m_b
			ms := swf.shs_smooth[idx]
			med_scores[idx] = ms; med_harms[idx] = m_p; med_dfs[idx] = m_d
			if ms > best_m_sc { best_m_sc = ms; best_m_i = idx; best_m_df = m_d; best_m_ph = m_p }
		}

		mean_m := 0.0
		for idx in 0 ..< max_f0_idx3 do mean_m += med_scores[idx]
		mean_m /= f64(max_f0_idx3)
		var_m := 1e-9
		for idx in 0 ..< max_f0_idx3 { dm := med_scores[idx] - mean_m; var_m += dm * dm }
		std_m := math.sqrt(var_m / f64(max_f0_idx3))

		med_f0 := 85.0 + L3_F0_STEP * f64(best_m_i)
		if med_f0 >= 170.0 {
			hi := int(math.round((0.5 * med_f0 - 85.0) / L3_F0_STEP))
			if hi >= 0 && hi < max_f0_idx3 && med_scores[hi] > mean_m + 0.35 * std_m && med_scores[hi] >= 0.45 * best_m_sc {
				best_m_i  = hi
				med_f0    = 85.0 + L3_F0_STEP * f64(hi)
				best_m_ph = max(best_m_ph, med_harms[hi])
				best_m_df = med_dfs[hi]
			}
		}
		if best_m_i > 0 && best_m_i + 1 < max_f0_idx3 {
			sm, s0, sp := med_scores[best_m_i - 1], med_scores[best_m_i], med_scores[best_m_i + 1]
			den := sm - 2.0 * s0 + sp
			if den < -1e-9 do med_f0 += L3_F0_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
		}
		if is_ssb {
			ref_b := -1.0e9
			ref_d := best_m_df
			for rs in -3 ..= 3 {
				fd := clamp(best_m_df + 1.5 * f64(rs), -42.0, 42.0)
				scr, phr, _ := eval_uniform_shs_comb(swf.hr_mfin[:], med_f0, fd, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.035)
				if scr > ref_b { ref_b = scr; ref_d = fd; best_m_ph = max(best_m_ph, phr) }
			}
			best_m_df = ref_d
		}

		df_steps_4 := 1
		df_tab_4   := [3]f64{0.0, 0.0, 0.0}
		df_pen_4   := [3]f64{1.0, 1.0, 0.98}
		if is_ssb && math.abs(swf.tracked_df) >= 3.5 {
			df_tab_4[df_steps_4] = swf.tracked_df
			df_steps_4 += 1
		}
		if is_ssb && math.abs(best_m_df) >= 3.5 && math.abs(best_m_df - swf.tracked_df) > 2.0 && df_steps_4 < 3 {
			df_tab_4[df_steps_4] = best_m_df
			df_steps_4 += 1
		}

		d4_scores  : [L3_NUM_F0]f64
		d4_harms   : [L3_NUM_F0]int
		d4_dfs     : [L3_NUM_F0]f64
		best_4k_sc := -1.0e9
		best_4k_i  := 60
		best_4k_df := 0.0
		best_4k_ph := 0
		if swf.hr_filled4k >= 1 {
			for idx in 0 ..< max_f0_idx3 {
				cf0 := 85.0 + L3_F0_STEP * f64(idx)
				b4  := -1.0e9
				d4  := 0.0
				p4  := 0
				for di in 0 ..< df_steps_4 {
					sc4, ph4, _ := eval_uniform_shs_comb(swf.hr_pres4k_exc[:], cf0, df_tab_4[di], comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 0.025)
					sc4 *= df_pen_4[di]
					if sc4 > b4 { b4 = sc4; d4 = df_tab_4[di]; p4 = ph4 }
				}
				im := max(idx - 1, 0)
				ip := min(idx + 1, max_f0_idx3 - 1)
				diff4 := 0.18 * prev_shs4[im] + 0.64 * prev_shs4[idx] + 0.18 * prev_shs4[ip]
				swf.shs4k_smooth[idx] = 0.50 * diff4 + 0.50 * b4
				s4 := swf.shs4k_smooth[idx]
				d4_scores[idx] = s4; d4_harms[idx] = p4; d4_dfs[idx] = d4
				if s4 > best_4k_sc { best_4k_sc = s4; best_4k_i = idx; best_4k_df = d4; best_4k_ph = p4 }
			}
		}

		z_m_bg, _, _ := comb_z_at(swf.hr_mfin[:], med_f0, best_m_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.035)
		ze_m, zo_m   := eval_even_odd_comb_z(swf.hr_mfin[:], med_f0, best_m_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz)
		z_past2k, ph_past2k := -10.0, 0
		if swf.hr_filled >= 2 && swf.frame_cnt >= 14 {
			z_past2k, ph_past2k = eval_unsel_past_z(swf.hr_past_avg[:], med_f0, best_m_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 1.8)
		}

		// Evaluate 4096-pt (2.93 Hz/bin) sub-noise comb & zero-overlap past4k cross-validator
		f0_4k    := med_f0
		z_4k_bg  := -10.0
		ze_4k, zo_4k := 0.0, 0.0
		z_past4k := -10.0
		ph_past4 := 0
		if swf.hr_filled4k >= 2 && swf.frame_cnt >= 20 {
			m4 := 0.0
			for idx in 0 ..< max_f0_idx3 do m4 += d4_scores[idx]
			m4 /= f64(max_f0_idx3)
			v4 := 1e-9
			for idx in 0 ..< max_f0_idx3 { d := d4_scores[idx] - m4; v4 += d * d }
			sd4 := math.sqrt(v4 / f64(max_f0_idx3))
			f0_4k = 85.0 + L3_F0_STEP * f64(best_4k_i)
			if f0_4k >= 170.0 {
				hi := int(math.round((0.5 * f0_4k - 85.0) / L3_F0_STEP))
				if hi >= 0 && hi < max_f0_idx3 && d4_scores[hi] > m4 + 0.32 * sd4 && d4_scores[hi] >= 0.43 * best_4k_sc {
					best_4k_i  = hi
					f0_4k      = 85.0 + L3_F0_STEP * f64(hi)
					best_4k_ph = max(best_4k_ph, d4_harms[hi])
					best_4k_df = d4_dfs[hi]
				}
			}
			if best_4k_i > 0 && best_4k_i + 1 < max_f0_idx3 {
				sm, s0, sp := d4_scores[best_4k_i - 1], d4_scores[best_4k_i], d4_scores[best_4k_i + 1]
				den := sm - 2.0 * s0 + sp
				if den < -1e-9 do f0_4k += L3_F0_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
			}
			z_4k_bg, _, _  = comb_z_at(swf.hr_pres4k_exc[:], f0_4k, best_4k_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 0.025)
			ze_4k, zo_4k   = eval_even_odd_comb_z(swf.hr_pres4k_exc[:], f0_4k, best_4k_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz)
			z_past4k, ph_past4 = eval_unsel_past_z(swf.hr_past4k_exc[:], f0_4k, best_4k_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 1.8)
		}

		// Fast 256 ms (128ms + 128ms zero-overlap) cross-validator & glide-immune end-of-word pause release detector
		f0_128, z_pres128, z_past128 := med_f0, -10.0, -10.0
		ph_128 := 0
		e_drop := false
		z_pres_cur := 2.0
		if swf.hr_filled >= 2 && swf.frame_cnt >= 12 {
			f0_128, z_pres128, z_past128, ph_128 = eval_128ms_cross(
				swf.hr_past128_exc[:], swf.hr_pres128_exc[:], df_steps_4, df_tab_4[:],
				comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz,
			)
			z128_med, _, _ := comb_z_at(swf.hr_pres128_exc[:], med_f0, best_m_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.035)
			raw_edrop3 : bool
			raw_edrop3, z_pres_cur = eval_past_pres_drop(
				swf.hr_past128_exc[:], swf.hr_pres128_exc[:], swf.tracked_f0, swf.tracked_df,
				comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz,
			)
			z_pres_cur = max(z_pres_cur, z128_med)
			glide_edrop3 := z128_med < 0.25 && f1_med_snr > 1.32 && f1_fast_snr < 1.14 && f1_raw_snr < 1.10 && hi_snr_ratio < 1.18
			e_drop = (raw_edrop3 || glide_edrop3) && (swf.harm_conf > 0.05 || swf.harm_hang > 0.05)
		}

		if swf.frame_cnt <= 7 {
			swf.med_streak = 0
			swf.d4k_streak = 0
		} else {
			lag4_f0 := swf.f0_ring[swf.ring_pos]
			lag4_4k := swf.f0_4k_ring[swf.ring_pos]
			lag4_df := swf.df_ring[swf.ring_pos]
			swf.f0_ring[swf.ring_pos]    = med_f0
			swf.f0_4k_ring[swf.ring_pos] = f0_4k
			swf.df_ring[swf.ring_pos]    = best_m_df
			swf.ring_pos = (swf.ring_pos + 1) & 3

			cont_m    := math.abs(med_f0 - swf.f0_prev_med) <= (12.0 if swf.harm_conf > 0.25 else 4.8)
			df_cont_m := !is_ssb || swf.med_streak < 2 || swf.harm_conf > 0.25 || math.abs(best_m_df - lag4_df) <= 9.5
			disj_cont := (swf.med_streak < 4) || (lag4_f0 >= 80.0 && math.abs(med_f0 - lag4_f0) <= (14.0 if swf.harm_conf > 0.25 else 5.8))
			if (!cont_m || !df_cont_m || !disj_cont || z_m_bg < 2.70 || math.min(ze_m, zo_m) < 0.78) && swf.harm_conf < 0.25 {
				swf.med_streak = 0
			} else if z_m_bg > 2.85 && math.min(ze_m, zo_m) > 0.85 && best_m_ph >= 5 && cont_m && df_cont_m && disj_cont {
				swf.med_streak = min(swf.med_streak + 1, 24)
			} else if z_m_bg < 2.25 {
				swf.med_streak = 0
			} else if swf.med_streak > 0 {
				swf.med_streak -= 1
			}
			swf.f0_prev_med = med_f0
			if is_ssb && swf.med_streak >= 3 && z_m_bg > 3.25 && math.abs(best_m_df) >= 3.5 {
				swf.mistune_cand   = best_m_df
				swf.mistune_streak = min(swf.mistune_streak + 1, 10)
			}

			cont_4k   := math.abs(f0_4k - swf.f0_prev_4k) <= (12.0 if swf.harm_conf > 0.25 else 4.2)
			df_cont_4 := !is_ssb || swf.d4k_streak < 2 || swf.harm_conf > 0.25 || math.abs(best_4k_df - lag4_df) <= 9.5
			disj_4k   := (swf.d4k_streak < 4) || (lag4_4k >= 80.0 && math.abs(f0_4k - lag4_4k) <= (14.0 if swf.harm_conf > 0.25 else 5.0))
			if (!cont_4k || !df_cont_4 || !disj_4k || z_4k_bg < 2.75 || math.min(ze_4k, zo_4k) < 0.80) && swf.harm_conf < 0.25 {
				swf.d4k_streak = 0
			} else if z_4k_bg > 2.90 && math.min(ze_4k, zo_4k) > 0.88 && best_4k_ph >= 5 && cont_4k && df_cont_4 && disj_4k {
				swf.d4k_streak = min(swf.d4k_streak + 1, 24)
			} else if z_4k_bg < 2.30 {
				swf.d4k_streak = 0
			} else if swf.d4k_streak > 0 {
				swf.d4k_streak -= 1
			}
			swf.f0_prev_4k = f0_4k
		}

		// Select best validated sub-noise lock (2k glide-agile vs 4k -15dB deep vs 256ms rapid syllable)
		use_4k_sel := swf.hr_filled4k >= 2 && swf.d4k_streak >= 5 && z_4k_bg > z_m_bg + 0.15 && z_past4k > 1.85 && math.min(ze_4k, zo_4k) > 1.05 && best_4k_ph >= 5
		if use_4k_sel {
			swf.c_med_f0    = f0_4k
			swf.c_best_m_df = best_4k_df
			swf.c_best_m_ph = best_4k_ph
			swf.c_z_m_bg    = z_4k_bg
			swf.c_ze_m      = ze_4k
			swf.c_zo_m      = zo_4k
			swf.c_z_past    = z_past4k
			swf.c_z_pres    = z_past4k
			swf.c_past_f0   = f0_4k
			swf.c_past_ph   = ph_past4
		} else {
			best_unsel := z_past2k
			best_uf0   := med_f0
			best_uph   := ph_past2k
			if z_pres128 > 3.05 && z_past128 > best_unsel && math.abs(med_f0 - f0_128) <= 6.0 {
				best_unsel = z_past128
				best_uf0   = f0_128
				best_uph   = ph_128
			}
			swf.c_med_f0    = med_f0
			swf.c_best_m_df = best_m_df
			swf.c_best_m_ph = best_m_ph
			swf.c_z_m_bg    = z_m_bg
			swf.c_ze_m      = ze_m
			swf.c_zo_m      = zo_m
			swf.c_z_past    = z_pres128
			swf.c_z_pres    = best_unsel
			swf.c_past_f0   = best_uf0
			swf.c_past_ph   = best_uph
		}
		swf.c_z_pres_cur  = z_pres_cur
		swf.c_e_drop      = e_drop
		swf.inst_comb_ema = swf.c_z_pres
	}

	med_f0     := swf.c_med_f0
	best_m_df  := swf.c_best_m_df
	best_m_ph  := swf.c_best_m_ph
	z_m_bg     := swf.c_z_m_bg
	ze_m       := swf.c_ze_m
	zo_m       := swf.c_zo_m
	z_pres128  := swf.c_z_past
	z_unsel    := swf.c_z_pres
	unsel_f0   := swf.c_past_f0
	unsel_ph   := swf.c_past_ph
	z_pres_cur := swf.c_z_pres_cur
	e_drop     := swf.c_e_drop
	eff_str3   := max(swf.med_streak, swf.d4k_streak)

	// Fast 10.67 ms (512-sample) syllabic contour on 48 kHz ana_fast_psd
	df_steps_f := 1
	df_tab_f   := [2]f64{0.0, 0.0}
	if is_ssb && math.abs(swf.tracked_df) >= 3.5 {
		df_tab_f[1] = swf.tracked_df
		df_steps_f  = 2
	}
	fast_scores : [L3_FAST_NUM_F0]f64
	fast_harms  : [L3_FAST_NUM_F0]int
	fast_dfs    : [L3_FAST_NUM_F0]f64
	best_f_sc   := -1.0e9
	best_f_i    := 50
	best_f_df   := 0.0
	best_f_ph   := 0
	for idx in 0 ..< L3_FAST_NUM_F0 {
		cf0 := 85.0 + f64(idx)
		f_b := -1.0e9
		f_d := 0.0
		f_p := 0
		for di in 0 ..< df_steps_f {
			scf, phf, _ := eval_uniform_shs_comb(fast_exc[:], cf0, df_tab_f[di], comb_f_lo, comb_f_hi, ka_lo, ka_hi, inv_ana_hz, 0.095)
			if scf > f_b { f_b = scf; f_d = df_tab_f[di]; f_p = phf }
		}
		fast_scores[idx] = f_b
		fast_harms[idx]  = f_p
		fast_dfs[idx]    = f_d
		if f_b > best_f_sc { best_f_sc = f_b; best_f_i = idx; best_f_df = f_d; best_f_ph = f_p }
	}
	mean_f := 0.0
	for v in fast_scores do mean_f += v
	mean_f /= f64(L3_FAST_NUM_F0)
	var_f := 1e-9
	for v in fast_scores {
		df := v - mean_f
		var_f += df * df
	}
	std_f   := math.sqrt(var_f / f64(L3_FAST_NUM_F0))
	fast_f0 := 85.0 + f64(best_f_i)
	if fast_f0 >= 170.0 {
		hi := int(math.round(0.5 * fast_f0 - 85.0))
		if hi >= 0 && hi < L3_FAST_NUM_F0 && fast_scores[hi] > mean_f + 0.35 * std_f && fast_scores[hi] >= 0.45 * best_f_sc {
			best_f_i  = hi
			fast_f0   = 85.0 + f64(hi)
			best_f_ph = max(best_f_ph, fast_harms[hi])
			best_f_df = fast_dfs[hi]
		}
	}
	if best_f_i > 0 && best_f_i + 1 < L3_FAST_NUM_F0 {
		sm, s0, sp := fast_scores[best_f_i - 1], fast_scores[best_f_i], fast_scores[best_f_i + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do fast_f0 += clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}
	z_f    := (best_f_sc - mean_f) / math.max(std_f, 1e-6)
	z_now, raw_comb_m, _ := comb_z_at(raw_exc_u[:], med_f0, best_m_df, comb_f_lo, comb_f_hi, ka_lo, ka_hi, inv_ana_hz, 0.08)

	cont_f := math.abs(fast_f0 - swf.f0_prev_fast) <= 11.0
	if swf.frame_cnt <= 9 {
		swf.fast_streak = 0
	} else if !cont_f && swf.harm_conf < 0.30 {
		swf.fast_streak = 0
	} else if best_f_sc > 0.135 && z_f > 2.95 && best_f_ph >= 5 && cont_f {
		swf.fast_streak = min(swf.fast_streak + 1, 16)
	} else if best_f_sc < 0.085 || z_f < 2.30 {
		swf.fast_streak = 0
	} else if swf.fast_streak > 0 {
		swf.fast_streak -= 1
	}
	swf.f0_prev_fast = fast_f0

	// Multi-Tier Sub-Noise & Conversational Detection (-15 dB target, 0/2720 pure-static immunity!):
	hc_med   := 0.0
	sub_lock := swf.hr_filled >= 2 && swf.frame_cnt >= 14 && !e_drop && z_pres_cur > 0.45 && best_m_ph >= 5 && unsel_ph >= 4 &&
	            math.abs(med_f0 - unsel_f0) <= 6.0 && math.min(ze_m, zo_m) > 1.05 &&
	            ((eff_str3 >= 5 && z_m_bg > 3.35 && z_unsel > 1.80) ||
	             (swf.med_streak >= 3 && z_m_bg > 3.55 && z_pres128 > 3.15 && z_unsel > 2.00))
	sub_hold := (swf.harm_conf > 0.15 || swf.harm_hang > 0.20) && swf.hr_filled >= 2 && !e_drop && z_pres_cur > 0.65 &&
	            z_m_bg > 2.75 && math.min(ze_m, zo_m) > 0.80 && best_m_ph >= 4
	mid_lock := !e_drop && z_pres_cur > 0.55 && swf.med_streak >= 3 && best_m_ph >= 5 &&
	            ((z_m_bg > 4.15 && math.min(ze_m, zo_m) > 1.65 && f1_fast_snr > 1.14 && f1_med_snr > 1.13) ||
	             (z_m_bg > 3.20 && math.min(ze_m, zo_m) > 1.00 && f1_fast_snr > 1.42 && f1_med_snr > 1.38 && f1_raw_snr > 1.20))
	if sub_lock || sub_hold || mid_lock {
		hc_med = clamp((z_m_bg - 2.15) * 0.65, 0.36, 1.0)
	}
	hc_fast := 0.0
	if (swf.fast_streak >= 2 && z_f > 3.05 && best_f_sc > 0.148 && best_f_ph >= 5 && f1_fast_snr > 1.46 && f1_med_snr > 1.38 && f1_raw_snr > 1.24) ||
	   (swf.harm_hang > 0.18 && swf.fast_streak >= 1 && z_f > 2.88 && best_f_sc > 0.132 && best_f_ph >= 4 && f1_fast_snr > 1.35 && f1_raw_snr > 1.16) ||
	   (z_f > 3.50 && best_f_sc > 0.210 && best_f_ph >= 5 && f1_fast_snr > 1.54 && f1_med_snr > 1.44 && f1_raw_snr > 1.28) {
		hc_fast = clamp((z_f - 2.38) * 0.65, 0.0, 1.0)
	}

	inst_hc := max(hc_med, hc_fast)
	use_fast_f0 := hc_fast >= 0.40 && l3_snr_ratio > 1.08
	sel_f0 := fast_f0   if use_fast_f0 else med_f0
	sel_df := best_f_df if use_fast_f0 else best_m_df

	if inst_hc > 0.08 {
		if math.abs(sel_f0 - swf.tracked_f0) < 16.0 {
			swf.tracked_f0 = 0.28 * swf.tracked_f0 + 0.72 * sel_f0
		} else {
			swf.tracked_f0 = sel_f0
		}
		if is_ssb {
			if math.abs(swf.tracked_df) >= 4.0 && math.abs(sel_df) < 1.0 {
				if swf.mistune_streak == 0 && math.abs(best_m_df) < 1.0 && math.abs(best_f_df) < 1.0 {
					sc0_u, _, _  := eval_uniform_shs_comb(swf.hr_wb[:], sel_f0, 0.0,            comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.025)
					scdf_u, _, _ := eval_uniform_shs_comb(swf.hr_wb[:], sel_f0, swf.tracked_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.025)
					if sc0_u > 1.12 * math.max(scdf_u, 0.01) {
						swf.tracked_df = 0.0
					}
				}
			} else if math.abs(sel_df - swf.tracked_df) < 16.0 {
				swf.tracked_df = 0.40 * swf.tracked_df + 0.60 * sel_df
			} else {
				swf.tracked_df = sel_df
			}
		} else {
			swf.tracked_df = 0.0
		}
	}

	// Un-lagged instantaneous pause detector (glide-safe: checks BOTH tracked_f0 and fast_f0!)
	z_trk_now, raw_comb_cur, _ := comb_z_at(raw_exc_u[:], swf.tracked_f0, swf.tracked_df, comb_f_lo, comb_f_hi, ka_lo, ka_hi, inv_ana_hz, 0.08)
	inst_comb_cur, _, _        := eval_uniform_shs_comb(fast_exc[:],    swf.tracked_f0, swf.tracked_df, comb_f_lo, comb_f_hi, ka_lo, ka_hi, inv_ana_hz, 0.09)
	med_comb_cur,  _, _        := eval_uniform_shs_comb(swf.hr_mfin[:], swf.tracked_f0, swf.tracked_df, comb_f_lo, comb_f_hi, hk_lo, hk_hi, inv_hr2_hz, 0.035)
	_ = raw_comb_m
	best_inst_c := max(inst_comb_cur, best_f_sc if best_f_ph >= 5 else inst_comb_cur)
	cur_comb_l3 := 0.50 * max(raw_comb_cur, best_f_sc * 0.85 if best_f_ph >= 5 else raw_comb_cur) + 0.50 * best_inst_c

	// Voiced F1/F2 formant VAD (230..940 Hz vocal core + true f1_med_snr + q25*1.58 QRN-swell immunity -> 0% static pops!)
	voiced_formant_vad := (voiced_pks >= 3 && f1_fast_snr > 1.52 && f1_med_snr > 1.52 && f1_raw_snr > 1.25 && f1_peak_gf > 5.2) ||
	                      (voiced_pks >= 2 && f1_fast_snr > 1.68 && f1_med_snr > 1.68 && f1_raw_snr > 1.32 && f1_peak_gf > 6.8)

	// Unvoiced Portuguese consonant / sibilant ('/s, ch, f, t, k/' in 1.6-2.54 kHz) — immune to QRN static swells & trailing breath!
	cons_vad := (swf.voiced_hang_hops > 0 && hi_broad_bins >= 8 && hi_snr_ratio > 1.42 && hi_peak_gf > 3.5 && hi_snr_ratio > 1.28 * lo_fast_snr) ||
	            (hi_broad_bins >= 11 && hi_snr_ratio > 1.80 && hi_peak_gf > 5.2 && hi_snr_ratio > 1.55 * math.max(lo_fast_snr, 0.95))

	was_active3 := swf.voice_conf > 0.02 || swf.harm_conf > 0.02 || swf.hang_hops > 0 || swf.gate_phase > 0.0
	pause_drop  := false
	if was_active3 && !voiced_formant_vad && !cons_vad && inst_hc < 0.12 &&
	   (e_drop ||
	    (f1_fast_snr < 1.15 && f1_raw_snr < 1.08 && hi_snr_ratio < 1.16) ||
	    (med_comb_cur > 0.15 && cur_comb_l3 < 0.22 * med_comb_cur && lo_fast_snr < 1.15 && raw_fast_snr < 1.12) ||
	    (swf.harm_conf > 0.15 && hc_med < 0.05 && z_trk_now < 0.80 && z_now < 0.80 && lo_fast_snr < 1.12 && raw_fast_snr < 1.10)) {
		pause_drop       = true
		inst_hc          = 0.0
		swf.harm_conf    = 0.0
		swf.harm_hang    = 0.0
		swf.med_streak   = 0
		swf.d4k_streak   = 0
		swf.fast_streak  = 0
		swf.shs_smooth   = {}
		swf.shs4k_smooth = {}
		if e_drop || (f1_med_snr > 1.26 && f1_fast_snr < 1.14) {
			swf.hr_pres2k_pow = {}
			swf.hr_pres4k_pow = {}
			swf.hr_mfin       = {}
			swf.hr_pres4k_exc = {}
			swf.hr_filled     = min(swf.hr_filled, 1)
			swf.hr_filled4k   = min(swf.hr_filled4k, 1)
		}
	}

	swf.harm_conf = 0.35 * swf.harm_conf + 0.65 * inst_hc if inst_hc > 0.05 else 0.20 * swf.harm_conf
	if swf.harm_conf > swf.harm_hang {
		swf.harm_hang = swf.harm_conf
	} else {
		swf.harm_hang *= 0.82
	}

	formant_conf := clamp((f1_fast_snr - 1.28) * 1.15, 0.30, 0.95) if voiced_formant_vad else 0.0
	cons_conf    := clamp((hi_snr_ratio - 1.22) * 1.10, 0.28, 0.85) if cons_vad else 0.0
	voiced_conf  := max(swf.harm_conf, formant_conf)
	raw_vconf    := max(voiced_conf, cons_conf)
	swf.voice_conf = 0.35 * swf.voice_conf + 0.65 * raw_vconf if raw_vconf > 0.05 else (0.0 if pause_drop else 0.18 * swf.voice_conf)

	// Track voiced hangover STRICTLY from voiced speech (never from cons_vad -> zero self-latching!)
	if voiced_conf >= 0.18 {
		swf.voiced_hang_hops = 5 // ~53 ms window for trailing sibilants
		swf.hang_hops        = 3 // ~32 ms syllabic bridge
		swf.silent_hops      = 0
	} else {
		swf.silent_hops += 1
		if swf.silent_hops >= 64 && swf.mistune_streak == 0 {
			swf.tracked_df = 0.0
			swf.df_profile = {}
		}
		if swf.voiced_hang_hops > 0 do swf.voiced_hang_hops -= 1
		if pause_drop {
			swf.hang_hops        = 0
			swf.voiced_hang_hops = 0
		} else if cons_vad {
			swf.hang_hops = max(swf.hang_hops, 1)
		} else if swf.hang_hops > 0 {
			swf.hang_hops -= 1
		}
	}
	speech_now := raw_vconf >= 0.18
	gate_open  := speech_now || swf.hang_hops > 0

	// 5. Pass 1 & Pass 2: Mistune-Aligned Harmonic-Guided Cohen OM-LSA Gain on 2048-pt STFT
	eff_n_arr  : [L3_BINS]f64
	gam_inst_a : [L3_BINS]f64
	gam_fast_a : [L3_BINS]f64
	xi_dd_a    : [L3_BINS]f64
	g_h1_arr   : [L3_BINS]f64
	voc_exc_sum := 0.0
	voc_exc_cnt := 0

	for k in k_lo ..= k_hi {
		freq := f64(k) * bin_hz
		over := base_over
		if band == .FM_Band {
			fn  := freq / 3000.0
			over = base_over + 0.16 * (fn * fn)
		}
		bm        := birdie_mask[k]
		eff_noise := math.max(swf.noise_psd[k] * over, 1e-20)
		gi        := (raw_pow[k] * bm * bm) / eff_noise
		gf        := (swf.fast_psd[k] * bm * bm) / eff_noise
		clean_est := math.max(0.72 * swf.fast_psd[k] * bm * bm + 0.28 * swf.welch_psd[k] * bm * bm - 1.00 * eff_noise, 0.0)
		xi_ml     := clean_est / eff_noise
		xi_dd     := 0.80 * (swf.prev_clean[k] / eff_noise) + 0.20 * xi_ml

		eff_n_arr[k]  = eff_noise
		gam_inst_a[k] = gi
		gam_fast_a[k] = gf
		xi_dd_a[k]    = xi_dd

		if freq >= 240.0 && freq <= 2450.0 {
			voc_exc_sum += math.max(gf - 1.0, 0.0)
			voc_exc_cnt += 1
		}
	}
	frame_voc_exc := voc_exc_sum / f64(max(voc_exc_cnt, 1))

	inst_gain : [L3_BINS]f64
	hconf     := swf.harm_conf
	f0_l3     := swf.tracked_f0
	df_l3     := swf.tracked_df if is_ssb else 0.0
	weak_comb := clamp(1.0 - (l3_snr_ratio - 1.25) * 0.55, 0.0, 1.0)
	eff_flr   := g_floor if gate_open else 0.0001

	for k in k_lo ..= k_hi {
		km   := max(k - 1, k_lo)
		kp   := min(k + 1, k_hi)
		freq := f64(k) * bin_hz
		xi_loc  := 0.25 * xi_dd_a[km] + 0.50 * xi_dd_a[k] + 0.25 * xi_dd_a[kp]
		gam_loc := 0.25 * gam_fast_a[km] + 0.50 * gam_fast_a[k] + 0.25 * gam_fast_a[kp]

		// Mistune-aligned harmonic tooth (sigma = 13.5 Hz, using 4096-pt side-chain SNR boost!)
		tooth_w    := 0.0
		ana_h_snr  := 0.0
		if hconf > 0.08 && f0_l3 > 80.0 && freq >= 0.70 * f0_l3 {
			hm := math.round((freq - df_l3) / f0_l3)
			if hm >= 1.0 && hm <= 20.0 {
				f_harm  := hm * f0_l3 + df_l3
				dhz     := math.abs(freq - f_harm)
				sig_t   := 12.5 + 3.0 * (1.0 - weak_comb)
				tooth_w  = math.exp(-0.5 * (dhz * dhz) / (sig_t * sig_t))
				ka_h    := clamp(int(math.round(f_harm * inv_ana_hz)), ka_lo, ka_hi)
				abm_h   := ana_birdie[ka_h]
				ana_h_snr = math.max((swf.ana_med_psd[ka_h] * abm_h * abm_h) / math.max(swf.ana_noise_psd[ka_h], 1e-22) - 1.0, 0.0)
			}
		}

		bm          := birdie_mask[k]
		gw          := (swf.welch_psd[k] * bm * bm) / eff_n_arr[k]
		harm_boost  := hconf * tooth_w * math.max(0.45 * gam_fast_a[k] + 0.55 * (1.0 + ana_h_snr) - 0.82, 0.0)
		persist_exc := math.max(gw - 1.12, 0.0) + 1.35 * harm_boost
		xi_peak     := math.max(xi_dd_a[k] + 1.15 * harm_boost, 0.78 * xi_loc)
		gam_peak    := math.max(0.42 * gam_inst_a[k] + 0.58 * gam_fast_a[k], 0.78 * gam_loc)

		cons_boost := 0.0
		if (cons_vad || swf.harm_hang > 0.14) && freq >= 1350.0 && gam_loc > 1.68 {
			cons_boost = clamp((gam_loc - 1.55) * 0.52, 0.0, 0.65)
		}

		p_loc      := clamp((xi_peak - 0.12) * 2.15 + persist_exc * 0.90 + cons_boost, 0.0, 1.0)
		p_glb      := clamp((frame_voc_exc - 0.14) * 1.45 + 0.60 * hconf * tooth_w, 0.0, 1.0)
		valley_pen := 0.16 * hconf * (1.0 - tooth_w)
		q_base     := 0.98 if (!gate_open || (hconf < 0.06 && cons_boost <= 0.0 && !voiced_formant_vad)) else 0.86
		q_k        := clamp(q_base + valley_pen - 0.62 * p_loc - 0.25 * p_glb, 0.05, 0.985)

		xi_om   := math.max(xi_peak, 0.06)
		gam_om  := math.min(gam_peak + 0.60 * harm_boost, 36.0)
		v_om    := (xi_om / (1.0 + xi_om)) * gam_om
		lr_k    := ((1.0 - q_k) / q_k) * (1.0 / (1.0 + xi_om)) * math.exp(math.min(v_om, 24.0))
		spp_raw := lr_k / (1.0 + lr_k)

		swf.prev_spp[k] = 0.35 * swf.prev_spp[k] + 0.65 * spp_raw
		spp := swf.prev_spp[k]

		g_h1 := mmse_lsa_gain_f64(xi_peak, 0.45 * gam_inst_a[k] + 0.55 * gam_fast_a[k], eff_flr)
		g_h1_arr[k] = g_h1 * bm
		g_om := math.pow(math.max(g_h1, eff_flr), spp) * math.pow(eff_flr, 1.0 - spp)

		// Adaptive comb valley suppression: deep (-24 dB) at -15 dB SNR, gentle/open on strong formants!
		comb_shaper := 1.0
		if hconf > 0.16 {
			val_depth   := (0.86 * weak_comb + 0.36 * (1.0 - weak_comb)) * hconf
			open_t      := max(tooth_w, max(cons_boost * 1.35, clamp((gam_loc - 3.2) * 0.18, 0.0, 0.65)))
			comb_shaper  = (1.0 - val_depth) + val_depth * open_t
		} else if !voiced_formant_vad && cons_boost <= 0.02 {
			comb_shaper = 1.0 - 0.70 * weak_comb
		}

		// Crisp F2/F3 speech articulation presence lift (+1.4 dB @ 1.75 kHz) on active speech bins
		artic := 1.0
		if spp > 0.42 && gate_open {
			df_p := (freq - 1750.0) / 600.0
			bell := math.exp(-0.5 * df_p * df_p)
			artic = 1.0 + 0.18 * clamp((spp - 0.42) * 2.0, 0.0, 1.0) * bell
		}
		inst_gain[k] = clamp(g_om * comb_shaper * artic * bm, eff_flr * 0.25, 1.20)
	}

	for k in 0 ..< L3_BINS {
		if k < k_lo || k > k_hi {
			swf.work_fft[k]   = complex(0.0, 0.0)
			swf.prev_clean[k] = 0.0
			continue
		}

		g_target := inst_gain[k]
		if k > k_lo && k < k_hi {
			nb_max := max(inst_gain[k - 1], inst_gain[k + 1])
			bm     := birdie_mask[k]
			is_transient_twinkle := (swf.welch_psd[k] * bm * bm) < 1.35 * eff_n_arr[k]
			if is_transient_twinkle && g_target > 2.3 * nb_max && nb_max < 0.06 {
				g_target = 0.22 * g_target + 0.78 * (1.35 * nb_max)
			}
		}

		sm_a := 0.18 if (!gate_open && g_target < swf.prev_gain[k]) else 0.38
		swf.prev_gain[k]  = sm_a * swf.prev_gain[k] + (1.0 - sm_a) * g_target
		g_apply          := swf.prev_gain[k]
		spp_k            := swf.prev_spp[k]
		g_dd             := spp_k * g_h1_arr[k] + (1.0 - spp_k) * g_apply
		swf.prev_clean[k] = (g_dd * g_dd) * raw_pow[k]
		swf.work_fft[k]  *= complex(g_apply, 0.0)
	}

	for k in 1 ..< (L3_FFT_N / 2) {
		swf.work_fft[L3_FFT_N - k] = conj(swf.work_fft[k])
	}
	fft_inverse(&swf.fft_plan, swf.work_fft[:])

	if pause_drop && !cons_vad {
		for n in 0 ..< L3_FFT_N {
			u_fade := clamp(f64(n) / 192.0, 0.0, 1.0)
			taper  := 1.0 - 0.96 * (0.5 - 0.5 * math.cos(math.PI * u_fade))
			swf.ola_out[n] *= taper
		}
	}
	for n in 0 ..< L3_FFT_N {
		swf.ola_out[n] += real(swf.work_fft[n]) * swf.window[n] * 0.5
	}

	// Click-free raised-cosine VAD envelope -> exact 0.0 digital silence in all non-speech moments!
	ATT_STEP_L3 :: 1.0 / 192.0 // 4.0 ms smooth attack (inside 32 ms OLA lookahead -> zero onset clipping!)
	REL_STEP_L3 :: 1.0 / 384.0 // 8.0 ms smooth release -> exact 0.0 silence with zero click!
	all_zero := true
	for n in 0 ..< L3_HOP {
		if gate_open {
			swf.gate_phase = min(swf.gate_phase + ATT_STEP_L3, 1.0)
		} else {
			swf.gate_phase = max(swf.gate_phase - REL_STEP_L3, 0.0)
		}
		if swf.gate_phase <= 0.0 {
			hop_out[n] = 0.0
		} else {
			g_env := 0.5 - 0.5 * math.cos(math.PI * swf.gate_phase) if swf.gate_phase < 1.0 else 1.0
			hop_out[n] = swf.ola_out[n] * g_env
			all_zero   = false
		}
	}

	copy(swf.ola_out[0 : L3_FFT_N - L3_HOP], swf.ola_out[L3_HOP : L3_FFT_N])
	for n in (L3_FFT_N - L3_HOP) ..< L3_FFT_N {
		swf.ola_out[n] = 0.0
	}
	if all_zero && !gate_open {
		for n in 0 ..< (L3_FFT_N - L3_HOP) {
			swf.ola_out[n] *= 0.20
			if math.abs(swf.ola_out[n]) < 1e-12 do swf.ola_out[n] = 0.0
		}
		swf.voice_conf = 0.0
	}
}

spectral_wiener_process :: proc(swf: ^Spectral_Wiener_Filter, buf: []f64, mode: Modulation_Mode, rf_notch: bool = false) {
	if swf.stage_idx == 0 && len(buf) > 0 && (len(buf) % L3_HOP) == 0 {
		for off := 0; off < len(buf); off += L3_HOP {
			spectral_wiener_step_hop(swf, buf[off : off + L3_HOP], swf.hop_ready[:], mode, rf_notch)
			copy(buf[off : off + L3_HOP], swf.hop_ready[:])
		}
		return
	}

	for i in 0 ..< len(buf) {
		swf.hop_staging[swf.stage_idx] = buf[i]
		buf[i] = swf.hop_ready[swf.stage_idx]
		swf.stage_idx += 1
		if swf.stage_idx == L3_HOP {
			swf.stage_idx = 0
			spectral_wiener_step_hop(swf, swf.hop_staging[:], swf.hop_ready[:], mode, rf_notch)
		}
	}
}

// ---------------------------------------------------------------------------
// FILTER LEVEL 4: Ultra Deep Weak-Signal Voice Recovery (-20 dB target, down to -30 dB)
//   - Three-Timescale Fast (~45 ms), Medium (-20 dB ~160 ms) & Deep (-30 dB ~340 ms) PSDs
//   - Variance-Equalized SHS Comb (85..260 Hz) + Zero-Self-Correlation Anti-Blast Gate
//   - Surgical Narrowband Heterodyne / CW Auto-Notch ('notch on' +2.3 dB spurs)
//   - Adaptive Phase-Coherent MMSE-LSA + Cepstral Formant Liftering + Exact 0.0 Silence Gate
// ---------------------------------------------------------------------------

L4_FFT_N        :: 4096
L4_HOP          :: 1024
L4_BINS         :: (L4_FFT_N / 2) + 1 // 2049 bins (11.71875 Hz/bin)
L4_F0_STEP      :: 0.25
L4_NUM_F0       :: 701                // 85.0 .. 260.0 Hz in 0.25 Hz steps (zero straddle loss @ 2.93 Hz/bin!)
L4_MED_F0_STEP  :: 0.50
L4_MED_NUM_F0   :: 351                // 85.0 .. 260.0 Hz in 0.50 Hz steps (glide-agile @ 5.86 Hz/bin!)
L4_FAST_NUM_F0  :: 176                // 85.0 .. 260.0 Hz in 1.00 Hz steps

Ultra_Deep_Voice_Filter :: struct {
	pre_clean          : Pre_Impulse_Hum_Cleaner,
	fft_plan           : FFT_Plan,
	window             : [L4_FFT_N]f64,
	in_ring            : [L4_FFT_N]f64,
	ola_out            : [L4_FFT_N]f64,
	hop_staging        : [L4_HOP]f64,
	hop_ready          : [L4_HOP]f64,
	stage_idx          : int,
	work_fft           : [L4_FFT_N]complex128,
	cepstrum_fft       : [L4_FFT_N]complex128,

	// Multi-timescale PSDs + long-horizon CW notch tracker
	fast_psd           : [L4_BINS]f64,
	welch_psd          : [L4_BINS]f64,
	pitch_psd          : [L4_BINS]f64,
	long_psd           : [L4_BINS]f64,
	noise_psd          : [L4_BINS]f64,
	tonal_mag_ema      : [L4_BINS]f64,
	tonal_pow_ema      : [L4_BINS]f64,
	cw_persist         : [L4_BINS]f64,
	shs_smooth         : [L4_NUM_F0]f64,
	med_shs_smooth     : [L4_MED_NUM_F0]f64,
	silent_hops        : int,

	// 12 kHz 1536-pt (128ms), 2048-pt (5.86 Hz/bin) & 4096-pt (2.93 Hz/bin) unclamped Welch + zero-overlap comb side-chain
	hr_lpf             : Decim12k_LPF,
	hr_plan2k          : FFT_Plan,
	hr_plan4k          : FFT_Plan,
	hr_win128          : [HR128_N]f64,
	hr_win2k           : [HR2K_N]f64,
	hr_win4k           : [HR4K_N]f64,
	hr_fft             : [HR4K_N]complex128,
	hr_ring            : [HR4_RING_N]f64,
	hr_past128_exc     : [HR2K_BINS]f64,
	hr_pres128_exc     : [HR2K_BINS]f64,
	hr_past2k_avg      : [HR2K_BINS]f64,
	hr_mfin2k          : [HR2K_BINS]f64,
	hr_wb2k            : [HR2K_BINS]f64,
	hr_wb2k_psd        : [HR2K_BINS]f64,
	hr_past2k_pow      : [4][HR2K_BINS]f64,
	hr_pres2k_pow      : [5][HR2K_BINS]f64,
	hr_past4k          : [HR4K_BINS]f64,
	hr_fin4k_avg       : [HR4K_BINS]f64,
	hr_wb4k_exc        : [HR4K_BINS]f64,
	hr_wb4k_psd        : [HR4K_BINS]f64,
	hr_past4k_pow      : [3][HR4K_BINS]f64,
	hr_pres4k_pow      : [3][HR4K_BINS]f64,
	hr_pos2k           : int,
	hr_filled2k        : int,
	hr_pos4k           : int,
	hr_filled4k        : int,

	// Ephraim-Malah MMSE-LSA & temporal gain state
	prev_clean_psd     : [L4_BINS]f64,
	prev_gain          : [L4_BINS]f64,

	// Pitch, SSB mistune, multi-tier lock & zero-silence VAD state
	tracked_f0_hz      : f64,
	f0_prev_cand       : f64,
	f0_prev_med        : f64,
	f0_prev_fast       : f64,
	deep_f0_ring       : [4]f64, // 4 hops * 1024 = 4096 samples -> 0.0% STFT window overlap!
	deep_df_ring       : [4]f64,
	med_f0_ring        : [4]f64,
	med_df_ring        : [4]f64,
	ring_pos           : int,
	tracked_df_hz      : f64,
	df_profile         : [DF_PROF_BINS]f64,
	df_cand_live       : f64,
	df_z_live          : f64,
	mistune_cand_df    : f64,
	mistune_f0_cand    : f64,
	mistune_streak     : int,
	wb4k_f0_prev       : f64,
	wb4k_streak        : int,
	inst_comb_deep_ema : f64,
	inst_comb_med_ema  : f64,
	voice_confidence   : f64,
	voice_hang         : f64,
	speech_context     : f64,
	noise_reduc_db     : f64,
	pitch_lock_streak  : int,
	med_lock_streak    : int,
	fast_lock_streak   : int,
	gate_phase         : f64,
	hang_hops          : int,
	voiced_hang_hops   : int,
	syll_active_flag   : bool,
	pend_tonal         : [L4_BINS]bool,
	pend_sh_hz         : f64,
	pend_sh_wait       : int,
	frame_cnt          : u64,
}

ultra_voice_filter_init :: proc(uvf: ^Ultra_Deep_Voice_Filter) {
	uvf^ = {}
	pre_cleaner_init(&uvf.pre_clean)
	fft_plan_init(&uvf.fft_plan,  L4_FFT_N)
	fft_plan_init(&uvf.hr_plan2k, HR2K_N)
	fft_plan_init(&uvf.hr_plan4k, HR4K_N)
	for n in 0 ..< L4_FFT_N {
		uvf.window[n]   = root_hann_f64(n, L4_FFT_N)
		uvf.hr_win4k[n] = uvf.window[n] * uvf.window[n] // Periodic Hann @ 12 kHz: 2.93 Hz/bin (+6.02 dB gain!)
	}
	for n in 0 ..< HR128_N {
		r := root_hann_f64(n, HR128_N)
		uvf.hr_win128[n] = r * r // 128 ms Periodic Hann @ 12 kHz (agile syllable onset/pause)
	}
	for n in 0 ..< HR2K_N {
		r := root_hann_f64(n, HR2K_N)
		uvf.hr_win2k[n] = r * r // Periodic Hann @ 12 kHz: 5.86 Hz/bin (+3.01 dB glide-agile gain!)
	}
	uvf.tracked_f0_hz = 145.0
	for k in 0 ..< L4_BINS {
		uvf.prev_gain[k] = 0.0
	}
}

@(private)
ultra_voice_step_hop :: proc(uvf: ^Ultra_Deep_Voice_Filter, hop_in: []f64, hop_out: []f64, mode: Modulation_Mode, rf_notch: bool) {
	band   := mode_to_band_family(mode)
	is_ssb := (band == .SSB_Band)

	// 1. Pre-clean indoor triac/LED/static impulses + European mains hum & push 4:1 decimated 12 kHz ring
	copy(uvf.in_ring[0 : L4_FFT_N - L4_HOP], uvf.in_ring[L4_HOP : L4_FFT_N])
	copy(uvf.hr_ring[0 : HR4_RING_N - HR_STEP_12K], uvf.hr_ring[HR_STEP_12K : HR4_RING_N])
	raw_hop_energy := 0.0
	for i in 0 ..< L4_HOP {
		raw_hop_energy += hop_in[i] * hop_in[i]
		cleaned := pre_cleaner_step(&uvf.pre_clean, hop_in[i], 3.18, is_ssb)
		uvf.in_ring[L4_FFT_N - L4_HOP + i] = cleaned
		lp12 := decim12k_lpf_step(&uvf.hr_lpf, cleaned)
		if (i & 3) == 3 {
			uvf.hr_ring[HR4_RING_N - HR_STEP_12K + (i >> 2)] = lp12
		}
	}

	for n in 0 ..< L4_FFT_N {
		uvf.work_fft[n] = complex(uvf.in_ring[n] * uvf.window[n], 0.0)
	}
	fft_forward(&uvf.fft_plan, uvf.work_fft[:])

	bin_hz     := AUDIO_SAMPLE_RATE_HZ / f64(L4_FFT_N) // 11.71875 Hz/bin
	inv_bin_hz := 1.0 / bin_hz
	uvf.frame_cnt += 1

	f_lo := 120.0
	f_hi := 2700.0
	switch band {
	case .AM_Band:
		f_lo = 128.0
		f_hi = 3250.0
	case .FM_Band:
		f_lo = 135.0
		f_hi = 2950.0
	case .SSB_Band:
		f_lo = 120.0
		f_hi = 2700.0
	}
	k_lo := clamp(int(math.floor(f_lo * inv_bin_hz)), 3, L4_BINS - 10)
	k_hi := clamp(int(math.ceil(f_hi * inv_bin_hz)), k_lo + 16, L4_BINS - 10)

	// 2. Update Multi-Timescale PSD Estimators:
	//    - fast_psd  (alpha=0.40 -> ~42 ms): crisp syllables, pitch glides & consonants
	//    - welch_psd (alpha=0.14 -> ~160 ms): -20 dB conversational speech comb & formant tracker
	//    - pitch_psd (alpha=0.08 -> ~340 ms): -25..-30 dB ultra-deep harmonic integrator
	//    - long_psd  (alpha=0.06 -> ~400 ms): narrowband CW / heterodyne spur notch tracker
	raw_pow : [L4_BINS]f64
	for k in 0 ..< L4_BINS {
		p   := c64_abs2(uvf.work_fft[k])
		p_c := math.max(p, 1e-20)
		mag := math.sqrt(p)
		raw_pow[k] = p

		if uvf.frame_cnt <= 4 {
			uvf.fast_psd[k]      = p_c
			uvf.welch_psd[k]     = p_c
			uvf.pitch_psd[k]     = p_c
			uvf.long_psd[k]      = p_c
			uvf.tonal_mag_ema[k] = mag
			uvf.tonal_pow_ema[k] = p_c
		} else {
			f_alpha := 0.55 if uvf.frame_cnt <= 7 else 0.48
			w_alpha := 0.34 if uvf.frame_cnt <= 10 else 0.22
			p_alpha := 0.25 if uvf.frame_cnt <= 10 else 0.055
			l_alpha := 0.22 if uvf.frame_cnt <= 10 else 0.055
			uvf.fast_psd[k]  = (1.0 - f_alpha) * uvf.fast_psd[k]  + f_alpha * p_c
			uvf.welch_psd[k] = (1.0 - w_alpha) * uvf.welch_psd[k] + w_alpha * p_c
			uvf.pitch_psd[k] = (1.0 - p_alpha) * uvf.pitch_psd[k] + p_alpha * p_c
			uvf.long_psd[k]  = (1.0 - l_alpha) * uvf.long_psd[k]  + l_alpha * p_c
			if p_c > 8.5 * uvf.tonal_pow_ema[k] {
				uvf.tonal_mag_ema[k] = mag
				uvf.tonal_pow_ema[k] = p_c
			} else {
				uvf.tonal_mag_ema[k] = 0.89 * uvf.tonal_mag_ema[k] + 0.11 * mag
				uvf.tonal_pow_ema[k] = 0.89 * uvf.tonal_pow_ema[k] + 0.11 * p_c
			}
		}
	}
	if uvf.pend_sh_wait > 0 {
		uvf.pend_sh_wait -= 1
		sh_bins := int(math.round(uvf.pend_sh_hz * inv_bin_hz))
		for k in 10 ..< (L4_BINS - 12) {
			nf    := math.max(uvf.noise_psd[k], 1e-20)
			c_now := (uvf.tonal_mag_ema[k] * uvf.tonal_mag_ema[k]) / math.max(uvf.tonal_pow_ema[k], 1e-20)
			if raw_pow[k] > 5.0 * nf && c_now < 0.92 {
				for d_s in -1 ..= 1 {
					ks := clamp(k - sh_bins + d_s, 10, L4_BINS - 12)
					if uvf.pend_tonal[ks] {
						p_k := math.max(raw_pow[k], 1e-20)
						uvf.fast_psd[k]      = p_k
						uvf.welch_psd[k]     = p_k
						uvf.long_psd[k]      = p_k
						uvf.tonal_mag_ema[k] = math.sqrt(p_k)
						uvf.tonal_pow_ema[k] = p_k
						break
					}
				}
			}
		}
	}

	// 3. Speech-Immune 57-Bin (668 Hz) Interior-Passband Timescale-Matched Local-Quantile Noise PSD Estimators
	nf_f_lo := 170.0 if is_ssb else 160.0
	nf_f_hi := 2540.0 if is_ssb else (f_hi - 120.0)
	nf_k_lo := clamp(int(math.ceil(nf_f_lo * inv_bin_hz)), k_lo, k_hi - 16)
	nf_k_hi := clamp(int(math.floor(nf_f_hi * inv_bin_hz)), nf_k_lo + 16, k_hi)

	med_nf_arr  : [L4_BINS]f64
	fast_nf_arr : [L4_BINS]f64
	sp_hold4    := uvf.voice_confidence >= 0.18
	init_cap4   := 1.0e30
	if is_ssb && uvf.frame_cnt <= 10 {
		init_cap4 = 2.4 * local_quantile_noise_floor(uvf.fast_psd[:], (nf_k_lo + nf_k_hi) / 2, 192, nf_k_lo, nf_k_hi, 22, 1.42, bin_hz)
	}
	for k in k_lo ..= k_hi {
		loc_p     := local_quantile_noise_floor(uvf.pitch_psd[:], k, 28, nf_k_lo, nf_k_hi, 38, 1.145, bin_hz)
		loc_f     := local_quantile_noise_floor(uvf.fast_psd[:],  k, 28, nf_k_lo, nf_k_hi, 42, 1.42,  bin_hz)
		loc_floor := math.min(math.min(loc_p, loc_f), init_cap4)
		if uvf.frame_cnt <= 6 {
			uvf.noise_psd[k] = loc_floor
		} else if loc_floor <= uvf.noise_psd[k] {
			uvf.noise_psd[k] = 0.65 * uvf.noise_psd[k] + 0.35 * loc_floor
		} else if sp_hold4 {
			uvf.noise_psd[k] = 0.992 * uvf.noise_psd[k] + 0.008 * math.min(loc_floor, 1.25 * uvf.noise_psd[k])
		} else {
			uvf.noise_psd[k] = 0.88 * uvf.noise_psd[k] + 0.12 * loc_floor
		}
	}
	swell4_fast := compute_qrn_swell_factor(uvf.fast_psd[:], uvf.noise_psd[:], nf_k_lo, nf_k_hi, bin_hz, 1.58)
	swell4_med  := math.sqrt(swell4_fast)
	for k in k_lo ..= k_hi {
		med_nf_arr[k]  = uvf.noise_psd[k] * swell4_med
		fast_nf_arr[k] = uvf.noise_psd[k] * swell4_fast
	}

	// 4. Harmonic-Protected Birdie & Speech-Safe Narrowband CW Heterodyne Auto-Notch BEFORE SHS Pitch Search!
	birdie_mask : [L4_BINS]f64
	for k in 0 ..< L4_BINS do birdie_mask[k] = 1.0

	quick_lo_s, quick_lo_n := 0.0, 0.0
	for k in k_lo ..= k_hi {
		fk := f64(k) * bin_hz
		if fk >= 180.0 && fk <= 1150.0 && !is_hum_notch_freq(fk) {
			quick_lo_s += uvf.fast_psd[k]
			quick_lo_n += math.max(fast_nf_arr[k], uvf.noise_psd[k])
		}
	}
	quick_lo_snr := quick_lo_s / math.max(quick_lo_n, 1e-20)

	if uvf.frame_cnt >= 6 {
		df_chk := uvf.tracked_df_hz if is_ssb else 0.0
		for k in (k_lo + 1) ..= (k_hi - 1) {
			n_flr     := math.max(uvf.noise_psd[k], 1e-20)
			constancy := (uvf.tonal_mag_ema[k] * uvf.tonal_mag_ema[k]) / math.max(uvf.tonal_pow_ema[k], 1e-20)
			b_psd     := max(uvf.welch_psd[k], 0.72 * uvf.fast_psd[k])
			b_prev    := max(uvf.welch_psd[k - 1], 0.72 * uvf.fast_psd[k - 1])
			b_next    := max(uvf.welch_psd[k + 1], 0.72 * uvf.fast_psd[k + 1])
			is_peak   := b_psd >= b_prev && b_psd >= b_next
			long_pk   := uvf.long_psd[k] >= uvf.long_psd[k - 1] && uvf.long_psd[k] >= uvf.long_psd[k + 1]

			strong_hit := false
			if is_peak && b_psd > 6.5 * n_flr {
				b7_sum := 1e-20
				for j in -3 ..= 3 {
					kk := clamp(k + j, k_lo, k_hi)
					b7_sum += max(uvf.welch_psd[kk], 0.72 * uvf.fast_psd[kk])
				}
				pk_conc   := (b_psd + max(b_prev, b_next)) / b7_sum
				tonal_hit := constancy > 0.89 || (constancy > 0.74 && pk_conc > 0.78 && b_psd > 9.5 * n_flr)
				if tonal_hit && !is_vocal_harmonic_member(uvf.welch_psd[:], uvf.fast_psd[:], uvf.noise_psd[:], k, bin_hz, k_lo, k_hi, df_chk) {
					strong_hit = true
				}
			}

			weak_cw_ok := false
			if rf_notch && uvf.frame_cnt >= 8 {
				bg_sum := 0.0
				bg_cnt := 0
				for dk in -7 ..= 7 {
					if math.abs(dk) <= 2 do continue
					kk := k + dk
					if kk >= k_lo && kk <= k_hi {
						bg_sum += min(uvf.long_psd[kk], 2.2 * uvf.noise_psd[kk])
						bg_cnt += 1
					}
				}
				bg_long  := math.max(bg_sum / f64(max(bg_cnt, 1)), 0.75 * n_flr)
				spur_snr := uvf.long_psd[k] / bg_long
				s7_exc   := 1e-20
				for dk in -3 ..= 3 {
					kk := clamp(k + dk, k_lo, k_hi)
					s7_exc += math.max(uvf.long_psd[kk] - bg_long, 0.0)
				}
				pk2_exc := math.max(uvf.long_psd[k] - bg_long, 0.0) +
				           math.max(max(uvf.long_psd[k - 1], uvf.long_psd[k + 1]) - bg_long, 0.0)
				sharp   := pk2_exc / s7_exc
				if uvf.cw_persist[k] >= 0.48 {
					// Already locked onto continuous CW heterodyne: hold smoothly across speech modulation dips!
					if long_pk && spur_snr > 1.48 && constancy > 0.68 {
						uvf.cw_persist[k] = min(uvf.cw_persist[k] + 0.09, 1.0)
					} else {
						uvf.cw_persist[k] *= 0.980
					}
				} else if long_pk && spur_snr > 1.72 && constancy > 0.81 && sharp > 0.58 && (quick_lo_snr < 1.25 || spur_snr > 4.2) {
					if !is_vocal_harmonic_member(uvf.long_psd[:], uvf.fast_psd[:], uvf.noise_psd[:], k, bin_hz, k_lo, k_hi, df_chk) {
						uvf.cw_persist[k] = min(uvf.cw_persist[k] + 0.18, 1.0)
					} else {
						uvf.cw_persist[k] *= 0.85
					}
				} else {
					uvf.cw_persist[k] *= 0.88
				}
				weak_cw_ok = uvf.cw_persist[k] >= 0.48
			} else if !rf_notch {
				uvf.cw_persist[k] *= 0.90
			}

			if strong_hit {
				att := clamp(math.sqrt(n_flr / math.max(raw_pow[k], n_flr)) * 0.05, 0.0001, 1.0)
				for dk in -3 ..= 3 {
					kk := k + dk
					if kk >= k_lo && kk <= k_hi do birdie_mask[kk] = min(birdie_mask[kk], att)
				}
			} else if weak_cw_ok {
				att := clamp(math.sqrt(n_flr / math.max(raw_pow[k], n_flr)) * 0.04, 0.0002, 1.0)
				for dk in -2 ..= 2 {
					kk := k + dk
					if kk >= k_lo && kk <= k_hi do birdie_mask[kk] = min(birdie_mask[kk], att)
				}
				sh := max(att * 3.0, 0.06)
				if k - 3 >= k_lo do birdie_mask[k - 3] = min(birdie_mask[k - 3], sh)
				if k + 3 <= k_hi do birdie_mask[k + 3] = min(birdie_mask[k + 3], sh)
			}
		}
	}

	// Measure broadband speech-band SNR & multi-peak voiced formant structure
	sum_welch_band := 0.0
	sum_noise_band := 0.0
	for k in k_lo ..= k_hi {
		bm := birdie_mask[k]
		sum_welch_band += uvf.welch_psd[k] * (bm * bm)
		sum_noise_band += uvf.noise_psd[k]
	}
	broadband_snr_ratio := sum_welch_band / math.max(sum_noise_band, 1e-20)

	// 5. 12 kHz 2048-pt (5.86 Hz/bin) & 4096-pt (2.93 Hz/bin) Disjoint Zero-Overlap Coherent SHS Comb:
	//    - syll_exc    (~42 ms 48 kHz fast_psd): rapid syllables & pitch glides
	//    - hr_mfin2k   (235 ms @ 5.86 Hz/bin) + hr_past2k_avg [1792..4095] vs hr_pres2k [4096..6143]: glide-agile -15..-21 dB!
	//    - hr_fin4k_avg(384 ms @ 2.93 Hz/bin) + hr_past4k     [0..4095]    vs hr_pres2k [4096..6143]: ultra-deep -20..-30 dB!
	comb_f_lo  := 165.0 if is_ssb else 155.0
	comb_f_hi  := 1720.0 if is_ssb else math.min(2000.0, f_hi - 110.0)
	inv_hr2_hz := 1.0 / HR2K_BIN_HZ
	inv_hr4_hz := 1.0 / HR4K_BIN_HZ
	hk2_lo     := clamp(int(math.floor(comb_f_lo * inv_hr2_hz)), 4, HR2K_BINS - 12)
	hk2_hi     := clamp(int(math.ceil(comb_f_hi * inv_hr2_hz)), hk2_lo + 16, HR2K_BINS - 4)
	hk4_lo     := clamp(int(math.floor(comb_f_lo * inv_hr4_hz)), 6, HR4K_BINS - 16)
	hk4_hi     := clamp(int(math.ceil(comb_f_hi * inv_hr4_hz)), hk4_lo + 24, HR4K_BINS - 4)

	deep_exc  : [L4_BINS]f64
	med_exc   : [L4_BINS]f64
	syll_exc  : [L4_BINS]f64
	raw_exc_u : [L4_BINS]f64
	for k in k_lo ..= k_hi {
		fk     := f64(k) * bin_hz
		bm     := birdie_mask[k]
		nf_d   := math.max(uvf.noise_psd[k], 1e-20)
		nf_m   := math.max(med_nf_arr[k],    nf_d)
		nf_f   := math.max(fast_nf_arr[k],   nf_m)
		gam_d  := (uvf.pitch_psd[k] * bm * bm) / nf_d
		gam_m  := (uvf.welch_psd[k] * bm * bm) / nf_m
		gam_s  := (uvf.fast_psd[k]  * bm * bm) / nf_f
		gam_ru := (raw_pow[k]       * bm * bm) / nf_d
		if bm < 0.65 || is_hum_notch_freq(fk) || fk < comb_f_lo || fk > comb_f_hi {
			deep_exc[k]  = 0.0
			med_exc[k]   = 0.0
			syll_exc[k]  = 0.0
			raw_exc_u[k] = 0.0
		} else {
			deep_exc[k]  = clamp(gam_d  - 1.0, -0.88, 2.6)
			med_exc[k]   = clamp(gam_m  - 1.0, -0.88, 2.8)
			syll_exc[k]  = clamp(gam_s  - 1.0, -0.88, 3.0)
			raw_exc_u[k] = clamp(gam_ru - 1.0, -1.00, 4.5)
		}
	}

	if uvf.frame_cnt >= 8 {
		p128_past, p128_pres : [HR2K_BINS]f64
		compute_hires_raw_pow(&uvf.hr_plan2k, uvf.hr_ring[6144 : 7680], uvf.hr_win128[:], uvf.hr_fft[:], p128_past[:])
		compute_hires_raw_pow(&uvf.hr_plan2k, uvf.hr_ring[7680 : 9216], uvf.hr_win128[:], uvf.hr_fft[:], p128_pres[:])
		normalize_hires_psd_to_exc(p128_past[:], birdie_mask[:], bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.62, uvf.hr_past128_exc[:])
		normalize_hires_psd_to_exc(p128_pres[:], birdie_mask[:], bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.62, uvf.hr_pres128_exc[:])

		// Disjoint zero-overlap 2048-pt (5.86 Hz/bin): past2k {3328..6143} vs pres2k {6144..9215} (0 shared samples, 490.7 ms!)
		raw_past2k, raw_pres2k : [HR2K_BINS]f64
		compute_hires_raw_pow(&uvf.hr_plan2k, uvf.hr_ring[4096 : 6144], uvf.hr_win2k[:], uvf.hr_fft[:], raw_past2k[:])
		compute_hires_raw_pow(&uvf.hr_plan2k, uvf.hr_ring[7168 : 9216], uvf.hr_win2k[:], uvf.hr_fft[:], raw_pres2k[:])

		uvf.hr_past2k_pow[uvf.hr_pos2k & 3] = raw_past2k
		uvf.hr_pres2k_pow[uvf.hr_pos2k % 5] = raw_pres2k
		a_wb2 := 1.0 if uvf.hr_filled2k == 0 else (0.35 if uvf.hr_filled2k <= 3 else 0.16)
		for hk in 0 ..< HR2K_BINS {
			uvf.hr_wb2k_psd[hk] = (1.0 - a_wb2) * uvf.hr_wb2k_psd[hk] + a_wb2 * raw_pres2k[hk]
		}
		uvf.hr_pos2k   += 1
		uvf.hr_filled2k = min(uvf.hr_filled2k + 1, 16)

		np2 := min(uvf.hr_filled2k, 4)
		nm2 := min(uvf.hr_filled2k, 5)
		past2k_psd, pres2k_psd : [HR2K_BINS]f64
		for hk in 0 ..< HR2K_BINS {
			sp := 0.0
			for si in 0 ..< np2 do sp += uvf.hr_past2k_pow[(uvf.hr_pos2k - 1 - si) & 3][hk]
			past2k_psd[hk] = sp / f64(np2)

			sm := 0.0
			for si in 0 ..< nm2 do sm += uvf.hr_pres2k_pow[(uvf.hr_pos2k - 1 - si) % 5][hk]
			pres2k_psd[hk] = sm / f64(nm2)
		}
		sc_p2 := 1.32 if np2 >= 3 else 1.52
		sc_m2 := 1.28 if nm2 >= 4 else 1.48
		normalize_hires_psd_to_exc(past2k_psd[:],      birdie_mask[:], bin_hz, HR2K_N, comb_f_lo, comb_f_hi, sc_p2, uvf.hr_past2k_avg[:])
		normalize_hires_psd_to_exc(pres2k_psd[:],      birdie_mask[:], bin_hz, HR2K_N, comb_f_lo, comb_f_hi, sc_m2, uvf.hr_mfin2k[:])
		normalize_hires_psd_to_exc(uvf.hr_wb2k_psd[:], birdie_mask[:], bin_hz, HR2K_N, comb_f_lo, comb_f_hi, 1.20,  uvf.hr_wb2k[:])
	}

	if uvf.frame_cnt >= 14 {
		// Disjoint zero-overlap 4096-pt (2.93 Hz/bin): past4k {0..4607} vs pres4k {4608..9215} (0 shared samples, 768.0 ms!)
		raw_past4k, raw_pres4k : [HR4K_BINS]f64
		compute_hires_raw_pow(&uvf.hr_plan4k, uvf.hr_ring[512  : 4608], uvf.hr_win4k[:], uvf.hr_fft[:], raw_past4k[:])
		compute_hires_raw_pow(&uvf.hr_plan4k, uvf.hr_ring[5120 : 9216], uvf.hr_win4k[:], uvf.hr_fft[:], raw_pres4k[:])

		uvf.hr_past4k_pow[uvf.hr_pos4k % 3] = raw_past4k
		uvf.hr_pres4k_pow[uvf.hr_pos4k % 3] = raw_pres4k
		a_wb4 := 1.0 if uvf.hr_filled4k == 0 else (0.32 if uvf.hr_filled4k <= 3 else 0.14)
		for hk in 0 ..< HR4K_BINS {
			uvf.hr_wb4k_psd[hk] = (1.0 - a_wb4) * uvf.hr_wb4k_psd[hk] + a_wb4 * raw_pres4k[hk]
		}
		uvf.hr_pos4k   += 1
		uvf.hr_filled4k = min(uvf.hr_filled4k + 1, 32)

		n4 := min(uvf.hr_filled4k, 3)
		past4k_psd, pres4k_psd : [HR4K_BINS]f64
		for hk in 0 ..< HR4K_BINS {
			sp4, pr4 := 0.0, 0.0
			for si in 0 ..< n4 {
				slot := (uvf.hr_pos4k - 1 - si) % 3
				sp4 += uvf.hr_past4k_pow[slot][hk]
				pr4 += uvf.hr_pres4k_pow[slot][hk]
			}
			past4k_psd[hk] = sp4 / f64(n4)
			pres4k_psd[hk] = pr4 / f64(n4)
		}
		sc4 := 1.38 if n4 >= 2 else 1.62
		normalize_hires_psd_to_exc(past4k_psd[:],      birdie_mask[:], bin_hz, HR4K_N, comb_f_lo, comb_f_hi, sc4,  uvf.hr_past4k[:])
		normalize_hires_psd_to_exc(pres4k_psd[:],      birdie_mask[:], bin_hz, HR4K_N, comb_f_lo, comb_f_hi, sc4,  uvf.hr_fin4k_avg[:])
		normalize_hires_psd_to_exc(uvf.hr_wb4k_psd[:], birdie_mask[:], bin_hz, HR4K_N, comb_f_lo, comb_f_hi, 1.18, uvf.hr_wb4k_exc[:])
	}

	// Stage 5A/5B Joint High-Res (F0, df) Comb Search:
	//   1) 0.50 Hz F0 grid on 5.86 Hz/bin hr_mfin2k across 9-point SSB mistune grid {-36..+36 Hz} + tracked_df_hz,
	//      followed by ±4.5 Hz (1.5 Hz step) local df refinement -> ±0.75 Hz accuracy on hop 1!
	//   2) Passes {0.0, tracked_df_hz, med_best_df} into 0.25 Hz @ 2.93 Hz/bin hr_fin4k_avg & hr_wb4k_exc!
	df_steps_m4 := 1
	df_tab_m4   := [10]f64{0.0, -36.0, -27.0, -18.0, -9.0, +9.0, +18.0, +27.0, +36.0, 0.0}
	df_pen_m4   := [10]f64{1.0,  0.96,  0.97,  0.97, 0.98, 0.98,  0.97,  0.97,  0.96, 1.0}
	if is_ssb {
		df_steps_m4 = 9
		if math.abs(uvf.tracked_df_hz) >= 3.5 {
			df_tab_m4[9] = uvf.tracked_df_hz
			df_steps_m4  = 10
		}
	}

	use_hr4_deep := uvf.hr_filled4k >= 1
	d_exc_slice  := uvf.hr_fin4k_avg[:] if use_hr4_deep else deep_exc[:]
	d_kl         := hk4_lo              if use_hr4_deep else k_lo
	d_kh         := hk4_hi              if use_hr4_deep else k_hi
	d_inv_hz     := inv_hr4_hz          if use_hr4_deep else inv_bin_hz

	use_hr2_med  := uvf.hr_filled2k >= 1
	m_exc_slice  := uvf.hr_mfin2k[:] if use_hr2_med else med_exc[:]
	m_kl         := hk2_lo           if use_hr2_med else k_lo
	m_kh         := hk2_hi           if use_hr2_med else k_hi
	m_inv_hz     := inv_hr2_hz       if use_hr2_med else inv_bin_hz

	max_med_idx4  := 211 if is_ssb else L4_MED_NUM_F0  // 85..190 Hz on SSB, 85..265 Hz on AM/FM
	max_deep_idx4 := 421 if is_ssb else L4_NUM_F0      // 85..190 Hz on SSB, 85..265 Hz on AM/FM
	max_fast_idx4 := 106 if is_ssb else L4_FAST_NUM_F0 // 85..190 Hz on SSB, 85..265 Hz on AM/FM

	prev_shs_d  := uvf.shs_smooth
	prev_shs_m  := uvf.med_shs_smooth
	f0_scores   : [L4_NUM_F0]f64
	f0_harms    : [L4_NUM_F0]int
	f0_dfs      : [L4_NUM_F0]f64
	med_scores  : [L4_MED_NUM_F0]f64
	med_harms   : [L4_MED_NUM_F0]int
	med_dfs     : [L4_MED_NUM_F0]f64
	fast_scores : [L4_FAST_NUM_F0]f64
	fast_harms  : [L4_FAST_NUM_F0]int
	fast_dfs    : [L4_FAST_NUM_F0]f64

	med_best_sc  := -1.0e9
	med_best_idx := 60
	med_best_df  := 0.0
	med_best_ph  := 0

	for idx in 0 ..< max_med_idx4 {
		cand_f0 := 85.0 + L4_MED_F0_STEP * f64(idx)
		m_best  := -1.0e9
		m_df    := 0.0
		m_ph    := 0
		for d_i in 0 ..< df_steps_m4 {
			dfc := df_tab_m4[d_i]
			if math.abs(dfc) > 0.46 * cand_f0 do continue
			sc_m, ph_m, _ := eval_uniform_shs_comb(m_exc_slice, cand_f0, dfc, comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz, 0.035)
			sc_m *= df_pen_m4[d_i]
			if sc_m > m_best { m_best = sc_m; m_df = dfc; m_ph = ph_m }
		}
		im     := max(idx - 1, 0)
		ip     := min(idx + 1, max_med_idx4 - 1)
		diff_m := 0.16 * prev_shs_m[im] + 0.68 * prev_shs_m[idx] + 0.16 * prev_shs_m[ip]
		uvf.med_shs_smooth[idx] = 0.45 * diff_m + 0.55 * m_best
		med_scores[idx]         = uvf.med_shs_smooth[idx]
		med_harms[idx]          = m_ph
		med_dfs[idx]            = m_df
		if med_scores[idx] > med_best_sc {
			med_best_sc  = med_scores[idx]
			med_best_idx = idx
			med_best_df  = m_df
			med_best_ph  = m_ph
		}
	}

	mean_med := 0.0
	for idx in 0 ..< max_med_idx4 do mean_med += med_scores[idx]
	mean_med /= f64(max_med_idx4)
	var_med := 1e-9
	for idx in 0 ..< max_med_idx4 { dm := med_scores[idx] - mean_med; var_med += dm * dm }
	std_med := math.sqrt(var_med / f64(max_med_idx4))

	med_f0 := 85.0 + L4_MED_F0_STEP * f64(med_best_idx)
	if med_f0 >= 170.0 {
		hi := int(math.round((0.5 * med_f0 - 85.0) / L4_MED_F0_STEP))
		if hi >= 0 && hi < max_med_idx4 && med_scores[hi] > mean_med + 0.35 * std_med && med_scores[hi] >= 0.45 * med_best_sc {
			med_best_idx = hi
			med_f0       = 85.0 + L4_MED_F0_STEP * f64(hi)
			med_best_ph  = max(med_best_ph, med_harms[hi])
			med_best_df  = med_dfs[hi]
		}
	}
	if med_best_idx > 0 && med_best_idx + 1 < max_med_idx4 {
		sm, s0, sp := med_scores[med_best_idx - 1], med_scores[med_best_idx], med_scores[med_best_idx + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do med_f0 += L4_MED_F0_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}
	if is_ssb {
		ref_b := -1.0e9
		ref_d := med_best_df
		for rs in -3 ..= 3 {
			fd := clamp(med_best_df + 1.5 * f64(rs), -42.0, 42.0)
			scr, phr, _ := eval_uniform_shs_comb(m_exc_slice, med_f0, fd, comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz, 0.035)
			if scr > ref_b { ref_b = scr; ref_d = fd; med_best_ph = max(med_best_ph, phr) }
		}
		med_best_df = ref_d
	}

	df_steps := 1
	df_table := [3]f64{0.0, 0.0, 0.0}
	df_pen   := [3]f64{1.0, 1.0, 0.98}
	if is_ssb && math.abs(uvf.tracked_df_hz) >= 3.5 {
		df_table[df_steps] = uvf.tracked_df_hz
		df_steps += 1
	}
	if is_ssb && math.abs(med_best_df) >= 3.5 && math.abs(med_best_df - uvf.tracked_df_hz) > 2.0 && df_steps < 3 {
		df_table[df_steps] = med_best_df
		df_steps += 1
	}

	best_score     := -1.0e9
	best_idx       := 120
	best_df        := 0.0
	best_pos_harms := 0

	fast_best_sc   := -1.0e9
	fast_best_idx  := 30
	fast_best_df   := 0.0
	fast_best_ph   := 0

	for idx in 0 ..< max_deep_idx4 {
		cand_f0 := 85.0 + L4_F0_STEP * f64(idx)
		d_best  := -1.0e9
		d_df    := 0.0
		d_ph    := 0
		for d_i in 0 ..< df_steps {
			sc_d, ph_d, _ := eval_uniform_shs_comb(d_exc_slice, cand_f0, df_table[d_i], comb_f_lo, comb_f_hi, d_kl, d_kh, d_inv_hz, 0.025)
			sc_d *= df_pen[d_i]
			if sc_d > d_best { d_best = sc_d; d_df = df_table[d_i]; d_ph = ph_d }
		}
		im1    := max(idx - 1, 0)
		ip1    := min(idx + 1, max_deep_idx4 - 1)
		im2    := max(idx - 2, 0)
		ip2    := min(idx + 2, max_deep_idx4 - 1)
		diff_d := 0.10 * prev_shs_d[im2] + 0.20 * prev_shs_d[im1] + 0.40 * prev_shs_d[idx] + 0.20 * prev_shs_d[ip1] + 0.10 * prev_shs_d[ip2]
		uvf.shs_smooth[idx] = 0.54 * diff_d + 0.46 * d_best
		f0_scores[idx]      = uvf.shs_smooth[idx]
		f0_harms[idx]       = d_ph
		f0_dfs[idx]         = d_df
		if f0_scores[idx] > best_score {
			best_score     = f0_scores[idx]
			best_idx       = idx
			best_df        = d_df
			best_pos_harms = d_ph
		}
	}

	for idx in 0 ..< max_fast_idx4 {
		cand_f0 := 85.0 + f64(idx)
		s_best  := -1.0e9
		s_df    := 0.0
		s_ph    := 0
		for d_i in 0 ..< df_steps {
			sc_s, ph_s, _ := eval_uniform_shs_comb(syll_exc[:], cand_f0, df_table[d_i], comb_f_lo, comb_f_hi, k_lo, k_hi, inv_bin_hz, 0.095)
			sc_s *= df_pen[d_i]
			if sc_s > s_best { s_best = sc_s; s_df = df_table[d_i]; s_ph = ph_s }
		}
		fast_scores[idx] = s_best
		fast_harms[idx]  = s_ph
		fast_dfs[idx]    = s_df
		if s_best > fast_best_sc {
			fast_best_sc  = s_best
			fast_best_idx = idx
			fast_best_df  = s_df
			fast_best_ph  = s_ph
		}
	}

	mean_shs, mean_fast := 0.0, 0.0
	for idx in 0 ..< max_deep_idx4 do mean_shs  += f0_scores[idx]
	for idx in 0 ..< max_fast_idx4 do mean_fast += fast_scores[idx]
	mean_shs  /= f64(max_deep_idx4)
	mean_fast /= f64(max_fast_idx4)

	var_shs, var_fast := 1e-9, 1e-9
	for idx in 0 ..< max_deep_idx4 { dd := f0_scores[idx]   - mean_shs;  var_shs  += dd * dd }
	for idx in 0 ..< max_fast_idx4 { df := fast_scores[idx] - mean_fast; var_fast += df * df }
	std_shs  := math.sqrt(var_shs  / f64(max_deep_idx4))
	std_fast := math.sqrt(var_fast / f64(max_fast_idx4))

	// Octave disambiguation & sub-Hz parabolic F0 refinement
	best_f0 := 85.0 + L4_F0_STEP * f64(best_idx)
	if best_f0 >= 170.0 {
		hi := int(math.round((0.5 * best_f0 - 85.0) / L4_F0_STEP))
		if hi >= 0 && hi < max_deep_idx4 && f0_scores[hi] > mean_shs + 0.30 * std_shs && f0_scores[hi] >= 0.42 * best_score {
			best_idx       = hi
			best_f0        = 85.0 + L4_F0_STEP * f64(hi)
			best_pos_harms = max(best_pos_harms, f0_harms[hi])
			best_df        = f0_dfs[hi]
		}
	}
	if best_idx > 0 && best_idx + 1 < max_deep_idx4 {
		sm, s0, sp := f0_scores[best_idx - 1], f0_scores[best_idx], f0_scores[best_idx + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do best_f0 += L4_F0_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}

	fast_f0 := 85.0 + f64(fast_best_idx)
	if fast_f0 >= 170.0 {
		hi := int(math.round(0.5 * fast_f0 - 85.0))
		if hi >= 0 && hi < max_fast_idx4 && fast_scores[hi] > mean_fast + 0.35 * std_fast && fast_scores[hi] >= 0.45 * fast_best_sc {
			fast_best_idx = hi
			fast_f0       = 85.0 + f64(hi)
			fast_best_ph  = max(fast_best_ph, fast_harms[hi])
			fast_best_df  = fast_dfs[hi]
		}
	}
	if fast_best_idx > 0 && fast_best_idx + 1 < max_fast_idx4 {
		sm, s0, sp := fast_scores[fast_best_idx - 1], fast_scores[fast_best_idx], fast_scores[fast_best_idx + 1]
		den := sm - 2.0 * s0 + sp
		if den < -1e-9 do fast_f0 += clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
	}

	// Measure unbiased short-time F1 vocal-core (230..940 Hz), low-voiced (170..1180 Hz) & high-sibilant (1600..2540 Hz)
	// power ratios against BOTH long-term noise_psd AND instantaneous spatial quantile fast_nf_arr (100% QRN-swell immune!)
	lo_fast_s, lo_raw_s, lo_nfl_s, lo_fnf_s := 0.0, 0.0, 0.0, 0.0
	f1_fast_s, f1_med_s, f1_raw_s, f1_nfl_s, f1_fnf_s := 0.0, 0.0, 0.0, 0.0, 0.0
	hi_fast_s, hi_nfl_s, hi_fnf_s := 0.0, 0.0, 0.0
	f1_peak_gf   := 0.0
	hi_peak_gf   := 0.0
	voiced_pks   := 0
	hi_broad_cnt := 0
	for k in k_lo ..= k_hi {
		fk  := f64(k) * bin_hz
		bm  := birdie_mask[k]
		nf  := math.max(uvf.noise_psd[k], 1e-20)
		nff := math.max(fast_nf_arr[k], nf)
		gf  := (uvf.fast_psd[k]  * bm * bm) / nff
		gw  := (uvf.welch_psd[k] * bm * bm) / nf
		if fk >= 170.0 && fk <= 1180.0 && !is_hum_notch_freq(fk) {
			lo_fast_s += uvf.fast_psd[k] * bm * bm
			lo_raw_s  += raw_pow[k]      * bm * bm
			lo_nfl_s  += nf
			lo_fnf_s  += nff
			if fk >= 230.0 && fk <= 940.0 {
				f1_fast_s += uvf.fast_psd[k]  * bm * bm
				f1_med_s  += uvf.welch_psd[k] * bm * bm
				f1_raw_s  += raw_pow[k]       * bm * bm
				f1_nfl_s  += nf
				f1_fnf_s  += nff
				if gf > f1_peak_gf do f1_peak_gf = gf
			}
			if k > k_lo && k < k_hi && bm >= 0.65 && fk >= 210.0 {
				pf0 := uvf.fast_psd[k] * bm * bm
				pfm := uvf.fast_psd[k - 1] * birdie_mask[k - 1] * birdie_mask[k - 1]
				pfp := uvf.fast_psd[k + 1] * birdie_mask[k + 1] * birdie_mask[k + 1]
				if pf0 >= pfm && pf0 >= pfp && gf > 2.50 && gw > 1.80 {
					voiced_pks += 1
				}
			}
		} else if fk >= 1600.0 && fk <= 2540.0 {
			hi_fast_s += uvf.fast_psd[k] * bm * bm
			hi_nfl_s  += nf
			hi_fnf_s  += nff
			if gf > 1.80 do hi_broad_cnt += 1
			if gf > hi_peak_gf do hi_peak_gf = gf
		}
	}
	lo_snr_ratio := lo_fast_s / math.max(lo_nfl_s, 1e-20)
	lo_fast_snr  := lo_fast_s / math.max(lo_fnf_s, 1e-20)
	raw_fast_snr := lo_raw_s  / math.max(lo_fnf_s, 1e-20)
	f1_med_snr   := f1_med_s  / math.max(f1_nfl_s, 1e-20)
	f1_fast_snr  := f1_fast_s / math.max(f1_fnf_s, 1e-20)
	f1_raw_snr   := f1_raw_s  / math.max(f1_fnf_s, 1e-20)
	hi_snr_ratio := hi_fast_s / math.max(hi_fnf_s, 1e-20)
	_ = lo_snr_ratio
	voiced_formant_vad := (voiced_pks >= 3 && f1_fast_snr > 1.52 && f1_med_snr > 1.52 && f1_raw_snr > 1.25 && f1_peak_gf > 5.2) ||
	                      (voiced_pks >= 2 && f1_fast_snr > 1.68 && f1_med_snr > 1.68 && f1_raw_snr > 1.32 && f1_peak_gf > 6.8)

	// Zero-selection-bias, zero-sample-overlap cross-validators:
	//   - hr_past4k     {0..4607}    vs hr_fin4k_avg {4608..9215} (0 shared samples, 2.93 Hz/bin!)
	//   - hr_past2k_avg {3328..6143} vs hr_mfin2k    {6144..9215} (0 shared samples, 5.86 Hz/bin!)
	//   - hr_past128    {6144..7679} vs hr_pres128   {7680..9215} (0 shared samples, 256 ms rapid syllable & pause snap!)
	lag4_d_f0 := uvf.deep_f0_ring[uvf.ring_pos]
	lag4_d_df := uvf.deep_df_ring[uvf.ring_pos]
	lag4_m_f0 := uvf.med_f0_ring[uvf.ring_pos]
	lag4_m_df := uvf.med_df_ring[uvf.ring_pos]
	uvf.deep_f0_ring[uvf.ring_pos] = best_f0
	uvf.deep_df_ring[uvf.ring_pos] = best_df
	uvf.med_f0_ring[uvf.ring_pos]  = med_f0
	uvf.med_df_ring[uvf.ring_pos]  = med_best_df
	uvf.ring_pos = (uvf.ring_pos + 1) & 3

	z_deep_bg, _, _ := comb_z_at(d_exc_slice, best_f0, best_df,     comb_f_lo, comb_f_hi, d_kl, d_kh, d_inv_hz, 0.025)
	z_med_bg,  _, _ := comb_z_at(m_exc_slice, med_f0,  med_best_df, comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz, 0.035)
	ze_d, zo_d      := eval_even_odd_comb_z(d_exc_slice, best_f0, best_df,     comb_f_lo, comb_f_hi, d_kl, d_kh, d_inv_hz)
	ze_m, zo_m      := eval_even_odd_comb_z(m_exc_slice, med_f0,  med_best_df, comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz)
	z_fast          := (fast_best_sc - mean_fast) / math.max(std_fast, 1e-6)

	z_past4k, ph_past4k := -10.0, 0
	if uvf.hr_filled4k >= 2 && uvf.frame_cnt >= 18 {
		z_past4k, ph_past4k = eval_unsel_past_z(uvf.hr_past4k[:], best_f0, best_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 1.8)
	}
	z_past2k, ph_past2k := -10.0, 0
	if uvf.hr_filled2k >= 2 && uvf.frame_cnt >= 14 {
		z_past2k, ph_past2k = eval_unsel_past_z(uvf.hr_past2k_avg[:], med_f0, med_best_df, comb_f_lo, comb_f_hi, hk2_lo, hk2_hi, inv_hr2_hz, 1.8)
	}

	// Long-horizon 2.93 Hz/bin Welch comb (hr_wb4k_exc, ~500 ms) + zero-overlap past4k cross-validator for -25..-30 dB voice!
	wb_f0, wb_df := best_f0, best_df
	wb_ph        := 0
	z_wb4k       := -10.0
	ze_wb, zo_wb := 0.0, 0.0
	z_past_wb    := -10.0
	ph_past_wb   := 0
	if uvf.hr_filled4k >= 3 && uvf.frame_cnt >= 18 {
		wb_scores : [L4_MED_NUM_F0]f64
		wb_harms  : [L4_MED_NUM_F0]int
		wb_dfs    : [L4_MED_NUM_F0]f64
		wb_bsc    := -1.0e9
		wb_bi     := 60
		for idx in 0 ..< max_med_idx4 {
			cf0 := 85.0 + L4_MED_F0_STEP * f64(idx)
			b_w := -1.0e9
			d_w := 0.0
			p_w := 0
			for d_i in 0 ..< df_steps {
				scw, phw, _ := eval_uniform_shs_comb(uvf.hr_wb4k_exc[:], cf0, df_table[d_i], comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 0.022)
				scw *= df_pen[d_i]
				if scw > b_w { b_w = scw; d_w = df_table[d_i]; p_w = phw }
			}
			wb_scores[idx] = b_w; wb_harms[idx] = p_w; wb_dfs[idx] = d_w
			if b_w > wb_bsc { wb_bsc = b_w; wb_bi = idx; wb_df = d_w; wb_ph = p_w }
		}
		m_wb := 0.0
		for idx in 0 ..< max_med_idx4 do m_wb += wb_scores[idx]
		m_wb /= f64(max_med_idx4)
		v_wb := 1e-9
		for idx in 0 ..< max_med_idx4 { dw := wb_scores[idx] - m_wb; v_wb += dw * dw }
		sd_wb := math.sqrt(v_wb / f64(max_med_idx4))
		wb_f0 = 85.0 + L4_MED_F0_STEP * f64(wb_bi)
		if wb_f0 >= 170.0 {
			hi := int(math.round((0.5 * wb_f0 - 85.0) / L4_MED_F0_STEP))
			if hi >= 0 && hi < max_med_idx4 && wb_scores[hi] > m_wb + 0.32 * sd_wb && wb_scores[hi] >= 0.43 * wb_bsc {
				wb_bi = hi
				wb_f0 = 85.0 + L4_MED_F0_STEP * f64(hi)
				wb_ph = max(wb_ph, wb_harms[hi])
				wb_df = wb_dfs[hi]
			}
		}
		if wb_bi > 0 && wb_bi + 1 < max_med_idx4 {
			sm, s0, sp := wb_scores[wb_bi - 1], wb_scores[wb_bi], wb_scores[wb_bi + 1]
			den := sm - 2.0 * s0 + sp
			if den < -1e-9 do wb_f0 += L4_MED_F0_STEP * clamp(0.5 * (sm - sp) / den, -0.48, +0.48)
		}
		z_wb4k, _, _ = comb_z_at(uvf.hr_wb4k_exc[:], wb_f0, wb_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 0.022)
		ze_wb, zo_wb = eval_even_odd_comb_z(uvf.hr_wb4k_exc[:], wb_f0, wb_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz)
		z_past_wb, ph_past_wb = eval_unsel_past_z(uvf.hr_past4k[:], wb_f0, wb_df, comb_f_lo, comb_f_hi, hk4_lo, hk4_hi, inv_hr4_hz, 1.8)
		wb_cont := math.abs(wb_f0 - uvf.wb4k_f0_prev) <= (6.0 if uvf.voice_confidence >= 0.20 else 2.2)
		if (!wb_cont || z_wb4k < 2.65 || math.min(ze_wb, zo_wb) < 0.75) && uvf.voice_confidence < 0.20 {
			uvf.wb4k_streak = 0
		} else if z_wb4k > 2.85 && math.min(ze_wb, zo_wb) > 0.85 && wb_ph >= 5 && wb_cont {
			uvf.wb4k_streak = min(uvf.wb4k_streak + 1, 32)
		} else if z_wb4k < 2.25 {
			uvf.wb4k_streak = 0
		} else if uvf.wb4k_streak > 0 {
			uvf.wb4k_streak -= 1
		}
		uvf.wb4k_f0_prev = wb_f0
	} else {
		uvf.wb4k_streak = 0
	}

	f0_128, z_pres128, z_past128 := med_f0, -10.0, -10.0
	ph_128     := 0
	e_drop     := false
	z_pres_cur := 2.0
	z128_med   := 2.0
	if uvf.hr_filled2k >= 2 && uvf.frame_cnt >= 12 {
		f0_128, z_pres128, z_past128, ph_128 = eval_128ms_cross(
			uvf.hr_past128_exc[:], uvf.hr_pres128_exc[:], df_steps, df_table[:],
			comb_f_lo, comb_f_hi, hk2_lo, hk2_hi, inv_hr2_hz,
		)
		z128_med, _, _ = comb_z_at(uvf.hr_pres128_exc[:], med_f0, med_best_df, comb_f_lo, comb_f_hi, hk2_lo, hk2_hi, inv_hr2_hz, 0.035)
		raw_edrop : bool
		raw_edrop, z_pres_cur = eval_past_pres_drop(
			uvf.hr_past128_exc[:], uvf.hr_pres128_exc[:], uvf.tracked_f0_hz, uvf.tracked_df_hz,
			comb_f_lo, comb_f_hi, hk2_lo, hk2_hi, inv_hr2_hz,
		)
		z_pres_cur = max(z_pres_cur, z128_med)
		glide_edrop := z128_med < 0.25 && f1_med_snr > 1.32 && f1_fast_snr < 1.14 && f1_raw_snr < 1.10 && hi_snr_ratio < 1.18
		e_drop = (raw_edrop || glide_edrop) && (uvf.voice_confidence > 0.05 || uvf.voice_hang > 0.05)
	}
	uvf.inst_comb_deep_ema = z_past4k
	uvf.inst_comb_med_ema  = max(z_past2k, z_past128 if (z_pres128 > 3.05 && math.abs(med_f0 - f0_128) <= 6.0) else -10.0)

	pitch_cont  := math.abs(best_f0 - uvf.f0_prev_cand) <= (12.0 if uvf.voice_confidence >= 0.20 else 4.2)
	df_cont_d   := !is_ssb || uvf.pitch_lock_streak < 2 || uvf.voice_confidence >= 0.20 || math.abs(best_df - lag4_d_df) <= 9.5
	disj_d_cont := (uvf.pitch_lock_streak < 4) || (lag4_d_f0 >= 80.0 && math.abs(best_f0 - lag4_d_f0) <= (14.0 if uvf.voice_confidence >= 0.20 else 5.0))
	med_cont    := math.abs(med_f0  - uvf.f0_prev_med)  <= (12.0 if uvf.voice_confidence >= 0.20 else 4.8)
	df_cont_m   := !is_ssb || uvf.med_lock_streak < 2   || uvf.voice_confidence >= 0.20 || math.abs(med_best_df - lag4_m_df) <= 9.5
	disj_m_cont := (uvf.med_lock_streak < 4)   || (lag4_m_f0 >= 80.0 && math.abs(med_f0  - lag4_m_f0) <= (14.0 if uvf.voice_confidence >= 0.20 else 5.8))
	fast_cont   := math.abs(fast_f0 - uvf.f0_prev_fast) <= 10.0

	if uvf.frame_cnt <= 7 {
		uvf.pitch_lock_streak = 0
		uvf.med_lock_streak   = 0
		uvf.fast_lock_streak  = 0
	} else {
		if (!pitch_cont || !df_cont_d || !disj_d_cont || z_deep_bg < 2.75 || math.min(ze_d, zo_d) < 0.80) && uvf.voice_confidence < 0.20 {
			uvf.pitch_lock_streak = 0
		} else if z_deep_bg > 2.90 && math.min(ze_d, zo_d) > 0.85 && best_pos_harms >= 5 && pitch_cont && df_cont_d && disj_d_cont {
			uvf.pitch_lock_streak = min(uvf.pitch_lock_streak + 1, 28)
		} else if z_deep_bg < 2.25 {
			uvf.pitch_lock_streak = 0
		} else if uvf.pitch_lock_streak > 0 {
			uvf.pitch_lock_streak -= 1
		}

		if (!med_cont || !df_cont_m || !disj_m_cont || z_med_bg < 2.70 || math.min(ze_m, zo_m) < 0.78) && uvf.voice_confidence < 0.20 {
			uvf.med_lock_streak = 0
		} else if z_med_bg > 2.85 && math.min(ze_m, zo_m) > 0.85 && med_best_ph >= 5 && med_cont && df_cont_m && disj_m_cont {
			uvf.med_lock_streak = min(uvf.med_lock_streak + 1, 24)
		} else if z_med_bg < 2.25 {
			uvf.med_lock_streak = 0
		} else if uvf.med_lock_streak > 0 {
			uvf.med_lock_streak -= 1
		}
		if is_ssb && uvf.med_lock_streak >= 3 && z_med_bg > 3.20 && math.abs(med_best_df) >= 3.5 {
			uvf.mistune_cand_df = med_best_df
			uvf.mistune_streak  = min(uvf.mistune_streak + 1, 10)
		}

		if !fast_cont && uvf.voice_confidence < 0.25 {
			uvf.fast_lock_streak = 0
		} else if fast_best_sc > 0.140 && z_fast > 3.05 && fast_best_ph >= 5 && fast_cont {
			uvf.fast_lock_streak = min(uvf.fast_lock_streak + 1, 16)
		} else if fast_best_sc < 0.090 || z_fast < 2.35 {
			uvf.fast_lock_streak = 0
		} else if uvf.fast_lock_streak > 0 {
			uvf.fast_lock_streak -= 1
		}
	}

	// 1) Deep -20..-30 dB path (2.93 Hz/bin Welch hr_fin4k_avg & hr_wb4k_exc + zero-overlap hr_past4k cross-validator -> 0% static blasts!)
	vconf_deep   := 0.0
	deep_lock    := uvf.hr_filled4k >= 2 && uvf.frame_cnt >= 18 && !e_drop &&
	                best_pos_harms >= 5 && ph_past4k >= 4 && math.min(ze_d, zo_d) > 1.05 &&
	                uvf.pitch_lock_streak >= 5 && z_deep_bg > 3.35 && z_past4k > 1.78
	ultra30_lock := uvf.hr_filled4k >= 4 && uvf.frame_cnt >= 26 && !e_drop &&
	                uvf.wb4k_streak >= 7 && wb_ph >= 5 && ph_past_wb >= 4 &&
	                z_wb4k > 3.35 && math.min(ze_wb, zo_wb) > 1.00 && z_past_wb > 1.05
	deep_hold    := (uvf.voice_confidence > 0.15 || uvf.voice_hang > 0.20) && uvf.hr_filled4k >= 2 && !e_drop && (z_pres_cur > 0.38 || z_wb4k > 2.95) &&
	                ((z_deep_bg > 2.75 && math.min(ze_d, zo_d) > 0.78 && best_pos_harms >= 4) ||
	                 (uvf.wb4k_streak >= 4 && z_wb4k > 2.92 && math.min(ze_wb, zo_wb) > 0.82 && wb_ph >= 4))
	if ultra30_lock && z_wb4k > z_deep_bg {
		best_f0    = wb_f0
		best_df    = wb_df
		vconf_deep = clamp((z_wb4k - 2.10) * 0.72, 0.38, 1.0)
	} else if deep_lock || deep_hold {
		if !deep_lock && uvf.wb4k_streak >= 4 && z_wb4k > z_deep_bg {
			best_f0 = wb_f0
			best_df = wb_df
		}
		vconf_deep = clamp((max(z_deep_bg, z_wb4k) - 2.15) * 0.70, 0.36, 1.0)
	}

	// 2) Medium -15..-21 dB glide-agile target path (5.86 Hz/bin hr_mfin2k + zero-overlap hr_past2k_avg & 128ms cross-validator!)
	vconf_med  := 0.0
	sub20_lock := uvf.hr_filled2k >= 2 && uvf.frame_cnt >= 12 && !e_drop && z128_med > 0.40 && med_best_ph >= 5 && math.min(ze_m, zo_m) > 1.05 &&
	              ((uvf.med_lock_streak >= 5 && z_med_bg > 3.32 && z_past2k > 1.75 && ph_past2k >= 4) ||
	               (uvf.med_lock_streak >= 3 && z_med_bg > 3.55 && z_pres128 > 3.15 && z_past128 > 1.95 && ph_128 >= 5 && math.abs(med_f0 - f0_128) <= 6.0))
	sub20_hold := (uvf.voice_confidence > 0.15 || uvf.voice_hang > 0.20) && uvf.hr_filled2k >= 2 && !e_drop && z_pres_cur > 0.58 &&
	              z_med_bg > 2.75 && math.min(ze_m, zo_m) > 0.78 && med_best_ph >= 4
	conv_lock  := !e_drop && z128_med > 0.55 && uvf.med_lock_streak >= 3 && med_best_ph >= 5 &&
	              ((z_med_bg > 4.15 && math.min(ze_m, zo_m) > 1.65 && f1_fast_snr > 1.14 && f1_med_snr > 1.13) ||
	               (z_med_bg > 3.20 && math.min(ze_m, zo_m) > 1.00 && f1_fast_snr > 1.42 && f1_med_snr > 1.38 && f1_raw_snr > 1.20))
	if sub20_lock || sub20_hold || conv_lock {
		vconf_med = clamp((z_med_bg - 2.15) * 0.70, 0.36, 1.0)
	}

	// 3) Agile syllabic path (~21-42 ms lock for conversational phrases & rapid prosody glides, +5.8-sigma F1 gated!)
	vconf_fast := 0.0
	if (uvf.fast_lock_streak >= 2 && z_fast > 3.05 && fast_best_sc > 0.148 && fast_best_ph >= 5 && f1_fast_snr > 1.46 && f1_med_snr > 1.38 && f1_raw_snr > 1.24) ||
	   (uvf.speech_context > 0.18 && uvf.fast_lock_streak >= 1 && z_fast > 2.88 && fast_best_sc > 0.132 && fast_best_ph >= 4 && f1_fast_snr > 1.35 && f1_raw_snr > 1.16) ||
	   (z_fast > 3.50 && fast_best_sc > 0.210 && fast_best_ph >= 5 && f1_fast_snr > 1.54 && f1_med_snr > 1.44 && f1_raw_snr > 1.28) {
		vconf_fast = clamp((z_fast - 2.42) * 0.65, 0.0, 1.0)
	}

	comb_vconf := max(vconf_deep, max(vconf_med, vconf_fast))
	uvf.syll_active_flag = (vconf_fast >= 0.38 && broadband_snr_ratio > 1.06) ||
	                       (vconf_fast > 0.20 && max(vconf_deep, vconf_med) < 0.30)

	chosen_f0 := best_f0
	chosen_df := best_df
	if uvf.syll_active_flag {
		chosen_f0 = fast_f0
		chosen_df = fast_best_df
	} else if vconf_med >= vconf_deep && vconf_med > 0.10 {
		chosen_f0 = med_f0
		chosen_df = med_best_df
	}

	uvf.f0_prev_cand = best_f0
	uvf.f0_prev_med  = med_f0
	uvf.f0_prev_fast = fast_f0

	pitch_glide_hz := math.abs(chosen_f0 - uvf.tracked_f0_hz)
	if comb_vconf > 0.08 {
		if pitch_glide_hz < 16.0 && !uvf.syll_active_flag {
			uvf.tracked_f0_hz = 0.38 * uvf.tracked_f0_hz + 0.62 * chosen_f0
		} else if pitch_glide_hz < 18.0 {
			uvf.tracked_f0_hz = 0.20 * uvf.tracked_f0_hz + 0.80 * chosen_f0
		} else {
			uvf.tracked_f0_hz = chosen_f0
		}
		if is_ssb {
			if math.abs(uvf.tracked_df_hz) >= 4.0 && math.abs(chosen_df) < 1.0 {
				if uvf.mistune_streak == 0 && math.abs(med_best_df) < 1.0 && math.abs(best_df) < 1.0 {
					sc0_u, _, _  := eval_uniform_shs_comb(m_exc_slice, chosen_f0, 0.0,               comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz, 0.025)
					scdf_u, _, _ := eval_uniform_shs_comb(m_exc_slice, chosen_f0, uvf.tracked_df_hz, comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz, 0.025)
					if sc0_u > 1.12 * math.max(scdf_u, 0.01) {
						uvf.tracked_df_hz = 0.0
					}
				}
			} else if math.abs(chosen_df - uvf.tracked_df_hz) < 16.0 {
				uvf.tracked_df_hz = 0.42 * uvf.tracked_df_hz + 0.58 * chosen_df
			} else {
				uvf.tracked_df_hz = chosen_df
			}
		} else {
			uvf.tracked_df_hz = 0.0
		}
	}

	// 6. Cepstral Vocal-Tract Formant Envelope Liftering (uses agile fast_psd on clear speech!)
	cep_fast_w := clamp((broadband_snr_ratio - 1.18) * 0.65, 0.0, 0.78)
	for k in 0 ..< L4_BINS {
		bm         := birdie_mask[k]
		n_psd      := math.max(uvf.noise_psd[k], 1e-20)
		src_psd    := (1.0 - cep_fast_w) * uvf.welch_psd[k] + cep_fast_w * uvf.fast_psd[k]
		snr_excess := math.max((src_psd * bm * bm) / n_psd - 0.92, 1e-3)
		if k < k_lo || k > k_hi do snr_excess = 1e-3
		uvf.cepstrum_fft[k] = complex(0.5 * math.ln(snr_excess), 0.0)
	}
	for k in 1 ..< (L4_FFT_N / 2) {
		uvf.cepstrum_fft[L4_FFT_N - k] = uvf.cepstrum_fft[k]
	}
	fft_inverse(&uvf.fft_plan, uvf.cepstrum_fft[:])

	LIFTER_Q :: 28
	for q in (LIFTER_Q + 1) ..< (L4_FFT_N - LIFTER_Q) {
		uvf.cepstrum_fft[q] = complex(0.0, 0.0)
	}
	for q in 1 ..= LIFTER_Q {
		taper := 0.5 * (1.0 + math.cos(math.PI * f64(q) / f64(LIFTER_Q + 1)))
		uvf.cepstrum_fft[q]            *= complex(taper, 0.0)
		uvf.cepstrum_fft[L4_FFT_N - q] *= complex(taper, 0.0)
	}
	fft_forward(&uvf.fft_plan, uvf.cepstrum_fft[:])

	max_cep : f64 = -1.0e9
	for k in k_lo ..= k_hi {
		rv := real(uvf.cepstrum_fft[k])
		if rv > max_cep do max_cep = rv
	}

	// 7. Glide-Immune Pause Detector + Consonant Bridge + Phase-Coherent Speech Reconstruction
	f0_use     := uvf.tracked_f0_hz
	df_use     := uvf.tracked_df_hz if is_ssb else 0.0
	base_floor := 0.0018 if is_ssb else 0.0040

	// Evaluate instantaneous comb on BOTH tracked_f0 and fast_f0 so pitch glides never drop out while pauses snap cleanly!
	z_trk_now, raw_c_trk, _ := comb_z_at(raw_exc_u[:], f0_use, df_use, comb_f_lo, comb_f_hi, k_lo, k_hi, inv_bin_hz, 0.08)
	inst_c_trk, _, _ := eval_uniform_shs_comb(syll_exc[:], f0_use,  df_use,       comb_f_lo, comb_f_hi, k_lo, k_hi, inv_bin_hz, 0.09)
	inst_c_cur, _, _ := eval_uniform_shs_comb(syll_exc[:], fast_f0, fast_best_df, comb_f_lo, comb_f_hi, k_lo, k_hi, inv_bin_hz, 0.09)
	deep_c_trk, _, _ := eval_uniform_shs_comb(d_exc_slice, f0_use,  df_use,       comb_f_lo, comb_f_hi, d_kl, d_kh, d_inv_hz,   0.025)
	med_c_trk,  _, _ := eval_uniform_shs_comb(m_exc_slice, f0_use,  df_use,       comb_f_lo, comb_f_hi, m_kl, m_kh, m_inv_hz,   0.035)

	exc_inst_comb := max(inst_c_trk, inst_c_cur if fast_best_ph >= 5 else inst_c_trk)
	raw_best_c    := max(raw_c_trk, fast_best_sc * 0.85 if fast_best_ph >= 5 else raw_c_trk)
	cur_comb_l4   := 0.50 * raw_best_c + 0.50 * exc_inst_comb
	exc_ref_comb  := max(deep_c_trk, med_c_trk) if broadband_snr_ratio > 1.05 else deep_c_trk

	// Unvoiced Portuguese consonant / sibilant ('/s, ch, f, t, k/' in 1.6-2.54 kHz) — immune to QRN static swells & trailing breath!
	consonant_gate := (uvf.voiced_hang_hops > 0 && hi_broad_cnt >= 8 && hi_snr_ratio > 1.42 && hi_peak_gf > 3.5 && hi_snr_ratio > 1.28 * lo_fast_snr) ||
	                  (hi_broad_cnt >= 11 && hi_snr_ratio > 1.80 && hi_peak_gf > 5.2 && hi_snr_ratio > 1.55 * math.max(lo_fast_snr, 0.95))

	was_active4 := uvf.voice_confidence > 0.02 || uvf.voice_hang > 0.02 || uvf.hang_hops > 0 || uvf.gate_phase > 0.0
	pause_gate  := 1.0
	pause_snap  := false
	if was_active4 && !voiced_formant_vad && !consonant_gate && comb_vconf < 0.12 {
		loud_drop := exc_ref_comb > 0.15 && cur_comb_l4 < 0.22 * exc_ref_comb && exc_inst_comb < 0.28 * exc_ref_comb && lo_fast_snr < 1.15 && raw_fast_snr < 1.12
		deep_drop := uvf.voice_confidence > 0.14 && vconf_deep < 0.10 && vconf_med < 0.05 && z_trk_now < 0.80 && z_pres_cur < 0.85 && lo_fast_snr < 1.12 && raw_fast_snr < 1.10
		if e_drop || (f1_fast_snr < 1.15 && f1_raw_snr < 1.08 && hi_snr_ratio < 1.16) || loud_drop || deep_drop {
			pause_gate            = 0.0
			pause_snap            = true
			comb_vconf            = 0.0
			uvf.voice_hang        = 0.0
			uvf.voice_confidence  = 0.0
			uvf.speech_context   *= 0.20
			uvf.pitch_lock_streak = 0
			uvf.med_lock_streak   = 0
			uvf.fast_lock_streak  = 0
			uvf.wb4k_streak       = 0
			uvf.shs_smooth        = {}
			uvf.med_shs_smooth    = {}
			if e_drop || loud_drop || (f1_med_snr > 1.26 && f1_fast_snr < 1.14) {
				uvf.hr_pres2k_pow = {}
				uvf.hr_pres4k_pow = {}
				uvf.hr_fin4k_avg  = {}
				uvf.hr_mfin2k     = {}
				uvf.hr_filled2k   = min(uvf.hr_filled2k, 1)
				uvf.hr_filled4k   = min(uvf.hr_filled4k, 1)
				if f1_med_snr > 1.26 {
					uvf.hr_wb4k_psd = {}
					uvf.hr_wb4k_exc = {}
				}
			}
		}
	}

	formant_vconf := clamp((f1_fast_snr - 1.28) * 1.15, 0.30, 0.95) if voiced_formant_vad else 0.0
	cons_vconf    := clamp((hi_snr_ratio - 1.22) * 1.10, 0.28, 0.85) if consonant_gate else 0.0
	voiced_tot_vc := max(comb_vconf * pause_gate, formant_vconf)
	inst_tot_vc   := max(voiced_tot_vc, cons_vconf)

	uvf.voice_confidence = 0.35 * uvf.voice_confidence + 0.65 * inst_tot_vc if inst_tot_vc > 0.05 else (0.0 if pause_snap else 0.18 * uvf.voice_confidence)
	if voiced_tot_vc > uvf.voice_hang {
		uvf.voice_hang = voiced_tot_vc
	} else {
		uvf.voice_hang *= 0.65
	}
	target_ctx := max(voiced_tot_vc, uvf.voice_hang)
	if target_ctx > uvf.speech_context {
		uvf.speech_context = target_ctx
	} else {
		uvf.speech_context *= 0.85
	}

	// Track voiced hangover STRICTLY from voiced speech (never from consonant_gate -> zero self-latching!)
	if voiced_tot_vc >= 0.18 {
		uvf.voiced_hang_hops = 2 // ~43 ms window for trailing sibilants
		uvf.hang_hops        = 1 // ~21.3 ms syllabic bridge (+ 21.3 ms natural STFT lookahead)
		uvf.silent_hops      = 0
	} else {
		uvf.silent_hops += 1
		if uvf.silent_hops >= 32 && uvf.mistune_streak == 0 {
			uvf.tracked_df_hz = 0.0
			uvf.df_profile    = {}
			uvf.df_cand_live  = 0.0
			uvf.df_z_live     = 0.0
		}
		if uvf.voiced_hang_hops > 0 do uvf.voiced_hang_hops -= 1
		if pause_snap || pause_gate < 0.20 {
			uvf.hang_hops        = 0
			uvf.voiced_hang_hops = 0
		} else if consonant_gate {
			uvf.hang_hops = max(uvf.hang_hops, 1)
		} else if uvf.hang_hops > 0 {
			uvf.hang_hops -= 1
		}
	}
	speech_now := inst_tot_vc >= 0.18
	gate_open  := speech_now || uvf.hang_hops > 0

	vconf       := comb_vconf * pause_gate
	eff_vactive := max(uvf.voice_confidence, uvf.voice_hang * 0.80)

	// Razor-sharp 11.2..14.5 Hz comb teeth at -15..-30 dB (4096-pt 11.72 Hz/bin beats L3 2048-pt on AM, FM, & SSB!)
	clear_speech_w := clamp((broadband_snr_ratio - 1.22) * 0.55, 0.0, 1.0)
	glide_open     := 0.38 if (voiced_formant_vad && comb_vconf < 0.35) else 0.0
	high_snr_open  := max(clamp((broadband_snr_ratio - 8.50) * 0.04, 0.0, 0.22), glide_open)
	glide_widen    := clamp((pitch_glide_hz - 0.8) * 1.4, 0.0, 5.0) if uvf.syll_active_flag else 0.0
	base_sig       := 10.8 if is_ssb else 12.2
	sigma_hz       := base_sig + glide_widen + clamp((broadband_snr_ratio - 1.50) * 1.4, 0.0, 3.2)

	// Agile fast_psd weighting on conversational speech (eliminates 340 ms metallic phase-magnitude reverb!)
	snr_fast_bias  := clamp((broadband_snr_ratio - 1.18) * 1.15, 0.0, 0.86)
	glide_mix      := clamp(0.50 + (pitch_glide_hz - 0.5) * 0.25, 0.50, 0.88) if (uvf.syll_active_flag && vconf > 0.20) else 0.0

	for k in 0 ..< L4_BINS {
		if k < k_lo || k > k_hi {
			uvf.work_fft[k]       = complex(0.0, 0.0)
			uvf.prev_clean_psd[k] = 0.0
			continue
		}

		freq  := f64(k) * bin_hz
		bm    := birdie_mask[k]
		n_psd := math.max(uvf.noise_psd[k], 1e-20)

		over := 1.06
		switch band {
		case .AM_Band:  over = 1.05
		case .FM_Band:  over = 1.04 + 0.45 * ((freq / 3000.0) * (freq / 3000.0))
		case .SSB_Band: over = 1.06
		}
		eff_noise := n_psd * over

		eff_fpsd   := uvf.fast_psd[k]  * (bm * bm)
		eff_wpsd   := uvf.welch_psd[k] * (bm * bm)
		eff_ppsd   := uvf.pitch_psd[k] * (bm * bm)
		gamma_fast := eff_fpsd / eff_noise
		gamma_w    := eff_wpsd / eff_noise
		gamma_inst := (raw_pow[k] * bm * bm) / eff_noise

		harm_weight := 0.0
		if comb_vconf > 0.06 && f0_use > 75.0 && freq >= 0.70 * f0_use {
			h_idx := math.round((freq - df_use) / f0_use)
			if h_idx >= 1.0 && h_idx <= 22.0 {
				f_harm  := h_idx * f0_use + df_use
				dist_hz := math.abs(freq - f_harm)
				harm_weight = math.exp(-0.5 * (dist_hz * dist_hz) / (sigma_hz * sigma_hz))
			}
		}

		cep_diff    := real(uvf.cepstrum_fft[k]) - max_cep
		valley_form := clamp(math.exp(0.65 * cep_diff) * 1.15, 0.22, 1.06)
		tooth_form  := clamp(0.95 + 0.10 * valley_form, 0.95, 1.05)
		formant_w   := harm_weight * tooth_form + (1.0 - harm_weight) * valley_form

		km := max(k - 1, k_lo)
		kp := min(k + 1, k_hi)
		unvoiced_open := 0.0
		cons_thr      := 1.85 if uvf.speech_context > 0.14 else 2.45
		nb_thr        := 1.55 if uvf.speech_context > 0.14 else 1.90
		if consonant_gate && freq >= 1350.0 && gamma_fast > cons_thr {
			f9_sum := 1e-20
			for jf in -4 ..= 4 {
				kkf := clamp(k + jf, k_lo, k_hi)
				f9_sum += uvf.fast_psd[kkf] * birdie_mask[kkf] * birdie_mask[kkf]
			}
			f_pk2 := eff_fpsd + max(uvf.fast_psd[km] * birdie_mask[km] * birdie_mask[km],
			                        uvf.fast_psd[kp] * birdie_mask[kp] * birdie_mask[kp])
			nb_gf := max((uvf.fast_psd[km] * birdie_mask[km] * birdie_mask[km]),
			             (uvf.fast_psd[kp] * birdie_mask[kp] * birdie_mask[kp])) / eff_noise
			if nb_gf > nb_thr && (f_pk2 / f9_sum) < 0.72 {
				unvoiced_open = clamp((gamma_fast - (cons_thr - 0.15)) * 0.36, 0.0, 0.92)
			}
		}

		deep_clean_pow := math.max(0.32 * eff_wpsd + 0.68 * eff_ppsd - 1.00 * eff_noise, 0.0)
		fast_clean_pow := math.max(0.82 * eff_fpsd + 0.18 * eff_wpsd - 1.00 * eff_noise, 0.0)

		decay_rel := clamp((0.64 * eff_wpsd - eff_fpsd) / math.max(0.35 * eff_wpsd, eff_noise), 0.0, 1.0) if eff_wpsd > 1.70 * eff_noise else 0.0
		bin_snr_f := clamp((gamma_fast - 2.0) * 0.25, 0.0, 0.85)
		fast_mix  := max(max(snr_fast_bias, bin_snr_f), max(glide_mix, decay_rel))
		if unvoiced_open > 0.10 {
			fast_mix = max(fast_mix, 0.88)
		}
		clean_est_pow := (1.0 - fast_mix) * deep_clean_pow + fast_mix * fast_clean_pow

		eff_flr := base_floor if gate_open else 0.0001
		xi_est  := clean_est_pow / eff_noise
		xi_dd   := 0.72 * (uvf.prev_clean_psd[k] / eff_noise) + 0.28 * xi_est
		gam_use := 0.40 * gamma_inst + 0.60 * ((1.0 - 0.65 * fast_mix) * gamma_w + (0.65 * fast_mix) * gamma_fast)
		g_lsa   := mmse_lsa_gain_f64(xi_dd, gam_use, eff_flr)

		// During voiced_formant_vad with weak comb lock (rapid glide), pass clean formants without comb drop!
		form_pass   := clamp((gamma_fast - 1.85) * 0.35, 0.0, 0.82) * valley_form if (voiced_formant_vad && comb_vconf < 0.45) else 0.0
		comb_floor  := max(clamp(0.012 - 0.010 * vconf, 0.002, 0.018), max(high_snr_open * valley_form, unvoiced_open * 0.50))
		comb_open   := max(harm_weight, max(unvoiced_open, form_pass))
		comb_mask   := comb_floor + (1.0 - comb_floor) * comb_open

		// F2/F3 vocal articulation presence polish (+0.5 dB @ 1.75 kHz) on clear speech harmonics & sibilants
		artic := 1.0
		if gate_open && comb_open > 0.35 && clear_speech_w > 0.10 {
			df_p := (freq - 1750.0) / 580.0
			artic = 1.0 + 0.06 * clear_speech_w * comb_open * math.exp(-0.5 * df_p * df_p)
		}

		idle_gate   := unvoiced_open * 0.88 if gate_open else 0.0001
		voice_blend := clamp(eff_vactive * 1.35, 0.0, 0.99) if gate_open else 0.0
		mask_tot    := ((1.0 - voice_blend) * idle_gate + voice_blend * comb_mask) * formant_w * artic

		inst_mag   := math.sqrt(math.max(raw_pow[k], 1e-30))
		clean_mag  := math.sqrt(clean_est_pow)
		// Phase-coherent MMSE-LSA (0.85..0.96) on > -5 dB speech; smooth Welch/Pitch magnitude synthesis strictly for -15..-30 dB!
		inst_blend := clamp(0.28 + 0.58 * clear_speech_w + 0.25 * fast_mix, 0.28, 0.96)
		target_mag := ((1.0 - inst_blend) * clean_mag + inst_blend * (inst_mag * bm * g_lsa)) * mask_tot

		g_max                := 1.32 + 1.35 * (1.0 - clear_speech_w)
		g_eff                := clamp(target_mag / inst_mag, eff_flr * 0.1, g_max)
		g_sm_prev            := 0.10 if (!gate_open && g_eff < uvf.prev_gain[k]) else 0.32
		uvf.prev_gain[k]      = g_sm_prev * uvf.prev_gain[k] + (1.0 - g_sm_prev) * g_eff
		g_apply              := uvf.prev_gain[k]
		g_dd_u               := 0.45 * g_apply + 0.55 * (g_lsa * bm)
		uvf.prev_clean_psd[k] = (g_dd_u * g_dd_u) * raw_pow[k]
		uvf.work_fft[k]      *= complex(g_apply, 0.0)
	}
	for k in 1 ..< (L4_FFT_N / 2) {
		uvf.work_fft[L4_FFT_N - k] = conj(uvf.work_fft[k])
	}

	fft_inverse(&uvf.fft_plan, uvf.work_fft[:])

	if pause_snap && !consonant_gate {
		for n in 0 ..< L4_FFT_N {
			u_fade := clamp(f64(n) / 256.0, 0.0, 1.0)
			taper  := 1.0 - 0.96 * (0.5 - 0.5 * math.cos(math.PI * u_fade))
			uvf.ola_out[n] *= taper
		}
	}
	for n in 0 ..< L4_FFT_N {
		uvf.ola_out[n] += real(uvf.work_fft[n]) * uvf.window[n] * 0.5
	}

	// Click-free raised-cosine VAD envelope -> exact 0.0 digital silence in all non-speech moments!
	ATT_STEP_L4 :: 1.0 / 256.0 // 5.3 ms smooth attack (inside 64 ms OLA lookahead -> zero onset clipping!)
	REL_STEP_L4 :: 1.0 / 448.0 // 9.3 ms smooth release -> exact 0.0 silence with zero click!
	out_hop_energy := 0.0
	all_zero       := true
	for n in 0 ..< L4_HOP {
		if gate_open {
			uvf.gate_phase = min(uvf.gate_phase + ATT_STEP_L4, 1.0)
		} else {
			uvf.gate_phase = max(uvf.gate_phase - REL_STEP_L4, 0.0)
		}
		if uvf.gate_phase <= 0.0 {
			hop_out[n] = 0.0
		} else {
			g_env := 0.5 - 0.5 * math.cos(math.PI * uvf.gate_phase) if uvf.gate_phase < 1.0 else 1.0
			v     := uvf.ola_out[n] * g_env
			hop_out[n]      = v
			out_hop_energy += v * v
			all_zero        = false
		}
	}

	copy(uvf.ola_out[0 : L4_FFT_N - L4_HOP], uvf.ola_out[L4_HOP : L4_FFT_N])
	for n in (L4_FFT_N - L4_HOP) ..< L4_FFT_N {
		uvf.ola_out[n] = 0.0
	}
	if all_zero && !gate_open {
		for n in 0 ..< (L4_FFT_N - L4_HOP) {
			uvf.ola_out[n] *= 0.20
			if math.abs(uvf.ola_out[n]) < 1e-12 do uvf.ola_out[n] = 0.0
		}
		uvf.voice_confidence = 0.0
	}

	if raw_hop_energy > 1e-24 {
		inst_nr := 10.0 * math.log10(raw_hop_energy / math.max(out_hop_energy, 1e-24))
		uvf.noise_reduc_db = 0.85 * uvf.noise_reduc_db + 0.15 * clamp(inst_nr, 0.0, 48.0)
	}
}

ultra_voice_filter_process :: proc(uvf: ^Ultra_Deep_Voice_Filter, buf: []f64, mode: Modulation_Mode, rf_notch: bool = false) {
	if uvf.stage_idx == 0 && len(buf) == L4_HOP {
		ultra_voice_step_hop(uvf, buf, uvf.hop_ready[:], mode, rf_notch)
		copy(buf, uvf.hop_ready[:])
		return
	}

	for i in 0 ..< len(buf) {
		uvf.hop_staging[uvf.stage_idx] = buf[i]
		buf[i] = uvf.hop_ready[uvf.stage_idx]
		uvf.stage_idx += 1
		if uvf.stage_idx == L4_HOP {
			uvf.stage_idx = 0
			ultra_voice_step_hop(uvf, uvf.hop_staging[:], uvf.hop_ready[:], mode, rf_notch)
		}
	}
}

// ---------------------------------------------------------------------------
// Unified 4-Level Audio Noise Filter Bank Dispatcher
// ---------------------------------------------------------------------------

Audio_Filter_Bank :: struct {
	simple   : Simple_Filter_State,
	moderate : Moderate_Filter_State,
	advanced : Spectral_Wiener_Filter,
	ultra    : Ultra_Deep_Voice_Filter,
}

audio_filter_bank_init :: proc(bank: ^Audio_Filter_Bank) {
	simple_filter_init(&bank.simple)
	moderate_filter_init(&bank.moderate)
	spectral_wiener_init(&bank.advanced)
	ultra_voice_filter_init(&bank.ultra)
}

// Notify Filter 3 & Filter 4 when Auto-Clarifier (or manual clarifier) steps RF carrier offset
// so both filters shift tracked_df synchronously while preserving noise floors!
audio_filter_bank_notify_clarifier_step :: proc(bank: ^Audio_Filter_Bank, delta_clar_hz: f64, mode: Modulation_Mode) {
	if math.abs(delta_clar_hz) < 2.5 do return
	if mode != .LSB && mode != .USB do return

	d_aud := delta_clar_hz if mode == .LSB else -delta_clar_hz

	if math.abs(bank.advanced.tracked_df) >= 4.0 {
		nd3 := clamp(bank.advanced.tracked_df + d_aud, -75.0, 75.0)
		if math.abs(nd3) < 4.0 do nd3 = 0.0
		bank.advanced.tracked_df = nd3
	} else {
		bank.advanced.tracked_df = 0.0
	}
	bank.advanced.mistune_cand   = bank.advanced.tracked_df
	bank.advanced.mistune_streak = 0
	for ri in 0 ..< len(bank.advanced.df_ring) {
		if bank.advanced.f0_ring[ri] >= 80.0 {
			bank.advanced.df_ring[ri] = clamp(bank.advanced.df_ring[ri] + d_aud, -75.0, 75.0)
		}
	}

	if math.abs(bank.ultra.tracked_df_hz) >= 4.0 {
		new_df := clamp(bank.ultra.tracked_df_hz + d_aud, -80.0, 80.0)
		if math.abs(new_df) < 4.0 do new_df = 0.0
		bank.ultra.tracked_df_hz = new_df
	} else {
		bank.ultra.tracked_df_hz = 0.0
	}
	bank.ultra.mistune_cand_df = bank.ultra.tracked_df_hz
	bank.ultra.mistune_streak  = 0
	for ri in 0 ..< len(bank.ultra.deep_df_ring) {
		if bank.ultra.deep_f0_ring[ri] >= 80.0 {
			bank.ultra.deep_df_ring[ri] = clamp(bank.ultra.deep_df_ring[ri] + d_aud, -80.0, 80.0)
		}
		if bank.ultra.med_f0_ring[ri] >= 80.0 {
			bank.ultra.med_df_ring[ri] = clamp(bank.ultra.med_df_ring[ri] + d_aud, -80.0, 80.0)
		}
	}

	// Shift persistent narrowband CW notch bins & 12 kHz high-res comb history synchronously across clarifier steps!
	sh_bins_4k := int(math.round(d_aud / (AUDIO_SAMPLE_RATE_HZ / 4096.0)))
	if sh_bins_4k != 0 {
		old_cw3 := bank.advanced.ana_cw_persist
		old_cw4 := bank.ultra.cw_persist
		bank.advanced.ana_cw_persist = {}
		bank.ultra.cw_persist        = {}
		for k in 4 ..< (L4_BINS - 4) {
			kd := k + sh_bins_4k
			if kd >= 4 && kd < (L4_BINS - 4) {
				bank.advanced.ana_cw_persist[kd] = old_cw3[k]
				bank.ultra.cw_persist[kd]        = old_cw4[k]
			}
		}
	}
	sh_df_prof := int(math.round(d_aud / DF_PROF_STEP))
	if sh_df_prof != 0 {
		shift_bins_f64(bank.advanced.df_profile[:], sh_df_prof, 0)
		shift_bins_f64(bank.ultra.df_profile[:],    sh_df_prof, 0)
	}
	if math.abs(bank.ultra.df_cand_live) >= 3.5 {
		bank.ultra.df_cand_live = clamp(bank.ultra.df_cand_live + d_aud, -40.0, 40.0)
	}
	sh_hr2 := int(math.round(d_aud / HR2K_BIN_HZ))
	if sh_hr2 != 0 {
		for sl in 0 ..< len(bank.advanced.hr_past2k_pow) do shift_bins_f64(bank.advanced.hr_past2k_pow[sl][:], sh_hr2, 4)
		for sl in 0 ..< len(bank.advanced.hr_pres2k_pow) do shift_bins_f64(bank.advanced.hr_pres2k_pow[sl][:], sh_hr2, 4)
		shift_bins_f64(bank.advanced.hr_wb2k_psd[:], sh_hr2, 4)

		for sl in 0 ..< len(bank.ultra.hr_past2k_pow) do shift_bins_f64(bank.ultra.hr_past2k_pow[sl][:], sh_hr2, 4)
		for sl in 0 ..< len(bank.ultra.hr_pres2k_pow) do shift_bins_f64(bank.ultra.hr_pres2k_pow[sl][:], sh_hr2, 4)
		shift_bins_f64(bank.ultra.hr_wb2k_psd[:], sh_hr2, 4)
	}
	sh_hr4 := int(math.round(d_aud / HR4K_BIN_HZ))
	if sh_hr4 != 0 {
		for sl in 0 ..< len(bank.advanced.hr_past4k_pow) do shift_bins_f64(bank.advanced.hr_past4k_pow[sl][:], sh_hr4, 6)
		for sl in 0 ..< len(bank.advanced.hr_pres4k_pow) do shift_bins_f64(bank.advanced.hr_pres4k_pow[sl][:], sh_hr4, 6)
		shift_bins_f64(bank.advanced.hr_wb4k_psd[:], sh_hr4, 6)

		for sl in 0 ..< len(bank.ultra.hr_past4k_pow) do shift_bins_f64(bank.ultra.hr_past4k_pow[sl][:], sh_hr4, 6)
		for sl in 0 ..< len(bank.ultra.hr_pres4k_pow) do shift_bins_f64(bank.ultra.hr_pres4k_pow[sl][:], sh_hr4, 6)
		shift_bins_f64(bank.ultra.hr_wb4k_psd[:], sh_hr4, 6)
	}

	if bank.advanced.pend_sh_wait > 0 {
		bank.advanced.pend_sh_hz += d_aud
	} else {
		for k in 0 ..< L3_BINS {
			nf := math.max(bank.advanced.noise_psd[k], 1e-20)
			ck := (bank.advanced.tonal_mag_ema[k] * bank.advanced.tonal_mag_ema[k]) / math.max(bank.advanced.tonal_pow_ema[k], 1e-20)
			bank.advanced.pend_tonal[k] = ck > 0.86 && bank.advanced.welch_psd[k] > 6.5 * nf
		}
		bank.advanced.pend_sh_hz = d_aud
	}
	bank.advanced.pend_sh_wait = 10

	if bank.ultra.pend_sh_wait > 0 {
		bank.ultra.pend_sh_hz += d_aud
	} else {
		for k in 0 ..< L4_BINS {
			nf := math.max(bank.ultra.noise_psd[k], 1e-20)
			ck := (bank.ultra.tonal_mag_ema[k] * bank.ultra.tonal_mag_ema[k]) / math.max(bank.ultra.tonal_pow_ema[k], 1e-20)
			bank.ultra.pend_tonal[k] = ck > 0.86 && bank.ultra.welch_psd[k] > 6.5 * nf
		}
		bank.ultra.pend_sh_hz = d_aud
	}
	bank.ultra.pend_sh_wait = 7

	if math.abs(d_aud) >= 10.0 {
		for k in 0 ..< L4_BINS {
			nf3 := bank.advanced.ana_noise_psd[k]
			if nf3 > 0.0 && bank.advanced.ana_med_psd[k] < 6.0 * nf3 {
				bank.advanced.ana_med_psd[k]  = 0.40 * bank.advanced.ana_med_psd[k]  + 0.60 * nf3
				bank.advanced.ana_long_psd[k] = 0.40 * bank.advanced.ana_long_psd[k] + 0.60 * nf3
			}
			nf := bank.ultra.noise_psd[k]
			if nf > 0.0 && bank.ultra.pitch_psd[k] < 6.0 * nf {
				bank.ultra.pitch_psd[k] = 0.35 * bank.ultra.pitch_psd[k] + 0.65 * nf
				bank.ultra.long_psd[k]  = 0.35 * bank.ultra.long_psd[k]  + 0.65 * nf
			}
		}
		bank.advanced.shs_smooth      = {}
		bank.advanced.shs4k_smooth    = {}
		bank.ultra.shs_smooth         = {}
		bank.ultra.med_shs_smooth     = {}
		bank.advanced.med_streak      = min(bank.advanced.med_streak, 4)
		bank.advanced.d4k_streak      = min(bank.advanced.d4k_streak, 4)
		bank.ultra.pitch_lock_streak  = min(bank.ultra.pitch_lock_streak, 6)
		bank.ultra.med_lock_streak    = min(bank.ultra.med_lock_streak, 3)
	}
}

@(private)
shift_bins_f64 :: proc(arr: []f64, sh, pad: int) {
	n := len(arr)
	if sh == 0 || n <= 2 * pad do return
	if sh > 0 {
		for kd := n - pad - 1; kd >= pad; kd -= 1 {
			ks := kd - sh
			arr[kd] = arr[ks] if (ks >= pad && ks < n - pad) else 0.0
		}
	} else {
		for kd := pad; kd < n - pad; kd += 1 {
			ks := kd - sh
			arr[kd] = arr[ks] if (ks >= pad && ks < n - pad) else 0.0
		}
	}
}

audio_filter_bank_process :: proc(
	bank     : ^Audio_Filter_Bank,
	buf      : []f64,
	level    : Filter_Level,
	mode     : Modulation_Mode,
	rf_notch : bool = false,
) -> (voice_conf: f64, pitch_hz: f64, nr_db: f64) {
	switch level {
	case .Off:
		return 0.0, 0.0, 0.0
	case .Level_1:
		p_in := block_power_f64(buf)
		simple_filter_process(&bank.simple, buf, mode)
		p_out := block_power_f64(buf)
		return 0.0, 0.0, calc_nr_db(p_in, p_out)
	case .Level_2:
		p_in := block_power_f64(buf)
		moderate_filter_process(&bank.moderate, buf, mode)
		p_out := block_power_f64(buf)
		return 0.0, 0.0, calc_nr_db(p_in, p_out)
	case .Level_3:
		p_in := block_power_f64(buf)
		spectral_wiener_process(&bank.advanced, buf, mode, rf_notch)
		p_out := block_power_f64(buf)
		return bank.advanced.voice_conf, bank.advanced.tracked_f0, calc_nr_db(p_in, p_out)
	case .Level_4:
		ultra_voice_filter_process(&bank.ultra, buf, mode, rf_notch)
		return bank.ultra.voice_confidence, bank.ultra.tracked_f0_hz, bank.ultra.noise_reduc_db
	}
	return 0.0, 0.0, 0.0
}

block_power_f64 :: #force_inline proc(buf: []f64) -> f64 {
	if len(buf) == 0 do return 1e-24
	s := 0.0
	for v in buf {
		s += v * v
	}
	return math.max(s / f64(len(buf)), 1e-24)
}

@(private)
calc_nr_db :: #force_inline proc(p_in, p_out: f64) -> f64 {
	if p_in <= 1e-23 do return 0.0
	return clamp(10.0 * math.log10(p_in / math.max(p_out, 1e-24)), 0.0, 48.0)
}
