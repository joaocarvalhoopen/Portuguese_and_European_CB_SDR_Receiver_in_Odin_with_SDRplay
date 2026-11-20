// Comprehensive Deterministic f64 DSP Verification & Odin Unit Test Suite.
// Verifies:
//   1. Custom f64 Radix-2 FFT/IFFT round-trip accuracy (< 1e-14) & Parseval identity
//   2. Pure-Odin f64 Exponential Integral E1(v) precision for Ephraim-Malah MMSE-LSA
//   3. Portuguese/CEPT 40-Channel CB frequency table (including Ch 22/23/24/25 & R/C skips)
//   4. >90 dB USB/LSB opposite-sideband rejection in 2048-pt Overlap-Save analytic filter
//   5. Full 6.0 MSamples/s (14-bit) -> 48 kHz demodulation for AM, FM, LSB, and USB
//   6. Monotonic noise suppression across Filter Levels 1, 2, 3, 4 on AM, FM, and SSB
//   7. Human voice recovery on SSB at -20 dB, -25 dB, and -30 dB under the noise floor!

package main

import "core:math"
import "core:os"
import "core:strings"
import "core:sync"
import "core:testing"
import "core:time"
import "vendor:miniaudio"

import sdr "../SDRplay_API_Odin_bindings/sdrplay_api_bindings"

// Measure spectral correlation of fundamental + harmonic voice bins [120..2500 Hz] between
// reference clean speech and recovered noisy speech using a 4096-pt f64 FFT.
@(private)
measure_speech_recovery_metrics :: proc(
	plan      : ^FFT_Plan,
	clean_ref : []f64,
	recovered : []f64,
) -> (spec_corr: f64, harmonic_snr_db: f64) {
	n := L4_FFT_N
	assert(len(clean_ref) >= n && len(recovered) >= n)

	c_fft := make([]complex128, n)
	r_fft := make([]complex128, n)
	defer delete(c_fft)
	defer delete(r_fft)

	for i in 0 ..< n {
		w := root_hann_f64(i, n)
		c_fft[i] = complex(clean_ref[i] * w, 0.0)
		r_fft[i] = complex(recovered[i] * w, 0.0)
	}
	fft_forward(plan, c_fft)
	fft_forward(plan, r_fft)

	bin_hz := AUDIO_SAMPLE_RATE_HZ / f64(n)
	// Evaluate full audible spectrum [35 Hz .. 12,000 Hz] so 50/100 Hz indoor hum,
	// SMPS switching whine, and high-frequency channel hiss are all accounted for honestly!
	k_lo   := int(math.ceil(35.0 / bin_hz))
	k_hi   := int(math.floor(12000.0 / bin_hz))
	k_vhi  := int(math.floor(2550.0 / bin_hz))

	// Determine peak clean bin threshold to partition active voice harmonic bins vs noise-only bins
	max_clean_mag := 0.0
	for k in k_lo ..= k_hi {
		m := c64_abs(c_fft[k])
		if m > max_clean_mag do max_clean_mag = m
	}

	dot     := 0.0
	norm_c2 := 0.0
	norm_r2 := 0.0

	sig_bins_pow   := 0.0
	sig_bins_cnt   := 0
	noise_bins_pow := 0.0
	noise_bins_cnt := 0

	for k in k_lo ..= k_hi {
		mc := c64_abs(c_fft[k])
		mr := c64_abs(r_fft[k])
		dot     += mc * mr
		norm_c2 += mc * mc
		norm_r2 += mr * mr

		pr := mr * mr
		if mc > 0.12 * max_clean_mag {
			sig_bins_pow += pr
			sig_bins_cnt += 1
		} else if k <= k_vhi && mc < 0.02 * max_clean_mag {
			noise_bins_pow += pr
			noise_bins_cnt += 1
		}
	}

	if norm_c2 > 1e-24 && norm_r2 > 1e-24 {
		spec_corr = dot / math.sqrt(norm_c2 * norm_r2)
	}

	// Compare total recovered voice harmonic power against equivalent speech-passband residual noise power
	mean_noise_per_bin := noise_bins_pow / f64(max(noise_bins_cnt, 1))
	total_pass_noise   := mean_noise_per_bin * f64(k_vhi - k_lo + 1)
	pure_voice_pow     := math.max(sig_bins_pow - mean_noise_per_bin * f64(sig_bins_cnt), 1e-24)
	harmonic_snr_db     = 10.0 * math.log10(pure_voice_pow / math.max(total_pass_noise, 1e-24))
	return
}

// Measure audio fundamental peak frequency (Hz), THD (%), fundamental power (dB), and specific tone bin power (dB)
@(private)
measure_audio_tone_fidelity_1024 :: proc(
	plan1k    : ^FFT_Plan,
	samples   : []f64,
	target_hz : f64,
	spur_hz   : f64 = 0.0,
) -> (peak_hz: f64, thd_pct: f64, fund_db: f64, spur_rej_db: f64) {
	N :: 1024
	assert(len(samples) >= N)
	buf := make([]complex128, N)
	defer delete(buf)

	mean := 0.0
	for i in 0 ..< N do mean += samples[i]
	mean /= f64(N)

	for i in 0 ..< N {
		w := blackman_harris_f64(i, N)
		buf[i] = complex((samples[i] - mean) * w, 0.0)
	}
	fft_forward(plan1k, buf)

	bin_hz := AUDIO_SAMPLE_RATE_HZ / f64(N) // 46.875 Hz/bin
	k_est  := clamp(int(math.round(target_hz / bin_hz)), 4, (N / 2) - 5)
	best_k := k_est
	best_p := -1.0
	for k in (k_est - 3) ..= (k_est + 3) {
		p := c64_abs2(buf[k])
		if p > best_p {
			best_p = p
			best_k = k
		}
	}

	m_l   := math.ln(math.max(c64_abs(buf[best_k - 1]), 1e-30))
	m_0   := math.ln(math.max(c64_abs(buf[best_k]),     1e-30))
	m_r   := math.ln(math.max(c64_abs(buf[best_k + 1]), 1e-30))
	denom := m_l - 2.0 * m_0 + m_r
	delta := 0.0
	if math.abs(denom) > 1e-18 {
		delta = clamp(0.5 * (m_l - m_r) / denom, -0.5, 0.5)
	}
	peak_hz = (f64(best_k) + delta) * bin_hz

	fund_pow := 0.0
	for k in max(1, best_k - 3) ..= min((N / 2) - 1, best_k + 3) {
		fund_pow += c64_abs2(buf[k])
	}

	harm_pow := 0.0
	for h in 2 ..= 5 {
		kh := int(math.round((f64(h) * peak_hz) / bin_hz))
		if kh + 3 < (N / 2) {
			for k in (kh - 3) ..= (kh + 3) {
				harm_pow += c64_abs2(buf[k])
			}
		}
	}

	thd_pct = 100.0 * math.sqrt(harm_pow / math.max(fund_pow, 1e-30))
	fund_db = 10.0 * math.log10(math.max(fund_pow, 1e-30))

	if spur_hz > 20.0 {
		ks := clamp(int(math.round(spur_hz / bin_hz)), 2, (N / 2) - 3)
		spur_pow := 0.0
		for k in (ks - 2) ..= (ks + 2) {
			if math.abs(f64(k - best_k)) > 3.0 {
				spur_pow += c64_abs2(buf[k])
			}
		}
		spur_rej_db = 10.0 * math.log10(math.max(fund_pow, 1e-30) / math.max(spur_pow, 1e-30))
	}
	return
}

run_complete_dsp_selftest :: proc() -> bool {
	screen_init()
	defer screen_shutdown()

	screen_println("================================================================================")
	screen_println("  RUNNING BUILT-IN f64 DSP, FFT, DEMODULATOR & -30dB VOICE RECOVERY SUITE")
	screen_println("================================================================================")
	all_ok := true

	// 1. FFT / IFFT Round-Trip & Parseval Energy Test (N = 4096)
	{
		plan := new(FFT_Plan)
		defer free(plan)
		fft_plan_init(plan, 4096)

		orig := make([]complex128, 4096)
		work := make([]complex128, 4096)
		defer delete(orig)
		defer delete(work)

		e_time := 0.0
		for i in 0 ..< 4096 {
			t  := f64(i) / 48000.0
			re := 0.7 * math.cos(2.0 * math.PI * 440.0 * t) - 0.3 * math.sin(2.0 * math.PI * 1875.3 * t)
			im := 0.4 * math.sin(2.0 * math.PI * 912.7 * t)
			orig[i] = complex(re, im)
			work[i] = orig[i]
			e_time += c64_abs2(orig[i])
		}

		fft_forward(plan, work)
		e_freq := 0.0
		for i in 0 ..< 4096 {
			e_freq += c64_abs2(work[i])
		}
		e_freq /= 4096.0
		fft_inverse(plan, work)

		err_sq := 0.0
		for i in 0 ..< 4096 {
			err_sq += c64_abs2(work[i] - orig[i])
		}
		rms_err := math.sqrt(err_sq / 4096.0)
		parseval_err := math.abs(e_time - e_freq) / e_time

		ok := rms_err < 1e-14 && parseval_err < 1e-13
		all_ok &= ok
		screen_printfln("  [1] 4096-pt f64 FFT/IFFT Round-Trip : RMS err = %.2e, Parseval err = %.2e -> %s",
			rms_err, parseval_err, "PASS" if ok else "FAIL")
	}

	// 2. Exponential Integral E1(v) Accuracy Test
	{
		e1_01 := expint_e1_f64(0.1)
		e1_10 := expint_e1_f64(1.0)
		e1_50 := expint_e1_f64(5.0)
		// NIST DLMF exact values:
		// E1(0.1) = 1.8229239584193906, E1(1.0) = 0.2193839343955203, E1(5.0) = 0.0011482955912753257
		err_max := max(
			math.abs(e1_01 - 1.8229239584193906),
			max(math.abs(e1_10 - 0.2193839343955203), math.abs(e1_50 - 0.0011482955912753257)),
		)
		ok := err_max < 1e-12
		all_ok &= ok
		screen_printfln("  [2] Ephraim-Malah f64 E1(v) Kernel  : Max abs error = %.2e -> %s",
			err_max, "PASS" if ok else "FAIL")
	}

	// 3. Portuguese / CEPT 40-Channel CB Frequency Table Verification
	{
		ok := CB_CHANNELS[0].freq_hz == 26_965_000.0 &&  // Ch 1
		      CB_CHANNELS[2].freq_hz == 26_985_000.0 &&  // Ch 3
		      CB_CHANNELS[3].freq_hz == 27_005_000.0 &&  // Ch 4 (R/C 26.995 skipped)
		      CB_CHANNELS[8].freq_hz == 27_065_000.0 &&  // Ch 9 Emergency
		      CB_CHANNELS[18].freq_hz == 27_185_000.0 && // Ch 19 Road/Truckers
		      CB_CHANNELS[21].freq_hz == 27_225_000.0 && // Ch 22
		      CB_CHANNELS[22].freq_hz == 27_255_000.0 && // Ch 23 (+30 kHz CEPT/FCC rule)
		      CB_CHANNELS[23].freq_hz == 27_235_000.0 && // Ch 24
		      CB_CHANNELS[24].freq_hz == 27_245_000.0 && // Ch 25
		      CB_CHANNELS[39].freq_hz == 27_405_000.0    // Ch 40
		all_ok &= ok
		screen_printfln("  [3] Portuguese/CEPT 40 CB Channels  : 26.965..27.405 MHz (Ch 22-25 verified) -> %s",
			"PASS" if ok else "FAIL")
	}

	// 4. 2048-pt Overlap-Save USB vs LSB Opposite-Sideband Rejection (> 90 dB)
	{
		cf := new(Channel_Selectivity_Filter)
		defer free(cf)
		channel_filter_init(cf)

		// Inject pure +1000 Hz USB complex tone vs -1000 Hz LSB image tone
		p_wanted := 0.0
		p_image  := 0.0
		for n in 0 ..< 4096 {
			phi := 2.0 * math.PI * 1000.0 * f64(n) / BASEBAND_RATE_HZ
			z_usb := complex(math.cos(phi), math.sin(phi))
			if channel_filter_push(cf, z_usb, .USB) && n >= 2048 {
				for s in cf.out_ready {
					p_wanted += c64_abs2(s)
				}
			}
		}
		channel_filter_reset(cf)
		for n in 0 ..< 4096 {
			phi := -2.0 * math.PI * 1000.0 * f64(n) / BASEBAND_RATE_HZ
			z_lsb := complex(math.cos(phi), math.sin(phi))
			if channel_filter_push(cf, z_lsb, .USB) && n >= 2048 {
				for s in cf.out_ready {
					p_image += c64_abs2(s)
				}
			}
		}
		rej_db := 10.0 * math.log10(p_wanted / math.max(p_image, 1e-30))
		ok := rej_db > 90.0
		all_ok &= ok
		screen_printfln("  [4] Analytic SSB Image Rejection    : %.1f dB opposite-sideband suppression -> %s",
			rej_db, "PASS" if ok else "FAIL")
	}

	// 5. Full 6.0 MSamples/s (14-bit) -> 48 kHz Demodulation Audit (AM, FM, LSB, USB + THD + Off-Freq + De-Emphasis)
	{
		modes  := [4]Modulation_Mode{.AM, .FM, .LSB, .USB}
		fe     := new(FrontEnd_Decimator)
		cf     := new(Channel_Selectivity_Filter)
		dm     := new(Demodulator_State)
		plan1k := new(FFT_Plan)
		defer free(fe)
		defer free(cf)
		defer free(dm)
		defer free(plan1k)

		channel_filter_init(cf)
		fft_plan_init(plan1k, 1024)
		demod_ok := true

		// 0.16 seconds at 6.0 MHz = 960,000 samples -> 7,680 samples at 48 kHz (7 full 1024-pt hops)
		N_6M  :: 960_000
		xi    := make([]i16, N_6M)
		xq    := make([]i16, N_6M)
		iq48  := make([]complex128, (N_6M / 125) + 64)
		aud   := make([]f64, 1024)
		defer delete(xi)
		defer delete(xq)
		defer delete(iq48)
		defer delete(aud)

		am_thd  := 99.0
		fm_thd  := 99.0
		fc_if   := -SDR_LO_OFFSET_HZ
		for m in modes {
			frontend_init(fe, 0.0)
			channel_filter_reset(cf)
			demod_init(dm)

			fm_ph := 0.0
			for n in 0 ..< N_6M {
				t_s  := f64(n) / SDR_SAMPLE_RATE_HZ
				a_re := math.cos(2.0 * math.PI * 800.0 * t_s)
				a_im := math.sin(2.0 * math.PI * 800.0 * t_s)
				p_if := 2.0 * math.PI * fc_if * t_s
				cif  := math.cos(p_if)
				sif  := math.sin(p_if)
				si, sq_v : f64
				switch m {
				case .AM:
					env  := 0.35 * (1.0 + 0.70 * a_re)
					si    = env * cif
					sq_v  = env * sif
				case .FM:
					fm_ph += (2.0 * math.PI * 1800.0 * a_re) / SDR_SAMPLE_RATE_HZ
					si     = 0.40 * math.cos(p_if + fm_ph)
					sq_v   = 0.40 * math.sin(p_if + fm_ph)
				case .USB:
					si    = 0.40 * (a_re * cif - a_im * sif)
					sq_v  = 0.40 * (a_re * sif + a_im * cif)
				case .LSB:
					si    = 0.40 * (a_re * cif + a_im * sif)
					sq_v  = 0.40 * (a_re * sif - a_im * cif)
				}
				xi[n] = i16(math.round(si * 8191.0) * 4.0)
				xq[n] = i16(math.round(sq_v * 8191.0) * 4.0)
			}

			n48      := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
			blocks   := 0
			last_rms := 0.0
			pk_hz    := 0.0
			thd_p    := 99.0
			for k in 0 ..< n48 {
				if channel_filter_push(cf, iq48[k], m) {
					demodulate_block_f64(dm, cf.out_ready[:], m, aud[0:1024])
					blocks += 1
					if blocks == 6 {
						last_rms = math.sqrt(block_power_f64(aud[0:1024]))
						pk_hz, thd_p, _, _ = measure_audio_tone_fidelity_1024(plan1k, aud[0:1024], 800.0)
					}
				}
			}
			if m == .AM do am_thd = thd_p
			if m == .FM do fm_thd = thd_p
			if last_rms < 0.05 || math.abs(pk_hz - 800.0) > 1.5 || thd_p > 0.50 {
				demod_ok = false
			}
		}

		// 5b. Off-frequency AM (+550 Hz carrier offset, clarifier OFF): verify Type-II SAM PLL locks with zero beat whistle!
		frontend_init(fe, 0.0)
		channel_filter_reset(cf)
		demod_init(dm)
		for n in 0 ..< N_6M {
			t_s  := f64(n) / SDR_SAMPLE_RATE_HZ
			a_re := math.cos(2.0 * math.PI * 800.0 * t_s)
			p_if := 2.0 * math.PI * (fc_if + 550.0) * t_s
			env  := 0.35 * (1.0 + 0.70 * a_re)
			xi[n] = i16(math.round(env * math.cos(p_if) * 8191.0) * 4.0)
			xq[n] = i16(math.round(env * math.sin(p_if) * 8191.0) * 4.0)
		}
		n48_am := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
		blks_am := 0
		am_off_pk, am_off_thd, am_whistle_rej : f64
		for k in 0 ..< n48_am {
			if channel_filter_push(cf, iq48[k], .AM) {
				demodulate_block_f64(dm, cf.out_ready[:], .AM, aud[0:1024])
				blks_am += 1
				if blks_am == 7 {
					am_off_pk, am_off_thd, _, am_whistle_rej = measure_audio_tone_fidelity_1024(plan1k, aud[0:1024], 800.0, 550.0)
				}
			}
		}
		am_sam_ok := dm.sam_lock_metric > 0.90 &&
		             math.abs(dm.carrier_off_hz - 550.0) < 2.0 &&
		             math.abs(am_off_pk - 800.0) < 1.5 &&
		             am_off_thd < 0.60 &&
		             am_whistle_rej > 45.0

		// 5c. Off-frequency FM (-600 Hz carrier offset, clarifier OFF) + CEPT 530.5 us (300 Hz) De-Emphasis check!
		frontend_init(fe, 0.0)
		channel_filter_reset(cf)
		demod_init(dm)
		fm_ph_off := 0.0
		for n in 0 ..< N_6M {
			t_s       := f64(n) / SDR_SAMPLE_RATE_HZ
			a_re      := math.cos(2.0 * math.PI * 800.0 * t_s)
			fm_ph_off += (2.0 * math.PI * 1800.0 * a_re) / SDR_SAMPLE_RATE_HZ
			p_if      := 2.0 * math.PI * (fc_if - 600.0) * t_s + fm_ph_off
			xi[n]      = i16(math.round(0.40 * math.cos(p_if) * 8191.0) * 4.0)
			xq[n]      = i16(math.round(0.40 * math.sin(p_if) * 8191.0) * 4.0)
		}
		n48_fm := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
		blks_fm := 0
		fm_off_pk, fm_off_thd : f64
		for k in 0 ..< n48_fm {
			if channel_filter_push(cf, iq48[k], .FM) {
				demodulate_block_f64(dm, cf.out_ready[:], .FM, aud[0:1024])
				blks_fm += 1
				if blks_fm == 6 {
					fm_off_pk, fm_off_thd, _, _ = measure_audio_tone_fidelity_1024(plan1k, aud[0:1024], 800.0)
				}
			}
		}
		// At t = 0.16s (7,680 samples @ DC_ALPHA=0.00018), 1st-order DC tracker reaches ~75% of -600 Hz (-449 Hz)
		// while the 110 Hz high-pass already yields < 0.02% THD on the recovered 800 Hz tone!
		fm_off_ok := dm.carrier_off_hz < -410.0 && dm.carrier_off_hz > -615.0 &&
		             math.abs(fm_off_pk - 800.0) < 1.5 &&
		             fm_off_thd < 0.50

		// Verify CEPT 530.5 us (f_c = 300 Hz) FM de-emphasis flattens +6 dB/oct pre-emphasized 400 Hz vs 2000 Hz tones!
		fm_deemph_db := [2]f64{}
		test_freqs   := [2]f64{400.0, 2000.0}
		for fi in 0 ..< 2 {
			f_tone  := test_freqs[fi]
			// Exact CEPT 530.5 us pre-emphasis transfer magnitude |H_pre(f)| = sqrt(1 + (f / 300)^2)
			pre_mag := math.sqrt(1.0 + (f_tone / 300.0) * (f_tone / 300.0))
			dev_hz  := 350.0 * pre_mag
			frontend_init(fe, 0.0)
			channel_filter_reset(cf)
			demod_init(dm)
			ph_d := 0.0
			for n in 0 ..< N_6M {
				t_s  := f64(n) / SDR_SAMPLE_RATE_HZ
				ph_d += (2.0 * math.PI * dev_hz * math.cos(2.0 * math.PI * f_tone * t_s)) / SDR_SAMPLE_RATE_HZ
				p_if := 2.0 * math.PI * fc_if * t_s + ph_d
				xi[n] = i16(math.round(0.40 * math.cos(p_if) * 8191.0) * 4.0)
				xq[n] = i16(math.round(0.40 * math.sin(p_if) * 8191.0) * 4.0)
			}
			nd := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
			bd := 0
			for k in 0 ..< nd {
				if channel_filter_push(cf, iq48[k], .FM) {
					demodulate_block_f64(dm, cf.out_ready[:], .FM, aud[0:1024])
					bd += 1
					if bd == 6 {
						_, _, fm_deemph_db[fi], _ = measure_audio_tone_fidelity_1024(plan1k, aud[0:1024], f_tone)
					}
				}
			}
		}
		deemph_flat_err := math.abs(fm_deemph_db[1] - fm_deemph_db[0])
		deemph_ok       := deemph_flat_err < 1.20

		demod_ok &= am_sam_ok && fm_off_ok && deemph_ok
		all_ok   &= demod_ok
		screen_printfln(
			"  [5] AM/FM/SSB Demod & Fidelity Audit: AM THD=%.3f%% (+550Hz Lock=OK) | FM THD=%.3f%% (DeEmph=%.2fdB) -> %s",
			am_thd, fm_thd, deemph_flat_err, "PASS" if demod_ok else "FAIL",
		)
	}

	// 6 & 7. 4-Level Indoor House Noise Filter Progression & -20/-25/-30 dB SSB Voice Recovery!
	{
		plan4k := new(FFT_Plan)
		bank   := new(Audio_Filter_Bank)
		defer free(plan4k)
		defer free(bank)
		fft_plan_init(plan4k, L4_FFT_N)

		// Synthesize 1.70 seconds (81,920 samples = 80 hops of 1024) of voiced human speech (F0 = 145 Hz, 14 harmonics)
		TOTAL_SAMPLES :: 80 * 1024
		clean_voice := make([]f64, TOTAL_SAMPLES)
		noisy_in    := make([]f64, TOTAL_SAMPLES)
		work_buf    := make([]f64, TOTAL_SAMPLES)
		defer delete(clean_voice)
		defer delete(noisy_in)
		defer delete(work_buf)

		// First 16 hops (~0.34s) establish realistic background indoor noise floor before speech syllable enters,
		// matching real CB radio operation where receiver listens to channel before operator keys/speaks.
		PRE_NOISE_SAMPLES :: 16 * 1024
		voice_pow_sum := 0.0
		voice_cnt     := 0
		for n in PRE_NOISE_SAMPLES ..< TOTAL_SAMPLES {
			t_s := f64(n - PRE_NOISE_SAMPLES) / AUDIO_SAMPLE_RATE_HZ
			f0  := 145.0
			s   := 0.0
			for h in 1 ..= 14 {
				fh  := f64(h) * f0
				d1  := (fh - 580.0) / 160.0
				d2  := (fh - 1580.0) / 260.0
				amp := (1.0 / (1.0 + d1 * d1) + 0.70 / (1.0 + d2 * d2)) / math.pow(f64(h), 0.30)
				s   += amp * math.cos(2.0 * math.PI * fh * t_s + 0.25 * f64(h))
			}
			clean_voice[n] = s * 0.25
			voice_pow_sum += clean_voice[n] * clean_voice[n]
			voice_cnt     += 1
		}
		clean_voice_rms := math.sqrt(voice_pow_sum / f64(voice_cnt))

		// Test progressive Filter Levels 1..4 on composite Indoor House Noise + SSB Band Noise at -20 dB, -25 dB, -30 dB
		snr_targets := [3]f64{-20.0, -25.0, -30.0}
		deep_ok     := true

		for target_snr_db in snr_targets {
			// Total composite noise power inside demodulated audio = Clean_Power * 10^(-target_snr_db / 10)
			// Composite noise = broadband SSB thermal static + 50/100/150 Hz house mains hum +
			// 1380 Hz SMPS switching tone + 100 Hz household triac spikes!
			target_noise_rms := clean_voice_rms * math.pow(10.0, -target_snr_db / 20.0)

			rng : u64 = 0x9E37_79B9_7F4A_7C15 + u64(math.abs(target_snr_db) * 100.0)
			raw_n_sum := 0.0
			for n in 0 ..< TOTAL_SAMPLES {
				t_s := f64(n) / AUDIO_SAMPLE_RATE_HZ
				g1, _ := xorshift_f64_normal(&rng)
				// Realistic indoor house noise mix:
				// broadband floor + 50/100/150 Hz mains buzz + 1385 Hz SMPS whine + triac clicks
				hum    := 0.75 * math.cos(2.0 * math.PI * 50.0 * t_s) +
				          0.55 * math.cos(2.0 * math.PI * 100.0 * t_s + 0.4) +
				          0.35 * math.cos(2.0 * math.PI * 150.0 * t_s + 1.1)
				smps   := 0.65 * math.cos(2.0 * math.PI * 1385.0 * t_s + 0.2)
				spike  := 3.8 if (n % 480) == 0 else 0.0
				comp_n := 0.85 * g1 + hum + smps + spike
				noisy_in[n] = comp_n
				raw_n_sum  += comp_n * comp_n
			}
			raw_n_rms := math.sqrt(raw_n_sum / f64(TOTAL_SAMPLES))
			n_scale   := target_noise_rms / raw_n_rms
			for n in 0 ..< TOTAL_SAMPLES {
				noisy_in[n] = clean_voice[n] + noisy_in[n] * n_scale
			}

			eval_start := TOTAL_SAMPLES - L4_FFT_N - 1024 // Compensate 3-hop OLA latency (3072 samples)
			raw_corr, raw_hsnr := measure_speech_recovery_metrics(
				plan4k,
				clean_voice[eval_start : eval_start + L4_FFT_N],
				noisy_in[eval_start : eval_start + L4_FFT_N],
			)

			// Run Filter Level 4 (Ultra Deep Voice Recovery)
			audio_filter_bank_init(bank)
			copy(work_buf, noisy_in)
			last_vc, last_f0, last_nr : f64
			for blk := 0; blk < TOTAL_SAMPLES; blk += 1024 {
				last_vc, last_f0, last_nr = audio_filter_bank_process(
					bank,
					work_buf[blk : blk + 1024],
					.Level_4,
					.LSB,
				)
			}

			// Note: 4096-pt OLA with hop=1024 has exact delay of 3072 samples (L4_FFT_N - L4_HOP)
			ref_slice := clean_voice[TOTAL_SAMPLES - 3072 - L4_FFT_N : TOTAL_SAMPLES - 3072]
			out_slice := work_buf[TOTAL_SAMPLES - L4_FFT_N : TOTAL_SAMPLES]
			out_corr, out_hsnr := measure_speech_recovery_metrics(plan4k, ref_slice, out_slice)
			snr_gain := out_hsnr - target_snr_db

			pass_this := out_corr > 0.62 && snr_gain > 18.0 && math.abs(last_f0 - 145.0) < 8.0
			deep_ok  &= pass_this
			_ = raw_hsnr
			_ = last_vc
			_ = last_nr

			screen_printfln(
				"  [SSB %+5.1f dB] Raw Corr=%.2f -> Filter 4 Corr=%.2f | Out SNR=%+5.1f dB (Gain +%4.1f dB) | F0=%.1f Hz -> %s",
				target_snr_db, raw_corr, out_corr, out_hsnr, snr_gain, last_f0, "PASS" if pass_this else "FAIL",
			)
		}

		// Also verify all 4 Filter Levels across AM, FM, and USB under -18 dB composite indoor house noise
		{
			rng_m : u64 = 0x1234_5678_9ABC_DEF0
			target_n_rms := clean_voice_rms * math.pow(10.0, 18.0 / 20.0)
			sum_m := 0.0
			for n in 0 ..< TOTAL_SAMPLES {
				t_s := f64(n) / AUDIO_SAMPLE_RATE_HZ
				g1, _ := xorshift_f64_normal(&rng_m)
				hum   := 1.2 * math.cos(2.0 * math.PI * 50.0 * t_s) +
				         0.9 * math.cos(2.0 * math.PI * 100.0 * t_s + 0.4) +
				         0.7 * math.cos(2.0 * math.PI * 150.0 * t_s + 1.1)
				smps  := 0.9 * math.cos(2.0 * math.PI * 1385.0 * t_s + 0.2)
				spike := 4.5 if (n % 480) == 0 else 0.0
				cn    := 0.70 * g1 + hum + smps + spike
				noisy_in[n] = cn
				sum_m += cn * cn
			}
			sc_m := target_n_rms / math.sqrt(sum_m / f64(TOTAL_SAMPLES))
			for n in 0 ..< TOTAL_SAMPLES {
				noisy_in[n] = clean_voice[n] + noisy_in[n] * sc_m
			}
		}
		modes_check := [3]Modulation_Mode{.AM, .FM, .USB}
		levels_ok   := true
		for m in modes_check {
			corrs : [5]f64
			corrs[0], _ = measure_speech_recovery_metrics(
				plan4k,
				clean_voice[TOTAL_SAMPLES - L4_FFT_N : TOTAL_SAMPLES],
				noisy_in[TOTAL_SAMPLES - L4_FFT_N : TOTAL_SAMPLES],
			)
			for lvl_i in 1 ..= 4 {
				audio_filter_bank_init(bank)
				copy(work_buf, noisy_in)
				for blk := 0; blk < TOTAL_SAMPLES; blk += 1024 {
					audio_filter_bank_process(bank, work_buf[blk : blk + 1024], Filter_Level(lvl_i), m)
				}
				delay := 0
				if lvl_i == 3 do delay = L3_FFT_N - L3_HOP
				if lvl_i == 4 do delay = L4_FFT_N - L4_HOP
				ref_s := clean_voice[TOTAL_SAMPLES - delay - L4_FFT_N : TOTAL_SAMPLES - delay]
				out_s := work_buf[TOTAL_SAMPLES - L4_FFT_N : TOTAL_SAMPLES]
				corrs[lvl_i], _ = measure_speech_recovery_metrics(plan4k, ref_s, out_s)
			}
			screen_printfln("  [Band %-3s] Speech Spectral Corr: Off=%.2f -> L1=%.2f -> L2=%.2f -> L3=%.2f -> L4=%.2f",
				mode_name(m), corrs[0], corrs[1], corrs[2], corrs[3], corrs[4])
			if !(corrs[4] > corrs[3] && corrs[3] > corrs[2] && corrs[2] > corrs[1] && corrs[1] > corrs[0]) {
				levels_ok = false
			}
		}

		all_ok &= (deep_ok && levels_ok)
	}

	// 8. End-to-End RF Frequency Accuracy Test (LO -50 kHz Offset, NCO Sign, SAM Carrier Lock & USB/LSB Audio Pitch)
	{
		fe   := new(FrontEnd_Decimator)
		cf   := new(Channel_Selectivity_Filter)
		dm   := new(Demodulator_State)
		plan := new(FFT_Plan)
		defer free(fe)
		defer free(cf)
		defer free(dm)
		defer free(plan)

		channel_filter_init(cf)
		fft_plan_init(plan, 4096)

		N_6M :: 720_000 // 0.12 s at 6.0 MSPS -> 5,760 samples at 48 kHz
		xi   := make([]i16, N_6M)
		xq   := make([]i16, N_6M)
		iq48 := make([]complex128, (N_6M / 125) + 64)
		aud  := make([]f64, 6144)
		spec := make([]complex128, 4096)
		defer delete(xi)
		defer delete(xq)
		defer delete(iq48)
		defer delete(aud)
		defer delete(spec)

		// (a) Ch 19 AM at exact 27,185,137.5 Hz (+137.5 Hz offset above 27.185 MHz, LO = 27.135 MHz)
		f_rf_am := channel_to_freq_hz(19) + 137.5
		f_lo_19 := hardware_lo_freq_hz(19)
		f_bb_am := f_rf_am - f_lo_19 // +50,137.5 Hz
		for n in 0 ..< N_6M {
			t_s := f64(n) / SDR_SAMPLE_RATE_HZ
			env := 0.35 * (1.0 + 0.5 * math.cos(2.0 * math.PI * 700.0 * t_s))
			ang := 2.0 * math.PI * math.mod(f_bb_am * t_s, 1.0)
			xi[n] = i16(math.round(env * math.cos(ang) * 8191.0) * 4.0)
			xq[n] = i16(math.round(env * math.sin(ang) * 8191.0) * 4.0)
		}
		frontend_init(fe, 0.0)
		channel_filter_reset(cf)
		demod_init(dm)
		n48 := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
		for k in 0 ..< n48 {
			if channel_filter_push(cf, iq48[k], .AM) {
				demodulate_block_f64(dm, cf.out_ready[:], .AM, aud[0:1024], cf.noise_power_lin)
			}
		}
		am_off_err := math.abs(dm.carrier_off_hz - 137.5)

		// (b) Ch 34 USB (27.345 MHz + 1000.0 Hz = 27.346000 MHz) & Ch 38 LSB (27.385 MHz - 1000.0 Hz = 27.384000 MHz)
		measure_ssb_tone_hz :: proc(
			fe: ^FrontEnd_Decimator, cf: ^Channel_Selectivity_Filter, dm: ^Demodulator_State, plan: ^FFT_Plan,
			ch: int, mode: Modulation_Mode, f_rf_tone: f64,
			xi, xq: []i16, iq48: []complex128, aud: []f64, spec: []complex128,
		) -> f64 {
			f_bb := f_rf_tone - hardware_lo_freq_hz(ch)
			for n in 0 ..< len(xi) {
				t_s := f64(n) / SDR_SAMPLE_RATE_HZ
				ang := 2.0 * math.PI * math.mod(f_bb * t_s, 1.0)
				xi[n] = i16(math.round(0.35 * math.cos(ang) * 8191.0) * 4.0)
				xq[n] = i16(math.round(0.35 * math.sin(ang) * 8191.0) * 4.0)
			}
			frontend_init(fe, 0.0)
			channel_filter_reset(cf)
			demod_init(dm)
			n48_s := frontend_process_iq_block(fe, xi, xq, 0.0, .Off, iq48)
			out_p := 0
			for k in 0 ..< n48_s {
				if channel_filter_push(cf, iq48[k], mode) {
					if out_p + 1024 <= len(aud) {
						demodulate_block_f64(dm, cf.out_ready[:], mode, aud[out_p : out_p + 1024])
						out_p += 1024
					}
				}
			}
			start := max(out_p - 4096, 0)
			for i in 0 ..< 4096 {
				spec[i] = complex(aud[start + i] * blackman_harris_f64(i, 4096), 0.0)
			}
			fft_forward(plan, spec)
			best_k := 1
			best_p := 0.0
			for k in 5 ..< 1024 {
				p := c64_abs2(spec[k])
				if p > best_p {
					best_p = p
					best_k = k
				}
			}
			lm := 0.5 * math.ln(math.max(c64_abs2(spec[best_k - 1]), 1e-30))
			l0 := 0.5 * math.ln(math.max(c64_abs2(spec[best_k]),     1e-30))
			lp := 0.5 * math.ln(math.max(c64_abs2(spec[best_k + 1]), 1e-30))
			d  := 0.5 * (lm - lp) / (lm - 2.0 * l0 + lp)
			return (f64(best_k) + d) * (AUDIO_SAMPLE_RATE_HZ / 4096.0)
		}

		usb_tone_hz := measure_ssb_tone_hz(fe, cf, dm, plan, 34, .USB, 27_346_000.0, xi, xq, iq48, aud, spec)
		lsb_tone_hz := measure_ssb_tone_hz(fe, cf, dm, plan, 38, .LSB, 27_384_000.0, xi, xq, iq48, aud, spec)

		freq_ok := am_off_err < 2.5 && math.abs(usb_tone_hz - 1000.0) < 0.5 && math.abs(lsb_tone_hz - 1000.0) < 0.5
		all_ok  &= freq_ok
		screen_printfln(
			"  [8] Exact RF Frequency Verification : SAM Off Err=%.2f Hz | USB=%.2f Hz | LSB=%.2f Hz -> %s",
			am_off_err, usb_tone_hz, lsb_tone_hz, "PASS" if freq_ok else "FAIL",
		)
	}

	// 9. 40-Channel × 3-Band (AM/FM/LSB/USB) Auto-Scanner + Auto-LNA & Auto-RF-Gain Verification
	{
		sc := new(CB_Auto_Scanner)
		defer free(sc)
		scanner_init(sc)
		scanner_begin_capture(sc, 96)

		// Synthesize 0.18 s (1,080,000 samples at 6.0 MSPS) at wideband LO = 26.880 MHz containing:
		//   Ch 9  (27.065000 MHz) : Exact unmodulated RF calibration carrier
		//   Ch 19 (27.185000 MHz) : Modulated AM voice
		//   Ch 31 (27.315000 MHz) : Modulated FM voice
		//   Ch 34 (27.345000 MHz) : Modulated USB voice
		//   Ch 38 (27.385000 MHz) : Modulated LSB voice
		N_SCAN :: 1_080_000
		xi_s   := make([]i16, 30_000)
		xq_s   := make([]i16, 30_000)
		defer delete(xi_s)
		defer delete(xq_s)

		rng_s  : u64 = 0x55AA_1234_9988_7766
		fm_rad := 0.0
		v_re   := 0.0
		v_im   := 0.0
		dt_6m  := 1.0 / SDR_SAMPLE_RATE_HZ

		f09 := channel_to_freq_hz(9)  - SCAN_LO_FREQ_HZ
		f19 := channel_to_freq_hz(19) - SCAN_LO_FREQ_HZ
		f31 := channel_to_freq_hz(31) - SCAN_LO_FREQ_HZ
		f34 := channel_to_freq_hz(34) - SCAN_LO_FREQ_HZ
		f38 := channel_to_freq_hz(38) - SCAN_LO_FREQ_HZ

		for base := 0; base < N_SCAN; base += len(xi_s) {
			for k in 0 ..< len(xi_s) {
				idx := base + k
				t_s := f64(idx) * dt_6m
				if (idx % 125) == 0 {
					env := 0.55 + 0.45 * math.sin(2.0 * math.PI * 4.5 * t_s)
					sr, si := 0.0, 0.0
					for h in 3 ..= 12 {
						fh  := f64(h) * 165.0
						amp := 0.25 / math.pow(f64(h), 0.3)
						ang := 2.0 * math.PI * fh * t_s + 0.2 * f64(h)
						sr += amp * math.cos(ang)
						si += amp * math.sin(ang)
					}
					v_re = sr * env
					v_im = si * env
				}
				fm_rad = wrap_pi_f64(fm_rad + (2.0 * math.PI * 2100.0 * v_re) * dt_6m)

				p09 := 2.0 * math.PI * math.mod(f09 * t_s, 1.0)
				p19 := 2.0 * math.PI * math.mod(f19 * t_s, 1.0)
				p31 := 2.0 * math.PI * math.mod(f31 * t_s, 1.0) + fm_rad
				p34 := 2.0 * math.PI * math.mod(f34 * t_s, 1.0)
				p38 := 2.0 * math.PI * math.mod(f38 * t_s, 1.0)

				// Sum all 5 channels + broadband 14-bit floor
				si := 0.05 * math.cos(p09)
				sq := 0.05 * math.sin(p09)

				am_c := 0.045 * (1.0 + 0.85 * v_re)
				si += am_c * math.cos(p19)
				sq += am_c * math.sin(p19)

				si += 0.05 * math.cos(p31)
				sq += 0.05 * math.sin(p31)

				c34, s34 := math.cos(p34), math.sin(p34)
				si += 0.045 * (v_re * c34 - v_im * s34)
				sq += 0.045 * (v_re * s34 + v_im * c34)

				c38, s38 := math.cos(p38), math.sin(p38)
				si += 0.045 * (v_re * c38 + v_im * s38)
				sq += 0.045 * (v_re * s38 - v_im * c38)

				gn_i, gn_q := xorshift_f64_normal(&rng_s)
				si += gn_i * 0.030
				sq += gn_q * 0.030

				xi_s[k] = i16(clamp(math.round(si * 8191.0) * 4.0, -32767.0, 32767.0))
				xq_s[k] = i16(clamp(math.round(sq * 8191.0) * 4.0, -32767.0, 32767.0))
			}
			scanner_process_iq_6m(sc, &xi_s[0], &xq_s[0], len(xi_s))
		}

		rep := scanner_build_report(sc, .All_Bands, 6.0)
		ch09_err := math.abs(rep.entries[8].measured_peak_hz - 27_065_000.0)
		scan_ok  := rep.active_count == 5 &&
		            ch09_err < 3.0 &&
		            rep.entries[18].active && rep.entries[18].best_mode == .AM &&
		            rep.entries[30].active && rep.entries[30].best_mode == .FM &&
		            rep.entries[33].active && rep.entries[33].best_mode == .USB &&
		            rep.entries[37].active && rep.entries[37].best_mode == .LSB

		// Verify Closed-Loop Auto-LNA & Auto-RF-Gain + RSPdx-R2 27 MHz LNA Limit (0..19)
		rx_g  := new(Receiver_Context)
		eng_g := new(SDR_Engine)
		defer free(rx_g)
		defer free(eng_g)
		receiver_context_init(rx_g)
		rx_g.sim_mode       = true
		eng_g.rx_ctx        = rx_g
		eng_g.streaming     = true
		eng_g.dev.hwVer     = sdr.SDRPLAY_RSPdxR2_ID

		// Simulate ADC overload (-10 dBFS, overload_events=1) -> Auto-LNA must step UP & Auto-RF-Gain must step UP
		rx_g.tel.callbacks_cnt   = 10
		rx_g.tel.overload_events = 1
		rx_g.tel.adc_dbfs        = -10.0
		rx_g.tel.adc_peak_pct    = 96.0
		sdr_engine_auto_gain_tick(eng_g)
		ovl_lna := rx_g.cfg.lna_state
		ovl_gr  := rx_g.cfg.rf_gr_db

		// Simulate weak antenna (-45 dBFS, 0 overloads, low gRdB=24) over 3 calm ticks -> Auto-LNA must step DOWN
		rx_g.cfg.rf_gr_db     = 24
		rx_g.tel.adc_dbfs     = -45.0
		rx_g.tel.adc_peak_pct = 10.0
		for _ in 0 ..< 3 {
			sdr_engine_auto_gain_tick(eng_g)
		}
		weak_lna := rx_g.cfg.lna_state
		weak_gr  := rx_g.cfg.rf_gr_db

		gain_ok := max_lna_state_for_27mhz(sdr.SDRPLAY_RSPdxR2_ID) == 19 &&
		           max_lna_state_for_27mhz(sdr.SDRPLAY_RSP1A_ID) == 6 &&
		           ovl_lna == 2 && ovl_gr > 32 &&
		           weak_lna < ovl_lna && weak_gr == 20

		both_ok := scan_ok && gain_ok
		all_ok  &= both_ok
		screen_printfln(
			"  [9] 40-Ch 3-Band Scanner & Auto-LNA : Active=%d/5 (AM/FM/USB/LSB), Ch9 Err=%.2f Hz, AutoLNA %d->%d -> %s",
			rep.active_count, ch09_err, ovl_lna, weak_lna, "PASS" if both_ok else "FAIL",
		)
	}

	// 10. Closed-Loop Multi-Mode Auto-Clarifier Test (AM +620Hz, FM -280Hz, USB +58Hz Voice, LSB -52Hz Voice & Birdie Immunity)
	{
		cf := new(Channel_Selectivity_Filter)
		dm := new(Demodulator_State)
		ac := new(Auto_Clarifier_State)
		defer free(cf)
		defer free(dm)
		defer free(ac)

		aud_blk : [CHAN_HOP_B]f64

		run_autoclar_case :: proc(
			cf       : ^Channel_Selectivity_Filter,
			dm       : ^Demodulator_State,
			ac       : ^Auto_Clarifier_State,
			aud_blk  : []f64,
			mode     : Modulation_Mode,
			tx_off   : f64,
			f0_voice : f64,
			noise_only: bool,
		) -> f64 {
			channel_filter_init(cf)
			demod_init(dm)
			auto_clarifier_init(ac)

			clar_hz := 0.0
			carr_ph := 0.0
			fm_mod  := 0.0
			rng     : u64 = 0xCAFE_BABE_1234_5678 + u64(int(mode) * 997)

			// Simulate 48 blocks (~1.02 s) at 48 kHz complex baseband with live closed-loop clarifier feedback
			for n in 0 ..< (48 * CHAN_HOP_B) {
				t_s := f64(n) / BASEBAND_RATE_HZ
				gn1, gn2 := xorshift_f64_normal(&rng)
				z_bb : complex128

				if noise_only {
					// Empty USB noise floor + strong 1385 Hz unmodulated indoor SMPS birdie (must NOT move clarifier!)
					bird := 0.06 * math.cos(2.0 * math.PI * 1385.0 * t_s)
					bsin := 0.06 * math.sin(2.0 * math.PI * 1385.0 * t_s)
					z_bb  = complex(0.015 * gn1 + bird, 0.015 * gn2 + bsin)
				} else {
					res_off := tx_off - clar_hz
					carr_ph = wrap_pi_f64(carr_ph + (2.0 * math.PI * res_off) / BASEBAND_RATE_HZ)
					cc := math.cos(carr_ph)
					cs := math.sin(carr_ph)

					// Analytic voiced speech (12 harmonics with realistic formant envelope, zero carrier in SSB!)
					vre := 0.0
					vim := 0.0
					for h in 1 ..= 12 {
						fh  := f64(h) * f0_voice
						d1  := (fh - 560.0) / 170.0
						d2  := (fh - 1540.0) / 260.0
						amp := (1.0 / (1.0 + d1 * d1) + 0.65 / (1.0 + d2 * d2)) / math.pow(f64(h), 0.30)
						ph  := 2.0 * math.PI * fh * t_s + 0.23 * f64(h)
						vre += amp * math.cos(ph)
						vim += amp * math.sin(ph)
					}
					vre *= 0.08
					vim *= 0.08

					switch mode {
					case .AM:
						env := 0.25 * (1.0 + 2.2 * vre)
						z_bb = complex(env * cc + 0.004 * gn1, env * cs + 0.004 * gn2)
					case .FM:
						fm_mod = wrap_pi_f64(fm_mod + (2.0 * math.PI * 1400.0 * vre * 3.0) / BASEBAND_RATE_HZ)
						tot_p := carr_ph + fm_mod
						z_bb   = complex(0.25 * math.cos(tot_p) + 0.004 * gn1, 0.25 * math.sin(tot_p) + 0.004 * gn2)
					case .USB:
						z_bb = complex((vre * cc - vim * cs) + 0.006 * gn1, (vre * cs + vim * cc) + 0.006 * gn2)
					case .LSB:
						z_bb = complex((vre * cc + vim * cs) + 0.006 * gn1, (vre * cs - vim * cc) + 0.006 * gn2)
					}
				}

				if channel_filter_push(cf, z_bb, mode) {
					demodulate_block_f64(dm, cf.out_ready[:], mode, aud_blk, cf.noise_power_lin)
					d_hz, do_adj := auto_clarifier_step(ac, dm, cf, aud_blk, mode, true)
					if do_adj {
						clar_hz = clamp(clar_hz + d_hz, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
					}
				}
			}
			return clar_hz
		}

		c_am    := run_autoclar_case(cf, dm, ac, aud_blk[:], .AM,  +620.0, 140.0, false)
		c_fm    := run_autoclar_case(cf, dm, ac, aud_blk[:], .FM,  -280.0, 140.0, false)
		c_usb   := run_autoclar_case(cf, dm, ac, aud_blk[:], .USB,  +58.0, 142.0, false)
		c_lsb   := run_autoclar_case(cf, dm, ac, aud_blk[:], .LSB,  -52.0, 148.0, false)
		c_noise := run_autoclar_case(cf, dm, ac, aud_blk[:], .USB,    0.0, 142.0, true)

		err_am  := math.abs(c_am  - (+620.0))
		err_fm  := math.abs(c_fm  - (-280.0))
		err_usb := math.abs(c_usb - (+58.0))
		err_lsb := math.abs(c_lsb - (-52.0))
		err_ns  := math.abs(c_noise)

		ac_ok := err_am < 2.0 && err_fm < 8.0 && err_usb < 3.0 && err_lsb < 3.0 && err_ns < 0.1
		all_ok &= ac_ok
		screen_printfln(
			"  [10] Closed-Loop f64 Auto-Clarifier : AM=%+.1fHz FM=%+.1fHz USB=%+.1fHz LSB=%+.1fHz Idle=%+.1fHz -> %s",
			c_am, c_fm, c_usb, c_lsb, c_noise, "PASS" if ac_ok else "FAIL",
		)
	}

	// 11. 40-Ch Braille 60s Waterfall ('sp'/'spetrum') & Adaptive Multi-Mode f64 Auto-Squelch
	{
		// (A) Verify Braille UTF-8 & monotonic 0..8 dot dB intensity ramp
		db_steps := [9]f64{0.0, 2.5, 6.0, 9.0, 13.0, 17.0, 22.0, 27.0, 34.0}
		braille_ok := true
		prev_dots  := -1
		utf8_tmp   : [3]u8
		for st in db_steps {
			m, nd := braille_db_to_mask(st)
			n_b   := encode_braille_utf8(utf8_tmp[:], 0, m)
			if nd <= prev_dots || n_b != 3 || utf8_tmp[0] != 0xE2 || (utf8_tmp[1] < 0xA0 || utf8_tmp[1] > 0xA3) {
				braille_ok = false
			}
			prev_dots = nd
		}

		// (B) Feed 60 full seconds of 6.0 MSPS 14-bit multi-channel RF while listening on Ch 27 (LO = 27.225 MHz)
		wf := new(Braille_Waterfall_State)
		defer free(wf)
		waterfall_init(wf)

		PKT_N :: 30_000 // Exact 5.0 ms period (every 200 Hz harmonic is 100% phase-continuous across packet boundaries)
		xi_w  := make([]i16, PKT_N)
		xq_w  := make([]i16, PKT_N)
		defer delete(xi_w)
		defer delete(xq_w)

		lo_27 := hardware_lo_freq_hz(27) // 27.225000 MHz
		f_c19 := channel_to_freq_hz(19) - lo_27            // -40.0 kHz (AM)
		f_c31 := channel_to_freq_hz(31) - lo_27            // +90.0 kHz (FM)
		f_c34 := (channel_to_freq_hz(34) + 1400.0) - lo_27 // +121.4 kHz (USB upper half!)
		f_c38 := (channel_to_freq_hz(38) - 1400.0) - lo_27 // +158.6 kHz (LSB lower half!)

		rng_w : u64 = 0x1357_9BDF_2468_ACE0
		for n in 0 ..< PKT_N {
			t_s := f64(n) / SDR_SAMPLE_RATE_HZ
			p19 := 2.0 * math.PI * math.mod(f_c19 * t_s, 1.0)
			p31 := 2.0 * math.PI * math.mod(f_c31 * t_s, 1.0) + 1.4 * math.sin(2.0 * math.PI * 1000.0 * t_s)
			p34 := 2.0 * math.PI * math.mod(f_c34 * t_s, 1.0)
			p38 := 2.0 * math.PI * math.mod(f_c38 * t_s, 1.0)
			g1, g2 := xorshift_f64_normal(&rng_w)

			si := 0.055 * (1.0 + 0.6 * math.cos(2.0 * math.PI * 800.0 * t_s)) * math.cos(p19) +
			      0.060 * math.cos(p31) + 0.055 * math.cos(p34) + 0.055 * math.cos(p38) + 0.022 * g1
			sq := 0.055 * (1.0 + 0.6 * math.cos(2.0 * math.PI * 800.0 * t_s)) * math.sin(p19) +
			      0.060 * math.sin(p31) + 0.055 * math.sin(p34) + 0.055 * math.sin(p38) + 0.022 * g2

			xi_w[n] = i16(clamp(math.round(si * 8191.0) * 4.0, -32767.0, 32767.0))
			xq_w[n] = i16(clamp(math.round(sq * 8191.0) * 4.0, -32767.0, 32767.0))
		}

		// Stream 60 full seconds (60 * 200 = 12,000 packets = 360,000,000 samples) into 60s history ring
		for _ in 0 ..< (60 * 200) {
			waterfall_process_iq_6m(wf, &xi_w[0], &xq_w[0], PKT_N, lo_27)
		}

		newest_slot := (wf.head - 1 + WF_HISTORY_SECONDS) % WF_HISTORY_SECONDS
		sl_new      := wf.history[newest_slot]
		flr         := sl_new.floor_half_dbfs
		_, quiet_dots := braille_db_to_mask(sl_new.half_dbfs[4][0] - flr) // Ch 5 quiet

		wf_ok := braille_ok &&
		         wf.history_count == 60 &&
		         sl_new.valid &&
		         quiet_dots == 0 &&
		         (sl_new.half_dbfs[18][0] - flr) > 14.0 && (sl_new.half_dbfs[18][1] - flr) > 14.0 && // Ch 19 AM both halves
		         (sl_new.half_dbfs[30][0] - flr) > 14.0 && (sl_new.half_dbfs[30][1] - flr) > 14.0 && // Ch 31 FM both halves
		         (sl_new.half_dbfs[33][1] - sl_new.half_dbfs[33][0]) > 12.0 &&                       // Ch 34 USB right half!
		         (sl_new.half_dbfs[37][0] - sl_new.half_dbfs[37][1]) > 12.0                          // Ch 38 LSB left half!

		// (C) Verify Adaptive Multi-Mode f64 Auto-Squelch (gain-step immunity, voice open, syllabic bridge, tail mute)
		cf_s := new(Channel_Selectivity_Filter)
		dm_s := new(Demodulator_State)
		defer free(cf_s)
		defer free(dm_s)
		channel_filter_init(cf_s)
		demod_init(dm_s)

		aud_s   : [CHAN_HOP_B]f64
		rng_sq  : u64 = 0x8877_6655_4433_2211
		muted_on_noise := true
		muted_on_jump  := true
		opened_on_tx   := false
		held_pause     := true
		closed_after   := false

		// 120 blocks (~2.56 s):
		//   blk 0..14  : baseline thermal noise -> must close & stay MUTED
		//   blk 15..29 : +12 dB sudden RF gain jump (4x amplitude noise) -> must stay MUTED!
		//   blk 30..54 : USB voiced speech (+10 dB SNR) -> must OPEN!
		//   blk 55..66 : 256 ms inter-word pause -> must HOLD OPEN (480 ms SSB syllabic bridge)!
		//   blk 67..119: end of transmission (noise only > 1.1 s) -> must CLOSE!
		for blk in 0 ..< 120 {
			n_amp := 0.010 if blk < 15 else 0.040 // +12 dB wideband gain jump at blk 15!
			has_voice := (blk >= 30 && blk <= 54)
			for i in 0 ..< CHAN_HOP_B {
				t_s := f64(blk * CHAN_HOP_B + i) / BASEBAND_RATE_HZ
				g1, g2 := xorshift_f64_normal(&rng_sq)
				zr := g1 * n_amp
				zi := g2 * n_amp
				if has_voice {
					for h in 2 ..= 10 {
						ang := 2.0 * math.PI * (f64(h) * 155.0) * t_s
						zr += 0.032 * math.cos(ang)
						zi += 0.032 * math.sin(ang)
					}
				}
				if channel_filter_push(cf_s, complex(zr, zi), .USB) {
					demodulate_block_f64(dm_s, cf_s.out_ready[:], .USB, aud_s[:], cf_s.noise_power_lin)
					vc_sim := 0.75 if has_voice else 0.05
					post_process_audio_block_f64(
						dm_s, aud_s[:], cf_s.sig_dbfs, cf_s.snr_db, vc_sim,
						true, -98.0, 0.90, .Level_2,
						true, cf_s.noise_dbfs, .USB, has_voice,
					)
				}
			}
			if blk >= 5  && blk <= 14 && dm_s.sq_is_open do muted_on_noise = false
			if blk >= 15 && blk <= 29 && dm_s.sq_is_open do muted_on_jump  = false
			if blk >= 36 && blk <= 54 && dm_s.sq_is_open do opened_on_tx   = true
			if blk >= 55 && blk <= 66 && !dm_s.sq_is_open do held_pause    = false
			if blk == 119 && !dm_s.sq_is_open do closed_after = true
		}

		sq_ok   := muted_on_noise && muted_on_jump && opened_on_tx && held_pause && closed_after
		st11_ok := wf_ok && sq_ok
		all_ok  &= st11_ok
		screen_printfln(
			"  [11] Braille 60s Waterfall & Auto-Sq: Hist=%d/60s USB/LSB Split=OK | AutoSq(Jump/Open/Hold/Mute)=%s -> %s",
			wf.history_count, "OK" if sq_ok else "ERR", "PASS" if st11_ok else "FAIL",
		)
	}

	// 12. CLI Command History & Arrow Up/Down/Left/Right + Backspace/Delete + Enter Line Editor Verification
	{
		hist := new(CLI_Command_History)
		ed   := new(CLI_Line_Editor)
		defer free(hist)
		defer free(ed)

		feed_str :: proc(ed: ^CLI_Line_Editor, hist: ^CLI_Command_History, s: string) -> Line_Edit_Action {
			last := Line_Edit_Action.Continue
			for i in 0 ..< len(s) {
				last = line_editor_feed_byte(ed, hist, s[i])
			}
			return last
		}

		// Populate history with 3 previous commands (plus a duplicate to verify dedup)
		cli_history_push(hist, "ch 27")
		cli_history_push(hist, "mode usb")
		cli_history_push(hist, "sp on")
		cli_history_push(hist, "sp on") // consecutive duplicate must be ignored
		dedup_ok := (hist.count == 3)

		// Start typing an unsubmitted draft "vol 80", then press Up x3 -> Down x1 -> Down x2 (draft preserved!) -> Up x2 ("mode usb")
		line_editor_reset(ed, hist)
		_ = feed_str(ed, hist, "vol 80")
		_ = feed_str(ed, hist, "\x1b[A\x1b[A\x1b[A") // Up x3 -> oldest "ch 27"
		up3_ok := (string(ed.buf[:ed.length]) == "ch 27")

		_ = feed_str(ed, hist, "\x1b[B\x1b[B\x1b[B") // Down x3 -> back to unsubmitted draft "vol 80"
		draft_ok := (string(ed.buf[:ed.length]) == "vol 80")

		_ = feed_str(ed, hist, "\x1b[A\x1b[A") // Up x2 -> "mode usb"
		recall_ok := (string(ed.buf[:ed.length]) == "mode usb")

		// Erase "usb" with Backspace (0x7F x3), type "lsb" -> "mode lsb"
		_ = feed_str(ed, hist, "\x7f\x7f\x7flsb")
		bksp_ok := (string(ed.buf[:ed.length]) == "mode lsb")

		// Move Left x3 (\x1b[D x3), forward-Delete x3 (\x1b[3~ x3), insert "fm", then mid-line insert:
		// Home (\x1b[H), Right x4 (\x1b[C x4), check cursor, End (\x1b[F), Enter (\r)
		_ = feed_str(ed, hist, "\x1b[D\x1b[D\x1b[D")
		_ = feed_str(ed, hist, "\x1b[3~\x1b[3~\x1b[3~")
		_ = feed_str(ed, hist, "fm")
		_ = feed_str(ed, hist, "\x1b[H\x1b[C\x1b[C\x1b[C\x1b[C")
		cursor_mid_ok := (ed.cursor == 4 && string(ed.buf[:ed.length]) == "mode fm")
		submit_act    := feed_str(ed, hist, "\x1b[F\r")
		final_cmd     := string(ed.buf[:ed.length])
		cli_history_push(hist, final_cmd)

		st12_ok := dedup_ok && up3_ok && draft_ok && recall_ok && bksp_ok &&
		           cursor_mid_ok && submit_act == .Submit && final_cmd == "mode fm" && hist.count == 4
		all_ok &= st12_ok
		screen_printfln(
			"  [12] CLI Up/Down History & Line Edit: Recall='mode usb' -> Edit='%s' (Hist=%d) -> %s",
			final_cmd, hist.count, "PASS" if st12_ok else "FAIL",
		)
	}

	// 13. Dedicated-Thread Double-Buffered Screen Engine & Lock-Free SPSC Audio Ring Verification
	{
		// Flush queued test output so far and verify background Screen Printer Thread Front/Back swaps
		screen_flush()
		sp_stats  := screen_get_stats()
		screen_ok := sp_stats.swaps_count > 0 && sp_stats.bytes_written > 0 && sp_stats.dropped_bytes == 0

		// Verify Lock-Free SPSC Audio Ring Buffer (262,144 samples, 64 ms pre-fill cushion, 0 underruns, 0 overflows)
		rx_r   := new(Receiver_Context)
		fake_d := new(miniaudio.device)
		defer free(rx_r)
		defer free(fake_d)
		receiver_context_init(rx_r)
		fake_d.pUserData = rawptr(rx_r)
		sync.atomic_store_explicit(&rx_r.audio_consumer_on, true, .Release)

		in_blk  : [1024]f64
		out_i32 : [2048]f32 // Dual-mono stereo frames (1024 * 2)

		// Pre-fill 8 blocks (8,192 samples >= AUDIO_PREFILL_SAMPLES = 6,144) before consumer pulls,
		// then stream 200 steady-state producer/consumer blocks (204,800 samples = 4.26 seconds @ 48 kHz)
		seq_w : int = 0
		seq_r : int = 0
		samples_exact := true

		for pre in 0 ..< 8 {
			_ = pre
			for i in 0 ..< 1024 {
				in_blk[i] = 0.5 * math.sin(0.01 * f64(seq_w))
				seq_w += 1
			}
			audio_ring_push_f64(rx_r, in_blk[:])
		}

		for step in 0 ..< 200 {
			_ = step
			for i in 0 ..< 1024 {
				in_blk[i] = 0.5 * math.sin(0.01 * f64(seq_w))
				seq_w += 1
			}
			audio_ring_push_f64(rx_r, in_blk[:])
			cb_miniaudio_data_callback(fake_d, rawptr(&out_i32[0]), nil, 1024)
			for i in 0 ..< 1024 {
				exp_f32 := f32(0.5 * math.sin(0.01 * f64(seq_r)))
				if out_i32[i * 2] != exp_f32 || out_i32[i * 2 + 1] != exp_f32 {
					samples_exact = false
				}
				seq_r += 1
			}
		}

		und_cnt := sync.atomic_load_explicit(&rx_r.underrun_cnt, .Relaxed)
		ovf_cnt := sync.atomic_load_explicit(&rx_r.overflow_cnt, .Relaxed)
		ring_ok := rx_r.ring_primed && samples_exact && und_cnt == 0 && ovf_cnt == 0
		st13_ok := screen_ok && ring_ok
		all_ok  &= st13_ok
		screen_printfln(
			"  [13] Double-Buf Screen & SPSC Ring  : Swaps=%d Drop=%d | 204k Frames Und=%d Ovf=%d -> %s",
			sp_stats.swaps_count, sp_stats.dropped_bytes, und_cnt, ovf_cnt, "PASS" if st13_ok else "FAIL",
		)
	}

	// 14. Raw 6.0 MSPS 14-Bit IQ_file_xxxx.samples Save, Auto-Increment, Read/Play/Stop/Seek, Timestamps & 40-Ch Playback
	{
		test_dir :: "/tmp/cb_iq_selftest_32b0"
		_ = os.remove_all(test_dir)
		_ = os.make_directory_all(test_dir)
		defer _ = os.remove_all(test_dir)

		rx_iq  := new(Receiver_Context)
		plan1k := new(FFT_Plan)
		defer {
			receiver_context_shutdown(rx_iq)
			free(rx_iq)
			free(plan1k)
		}
		receiver_context_init(rx_iq)
		fft_plan_init(plan1k, 1024)

		rx_iq.cfg.channel         = 19
		rx_iq.cfg.mode            = .AM
		rx_iq.cfg.filter_level    = .Off
		rx_iq.cfg.squelch_enabled = false
		rx_iq.cfg.auto_squelch    = false
		rx_iq.cfg.auto_clarifier  = false
		rx_iq.cfg.clarifier_hz    = 0.0

		t0_ns := civil_to_unix_ns(2026, 11, 14, 21, 30, 0, 0) // 2026-11-14 21:30:00.000 UTC
		lo_19 := hardware_lo_freq_hz(19)                       // 27,135,000.0 Hz

		// Start Recording #1 -> must create IQ_file_0000.samples
		p0_raw, ok_sv0 := iq_save_start(&rx_iq.iq_eng, test_dir, false, t0_ns)
		p0_buf : [256]u8
		p0_len := min(len(p0_raw), len(p0_buf))
		copy(p0_buf[:p0_len], p0_raw[:p0_len])
		path_0000 := string(p0_buf[:p0_len])
		name_0_ok := ok_sv0 && strings.has_suffix(path_0000, "IQ_file_0000.samples")

		// Synthesize 25 blocks (0.25 s = 1,500,000 samples @ 6.0 MSPS) carrying 4 simultaneous CB channels:
		//   - Ch 19 (27.185 MHz, +50 kHz):  AM  700 Hz
		//   - Ch 31 (27.315 MHz, +180 kHz): FM  1000 Hz
		//   - Ch 34 (27.345 MHz, +210 kHz): USB 1200 Hz
		//   - Ch 38 (27.385 MHz, +250 kHz): LSB 900 Hz
		NUM_REC_BLKS :: 25
		xi_blk := make([]i16, IQ_BLOCK_SAMPLES)
		xq_blk := make([]i16, IQ_BLOCK_SAMPLES)
		defer delete(xi_blk)
		defer delete(xq_blk)

		f_off_19 := channel_to_freq_hz(19) - lo_19
		f_off_31 := channel_to_freq_hz(31) - lo_19
		f_off_34 := channel_to_freq_hz(34) - lo_19
		f_off_38 := channel_to_freq_hz(38) - lo_19
		fm_ph_31 := 0.0

		for b in 0 ..< NUM_REC_BLKS {
			blk_ts_ns := t0_ns + i64(b) * 10_000_000
			base_s    := b * IQ_BLOCK_SAMPLES
			for n in 0 ..< IQ_BLOCK_SAMPLES {
				t_s := f64(base_s + n) / SDR_SAMPLE_RATE_HZ

				// 1. Ch 19 AM (700 Hz)
				env19 := 0.16 * (1.0 + 0.70 * math.cos(2.0 * math.PI * 700.0 * t_s))
				p19   := 2.0 * math.PI * f_off_19 * t_s
				si    := env19 * math.cos(p19)
				sq_v  := env19 * math.sin(p19)

				// 2. Ch 31 FM (1000 Hz, +-2.0 kHz dev)
				fm_ph_31 += (2.0 * math.PI * 2000.0 * math.cos(2.0 * math.PI * 1000.0 * t_s)) / SDR_SAMPLE_RATE_HZ
				p31      := 2.0 * math.PI * f_off_31 * t_s + fm_ph_31
				si       += 0.16 * math.cos(p31)
				sq_v     += 0.16 * math.sin(p31)

				// 3. Ch 34 USB (+1200 Hz analytic)
				p34  := 2.0 * math.PI * (f_off_34 + 1200.0) * t_s
				si   += 0.16 * math.cos(p34)
				sq_v += 0.16 * math.sin(p34)

				// 4. Ch 38 LSB (-900 Hz analytic)
				p38  := 2.0 * math.PI * (f_off_38 - 900.0) * t_s
				si   += 0.16 * math.cos(p38)
				sq_v += 0.16 * math.sin(p38)

				xi_blk[n] = i16(math.round(clamp(si, -0.95, 0.95) * 8191.0) * 4.0)
				xq_blk[n] = i16(math.round(clamp(sq_v, -0.95, 0.95) * 8191.0) * 4.0)
			}
			receiver_process_sdr_samples(rx_iq, &xi_blk[0], &xq_blk[0], IQ_BLOCK_SAMPLES, lo_19, blk_ts_ns, false)
		}

		stop0_ok  := iq_save_stop(&rx_iq.iq_eng, false)
		drops_0   := sync.atomic_load_explicit(&rx_iq.iq_eng.rec_dropped_blocks, .Relaxed)

		// Start Recording #2 -> must auto-increment to IQ_file_0001.samples!
		p1_raw, ok_sv1 := iq_save_start(&rx_iq.iq_eng, test_dir, false, t0_ns + 5_000_000_000)
		p1_buf : [256]u8
		p1_len := min(len(p1_raw), len(p1_buf))
		copy(p1_buf[:p1_len], p1_raw[:p1_len])
		path_0001 := string(p1_buf[:p1_len])
		receiver_process_sdr_samples(rx_iq, &xi_blk[0], &xq_blk[0], IQ_BLOCK_SAMPLES, lo_19, t0_ns + 5_000_000_000, false)
		stop1_ok  := iq_save_stop(&rx_iq.iq_eng, false)
		name_1_ok := ok_sv1 && strings.has_suffix(path_0001, "IQ_file_0001.samples") && stop1_ok

		// Read/Load IQ_file_0000.samples paused at 00:00.00
		open_ok := iq_file_open(&rx_iq.iq_eng, path_0000, false, false)
		hdr_ok  := open_ok &&
		           rx_iq.iq_eng.play_info.total_blocks == NUM_REC_BLKS &&
		           rx_iq.iq_eng.play_info.start_ns == t0_ns &&
		           math.abs(rx_iq.iq_eng.play_info.duration_sec - 0.25) < 1e-6

		tel_0   := receiver_get_telemetry(rx_iq)
		dt_buf0 : [48]u8
		dt_str0 := format_datetime_utc(tel_0.stream_timestamp_ns, dt_buf0[:], true)
		ts0_ok  := tel_0.iq_file_mode && !tel_0.iq_playing &&
		           tel_0.stream_timestamp_ns == t0_ns &&
		           dt_str0 == "2026-11-14 21:30:00.000 UTC"

		// Test seek by HH:MM:SS.mmm, full ISO Date+Time, forward (+0.06s), backward (-0.04s), play & stop!
		s_hms, ok_hms := parse_seek_timestamp("21:30:00.120", t0_ns, 0.25)
		s_iso, ok_iso := parse_seek_timestamp("2026-11-14 21:30:00.180", t0_ns, 0.25)
		seek1_ok := ok_hms && iq_seek_to_seconds(&rx_iq.iq_eng, s_hms, false) &&
		            receiver_get_telemetry(rx_iq).stream_timestamp_ns == (t0_ns + 120_000_000)
		fwd_ok   := iq_seek_relative(&rx_iq.iq_eng, +0.06, false) &&
		            receiver_get_telemetry(rx_iq).stream_timestamp_ns == (t0_ns + 180_000_000) &&
		            ok_iso && math.abs(s_iso - 0.18) < 1e-4
		bwd_ok   := iq_seek_relative(&rx_iq.iq_eng, -0.08, false) &&
		            receiver_get_telemetry(rx_iq).stream_timestamp_ns == (t0_ns + 100_000_000)

		// Test Play -> advance real-time blocks -> Stop/Pause
		play_ok  := iq_play_cmd(&rx_iq.iq_eng, "", false) && iq_is_playing(&rx_iq.iq_eng)
		time.sleep(45 * time.Millisecond)
		pause_ok := iq_stop_pause_cmd(&rx_iq.iq_eng, false) && !iq_is_playing(&rx_iq.iq_eng)
		blk_after_play := sync.atomic_load_explicit(&rx_iq.iq_eng.play_cur_block, .Acquire)
		transport_ok   := seek1_ok && fwd_ok && bwd_ok && play_ok && pause_ok && blk_after_play >= 11

		// Verify all 4 recorded CB channels (AM, FM, USB, LSB) demodulate exact tones from IQ_file_0000.samples!
		IQ_Ch_Check :: struct {
			ch   : int,
			mode : Modulation_Mode,
			tone : f64,
		}
		checks := [4]IQ_Ch_Check{
			{19, .AM,  700.0},
			{31, .FM,  1000.0},
			{34, .USB, 1200.0},
			{38, .LSB, 900.0},
		}
		disk_slot  := new(IQ_Raw_Disk_Slot)
		aud_check  := make([]f64, 1024)
		defer free(disk_slot)
		defer delete(aud_check)

		sync.atomic_store_explicit(&rx_iq.audio_consumer_on, true, .Release)
		multich_ok := true
		for chk in checks {
			rx_iq.cfg.channel = chk.ch
			rx_iq.cfg.mode    = chk.mode
			frontend_init(&rx_iq.frontend, 0.0)
			channel_filter_reset(&rx_iq.chan_filter)
			demod_init(&rx_iq.demod)
			iq_flush_audio_ring_clean(rx_iq)

			// Feed 16 recorded 10ms blocks (0.16s = 7,680 samples @ 48 kHz) straight from IQ_file_0000.samples
			for b in 0 ..< 16 {
				off := i64(IQ_HEADER_BYTES) + i64(b) * i64(IQ_BLOCK_DISK_BYTES)
				nr, rerr := os.read_at(rx_iq.iq_eng.play_file, disk_slot.raw[:], off)
				if rerr != nil || nr != IQ_BLOCK_DISK_BYTES {
					multich_ok = false
					break
				}
				meta, mok := iq_slot_unpack_header(disk_slot)
				if !mok {
					multich_ok = false
					break
				}
				xi_s := iq_slot_xi_slice(disk_slot)
				xq_s := iq_slot_xq_slice(disk_slot)
				receiver_process_sdr_samples(rx_iq, &xi_s[0], &xq_s[0], meta.valid_samples, meta.lo_freq_hz, meta.timestamp_ns, true)
			}

			// Read steady-state audio hop from SPSC ring and verify exact demodulated tone frequency!
			r_idx := sync.atomic_load_explicit(&rx_iq.ring_read,  .Relaxed)
			w_idx := sync.atomic_load_explicit(&rx_iq.ring_write, .Acquire)
			avail := int(w_idx - r_idx)
			if avail >= 6144 {
				start_r := r_idx + u32(avail - 1536)
				for k in 0 ..< 1024 {
					aud_check[k] = f64(rx_iq.audio_ring[(start_r + u32(k)) & AUDIO_RING_MASK])
				}
				pk_f, _, _, _ := measure_audio_tone_fidelity_1024(plan1k, aud_check, chk.tone)
				if math.abs(pk_f - chk.tone) > 2.5 {
					multich_ok = false
				}
			} else {
				multich_ok = false
			}
		}
		sync.atomic_store_explicit(&rx_iq.audio_consumer_on, false, .Release)

		// Close IQ file & verify clean return to LIVE mode
		live_ok := iq_close_return_to_live(&rx_iq.iq_eng, nil, false) && !iq_is_file_mode(&rx_iq.iq_eng)

		st14_ok := name_0_ok && stop0_ok && drops_0 == 0 && name_1_ok && hdr_ok && ts0_ok && transport_ok && multich_ok && live_ok
		all_ok  &= st14_ok
		screen_printfln(
			"  [14] IQ_file_0000/0001 Save & Play  : Drops=%d | TS=%s | Seek/Play/Stop & 4-Band RX=OK -> %s",
			drops_0, dt_str0[:19], "PASS" if st14_ok else "FAIL",
		)
	}

	// 15. Ch 25 (27.245000 MHz LSB) Distant Nighttime Multi-Syllable Speech Intelligibility & Sub-Noise-Floor Recovery
	{
		fe     := new(FrontEnd_Decimator)
		cf     := new(Channel_Selectivity_Filter)
		dm     := new(Demodulator_State)
		ac     := new(Auto_Clarifier_State)
		bank   := new(Audio_Filter_Bank)
		plan1k := new(FFT_Plan)
		plan4k := new(FFT_Plan)
		defer free(fe)
		defer free(cf)
		defer free(dm)
		defer free(ac)
		defer free(bank)
		defer free(plan1k)
		defer free(plan4k)
		fft_plan_init(plan1k, 1024)
		fft_plan_init(plan4k, 4096)

		// Part A: Verify Ch 25 (27.245000 MHz, LO 27.195000 MHz) 6.0 MSPS 14-Bit LSB wideband impulse blanking
		N_WB :: 720_000 // 120 ms @ 6.0 MSPS -> 5,760 samples @ 48 kHz
		xi_wb  := make([]i16, N_WB)
		xq_wb  := make([]i16, N_WB)
		iq_wb  := make([]complex128, (N_WB / 125) + 64)
		wb_aud := make([]f64, 6144)
		wb_sp  := make([]complex128, 4096)
		defer delete(xi_wb)
		defer delete(xq_wb)
		defer delete(iq_wb)
		defer delete(wb_aud)
		defer delete(wb_sp)

		f_ch25    := channel_to_freq_hz(25)  // 27,245,000.0 Hz
		f_lo25    := hardware_lo_freq_hz(25) // 27,195,000.0 Hz (-50 kHz low-side injection)
		// A 130 Hz male fundamental on Ch 25 LSB sits at 27,245,000 - 130 = 27,244,870 Hz (+49,870 Hz above LO!)
		f_if_130  := (f_ch25 - 130.0) - f_lo25
		rng_wb    : u64 = 0x2525_CB25_DEAD_BEEF
		for n in 0 ..< N_WB {
			t_s    := f64(n) / SDR_SAMPLE_RATE_HZ
			g1, g2 := xorshift_f64_normal(&rng_wb)
			ang    := 2.0 * math.PI * math.mod(f_if_130 * t_s, 1.0)
			// Inject severe 2 us indoor ignition/triac RF impulse bursts every 1 ms (6,000 samples @ 6.0 MSPS)
			rem    := n % 6000
			imp    := 7200.0 if (rem >= 250 && rem < 262) else 0.0
			xi_wb[n] = i16(clamp(math.round((240.0 * math.cos(ang) + 180.0 * g1 + imp) * 4.0), -32760.0, 32760.0))
			xq_wb[n] = i16(clamp(math.round((240.0 * math.sin(ang) + 180.0 * g2 - imp) * 4.0), -32760.0, 32760.0))
		}
		frontend_init(fe, 0.0)
		channel_filter_init(cf)
		demod_init(dm)
		n48_wb  := frontend_process_iq_block(fe, xi_wb, xq_wb, 0.0, .Level_3, iq_wb)
		wb_blks := 0
		wb_pos  := 0
		for k in 0 ..< n48_wb {
			if channel_filter_push(cf, iq_wb[k], .LSB) {
				if wb_pos + 1024 <= len(wb_aud) {
					demodulate_block_f64(dm, cf.out_ready[:], .LSB, wb_aud[wb_pos : wb_pos + 1024], cf.noise_power_lin)
					wb_pos  += 1024
					wb_blks += 1
				}
			}
		}
		st_wb := max(wb_pos - 4096, 0)
		for i in 0 ..< 4096 {
			wb_sp[i] = complex(wb_aud[st_wb + i] * blackman_harris_f64(i, 4096), 0.0)
		}
		fft_forward(plan4k, wb_sp)
		bk_wb, bp_wb := 8, 0.0
		for k in 8 ..= 20 {
			p := c64_abs2(wb_sp[k])
			if p > bp_wb {
				bp_wb = p
				bk_wb = k
			}
		}
		lm_wb  := 0.5 * math.ln(math.max(c64_abs2(wb_sp[bk_wb - 1]), 1e-30))
		l0_wb  := 0.5 * math.ln(math.max(c64_abs2(wb_sp[bk_wb]),     1e-30))
		lp_wb  := 0.5 * math.ln(math.max(c64_abs2(wb_sp[bk_wb + 1]), 1e-30))
		den_wb := lm_wb - 2.0 * l0_wb + lp_wb
		del_wb := clamp(0.5 * (lm_wb - lp_wb) / den_wb, -0.5, 0.5) if math.abs(den_wb) > 1e-18 else 0.0
		pk_130 := (f64(bk_wb) + del_wb) * (AUDIO_SAMPLE_RATE_HZ / 4096.0)
		ch25_rf_ok := f_ch25 == 27_245_000.0 && wb_blks >= 4 && fe.nb_blanked_cnt >= 20 && math.abs(pk_130 - 130.0) < 1.5

		// Part B: 96-Block (2.048 s) Distant Ch 25 LSB Multi-Syllable Conversation Utterance @ -12.0 dB Channel SNR
		// Includes gliding male prosody F0(t) = 126..156 Hz, moving vowel formants (/o/ -> /s-a/ -> /i/),
		// unvoiced 1.7-2.6 kHz sibilant consonant onset, inter-word pauses, -36 Hz rig mistune & 1420 Hz birdie!
		N_SYL_BLKS :: 112
		N_SYL_SAMP :: N_SYL_BLKS * CHAN_HOP_B // 114,688 samples (~2.389 s @ 48 kHz)
		clean_ref  := make([]f64, N_SYL_SAMP)
		iq_ch25    := make([]complex128, N_SYL_SAMP)
		out_audio  := make([]f64, N_SYL_SAMP)
		defer delete(clean_ref)
		defer delete(iq_ch25)
		defer delete(out_audio)

		ph_harm : [18]f64
		for h in 0 ..< 18 do ph_harm[h] = 0.31 * f64(h + 1)
		rng_cons : u64 = 0x7788_99AA_BBCC_DDEE
		cons_bp1, cons_bp2 : f64

		sig_iq_pow := 0.0
		sig_iq_cnt := 0
		iq_sig_re  := make([]f64, N_SYL_SAMP)
		iq_sig_im  := make([]f64, N_SYL_SAMP)
		defer delete(iq_sig_re)
		defer delete(iq_sig_im)

		for n in 0 ..< N_SYL_SAMP {
			t_s := f64(n) / BASEBAND_RATE_HZ
			env_v := 0.0
			env_c := 0.0
			f0_t  := 135.0
			f1_t  := 500.0
			f2_t  := 1200.0
			f3_t  := 2350.0

			if t_s >= 0.26 && t_s < 0.70 {
				// Syllable 1 (/bo/): warm male low F0 gliding 131 -> 138 Hz
				u     := (t_s - 0.26) / 0.44
				env_v  = math.sin(math.PI * clamp(u * 6.0, 0.0, 0.5)) * math.sin(math.PI * clamp((1.0 - u) * 5.0, 0.0, 0.5))
				f0_t   = 131.0 + 7.0 * u
				f1_t   = 470.0 + 60.0 * u
				f2_t   = 960.0 + 180.0 * u
				f3_t   = 2280.0
			} else if t_s >= 0.96 && t_s < 1.03 {
				// Unvoiced Portuguese sibilant '/s/' onset (1.7 - 2.55 kHz) before Syllable 2!
				u     := (t_s - 0.96) / 0.07
				env_c  = math.sin(math.PI * u) * 0.55
			} else if t_s >= 1.03 && t_s < 1.45 {
				// Syllable 2 (/sa/): open vowel with rising-falling intonation 138 -> 146 -> 142 Hz
				u     := (t_s - 1.03) / 0.42
				env_v  = math.sin(math.PI * clamp(u * 7.0, 0.0, 0.5)) * math.sin(math.PI * clamp((1.0 - u) * 5.0, 0.0, 0.5))
				f0_t   = 138.0 + 8.0 * math.sin(math.PI * u * 0.85)
				f1_t   = 730.0 - 90.0 * u
				f2_t   = 1340.0 + 140.0 * u
				f3_t   = 2390.0
			} else if t_s >= 1.72 && t_s < 2.08 {
				// Syllable 3 (/ti/): high-F2 front vowel clarity test (F2 = 2040 Hz!) with falling cadence 142 -> 133 Hz
				u     := (t_s - 1.72) / 0.36
				env_v  = math.sin(math.PI * clamp(u * 6.0, 0.0, 0.5)) * math.sin(math.PI * clamp((1.0 - u) * 4.5, 0.0, 0.5))
				f0_t   = 142.0 - 9.0 * u
				f1_t   = 360.0
				f2_t   = 2040.0
				f3_t   = 2520.0
			}

			vre := 0.0
			vim := 0.0
			if env_v > 1e-5 {
				max_h := min(int(math.floor(2680.0 / f0_t)), 18)
				for h in 1 ..= max_h {
					fh  := f64(h) * f0_t
					ph_harm[h - 1] = wrap_pi_f64(ph_harm[h - 1] + (2.0 * math.PI * fh) / BASEBAND_RATE_HZ)
					d1  := (fh - f1_t) / 155.0
					d2  := (fh - f2_t) / 230.0
					d3  := (fh - f3_t) / 260.0
					chest := 0.42 / math.pow(f64(h), 0.55)
					amp := chest + (1.0 / (1.0 + d1 * d1) + 0.72 / (1.0 + d2 * d2) + 0.42 / (1.0 + d3 * d3)) / math.pow(f64(h), 0.28)
					vre += env_v * amp * math.cos(ph_harm[h - 1])
					vim += env_v * amp * math.sin(ph_harm[h - 1])
				}
			}
			if env_c > 1e-5 {
				// Bandlimited 1.95 kHz sibilant burst + Hilbert-pair approximation
				wn, _ := xorshift_f64_normal(&rng_cons)
				// Resonator centered near 2050 Hz (w0 = 2*pi*2050/48000 = 0.2683 rad)
				r_pole := 0.94
				y_c    := wn - r_pole * r_pole * cons_bp2
				y_cur  := 2.0 * r_pole * math.cos(0.2683) * cons_bp1 - (r_pole * r_pole) * cons_bp2 + 0.12 * y_c
				cons_bp2 = cons_bp1
				cons_bp1 = y_cur
				vre += env_c * y_cur * 2.4
				vim += env_c * cons_bp2 * 2.4
			}

			clean_ref[n]  = vre * 0.08
			iq_sig_re[n]  = vre * 0.08
			iq_sig_im[n]  = vim * 0.08
			if env_v > 0.25 || env_c > 0.15 {
				sig_iq_pow += iq_sig_re[n] * iq_sig_re[n] + iq_sig_im[n] * iq_sig_im[n]
				sig_iq_cnt += 1
			}
		}

		// Model realistic suburban Portuguese nighttime Ch 25 LSB sub-noise-floor DX:
		//   - Pure in-band 2.675 kHz white Gaussian thermal static at +6.0 dB ABOVE active speech (SNR_thermal = -6.0 dB, 4x weaker than static)
		//   - Plus -1420 Hz in-band LSB RF heterodyne whistle (+1420 Hz audio), 100 Hz household wideband RF triac switching spikes,
		//     and 50/100/150 Hz European power-supply hum -> total interference+noise SNR = -10.0 dB below noise floor!
		active_sig_pow := sig_iq_pow / f64(max(sig_iq_cnt, 1))
		target_lsb_thermal_pow := active_sig_pow * math.pow(10.0, 6.0 / 10.0)
		sigma_iq := math.sqrt(target_lsb_thermal_pow / (2.0 * (2675.0 / BASEBAND_RATE_HZ)))

		tx_mistune_hz := -36.0 // Distant LSB rig 36 Hz off-frequency
		rng_ch25 : u64 = 0x1122_3344_5566_7788
		for n in 0 ..< N_SYL_SAMP {
			t_s    := f64(n) / BASEBAND_RATE_HZ
			gn1, gn2 := xorshift_f64_normal(&rng_ch25)
			c_off  := math.cos(2.0 * math.PI * tx_mistune_hz * t_s)
			s_off  := math.sin(2.0 * math.PI * tx_mistune_hz * t_s)
			vre    := iq_sig_re[n]
			vim    := iq_sig_im[n]
			// Analytic LSB modulation shifted by tx_mistune_hz: conj(v(t)) * exp(j*2*pi*tx_off*t)
			s_re   := vre * c_off + vim * s_off
			s_im   := vre * s_off - vim * c_off
			// Steady -1635 Hz in-band LSB RF heterodyne whistle + 100 Hz wideband RF triac switching transients
			wh_re  := sigma_iq * 0.115 * math.cos(-2.0 * math.PI * 1635.0 * t_s)
			wh_im  := sigma_iq * 0.115 * math.sin(-2.0 * math.PI * 1635.0 * t_s)
			imp    := 0.0
			if (n % 480) == 17 {
				imp = sigma_iq * 14.0 // 100 Hz household triac switching spike (>13 dB wideband IQ transient)
			}
			iq_ch25[n] = complex(s_re + sigma_iq * gn1 + wh_re + imp, s_im + sigma_iq * gn2 + wh_im - 0.5 * imp)
		}

		// Helper: compute 8-subband short-time envelope intelligibility correlation (STOI-like, 125..2720 Hz)
		// + post-AGC active-speech-to-interword-pause contrast (dB)
		eval_ch25_intelligibility :: proc(
			plan1k    : ^FFT_Plan,
			ref_full  : []f64,
			out_full  : []f64,
			ola_delay : int,
			t_start   : f64 = 0.76,
		) -> (env_corr: f64, contrast_db: f64) {
			edges := [9]f64{125.0, 260.0, 450.0, 700.0, 1050.0, 1450.0, 1900.0, 2350.0, 2720.0}
			bin_hz := AUDIO_SAMPLE_RATE_HZ / 1024.0
			c_buf  := make([]complex128, 1024)
			r_buf  := make([]complex128, 1024)
			defer delete(c_buf)
			defer delete(r_buf)

			// Evaluate post-pull-in steady-state speech + '/s/' consonant + inter-word pauses
			start_s := int(t_start * AUDIO_SAMPLE_RATE_HZ) + ola_delay
			end_s   := len(out_full) - 1024
			MAX_FR  :: 280
			ref_env : [8][MAX_FR]f64
			out_env : [8][MAX_FR]f64
			n_fr    := 0

			for pos := start_s; pos + 1024 <= end_s && n_fr < MAX_FR; pos += 512 {
				r_pos := pos - ola_delay
				if r_pos < 0 || r_pos + 1024 > len(ref_full) do continue
				for i in 0 ..< 1024 {
					w := root_hann_f64(i, 1024)
					c_buf[i] = complex(ref_full[r_pos + i] * w, 0.0)
					r_buf[i] = complex(out_full[pos + i]   * w, 0.0)
				}
				fft_forward(plan1k, c_buf)
				fft_forward(plan1k, r_buf)
				for b in 0 ..< 8 {
					k0 := max(int(math.ceil(edges[b]      / bin_hz)), 1)
					k1 := min(int(math.floor(edges[b + 1] / bin_hz)), 511)
					pc, pr := 0.0, 0.0
					for k in k0 ..= k1 {
						pc += c64_abs2(c_buf[k])
						pr += c64_abs2(r_buf[k])
					}
					ref_env[b][n_fr] = math.sqrt(pc)
					out_env[b][n_fr] = math.sqrt(pr)
				}
				n_fr += 1
			}

			corr_sum := 0.0
			w_sum    := 0.0
			for b in 0 ..< 8 {
				mc, mr := 0.0, 0.0
				for f in 0 ..< n_fr {
					mc += ref_env[b][f]
					mr += out_env[b][f]
				}
				mc /= f64(max(n_fr, 1))
				mr /= f64(max(n_fr, 1))
				num, dc, dr := 0.0, 1e-24, 1e-24
				for f in 0 ..< n_fr {
					ac_v := ref_env[b][f] - mc
					ar_v := out_env[b][f] - mr
					num  += ac_v * ar_v
					dc   += ac_v * ac_v
					dr   += ar_v * ar_v
				}
				wb := math.sqrt(mc + 1e-9)
				corr_sum += wb * (num / math.sqrt(dc * dr))
				w_sum    += wb
			}
			env_corr = corr_sum / math.max(w_sum, 1e-12)

			// Measure post-AGC speaker RMS during active syllables vs inter-word silence pauses
			sp_pow, sp_cnt := 0.0, 0
			pa_pow, pa_cnt := 0.0, 0
			for idx := start_s; idx < len(out_full) - 256; idx += 1 {
				r_idx := idx - ola_delay
				if r_idx < 0 do continue
				t_ref := f64(r_idx) / AUDIO_SAMPLE_RATE_HZ
				in_syl := (t_ref >= 1.06 && t_ref <= 1.41) ||
				          (t_ref >= 1.75 && t_ref <= 2.04)
				in_pau := (t_ref >= 0.78 && t_ref <= 0.90) ||
				          (t_ref >= 1.53 && t_ref <= 1.66) ||
				          (t_ref >= 2.16 && t_ref <= 2.28)
				v := out_full[idx]
				if in_syl {
					sp_pow += v * v
					sp_cnt += 1
				} else if in_pau {
					pa_pow += v * v
					pa_cnt += 1
				}
			}
			sp_rms := math.sqrt(sp_pow / f64(max(sp_cnt, 1)))
			pa_rms := math.sqrt(pa_pow / f64(max(pa_cnt, 1)))
			contrast_db = 20.0 * math.log10(math.max(sp_rms, 1e-12) / math.max(pa_rms, 1e-12))
			return
		}

		test_lvls := [3]Filter_Level{.Off, .Level_3, .Level_4}
		ec_arr    : [3]f64
		ct_arr    : [3]f64
		clar_end  := 0.0
		aud_hop   : [CHAN_HOP_B]f64

		for li in 0 ..< 3 {
			fl := test_lvls[li]
			channel_filter_init(cf)
			demod_init(dm)
			auto_clarifier_init(ac)
			audio_filter_bank_init(bank)

			clar_hz := 0.0
			c_ph    := 0.0
			w_pos   := 0
			for n in 0 ..< N_SYL_SAMP {
				c_ph = wrap_pi_f64(c_ph - (2.0 * math.PI * clar_hz) / BASEBAND_RATE_HZ)
				z_rot := iq_ch25[n] * complex(math.cos(c_ph), math.sin(c_ph))
				if channel_filter_push(cf, z_rot, .LSB) {
					demodulate_block_f64(dm, cf.out_ready[:], .LSB, aud_hop[:], cf.noise_power_lin)
					// Add 50/100/150 Hz European power-supply / ground-loop mains hum in baseband audio
					for k_s in 0 ..< CHAN_HOP_B {
						t_a := f64(w_pos + k_s) / AUDIO_SAMPLE_RATE_HZ
						aud_hop[k_s] += 0.008 * math.cos(2.0 * math.PI * 50.0  * t_a) +
						                0.006 * math.cos(2.0 * math.PI * 100.0 * t_a + 0.4) +
						                0.004 * math.cos(2.0 * math.PI * 150.0 * t_a + 1.1)
					}
					d_hz, do_adj := auto_clarifier_step(ac, dm, cf, aud_hop[:], .LSB, true)
					if do_adj {
						old_c   := clar_hz
						clar_hz  = clamp(clar_hz + d_hz, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
						audio_filter_bank_notify_clarifier_step(bank, clar_hz - old_c, .LSB)
					}
					vc, _, _ := audio_filter_bank_process(bank, aud_hop[:], fl, .LSB)
					post_process_audio_block_f64(dm, aud_hop[:], cf.sig_dbfs, cf.snr_db, vc, false, -120.0, 0.90, fl)
					if w_pos + CHAN_HOP_B <= len(out_audio) {
						copy(out_audio[w_pos : w_pos + CHAN_HOP_B], aud_hop[:])
						w_pos += CHAN_HOP_B
					}
				}
			}
			clar_end = clar_hz
			delay := CHAN_FIR_LEN / 2 // 512-sample linear-phase FIR group delay of Channel_Selectivity_Filter
			if fl == .Level_3 do delay += L3_FFT_N - L3_HOP
			if fl == .Level_4 do delay += L4_FFT_N - L4_HOP
			ec_arr[li], ct_arr[li] = eval_ch25_intelligibility(plan1k, clean_ref[:w_pos], out_audio[:w_pos], delay)
		}

		clar_ok := math.abs(clar_end - tx_mistune_hz) < 5.0
		int_ok  := ec_arr[1] > ec_arr[0] + 0.22 && ec_arr[1] > 0.65 && ec_arr[2] > 0.72 && ec_arr[2] >= ec_arr[1] - 0.05
		sil_ok  := ct_arr[1] > 15.0 && ct_arr[2] > 20.0 && (ct_arr[2] > ct_arr[1] || (ct_arr[1] > 60.0 && ct_arr[2] > 60.0))
		st15_ok := ch25_rf_ok && clar_ok && int_ok && sil_ok
		all_ok  &= st15_ok
		screen_printfln(
			"  [15] Ch25 LSB -10dB DX Speech Intel : Clar=%+.1fHz | STOI Off=%.2f->F3=%.2f->F4=%.2f | Sil F3=+%4.1f/F4=+%4.1fdB -> %s",
			clar_end, ec_arr[0], ec_arr[1], ec_arr[2], ct_arr[1], ct_arr[2], "PASS" if st15_ok else "FAIL",
		)

		// -------------------------------------------------------------------------
		// Stage 16: Ch 34 LSB <350Hz TX-Cut Male Voice + Notch ON:
		//           Filter 3 @ -15 dB SNR & Filter 4 @ -20 dB SNR + Exact 0.0 Pause Silence
		// -------------------------------------------------------------------------
		N_CH34_BLKS :: 150
		N_CH34_SAMP :: N_CH34_BLKS * CHAN_HOP_B // 153,600 samples (3.200 s @ 48 kHz)
		ref34       := make([]f64, N_CH34_SAMP)
		sig34_re    := make([]f64, N_CH34_SAMP)
		sig34_im    := make([]f64, N_CH34_SAMP)
		iq34        := make([]complex128, N_CH34_SAMP)
		out34       := make([]f64, N_CH34_SAMP)
		defer delete(ref34)
		defer delete(sig34_re)
		defer delete(sig34_im)
		defer delete(iq34)
		defer delete(out34)

		ph34 : [22]f64
		for h in 0 ..< 22 do ph34[h] = 0.27 * f64(h + 1)
		sig34_pow := 0.0
		sig34_cnt := 0

		for n in 0 ..< N_CH34_SAMP {
			t_s   := f64(n) / BASEBAND_RATE_HZ
			env_v := 0.0
			f0_t  := 96.0
			f1_t  := 540.0
			f2_t  := 1180.0
			f3_t  := 2320.0

			if t_s >= 0.38 && t_s < 1.05 {
				u     := (t_s - 0.38) / 0.67
				env_v  = math.sin(math.PI * clamp(u * 6.5, 0.0, 0.5)) * math.sin(math.PI * clamp((1.0 - u) * 6.5, 0.0, 0.5))
				f0_t   = 95.0 + 7.0 * math.sin(math.PI * u)
				f1_t   = 520.0 + 110.0 * u
				f2_t   = 1080.0 + 260.0 * u
				f3_t   = 2320.0
			} else if t_s >= 1.52 && t_s < 2.18 {
				u     := (t_s - 1.52) / 0.66
				env_v  = math.sin(math.PI * clamp(u * 6.5, 0.0, 0.5)) * math.sin(math.PI * clamp((1.0 - u) * 6.5, 0.0, 0.5))
				f0_t   = 103.0 - 11.0 * u
				f1_t   = 580.0 - 130.0 * u
				f2_t   = 1460.0 + 380.0 * u
				f3_t   = 2440.0
			}

			vre, vim := 0.0, 0.0
			if env_v > 1e-5 {
				max_h := min(int(math.floor(2680.0 / f0_t)), 22)
				for h in 1 ..= max_h {
					fh := f64(h) * f0_t
					ph34[h - 1] = wrap_pi_f64(ph34[h - 1] + (2.0 * math.PI * fh) / BASEBAND_RATE_HZ)
					// Real CB SSB <350 Hz TX crystal/DSP high-pass cutoff (attenuates h=1,2,3 heavily!)
					tx_hp := clamp((fh - 320.0) / 85.0, 0.03, 1.0)
					d1    := (fh - f1_t) / 150.0
					d2    := (fh - f2_t) / 220.0
					d3    := (fh - f3_t) / 260.0
					amp   := tx_hp * (1.0 / (1.0 + d1 * d1) + 0.78 / (1.0 + d2 * d2) + 0.44 / (1.0 + d3 * d3)) / math.pow(f64(h), 0.22)
					vre  += env_v * amp * math.cos(ph34[h - 1])
					vim  += env_v * amp * math.sin(ph34[h - 1])
				}
			}
			ref34[n]    = vre * 0.08
			sig34_re[n] = vre * 0.08
			sig34_im[n] = vim * 0.08
			if env_v > 0.35 {
				sig34_pow += sig34_re[n] * sig34_re[n] + sig34_im[n] * sig34_im[n]
				sig34_cnt += 1
			}
		}

		act34_pow   := sig34_pow / f64(max(sig34_cnt, 1))
		mistune34   := -19.0 // Realistic Ch 34 LSB rig offset (-19 Hz)
		snrs_34     := [2]f64{-15.0, -20.0}
		lvls_34     := [2]Filter_Level{.Level_3, .Level_4}
		sp_rms_34   : [2]f64
		pau_max_34  : [2]f64
		ec_34       : [2]f64
		clar_34     : [2]f64

		for ti in 0 ..< 2 {
			snr_target := snrs_34[ti]
			fl_t       := lvls_34[ti]
			therm_pow  := act34_pow * math.pow(10.0, -snr_target / 10.0)
			sig_iq34   := math.sqrt(therm_pow / (2.0 * (2675.0 / BASEBAND_RATE_HZ)))
			rng34      : u64 = 0x3434_CB34_A5A5_5A5A ~ u64(ti * 0x13579BDF)

			for n in 0 ..< N_CH34_SAMP {
				t_s      := f64(n) / BASEBAND_RATE_HZ
				g1, g2   := xorshift_f64_normal(&rng34)
				c_o      := math.cos(2.0 * math.PI * mistune34 * t_s)
				s_o      := math.sin(2.0 * math.PI * mistune34 * t_s)
				vre      := sig34_re[n]
				vim      := sig34_im[n]
				s_re     := vre * c_o + vim * s_o
				s_im     := vre * s_o - vim * c_o
				// Weak continuous +1008 Hz LSB heterodyne spur (like Ch 34 1007.8 Hz carrier)
				wh_amp   := sig_iq34 * 0.055
				wh_re    := wh_amp * math.cos(-2.0 * math.PI * 1008.0 * t_s)
				wh_im    := wh_amp * math.sin(-2.0 * math.PI * 1008.0 * t_s)
				iq34[n]   = complex(s_re + sig_iq34 * g1 + wh_re, s_im + sig_iq34 * g2 + wh_im)
			}

			channel_filter_init(cf)
			demod_init(dm)
			auto_clarifier_init(ac)
			audio_filter_bank_init(bank)

			clar_hz := 0.0
			c_ph    := 0.0
			w_pos   := 0
			for n in 0 ..< N_CH34_SAMP {
				c_ph  = wrap_pi_f64(c_ph - (2.0 * math.PI * clar_hz) / BASEBAND_RATE_HZ)
				z_rot := iq34[n] * complex(math.cos(c_ph), math.sin(c_ph))
				if channel_filter_push(cf, z_rot, .LSB) {
					demodulate_block_f64(dm, cf.out_ready[:], .LSB, aud_hop[:], cf.noise_power_lin)
					d_hz, do_adj := auto_clarifier_step(ac, dm, cf, aud_hop[:], .LSB, true)
					if do_adj {
						old_c   := clar_hz
						clar_hz  = clamp(clar_hz + d_hz, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
						audio_filter_bank_notify_clarifier_step(bank, clar_hz - old_c, .LSB)
					}
					vc, _, _ := audio_filter_bank_process(bank, aud_hop[:], fl_t, .LSB, true)
					post_process_audio_block_f64(
						dm, aud_hop[:], cf.sig_dbfs, cf.snr_db, vc,
						false, -120.0, 0.90, fl_t, false, cf.noise_dbfs, .LSB, ac.locked,
					)
					if w_pos + CHAN_HOP_B <= len(out34) {
						copy(out34[w_pos : w_pos + CHAN_HOP_B], aud_hop[:])
						w_pos += CHAN_HOP_B
					}
				}
			}

			clar_34[ti] = clar_hz
			delay := CHAN_FIR_LEN / 2 + ((L3_FFT_N - L3_HOP) if fl_t == .Level_3 else (L4_FFT_N - L4_HOP))

			sp_p, sp_c := 0.0, 0
			p_max      := 0.0
			for idx := int(0.45 * AUDIO_SAMPLE_RATE_HZ) + delay; idx < w_pos - 256; idx += 1 {
				t_ref := f64(idx - delay) / AUDIO_SAMPLE_RATE_HZ
				in_sp := (t_ref >= 0.56 && t_ref <= 0.98) || (t_ref >= 1.66 && t_ref <= 2.10)
				in_pa := (t_ref >= 1.18 && t_ref <= 1.46) || (t_ref >= 2.32 && t_ref <= 3.12)
				v     := out34[idx]
				if in_sp {
					sp_p += v * v
					sp_c += 1
				} else if in_pa {
					if math.abs(v) > p_max do p_max = math.abs(v)
				}
			}
			sp_rms_34[ti]  = math.sqrt(sp_p / f64(max(sp_c, 1)))
			pau_max_34[ti] = p_max
			ec_34[ti], _   = eval_ch25_intelligibility(plan1k, ref34[:w_pos], out34[:w_pos], delay, 0.45)
		}

		// Also verify <350 Hz TX-cut male LSB Auto-Clarifier zero-beat lock (@ -1.0 dB conversational SNR)
		clar_txcut := 0.0
		{
			therm_c := act34_pow * math.pow(10.0, 1.0 / 10.0)
			sig_c   := math.sqrt(therm_c / (2.0 * (2675.0 / BASEBAND_RATE_HZ)))
			rng_c   : u64 = 0xCAFE_3434_BEEF_1234
			channel_filter_init(cf)
			demod_init(dm)
			auto_clarifier_init(ac)
			c_ph := 0.0
			for n in 0 ..< N_CH34_SAMP {
				t_s    := f64(n) / BASEBAND_RATE_HZ
				g1, g2 := xorshift_f64_normal(&rng_c)
				c_o    := math.cos(2.0 * math.PI * mistune34 * t_s)
				s_o    := math.sin(2.0 * math.PI * mistune34 * t_s)
				s_re   := sig34_re[n] * c_o + sig34_im[n] * s_o
				s_im   := sig34_re[n] * s_o - sig34_im[n] * c_o
				c_ph    = wrap_pi_f64(c_ph - (2.0 * math.PI * clar_txcut) / BASEBAND_RATE_HZ)
				z_rot  := complex(s_re + sig_c * g1, s_im + sig_c * g2) * complex(math.cos(c_ph), math.sin(c_ph))
				if channel_filter_push(cf, z_rot, .LSB) {
					demodulate_block_f64(dm, cf.out_ready[:], .LSB, aud_hop[:], cf.noise_power_lin)
					d_hz, do_adj := auto_clarifier_step(ac, dm, cf, aud_hop[:], .LSB, true)
					if do_adj {
						clar_txcut = clamp(clar_txcut + d_hz, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
					}
				}
			}
		}

		clar34_ok := math.abs(clar_txcut - mistune34) < 5.0 && math.abs(clar_34[0]) < 30.0 && math.abs(clar_34[1]) < 30.0
		f3_15_ok  := sp_rms_34[0] > 0.12 && ec_34[0] > 0.55 && pau_max_34[0] == 0.0
		f4_20_ok  := sp_rms_34[1] > 0.12 && ec_34[1] > 0.52 && pau_max_34[1] == 0.0
		st16_ok   := clar34_ok && f3_15_ok && f4_20_ok
		all_ok    &= st16_ok
		screen_printfln(
			"  [16] Ch34 LSB -15dB(F3)/-20dB(F4)+Notch: Clar=%+.1fHz | STOI=%.2f/%.2f | PauseMax=%.6f/%.6f -> %s",
			clar_txcut, ec_34[0], ec_34[1], pau_max_34[0], pau_max_34[1], "PASS" if st16_ok else "FAIL",
		)
	}

	screen_println("================================================================================")
	if all_ok {
		screen_println("  OVERALL SELFTEST RESULT: *** ALL TESTS PASSED ***")
	} else {
		screen_println("  OVERALL SELFTEST RESULT: *** SOME TESTS FAILED ***")
	}
	screen_println("================================================================================")
	screen_flush()
	return all_ok
}

@(test)
odin_unit_test_dsp_suite :: proc(t: ^testing.T) {
	ok := run_complete_dsp_selftest()
	testing.expect(t, ok, "Complete f64 CB SDR DSP & -30dB SSB Voice Recovery suite must pass")
}
