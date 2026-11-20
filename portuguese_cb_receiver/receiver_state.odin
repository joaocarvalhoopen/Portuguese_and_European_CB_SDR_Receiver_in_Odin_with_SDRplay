// Shared Receiver Configuration, f64 Audio Ring Buffer, Full 6 MSPS -> 48 kHz DSP Coordinator,
// and Thread-Safe Live Telemetry Snapshot.

package main

import "core:math"
import "core:sync"
import "core:time"

AUDIO_RING_CAPACITY   :: 1 << 18 // 262,144 mono f64 audio frames (~5.46 seconds at 48 kHz, lock-free SPSC)
AUDIO_RING_MASK       :: u32(AUDIO_RING_CAPACITY - 1)
AUDIO_PREFILL_SAMPLES :: 6144    // 128.0 ms pre-buffer cushion (3x1024 HW DMA periods + 3x1024 anti-jitter cushion)
DECIM_CHUNK_MAX_IQ    :: 65536
DECIM_OUT_MAX_48K     :: (DECIM_CHUNK_MAX_IQ / 125) + 64

// User-adjustable runtime settings (protected by cfg_mutex)
Receiver_Config :: struct {
	channel         : int,
	mode            : Modulation_Mode,
	filter_level    : Filter_Level,
	clarifier_hz    : f64,  // [-5000.0 .. +5000.0] Hz fine offset
	auto_clarifier  : bool, // Closed-loop Auto-Clarifier (AM SAM PLL, FM DC, SSB vocal harmonic mistune)
	volume_pct        : f64,  // [0.0 .. 200.0] %
	squelch_enabled   : bool,
	auto_squelch      : bool, // Adaptive multi-mode noise-floor + voice/PLL coherence Auto-Squelch
	squelch_dbfs      : f64,  // [-135.0 .. 0.0] dBFS manual threshold
	hw_agc_enabled    : bool,
	auto_rf_gain      : bool, // Closed-loop Auto RF Gain (HW AGC + software ADC window supervisor)
	auto_lna          : bool, // Closed-loop Auto LNA state controller (overload & weak-signal tracking)
	rf_gr_db          : i32,  // [20 .. 59] dB IF gain reduction
	lna_state         : u8,   // [0 .. max_lna] LNA attenuation step
	antenna_port      : int,  // 0=Ant A, 1=Ant B, 2=Ant C (RSPdx / RSPdx-R2 / RSP2)
	rf_notch          : bool, // FM/MW broadcast notch filter
	ppm_cal           : f64,  // Crystal PPM correction
	monitor_sec       : int,  // Plain-text periodic telemetry interval (0 = off)
	waterfall_enabled : bool, // Periodic Braille 40-ch spectrum + 60s waterfall ('sp on' / 'sp off')
	waterfall_sec     : int,  // Refresh interval in seconds when waterfall_enabled is true (default 2s)
	lo_override_hz    : f64,  // >0 overrides hardware LO during wideband 40-channel scan
	mute_for_scan     : bool, // Mutes audio ring output during wideband scan LO hop

	// Hardware update dirty flags consumed by SDR engine
	hw_dirty_rf     : bool,
	hw_dirty_gain   : bool,
	hw_dirty_agc    : bool,
	hw_dirty_ant    : bool,
	hw_dirty_notch  : bool,
	hw_dirty_ppm    : bool,
}

// Live read-only telemetry snapshot for plain-text CLI display
Telemetry_Snapshot :: struct {
	streaming        : bool,
	simulation_mode  : bool,
	device_name      : string,
	device_serial    : string,

	channel          : int,
	nominal_freq_hz  : f64,
	clarifier_hz     : f64,
	auto_clarifier   : bool,
	clar_locked      : bool,
	effective_hz     : f64,
	lo_freq_hz       : f64,
	mode             : Modulation_Mode,
	filter_level     : Filter_Level,

	sig_dbfs         : f64,
	noise_dbfs       : f64,
	snr_db           : f64,
	adc_dbfs         : f64,
	adc_peak_pct     : f64,
	carrier_off_hz   : f64,

	squelch_open      : bool,
	squelch_enabled   : bool,
	auto_squelch      : bool,
	squelch_dbfs      : f64,
	eff_sq_dbfs       : f64,
	volume_pct        : f64,
	waterfall_enabled : bool,

	audio_rms_dbfs   : f64,
	audio_peak_pct   : f64,
	agc_gain_db      : f64,
	filter_nr_db     : f64,
	voice_conf_pct   : f64,
	voice_pitch_hz   : f64,

	curr_total_gain  : f64,
	rf_gr_db         : i32,
	hw_curr_gr_db    : i32,
	hw_lna_gr_db     : i32,
	lna_state        : u8,
	max_lna_state    : u8,
	hw_agc_enabled   : bool,
	auto_rf_gain     : bool,
	auto_lna         : bool,
	antenna_port     : int,
	rf_notch         : bool,
	ppm_cal          : f64,
	scan_active      : bool,
	scan_band_label  : string,

	// Accurate stream Date & Time (live UTC clock when LIVE; recorded UTC timestamp when playing/paused on IQ file!)
	stream_timestamp_ns : i64,

	// Raw 6.0 MSPS 14-Bit IQ_file_xxxx.samples Recorder & Player State
	iq_saving        : bool,
	iq_save_name_buf : [64]u8,
	iq_save_name_len : int,
	iq_save_sec      : f64,
	iq_save_mb       : f64,
	iq_save_drops    : u64,

	iq_file_mode     : bool,
	iq_playing       : bool,
	iq_play_name_buf : [64]u8,
	iq_play_name_len : int,
	iq_play_pos_sec  : f64,
	iq_play_tot_sec  : f64,
	iq_file_start_ns : i64,
	iq_file_end_ns   : i64,

	total_iq_samples : u64,
	callbacks_cnt    : u64,
	overload_events  : u64,
	blanked_impulses : u64,
	audio_underruns  : u64,
	audio_overflows  : u64,
	ring_fill_pct    : f64,
}

Receiver_Context :: struct {
	running          : bool,
	sim_mode         : bool,
	sim_snr_db       : f64,
	sim_custom_ch    : int,
	sim_custom_mode  : Modulation_Mode,

	cfg_mutex        : sync.Mutex,
	cfg              : Receiver_Config,

	// Complete heap-allocated f64 DSP chain (owned exclusively by stream/sim thread)
	frontend         : FrontEnd_Decimator,
	chan_filter      : Channel_Selectivity_Filter,
	demod            : Demodulator_State,
	auto_clar        : Auto_Clarifier_State,
	filter_bank      : Audio_Filter_Bank,
	scanner          : CB_Auto_Scanner,
	waterfall        : Braille_Waterfall_State,
	iq_eng           : IQ_Recorder_Player,
	last_mode        : Modulation_Mode,
	last_channel     : int,
	last_squelch_en  : bool,

	// Scratch buffers for zero-allocation callback execution
	iq_48k_scratch   : [DECIM_OUT_MAX_48K]complex128,
	audio_block_1024 : [CHAN_HOP_B]f64,

	// Lock-free Single-Producer / Single-Consumer (SPSC) f64 Audio Ring Buffer
	// Producer (SDR/Sim DSP thread) writes ring_write; Consumer (miniaudio callback) writes ring_read.
	audio_consumer_on : bool,
	ring_primed       : bool,
	audio_ring        : [AUDIO_RING_CAPACITY]f64,
	ring_write        : u32,
	ring_read         : u32,
	underrun_cnt      : u64,
	overflow_cnt      : u64,

	// Telemetry mutex & live counters
	tel_mutex        : sync.Mutex,
	tel              : Telemetry_Snapshot,

	// Optional on-demand 48 kHz complex128 I/Q capture buffer for deep channel/filter diagnostics
	diag_mutex       : sync.Mutex,
	diag_iq_buf      : []complex128,
	diag_iq_write    : int,
	diag_capturing   : bool,
}

receiver_context_init :: proc(rx: ^Receiver_Context) {
	rx.running         = true
	rx.sim_snr_db      = -24.0 // Default simulation SNR: deep -24 dB weak voice under house + thermal noise!
	rx.sim_custom_ch   = 0
	rx.sim_custom_mode = .AM

	rx.cfg = Receiver_Config{
		channel           = CB_DEFAULT_CHANNEL, // Ch 19 (27.185 MHz)
		mode              = .AM,
		filter_level      = .Level_2, // Clean natural voice + house EMI removal on startup ('f 4' for -30 dB deep voice!)
		clarifier_hz      = 0.0,
		auto_clarifier    = true,     // Closed-loop Auto-Clarifier enabled by default across AM, FM & SSB!
		volume_pct        = 90.0,
		squelch_enabled   = false,
		auto_squelch      = false,
		squelch_dbfs      = -98.0,
		hw_agc_enabled    = true,
		auto_rf_gain      = true,
		auto_lna          = true,
		rf_gr_db          = 32,
		lna_state         = 0,
		antenna_port      = 0,
		rf_notch          = false, // Disabled by default so no broadcast notch attenuates 27 MHz HF path
		ppm_cal           = 0.0,
		monitor_sec       = 0,
		waterfall_enabled = false,
		waterfall_sec     = 2,
		lo_override_hz    = 0.0,
		mute_for_scan     = false,
	}

	frontend_init(&rx.frontend, rx.cfg.clarifier_hz)
	channel_filter_init(&rx.chan_filter)
	demod_init(&rx.demod)
	auto_clarifier_init(&rx.auto_clar)
	audio_filter_bank_init(&rx.filter_bank)
	scanner_init(&rx.scanner)
	waterfall_init(&rx.waterfall)
	iq_engine_init(&rx.iq_eng, rx)
	rx.last_mode    = rx.cfg.mode
	rx.last_channel = rx.cfg.channel

	rx.tel = Telemetry_Snapshot{
		device_name        = "Initializing...",
		device_serial      = "-",
		channel            = rx.cfg.channel,
		nominal_freq_hz    = channel_to_freq_hz(rx.cfg.channel),
		auto_clarifier     = rx.cfg.auto_clarifier,
		effective_hz       = channel_to_freq_hz(rx.cfg.channel),
		lo_freq_hz         = hardware_lo_freq_hz(rx.cfg.channel),
		mode               = rx.cfg.mode,
		filter_level       = rx.cfg.filter_level,
		sig_dbfs           = -115.0,
		noise_dbfs         = -120.0,
		audio_rms_dbfs     = -90.0,
		squelch_open       = true,
		squelch_dbfs       = rx.cfg.squelch_dbfs,
		eff_sq_dbfs        = rx.cfg.squelch_dbfs,
		volume_pct         = rx.cfg.volume_pct,
		rf_gr_db           = rx.cfg.rf_gr_db,
		hw_curr_gr_db      = rx.cfg.rf_gr_db,
		max_lna_state      = 19,
		hw_agc_enabled     = rx.cfg.hw_agc_enabled,
		auto_rf_gain       = rx.cfg.auto_rf_gain,
		auto_lna           = rx.cfg.auto_lna,
		rf_notch           = rx.cfg.rf_notch,
		ppm_cal            = rx.cfg.ppm_cal,
		scan_band_label    = "OFF",
		stream_timestamp_ns= time.now()._nsec,
	}
}

receiver_context_shutdown :: proc(rx: ^Receiver_Context) {
	if rx == nil do return
	iq_engine_shutdown(&rx.iq_eng)
}

// Lock-free SPSC push: SDR/DSP thread writes f64 audio samples with zero mutex contention
audio_ring_push_f64 :: proc(rx: ^Receiver_Context, samples: []f64) {
	n_in := len(samples)
	if n_in == 0 do return

	// When running headless without an active audio playback device, no consumer drains ring_read
	if !sync.atomic_load_explicit(&rx.audio_consumer_on, .Relaxed) {
		return
	}

	w     := sync.atomic_load_explicit(&rx.ring_write, .Relaxed)
	r     := sync.atomic_load_explicit(&rx.ring_read,  .Acquire)
	used  := int(w - r)
	avail := max(AUDIO_RING_CAPACITY - used, 0)
	n_wr  := min(n_in, avail)

	for i in 0 ..< n_wr {
		rx.audio_ring[(w + u32(i)) & AUDIO_RING_MASK] = samples[i]
	}
	if n_wr > 0 {
		sync.atomic_store_explicit(&rx.ring_write, w + u32(n_wr), .Release)
	}
	if n_wr < n_in {
		_ = sync.atomic_add_explicit(&rx.overflow_cnt, u64(n_in - n_wr), .Relaxed)
	}
}

audio_ring_fill_fraction :: proc(rx: ^Receiver_Context) -> f64 {
	w    := sync.atomic_load_explicit(&rx.ring_write, .Relaxed)
	r    := sync.atomic_load_explicit(&rx.ring_read,  .Relaxed)
	used := clamp(int(w - r), 0, AUDIO_RING_CAPACITY)
	return (f64(used) / f64(AUDIO_RING_CAPACITY)) * 100.0
}

// Process arbitrary-length 6.0 MSamples/s 14-bit I/Q callback buffers through the entire f64 DSP chain
receiver_process_sdr_samples :: proc(
	rx             : ^Receiver_Context,
	xi             : [^]i16,
	xq             : [^]i16,
	num_samples    : int,
	override_lo_hz : f64  = 0.0,
	override_ts_ns : i64  = 0,
	from_iq_player : bool = false,
) {
	// When playing/paused on a recorded IQ_file_xxxx.samples file, ignore live hardware/sim callbacks
	if !from_iq_player && iq_is_file_mode(&rx.iq_eng) {
		return
	}

	// Snapshot active user controls once per callback
	sync.mutex_lock(&rx.cfg_mutex)
	cfg := rx.cfg
	sync.mutex_unlock(&rx.cfg_mutex)

	active_lo_hz := override_lo_hz if override_lo_hz > 0.0 else (cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel))
	ts_ns        := override_ts_ns if override_ts_ns > 0 else time.now()._nsec

	// Non-blocking lock-free SPSC raw 6.0 MSPS 14-Bit spectrum recording ('save' -> IQ_file_xxxx.samples)
	if !from_iq_player && iq_is_saving(&rx.iq_eng) {
		iq_recorder_feed_6m(&rx.iq_eng, xi, xq, num_samples, active_lo_hz, cfg.rf_gr_db, cfg.lna_state, ts_ns)
	}

	if cfg.mode != rx.last_mode || cfg.channel != rx.last_channel {
		if cfg.channel != rx.last_channel {
			channel_filter_reset(&rx.chan_filter)
		}
		demod_reset_mode(&rx.demod)
		auto_clarifier_reset(&rx.auto_clar)
		rx.last_mode    = cfg.mode
		rx.last_channel = cfg.channel
	}
	if cfg.squelch_enabled && !rx.last_squelch_en {
		rx.demod.sq_is_open      = false
		rx.demod.sq_hold_samples = 0
	}
	rx.last_squelch_en = cfg.squelch_enabled

	// Feed continuous 40-channel 60-second Braille Waterfall analyzer uninterruptedly at 6.0 MSPS (with exact timestamp!)
	waterfall_process_iq_6m(&rx.waterfall, xi, xq, num_samples, active_lo_hz, ts_ns)

	// If wideband 40-channel scanner is actively capturing, route 6 MSPS I/Q to scanner!
	if cfg.mute_for_scan {
		scan_cap, _, _ := scanner_is_capture_done(&rx.scanner)
		if scan_cap {
			scanner_process_iq_6m(&rx.scanner, xi, xq, num_samples, active_lo_hz)
		}
		// Still track raw 14-bit ADC wideband power for Auto-LNA / Auto-RF-Gain during scanning
		SCALE_I16 :: f64(1.0 / 32768.0)
		p_acc := 0.0
		pk_b  := 0.0
		step_cnt := 0
		for n := 0; n < num_samples; n += 16 {
			ri := f64(xi[n]) * SCALE_I16
			rq := f64(xq[n]) * SCALE_I16
			m2 := ri * ri + rq * rq
			p_acc += m2
			if m2 > pk_b do pk_b = m2
			step_cnt += 1
		}
		if step_cnt > 0 {
			rx.frontend.adc_power_ema = 0.90 * rx.frontend.adc_power_ema + 0.10 * math.max(p_acc / f64(step_cnt), 1e-16)
			rx.frontend.adc_peak      = math.sqrt(pk_b)
		}
		sync.mutex_lock(&rx.tel_mutex)
		rx.tel.total_iq_samples   += u64(num_samples)
		rx.tel.callbacks_cnt      += 1
		rx.tel.adc_dbfs            = 10.0 * math.log10(math.max(rx.frontend.adc_power_ema, 1e-16))
		rx.tel.adc_peak_pct        = rx.frontend.adc_peak * 100.0
		rx.tel.stream_timestamp_ns = ts_ns
		sync.mutex_unlock(&rx.tel_mutex)
		return
	}

	// Compute exact NCO base frequency offset to mix ANY selected CB channel (1..40) to 0 Hz baseband
	// whether running on live hardware (-SDR_LO_OFFSET_HZ = +50 kHz) or playing back a recorded IQ file!
	base_offset_hz := channel_to_freq_hz(cfg.channel) - active_lo_hz

	vol_lin        := clamp(cfg.volume_pct / 100.0, 0.0, 4.0)
	last_vc        := 0.0
	last_f0        := 0.0
	last_nr        := 0.0
	updated_filter := false

	offset := 0
	for offset < num_samples {
		chunk := min(num_samples - offset, DECIM_CHUNK_MAX_IQ)
		i_slice := xi[offset : offset + chunk]
		q_slice := xq[offset : offset + chunk]
		offset += chunk

		// 1. 6.0 MHz -> 240 kHz (Blanker) -> 48 kHz Polyphase f64 Decimation
		n48 := frontend_process_iq_block(
			&rx.frontend,
			i_slice,
			q_slice,
			cfg.clarifier_hz,
			cfg.filter_level,
			rx.iq_48k_scratch[:],
			base_offset_hz,
		)

		if rx.diag_capturing && n48 > 0 {
			sync.mutex_lock(&rx.diag_mutex)
			if rx.diag_capturing && rx.diag_iq_write < len(rx.diag_iq_buf) {
				can_copy := min(n48, len(rx.diag_iq_buf) - rx.diag_iq_write)
				copy(rx.diag_iq_buf[rx.diag_iq_write : rx.diag_iq_write + can_copy], rx.iq_48k_scratch[0 : can_copy])
				rx.diag_iq_write += can_copy
				if rx.diag_iq_write >= len(rx.diag_iq_buf) {
					rx.diag_capturing = false
				}
			}
			sync.mutex_unlock(&rx.diag_mutex)
		}

		// 2. Feed 48 kHz complex samples into 2048-pt Overlap-Save Channel Filter
		for k in 0 ..< n48 {
			if channel_filter_push(&rx.chan_filter, rx.iq_48k_scratch[k], cfg.mode) {
				// 1024 exact linear-convolution complex samples ready!
				// 3. Optimal f64 Demodulator (SAM / FM-PLL / Coherent Analytic SSB with Channel IF AGC)
				demodulate_block_f64(
					&rx.demod,
					rx.chan_filter.out_ready[:],
					cfg.mode,
					rx.audio_block_1024[:],
					rx.chan_filter.noise_power_lin,
				)

				// 3b. Closed-Loop f64 Auto-Clarifier (tracks AM carrier, FM offset & SSB vocal mistune)
				d_clar, do_adj := auto_clarifier_step(
					&rx.auto_clar,
					&rx.demod,
					&rx.chan_filter,
					rx.audio_block_1024[:],
					cfg.mode,
					cfg.auto_clarifier,
				)
				if do_adj {
					applied_step := 0.0
					sync.mutex_lock(&rx.cfg_mutex)
					if rx.cfg.auto_clarifier && rx.cfg.channel == cfg.channel && rx.cfg.mode == cfg.mode {
						old_clar           := rx.cfg.clarifier_hz
						rx.cfg.clarifier_hz = clamp(rx.cfg.clarifier_hz + d_clar, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
						cfg.clarifier_hz    = rx.cfg.clarifier_hz
						applied_step        = rx.cfg.clarifier_hz - old_clar
					}
					sync.mutex_unlock(&rx.cfg_mutex)
					if applied_step != 0.0 {
						audio_filter_bank_notify_clarifier_step(&rx.filter_bank, applied_step, cfg.mode)
					}
				}

				// 4. Band-Specific 4-Level f64 Audio Noise Filter
				vc, f0, nr := audio_filter_bank_process(
					&rx.filter_bank,
					rx.audio_block_1024[:],
					cfg.filter_level,
					cfg.mode,
					cfg.rf_notch,
				)
				last_vc        = vc
				last_f0        = f0
				last_nr        = nr
				updated_filter = true

				// 5. Post-Filter Speech AGC, Soft-Knee Limiter, Manual/Auto Squelch & Volume
				post_process_audio_block_f64(
					&rx.demod,
					rx.audio_block_1024[:],
					rx.chan_filter.sig_dbfs,
					rx.chan_filter.snr_db,
					vc,
					cfg.squelch_enabled,
					cfg.squelch_dbfs,
					vol_lin,
					cfg.filter_level,
					cfg.auto_squelch,
					rx.chan_filter.noise_dbfs,
					cfg.mode,
					rx.auto_clar.locked && cfg.auto_clarifier,
				)

				// 6. Enqueue f64 audio to miniaudio playback ring buffer (unless suppressed during paused seek preview)
				if !sync.atomic_load_explicit(&rx.iq_eng.play_suppress_aud, .Relaxed) {
					audio_ring_push_f64(rx, rx.audio_block_1024[:])
				}
			}
		}
	}

	// Publish thread-safe telemetry snapshot
	fill_pct := audio_ring_fill_fraction(rx)
	sync.mutex_lock(&rx.tel_mutex)
	rx.tel.total_iq_samples   += u64(num_samples)
	rx.tel.callbacks_cnt      += 1
	rx.tel.channel             = cfg.channel
	rx.tel.nominal_freq_hz     = channel_to_freq_hz(cfg.channel)
	rx.tel.clarifier_hz        = cfg.clarifier_hz
	rx.tel.auto_clarifier      = cfg.auto_clarifier
	rx.tel.clar_locked         = rx.auto_clar.locked && cfg.auto_clarifier
	rx.tel.effective_hz        = effective_rx_freq_hz(cfg.channel, cfg.clarifier_hz)
	rx.tel.lo_freq_hz          = active_lo_hz
	rx.tel.mode                = cfg.mode
	rx.tel.filter_level        = cfg.filter_level
	rx.tel.stream_timestamp_ns = ts_ns

	rx.tel.sig_dbfs            = rx.chan_filter.sig_dbfs
	rx.tel.noise_dbfs          = rx.chan_filter.noise_dbfs
	rx.tel.snr_db              = rx.chan_filter.snr_db
	rx.tel.adc_dbfs            = 10.0 * math.log10(math.max(rx.frontend.adc_power_ema, 1e-16))
	rx.tel.adc_peak_pct        = rx.frontend.adc_peak * 100.0
	rx.tel.carrier_off_hz      = rx.demod.carrier_off_hz

	rx.tel.squelch_open        = rx.demod.sq_is_open
	rx.tel.squelch_enabled     = cfg.squelch_enabled
	rx.tel.auto_squelch        = cfg.auto_squelch
	rx.tel.squelch_dbfs        = cfg.squelch_dbfs
	rx.tel.eff_sq_dbfs         = rx.demod.eff_sq_thresh_dbfs
	rx.tel.volume_pct          = cfg.volume_pct
	rx.tel.waterfall_enabled   = cfg.waterfall_enabled

	rx.tel.audio_rms_dbfs      = 20.0 * math.log10(math.max(rx.demod.audio_rms, 1e-9))
	rx.tel.audio_peak_pct      = rx.demod.audio_peak * 100.0
	rx.tel.agc_gain_db         = 20.0 * math.log10(math.max(rx.demod.agc_gain, 1e-6))
	if updated_filter {
		rx.tel.filter_nr_db   = last_nr
		rx.tel.voice_conf_pct = last_vc * 100.0
		rx.tel.voice_pitch_hz = last_f0
	}

	rx.tel.rf_gr_db            = cfg.rf_gr_db
	rx.tel.lna_state           = cfg.lna_state
	rx.tel.hw_agc_enabled      = cfg.hw_agc_enabled
	rx.tel.auto_rf_gain        = cfg.auto_rf_gain
	rx.tel.auto_lna            = cfg.auto_lna
	rx.tel.antenna_port        = cfg.antenna_port
	rx.tel.rf_notch            = cfg.rf_notch
	rx.tel.ppm_cal             = cfg.ppm_cal
	rx.tel.blanked_impulses    = rx.frontend.nb_blanked_cnt
	rx.tel.ring_fill_pct       = fill_pct
	sync.mutex_unlock(&rx.tel_mutex)
}

receiver_get_telemetry :: proc(rx: ^Receiver_Context) -> Telemetry_Snapshot {
	sync.mutex_lock(&rx.cfg_mutex)
	cfg := rx.cfg
	sync.mutex_unlock(&rx.cfg_mutex)

	sync.mutex_lock(&rx.scanner.mutex)
	scan_on := rx.scanner.enabled
	scan_bf := rx.scanner.band_filter
	sync.mutex_unlock(&rx.scanner.mutex)

	sync.mutex_lock(&rx.tel_mutex)
	snap := rx.tel
	sync.mutex_unlock(&rx.tel_mutex)
	snap.audio_underruns = sync.atomic_load_explicit(&rx.underrun_cnt, .Relaxed)
	snap.audio_overflows = sync.atomic_load_explicit(&rx.overflow_cnt, .Relaxed)
	snap.ring_fill_pct   = audio_ring_fill_fraction(rx)

	snap.channel           = cfg.channel
	snap.nominal_freq_hz   = channel_to_freq_hz(cfg.channel)
	snap.clarifier_hz      = cfg.clarifier_hz
	snap.auto_clarifier    = cfg.auto_clarifier
	snap.effective_hz      = effective_rx_freq_hz(cfg.channel, cfg.clarifier_hz)
	snap.lo_freq_hz        = cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel)
	snap.mode              = cfg.mode
	snap.filter_level      = cfg.filter_level
	snap.squelch_enabled   = cfg.squelch_enabled
	snap.auto_squelch      = cfg.auto_squelch
	snap.squelch_dbfs      = snap.eff_sq_dbfs if cfg.auto_squelch else cfg.squelch_dbfs
	snap.volume_pct        = cfg.volume_pct
	snap.waterfall_enabled = cfg.waterfall_enabled
	snap.rf_gr_db          = snap.hw_curr_gr_db if (cfg.hw_agc_enabled && snap.hw_curr_gr_db >= 20) else cfg.rf_gr_db
	snap.lna_state         = cfg.lna_state
	snap.hw_agc_enabled    = cfg.hw_agc_enabled
	snap.auto_rf_gain      = cfg.auto_rf_gain
	snap.auto_lna          = cfg.auto_lna
	snap.antenna_port      = cfg.antenna_port
	snap.rf_notch          = cfg.rf_notch
	snap.ppm_cal           = cfg.ppm_cal
	snap.scan_active       = scan_on
	snap.scan_band_label   = scan_band_name(scan_bf) if scan_on else "OFF"

	// Populate live clock or recorded IQ_file_xxxx.samples date/time + transport state
	if !iq_is_file_mode(&rx.iq_eng) {
		snap.stream_timestamp_ns = time.now()._nsec
	}
	iq_fill_telemetry_snapshot(&rx.iq_eng, &snap)
	return snap
}
