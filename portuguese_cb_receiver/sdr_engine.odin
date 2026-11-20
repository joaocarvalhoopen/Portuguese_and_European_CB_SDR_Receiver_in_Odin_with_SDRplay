// SDRplay API v3.15 Hardware Engine (6.0 MSamples/s @ 14-Bit ADC Depth),
// Closed-Loop Auto-LNA & Auto-RF-Gain Controller, and Multi-Channel 6.0 MSamples/s
// Synthetic CB Spectrum + Indoor House Noise Simulator (-sim).
// Uses the workspace's SDRplay_API_Odin_bindings package directly.

package main

import "base:runtime"
import "core:math"
import "core:sync"
import "core:thread"
import "core:time"

import sdr "../SDRplay_API_Odin_bindings/sdrplay_api_bindings"

SDR_Engine :: struct {
	rx_ctx            : ^Receiver_Context,
	dev               : sdr.DeviceT,
	params            : ^sdr.DeviceParamsT,
	api_opened        : bool,
	device_selected   : bool,
	streaming         : bool,
	serial_buf        : [sdr.SDRPLAY_MAX_SER_NO_LEN]u8,
	serial_len        : int,

	// Closed-loop Auto-LNA & Auto-RF-Gain tracking state
	last_overload_cnt : u64,
	calm_lna_ticks    : int,

	// Simulation thread when running in -sim mode
	sim_thread        : ^thread.Thread,
	sim_stop          : bool,
}

// Band-accurate maximum LNAstate for 27 MHz CB (12-50 MHz / <60 MHz HF sub-band) per SDRplay API v3.15 headers:
//   - RSPdx / RSPdx-R2 (12..50 MHz): RSPDX_NUM_LNA_STATES_AMPORT2_12_50 = 20 -> valid 0..19!
//   - RSP1A / RSP1B (< 60 MHz)     : RSPIA_NUM_LNA_STATES_AM = 7            -> valid 0..6!
//   - RSP2 (< 420 MHz)             : RSPII_NUM_LNA_STATES = 9               -> valid 0..8!
//   - RSPduo (< 60 MHz)            : RSPDUO_NUM_LNA_STATES_AM = 7           -> valid 0..6!
//   - RSP1                         : 4 states                               -> valid 0..3!
max_lna_state_for_27mhz :: proc(hw_ver: u8) -> u8 {
	switch hw_ver {
	case sdr.SDRPLAY_RSPdx_ID, sdr.SDRPLAY_RSPdxR2_ID:
		return u8(sdr.RSPDX_NUM_LNA_STATES_AMPORT2_12_50 - 1) // 19
	case sdr.SDRPLAY_RSP1A_ID, sdr.SDRPLAY_RSP1B_ID:
		return u8(sdr.RSPIA_NUM_LNA_STATES_AM - 1)            // 6
	case sdr.SDRPLAY_RSP2_ID:
		return u8(sdr.RSPII_NUM_LNA_STATES - 1)               // 8
	case sdr.SDRPLAY_RSPduo_ID:
		return u8(sdr.RSPDUO_NUM_LNA_STATES_AM - 1)           // 6
	case sdr.SDRPLAY_RSP1_ID:
		return 3
	}
	return 19 // Default / Simulator (RSPdx-R2 27 MHz profile)
}

// C callback invoked on SDRplay streaming thread at 6.0 MSamples/s (14-bit ADC in i16)
cb_sdr_stream_callback :: proc "c" (
	xi          : [^]i16,
	xq          : [^]i16,
	params      : ^sdr.StreamCbParamsT,
	num_samples : u32,
	reset       : u32,
	cb_context  : rawptr,
) {
	context = runtime.default_context()
	if cb_context == nil || num_samples == 0 do return

	eng := (^SDR_Engine)(cb_context)
	rx  := eng.rx_ctx
	if rx == nil || !rx.running do return

	receiver_process_sdr_samples(rx, xi, xq, int(num_samples))
}

// C callback invoked by SDRplay API for gain changes, ADC overload, or device removal
cb_sdr_event_callback :: proc "c" (
	event_id   : sdr.EventT,
	tuner      : sdr.TunerSelectT,
	params     : ^sdr.EventParamsT,
	cb_context : rawptr,
) {
	context = runtime.default_context()
	if cb_context == nil do return

	eng := (^SDR_Engine)(cb_context)
	rx  := eng.rx_ctx

	#partial switch event_id {
	case .PowerOverloadChange:
		sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Ctrl_OverloadMsgAck, .None)
		if rx != nil && params != nil {
			if params.powerOverloadParams.powerOverloadChangeType == .Overload_Detected {
				sync.mutex_lock(&rx.tel_mutex)
				rx.tel.overload_events += 1
				sync.mutex_unlock(&rx.tel_mutex)
			}
		}

	case .GainChange:
		if rx != nil && params != nil {
			sync.mutex_lock(&rx.tel_mutex)
			rx.tel.curr_total_gain = params.gainParams.currGain
			rx.tel.hw_curr_gr_db   = i32(params.gainParams.gRdB)
			rx.tel.hw_lna_gr_db    = i32(params.gainParams.lnaGRdB)
			sync.mutex_unlock(&rx.tel_mutex)
		}

	case .DeviceRemoved, .DeviceFailure:
		if rx != nil {
			rx.running = false
		}
	}
}

// Open SDRplay API, select RSP hardware, configure 6.0 MSamples/s 14-bit Zero-IF (+50 kHz offset), and start streaming
sdr_engine_start_hardware :: proc(eng: ^SDR_Engine, rx: ^Receiver_Context) -> bool {
	eng.rx_ctx = rx

	err := sdr.sdrplay_api_Open()
	if err != .Success {
		screen_eprintfln("  [ERROR] sdrplay_api_Open failed: %s", sdr.error_string(err))
		screen_eprintln("          Ensure 'sdrplay_apiService' is running or pass '-sim' for RF simulator mode.")
		return false
	}
	eng.api_opened = true

	api_ver : f32
	err = sdr.sdrplay_api_ApiVersion(&api_ver)
	if err != .Success {
		screen_eprintfln("  [ERROR] sdrplay_api_ApiVersion failed: %s", sdr.error_string(err))
		sdr_engine_stop(eng)
		return false
	}

	sdr.sdrplay_api_LockDeviceApi()
	devices  : [sdr.SDRPLAY_MAX_DEVICES]sdr.DeviceT
	num_devs : u32 = 0
	err = sdr.sdrplay_api_GetDevices(&devices[0], &num_devs, sdr.SDRPLAY_MAX_DEVICES)
	if err != .Success || num_devs == 0 {
		sdr.sdrplay_api_UnlockDeviceApi()
		if err != .Success {
			screen_eprintfln("  [ERROR] sdrplay_api_GetDevices failed: %s", sdr.error_string(err))
		} else {
			screen_eprintln("  [ERROR] No SDRplay RSP devices found on USB. (Tip: use '-sim' to test without hardware.)")
		}
		sdr_engine_stop(eng)
		return false
	}

	chosen_idx := -1
	for i in 0 ..< int(num_devs) {
		d := &devices[i]
		if d.valid != 0 {
			if chosen_idx < 0 || d.hwVer == sdr.SDRPLAY_RSPdxR2_ID || d.hwVer == sdr.SDRPLAY_RSPdx_ID {
				chosen_idx = i
			}
		}
	}

	if chosen_idx < 0 {
		sdr.sdrplay_api_UnlockDeviceApi()
		screen_eprintln("  [ERROR] SDRplay device enumerated, but none is currently available (valid == 0).")
		sdr_engine_stop(eng)
		return false
	}

	eng.dev = devices[chosen_idx]

	if eng.dev.hwVer == sdr.SDRPLAY_RSPduo_ID {
		eng.dev.rspDuoMode = .Single_Tuner
		eng.dev.tuner      = .Tuner_A
	}

	err = sdr.sdrplay_api_SelectDevice(&eng.dev)
	sdr.sdrplay_api_UnlockDeviceApi()
	if err != .Success {
		screen_eprintfln("  [ERROR] sdrplay_api_SelectDevice failed: %s", sdr.error_string(err))
		sdr_engine_stop(eng)
		return false
	}
	eng.device_selected = true

	sdr.sdrplay_api_DebugEnable(eng.dev.dev, .Disable)

	err = sdr.sdrplay_api_GetDeviceParams(eng.dev.dev, &eng.params)
	if err != .Success || eng.params == nil || eng.params.devParams == nil || eng.params.rxChannelA == nil {
		screen_eprintfln("  [ERROR] sdrplay_api_GetDeviceParams failed: %s", sdr.error_string(err))
		sdr_engine_stop(eng)
		return false
	}

	max_lna := max_lna_state_for_27mhz(eng.dev.hwVer)
	sync.mutex_lock(&rx.cfg_mutex)
	if rx.cfg.lna_state > max_lna {
		rx.cfg.lna_state = max_lna
	}
	cfg := rx.cfg
	sync.mutex_unlock(&rx.cfg_mutex)

	dp := eng.params.devParams
	ch := eng.params.rxChannelA

	// 6.0 MSamples/s -> maximum 14-bit ADC mode on SDRplay hardware!
	dp.fsFreq.fsHz = SDR_SAMPLE_RATE_HZ
	dp.ppm         = cfg.ppm_cal

	notch_u8 : u8 = 1 if cfg.rf_notch else 0
	switch eng.dev.hwVer {
	case sdr.SDRPLAY_RSPdx_ID, sdr.SDRPLAY_RSPdxR2_ID:
		dp.rspDxParams.hdrEnable        = 0
		dp.rspDxParams.biasTEnable      = 0
		dp.rspDxParams.rfNotchEnable    = notch_u8
		dp.rspDxParams.rfDabNotchEnable = notch_u8
		switch cfg.antenna_port {
		case 1:  dp.rspDxParams.antennaSel = .RspDx_ANTENNA_B
		case 2:  dp.rspDxParams.antennaSel = .RspDx_ANTENNA_C
		case:    dp.rspDxParams.antennaSel = .RspDx_ANTENNA_A
		}
		ch.rspDxTunerParams.hdrBw = .HDRMODE_BW_1_700

	case sdr.SDRPLAY_RSP1A_ID, sdr.SDRPLAY_RSP1B_ID:
		dp.rsp1aParams.rfNotchEnable    = notch_u8
		dp.rsp1aParams.rfDabNotchEnable = notch_u8
		ch.rsp1aTunerParams.biasTEnable = 0

	case sdr.SDRPLAY_RSP2_ID:
		ch.rsp2TunerParams.biasTEnable   = 0
		ch.rsp2TunerParams.rfNotchEnable = notch_u8
		ch.rsp2TunerParams.antennaSel    = .Rsp2_ANTENNA_B if cfg.antenna_port == 1 else .Rsp2_ANTENNA_A

	case sdr.SDRPLAY_RSPduo_ID:
		ch.rspDuoTunerParams.biasTEnable      = 0
		ch.rspDuoTunerParams.rfNotchEnable    = notch_u8
		ch.rspDuoTunerParams.rfDabNotchEnable = notch_u8
	}

	lo_hz := cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel)
	ch.tunerParams.rfFreq.rfHz = lo_hz
	// BW_1_536 (+-768 kHz analog passband) keeps +50 kHz completely flat AND captures all 40 CB channels at once!
	ch.tunerParams.bwType      = .BW_1_536
	ch.tunerParams.ifType      = .IF_Zero
	ch.tunerParams.loMode      = .LO_Auto

	ch.tunerParams.gain.gRdB     = clamp(cfg.rf_gr_db, 20, 59)
	ch.tunerParams.gain.LNAstate = min(cfg.lna_state, max_lna)
	ch.tunerParams.gain.minGr    = .NORMAL_MIN_GR

	ch.tunerParams.dcOffsetTuner.dcCal           = 3
	ch.tunerParams.dcOffsetTuner.speedUp         = 0
	ch.tunerParams.dcOffsetTuner.trackTime       = 1
	ch.tunerParams.dcOffsetTuner.refreshRateTime = 2048

	ch.ctrlParams.dcOffset.DCenable           = 1
	ch.ctrlParams.dcOffset.IQenable           = 1
	ch.ctrlParams.decimation.enable           = 0
	ch.ctrlParams.decimation.decimationFactor = 1
	ch.ctrlParams.decimation.wideBandSignal   = 0
	// Use smooth 5 Hz hardware AGC loop at -18 dBFS setpoint so strong adjacent CB channels
	// do not cause syllabic gain pumping on weak distant LSB stations while maximizing 14-bit ADC bits!
	ch.ctrlParams.agc.enable                  = .AGC_5HZ if cfg.hw_agc_enabled else .AGC_DISABLE
	ch.ctrlParams.agc.setPoint_dBfs           = -18
	ch.ctrlParams.adsbMode                    = .DECIMATION

	cb_fns := sdr.CallbackFnsT{
		StreamACbFn = cb_sdr_stream_callback,
		StreamBCbFn = cb_sdr_stream_callback,
		EventCbFn   = cb_sdr_event_callback,
	}

	err = sdr.sdrplay_api_Init(eng.dev.dev, &cb_fns, rawptr(eng))
	if err != .Success {
		screen_eprintfln("  [ERROR] sdrplay_api_Init (6.0 MSPS 14-bit) failed: %s", sdr.error_string(err))
		sdr_engine_stop(eng)
		return false
	}
	eng.streaming = true

	hw_name := sdr.hw_version_name(eng.dev.hwVer)
	raw_ser := sdr.serial_string(&eng.dev)
	eng.serial_len = min(len(raw_ser), len(eng.serial_buf))
	copy(eng.serial_buf[:eng.serial_len], transmute([]u8)(raw_ser[:eng.serial_len]))
	ser_str := string(eng.serial_buf[:eng.serial_len])

	sync.mutex_lock(&rx.tel_mutex)
	rx.tel.streaming       = true
	rx.tel.simulation_mode = false
	rx.tel.device_name     = hw_name
	rx.tel.device_serial   = ser_str
	rx.tel.max_lna_state   = max_lna
	rx.tel.hw_curr_gr_db   = clamp(cfg.rf_gr_db, 20, 59)
	sync.mutex_unlock(&rx.tel_mutex)

	screen_printfln("  [SDR] Connected: %s (Serial: %s, API v%.2f, 27MHz LNA States: 0..%d)",
		hw_name, ser_str, api_ver, max_lna)
	screen_println("  [SDR] Streaming: 6.000 MSamples/s @ 14-Bit ADC Depth (IF=Zero, LO Offset=-50 kHz, BW=1.536 MHz)")
	return true
}

// Apply any live runtime parameter updates requested by CLI / Scanner / Auto-LNA to active SDRplay hardware
sdr_engine_apply_pending_updates :: proc(eng: ^SDR_Engine) {
	if !eng.streaming || eng.rx_ctx == nil do return

	rx := eng.rx_ctx
	sync.mutex_lock(&rx.cfg_mutex)
	cfg := rx.cfg
	rx.cfg.hw_dirty_rf    = false
	rx.cfg.hw_dirty_gain  = false
	rx.cfg.hw_dirty_agc   = false
	rx.cfg.hw_dirty_ant   = false
	rx.cfg.hw_dirty_notch = false
	rx.cfg.hw_dirty_ppm   = false
	sync.mutex_unlock(&rx.cfg_mutex)

	// In simulator mode, flags are cleared and sim_worker_proc reads rx.cfg directly
	if eng.params == nil || !eng.device_selected do return

	dp := eng.params.devParams
	ch := eng.params.rxChannelA

	if cfg.hw_dirty_rf {
		lo_hz := cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel)
		ch.tunerParams.rfFreq.rfHz = lo_hz
		sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Tuner_Frf, .None)
	}

	if cfg.hw_dirty_agc {
		ch.ctrlParams.agc.enable        = .AGC_5HZ if cfg.hw_agc_enabled else .AGC_DISABLE
		ch.ctrlParams.agc.setPoint_dBfs = -18
		sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Ctrl_Agc, .None)
	}

	if cfg.hw_dirty_gain {
		max_lna := max_lna_state_for_27mhz(eng.dev.hwVer)
		ch.tunerParams.gain.gRdB     = clamp(cfg.rf_gr_db, 20, 59)
		ch.tunerParams.gain.LNAstate = min(cfg.lna_state, max_lna)
		sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Tuner_Gr, .None)
	}

	if cfg.hw_dirty_ppm {
		dp.ppm = cfg.ppm_cal
		sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Dev_Ppm, .None)
	}

	if cfg.hw_dirty_ant {
		switch eng.dev.hwVer {
		case sdr.SDRPLAY_RSPdx_ID, sdr.SDRPLAY_RSPdxR2_ID:
			switch cfg.antenna_port {
			case 1:  dp.rspDxParams.antennaSel = .RspDx_ANTENNA_B
			case 2:  dp.rspDxParams.antennaSel = .RspDx_ANTENNA_C
			case:    dp.rspDxParams.antennaSel = .RspDx_ANTENNA_A
			}
			sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .None, .RspDx_AntennaControl)
		case sdr.SDRPLAY_RSP2_ID:
			ch.rsp2TunerParams.antennaSel = .Rsp2_ANTENNA_B if cfg.antenna_port == 1 else .Rsp2_ANTENNA_A
			sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Rsp2_AntennaControl, .None)
		}
	}

	if cfg.hw_dirty_notch {
		notch_u8 : u8 = 1 if cfg.rf_notch else 0
		switch eng.dev.hwVer {
		case sdr.SDRPLAY_RSPdx_ID, sdr.SDRPLAY_RSPdxR2_ID:
			dp.rspDxParams.rfNotchEnable    = notch_u8
			dp.rspDxParams.rfDabNotchEnable = notch_u8
			ext := sdr.ReasonForUpdateExtension1T(u32(sdr.ReasonForUpdateExtension1T.RspDx_RfNotchControl) | u32(sdr.ReasonForUpdateExtension1T.RspDx_RfDabNotchControl))
			sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .None, ext)
		case sdr.SDRPLAY_RSP1A_ID, sdr.SDRPLAY_RSP1B_ID:
			dp.rsp1aParams.rfNotchEnable    = notch_u8
			dp.rsp1aParams.rfDabNotchEnable = notch_u8
			r := sdr.ReasonForUpdateT(u32(sdr.ReasonForUpdateT.Rsp1a_RfNotchControl) | u32(sdr.ReasonForUpdateT.Rsp1a_RfDabNotchControl))
			sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, r, .None)
		case sdr.SDRPLAY_RSP2_ID:
			ch.rsp2TunerParams.rfNotchEnable = notch_u8
			sdr.sdrplay_api_Update(eng.dev.dev, eng.dev.tuner, .Rsp2_RfNotchControl, .None)
		}
	}
}

// Closed-loop 250 ms supervisory controller for Auto LNA + Auto RF Gain.
// Coordinates SDRplay RF LNA attenuation steps (0..max_lna_27mhz) with baseband IF gRdB (20..59 dB)
// so strong HF/local signals never overload the 14-bit ADC while weak signals get maximum sensitivity!
sdr_engine_auto_gain_tick :: proc(eng: ^SDR_Engine) {
	if eng == nil || eng.rx_ctx == nil || !eng.streaming do return
	rx := eng.rx_ctx
	if iq_is_file_mode(&rx.iq_eng) do return

	sync.mutex_lock(&rx.cfg_mutex)
	cfg := rx.cfg
	sync.mutex_unlock(&rx.cfg_mutex)

	if !cfg.auto_lna && !cfg.auto_rf_gain do return
	if cfg.mute_for_scan do return // Never step RF/IF gain mid-FFT-capture during wideband 40-ch scan!

	sync.mutex_lock(&rx.tel_mutex)
	ovl_cnt   := rx.tel.overload_events
	adc_dbfs  := rx.tel.adc_dbfs
	adc_peak  := rx.tel.adc_peak_pct
	hw_gr     := rx.tel.hw_curr_gr_db
	cb_cnt    := rx.tel.callbacks_cnt
	sync.mutex_unlock(&rx.tel_mutex)

	if cb_cnt < 4 do return // Wait until initial stream callbacks populate ADC power metrics

	new_ovl := ovl_cnt > eng.last_overload_cnt
	eng.last_overload_cnt = ovl_cnt

	max_lna   := max_lna_state_for_27mhz(eng.dev.hwVer)
	new_lna   := min(cfg.lna_state, max_lna)
	new_gr    := clamp(cfg.rf_gr_db, 20, 59)
	eff_gr    := hw_gr if (cfg.hw_agc_enabled && hw_gr >= 20 && !rx.sim_mode) else new_gr
	dirty_gr  := false
	dirty_agc := false

	// 1. Auto RF Gain (IF gRdB 20..59 dB + Hardware AGC setpoint -30 dBFS)
	if cfg.auto_rf_gain {
		if !cfg.hw_agc_enabled {
			cfg.hw_agc_enabled = true
			dirty_agc          = true
		}
		// Software supervisory tracking (keeps gRdB synced & protects when HW AGC is saturated or in -sim)
		if new_ovl || adc_peak > 85.0 || adc_dbfs > -13.0 {
			new_gr = clamp(eff_gr + 5, 20, 59)
		} else if adc_dbfs > -16.5 && eff_gr < 59 {
			new_gr = clamp(eff_gr + 2, 20, 59)
		} else if adc_dbfs < -25.0 && adc_peak < 35.0 && eff_gr > 20 {
			new_gr = clamp(eff_gr - 2, 20, 59)
		}
		if new_gr != cfg.rf_gr_db {
			dirty_gr = true
		}
		eff_gr = new_gr
	}

	// 2. Auto LNA State (0 = max sensitivity .. max_lna = max front-end protection)
	// Note: SDRplay hardware AGC NEVER changes LNAstate; this loop manages LNAstate automatically!
	if cfg.auto_lna {
		if new_ovl || adc_peak > 85.0 || adc_dbfs > -13.0 {
			// Immediate front-end overload protection: step LNA attenuation UP
			step := u8(2) if new_ovl else u8(1)
			new_lna = min(new_lna + step, max_lna)
			eng.calm_lna_ticks = 0
		} else if adc_dbfs > -19.0 && eff_gr >= 52 && new_lna < max_lna {
			// Baseband IF attenuation nearing 59 dB ceiling -> hand off attenuation to RF LNA
			new_lna = min(new_lna + 1, max_lna)
			eng.calm_lna_ticks = 0
		} else if !new_ovl && adc_peak < 42.0 && adc_dbfs < -28.0 && eff_gr <= 32 && new_lna > 0 {
			// Plenty of ADC headroom & IF gain near maximum -> step LNA attenuation DOWN to 0 (min NF) after 3 calm ticks
			eng.calm_lna_ticks += 1
			if eng.calm_lna_ticks >= 3 {
				new_lna -= 1
				eng.calm_lna_ticks = 0
			}
		} else {
			eng.calm_lna_ticks = 0
		}

		if new_lna != cfg.lna_state {
			dirty_gr = true
		}
	}

	if dirty_gr || dirty_agc {
		sync.mutex_lock(&rx.cfg_mutex)
		if cfg.auto_rf_gain {
			rx.cfg.rf_gr_db       = new_gr
			rx.cfg.hw_agc_enabled = true
			if dirty_agc do rx.cfg.hw_dirty_agc = true
		}
		if cfg.auto_lna {
			rx.cfg.lna_state = new_lna
		}
		if dirty_gr do rx.cfg.hw_dirty_gain = true
		sync.mutex_unlock(&rx.cfg_mutex)

		sync.mutex_lock(&rx.tel_mutex)
		rx.tel.hw_curr_gr_db = new_gr
		sync.mutex_unlock(&rx.tel_mutex)

		sdr_engine_apply_pending_updates(eng)
	}
}

// ---------------------------------------------------------------------------
// 6.0 MSamples/s 14-Bit Multi-Channel CB Spectrum + Indoor House EMI Simulator (-sim)
// ---------------------------------------------------------------------------

// Synthesize human-like harmonic voice waveform at baseband time t_sec
synthesize_test_voice_f64 :: #force_inline proc(t_sec: f64) -> f64 {
	syl_phase := math.sin(2.0 * math.PI * 1.6 * t_sec) + 0.5 * math.sin(2.0 * math.PI * 3.1 * t_sec + 0.7)
	env       := clamp((syl_phase + 0.35) * 0.85, 0.0, 1.0)
	if env <= 0.001 do return 0.0

	f0_base  := 142.0
	phi_fund := 2.0 * math.PI * (f0_base * t_sec + (14.0 / (2.0 * math.PI * 1.1)) * math.sin(2.0 * math.PI * 1.1 * t_sec))
	f1       := 540.0 + 140.0 * math.sin(2.0 * math.PI * 1.9 * t_sec)
	f2       := 1520.0 + 340.0 * math.cos(2.0 * math.PI * 2.4 * t_sec)

	s := 0.0
	for h in 1 ..= 16 {
		fh  := f64(h) * f0_base
		d1  := (fh - f1) / 140.0
		d2  := (fh - f2) / 240.0
		amp := (1.0 / (1.0 + d1 * d1) + 0.65 / (1.0 + d2 * d2)) / math.pow(f64(h), 0.35)
		s   += amp * math.cos(f64(h) * phi_fund + 0.31 * f64(h))
	}
	return s * env * 0.36
}

// Deterministic fast xorshift64* PRNG for repeatable Gaussian + impulsive house noise in f64
xorshift_f64_normal :: #force_inline proc(state: ^u64) -> (g1: f64, g2: f64) {
	next_u64 :: #force_inline proc(s: ^u64) -> u64 {
		x := s^
		x ~= x >> 12
		x ~= x << 25
		x ~= x >> 27
		s^ = x
		return x * 0x2545F4914F6CDD1D
	}
	u1 := (f64(next_u64(state) >> 11) + 1.0) * (1.0 / 9007199254740993.0)
	u2 := (f64(next_u64(state) >> 11) + 1.0) * (1.0 / 9007199254740993.0)
	r  := math.sqrt(-2.0 * math.ln(u1))
	th := 2.0 * math.PI * u2
	return r * math.cos(th), r * math.sin(th)
}

@(private)
Sim_Station :: struct {
	channel : int,
	mode    : Modulation_Mode,
	snr_db  : f64,
}

@(private)
sim_worker_proc :: proc(t: ^thread.Thread) {
	eng := (^SDR_Engine)(t.data)
	rx  := eng.rx_ctx

	SIM_PKT :: 30_000 // 5.0 ms packet at 6,000,000 samples/sec
	xi_buf  := make([]i16, SIM_PKT)
	xq_buf  := make([]i16, SIM_PKT)
	defer delete(xi_buf)
	defer delete(xq_buf)

	rng_state  : u64 = 0xCAFE_BABE_1234_5678
	sample_idx : u64 = 0
	fm_int_rad : f64 = 0.0
	voice_cur  : f64 = 0.0
	voice_hilb : f64 = 0.0
	voice2_cur : f64 = 0.0
	voice2_hlb : f64 = 0.0
	voice2_fm  : f64 = 0.0

	dt_6m   := 1.0 / SDR_SAMPLE_RATE_HZ
	sim_t0  := time.tick_now()
	pkt_idx : i64 = 0

	for !eng.sim_stop && rx.running {

		sync.mutex_lock(&rx.cfg_mutex)
		cfg         := rx.cfg
		user_snr_db := rx.sim_snr_db
		custom_ch   := rx.sim_custom_ch
		custom_mode := rx.sim_custom_mode
		sync.mutex_unlock(&rx.cfg_mutex)

		// Active hardware LO in Hz (either wideband scan LO 26.880 MHz or channel LO - 50 kHz)
		lo_hz := cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel)

		// Simulated antenna + RF gain scaling:
		// Port C (BNC=2) has strongest 27 MHz coupling (+2 dB), Port A (0) nominal (0 dB), Port B (1) disconnected (-18 dB)
		ant_db := 0.0
		if cfg.antenna_port == 2 do ant_db = +2.5
		if cfg.antenna_port == 1 do ant_db = -18.0
		gain_Offset_db := (34.0 - f64(cfg.rf_gr_db)) - f64(cfg.lna_state) * 3.0 + ant_db
		rf_gain_lin    := clamp(math.pow(10.0, gain_Offset_db / 20.0), 0.02, 8.0)

		noise_sigma      := 0.045 * rf_gain_lin + 0.003 // 0.003 = internal receiver thermal floor
		inband_noise_rms := noise_sigma * math.sqrt(2680.0 / SDR_SAMPLE_RATE_HZ)

		// Multi-station realistic Portuguese / European CB spectrum across all 3 bands (AM, FM, SSB [LSB & USB]):
		//   Ch 19 (27.185 MHz) AM  | Ch 31 (27.315 MHz) FM
		//   Ch 34 (27.345 MHz) USB | Ch 38 (27.385 MHz) LSB
		stations : [5]Sim_Station
		num_st   := 4
		stations[0] = {19, .AM,  max(user_snr_db, 18.0) if cfg.lo_override_hz > 0.0 else (user_snr_db if cfg.channel == 19 else 18.0)}
		stations[1] = {31, .FM,  20.0 if cfg.channel != 31 else max(user_snr_db, 14.0)}
		stations[2] = {34, .USB, 17.0 if cfg.channel != 34 else user_snr_db}
		stations[3] = {38, .LSB, 16.0 if cfg.channel != 38 else user_snr_db}
		if custom_ch > 0 && custom_ch != 19 && custom_ch != 31 && custom_ch != 34 && custom_ch != 38 {
			stations[4] = {custom_ch, custom_mode, user_snr_db}
			num_st = 5
		}

		st_freq_if : [5]f64
		st_scale   : [5]f64
		st_active  : [5]bool
		for s_i in 0 ..< num_st {
			df := channel_to_freq_hz(stations[s_i].channel) - lo_hz
			// Analog BW_1_536 passband is [-768 kHz, +768 kHz]
			if math.abs(df) <= 740_000.0 && cfg.antenna_port != 1 {
				st_active[s_i]  = true
				st_freq_if[s_i] = df
				t_rms          := inband_noise_rms * math.pow(10.0, stations[s_i].snr_db / 20.0)
				st_scale[s_i]   = t_rms / 0.30
			}
		}

		for n in 0 ..< SIM_PKT {
			if (sample_idx % 125) == 0 {
				t_s := f64(sample_idx) * dt_6m
				// Voice 1 (F0 ~ 142 Hz, continuous natural syllabic cadence without flat-topping)
				syl1 := 0.65 * math.sin(2.0 * math.PI * 3.2 * t_s) + 0.35 * math.sin(2.0 * math.PI * 5.7 * t_s + 0.7)
				env1 := 0.58 + 0.40 * syl1
				f0_1 := 142.0
				phi1 := 2.0 * math.PI * (f0_1 * t_s + (14.0 / (2.0 * math.PI * 1.1)) * math.sin(2.0 * math.PI * 1.1 * t_s))
				f1_1 := 540.0 + 140.0 * math.sin(2.0 * math.PI * 1.9 * t_s)
				f2_1 := 1520.0 + 340.0 * math.cos(2.0 * math.PI * 2.4 * t_s)
				sr1, si1 := 0.0, 0.0
				for h in 1 ..= 14 {
					fh  := f64(h) * f0_1
					d1  := (fh - f1_1) / 140.0
					d2  := (fh - f2_1) / 240.0
					amp := (1.0 / (1.0 + d1 * d1) + 0.65 / (1.0 + d2 * d2)) / math.pow(f64(h), 0.35)
					ang := f64(h) * phi1 + 0.31 * f64(h)
					sr1 += amp * math.cos(ang)
					si1 += amp * math.sin(ang)
				}
				voice_cur  = sr1 * env1 * 0.36
				voice_hilb = si1 * env1 * 0.36

				// Voice 2 (F0 ~ 185 Hz, distinct operator for FM/USB channels)
				env2 := clamp(0.65 + 0.35 * math.sin(2.0 * math.PI * 2.3 * t_s + 1.2), 0.25, 1.0)
				f0_2 := 185.0
				phi2 := 2.0 * math.PI * (f0_2 * t_s + (11.0 / (2.0 * math.PI * 1.4)) * math.sin(2.0 * math.PI * 1.4 * t_s))
				sr2, si2, sfm2 := 0.0, 0.0, 0.0
				for h in 1 ..= 12 {
					fh     := f64(h) * f0_2
					d1     := (fh - 620.0) / 160.0
					d2     := (fh - 1680.0) / 260.0
					amp    := (1.0 / (1.0 + d1 * d1) + 0.60 / (1.0 + d2 * d2)) / math.pow(f64(h), 0.35)
					ang    := f64(h) * phi2 + 0.19 * f64(h)
					c_ang  := math.cos(ang)
					sr2    += amp * c_ang
					si2    += amp * math.sin(ang)
					// CEPT 530 us (f_c = 300 Hz) +6 dB/oct CB FM transmitter pre-emphasis
					pre_em := math.min(math.sqrt(1.0 + (fh / 300.0) * (fh / 300.0)) * 0.34, 2.8)
					sfm2   += amp * pre_em * c_ang
				}
				voice2_cur = sr2  * env2 * 0.36
				voice2_hlb = si2  * env2 * 0.36
				voice2_fm  = clamp(sfm2 * env2 * 0.36, -0.98, +0.98)
			}

			t_s   := f64(sample_idx) * dt_6m
			fm_int_rad = wrap_pi_f64(fm_int_rad + (2.0 * math.PI * 1950.0 * voice2_fm) * dt_6m)

			sig_i := 0.0
			sig_q := 0.0

			for s_i in 0 ..< num_st {
				if !st_active[s_i] do continue
				cyc    := math.mod(st_freq_if[s_i] * t_s, 1.0)
				phi_if := 2.0 * math.PI * cyc
				sc     := st_scale[s_i]

				switch stations[s_i].mode {
				case .AM:
					carr := sc * 1.8 * (1.0 + 0.82 * voice_cur)
					sig_i += carr * math.cos(phi_if)
					sig_q += carr * math.sin(phi_if)
				case .FM:
					ang    := phi_if + fm_int_rad
					fm_amp := sc * 2.0
					sig_i  += fm_amp * math.cos(ang)
					sig_q  += fm_amp * math.sin(ang)
				case .USB:
					cif := math.cos(phi_if)
					sif := math.sin(phi_if)
					ai  := voice2_cur * sc
					aq  := voice2_hlb * sc
					sig_i += ai * cif - aq * sif
					sig_q += ai * sif + aq * cif
				case .LSB:
					cif := math.cos(phi_if)
					sif := math.sin(phi_if)
					ai  :=  voice_cur  * sc
					aq  := -voice_hilb * sc
					sig_i += ai * cif - aq * sif
					sig_q += ai * sif + aq * cif
				}
			}

			// Composite Indoor House Noise: thermal Gaussian + 50/100 Hz hum + 1380 Hz SMPS birdie + triac spikes
			gn_i, gn_q := xorshift_f64_normal(&rng_state)
			fc_tuned   := -SDR_LO_OFFSET_HZ
			birdie_ang := 2.0 * math.PI * math.mod((fc_tuned + 1380.0) * t_s, 1.0)
			hum_ang    := 2.0 * math.PI * math.mod((fc_tuned + 100.0) * t_s, 1.0)

			ni := gn_i * noise_sigma + (0.0012 * math.cos(birdie_ang) + 0.0010 * math.cos(hum_ang)) * rf_gain_lin
			nq := gn_q * noise_sigma + (0.0012 * math.sin(birdie_ang) + 0.0010 * math.sin(hum_ang)) * rf_gain_lin

			// 100 Hz indoor triac dimmer microsecond spike (6 samples = 1 us every 10 ms)
			if (sample_idx % 60_000) < 6 {
				ni += 0.25 * rf_gain_lin
				nq -= 0.20 * rf_gain_lin
			}

			// Quantize to 14-bit SDRplay ADC resolution scaled into 16-bit container
			tot_i := sig_i + ni
			tot_q := sig_q + nq
			q14_i := math.round(clamp(tot_i, -0.999, +0.999) * 8191.0) * 4.0
			q14_q := math.round(clamp(tot_q, -0.999, +0.999) * 8191.0) * 4.0
			xi_buf[n] = i16(clamp(q14_i, -32767.0, 32767.0))
			xq_buf[n] = i16(clamp(q14_q, -32767.0, 32767.0))

			sample_idx += 1
		}

		receiver_process_sdr_samples(rx, &xi_buf[0], &xq_buf[0], SIM_PKT)

		// Monotonic crystal-accurate 5.000 ms packet pacing (eliminates cumulative OS timer-slack drift)
		pkt_idx        += 1
		target_elapsed := time.Duration(pkt_idx * 5) * time.Millisecond
		actual_elapsed := time.tick_since(sim_t0)
		if actual_elapsed < target_elapsed {
			time.sleep(target_elapsed - actual_elapsed)
		} else if actual_elapsed - target_elapsed > 250 * time.Millisecond {
			sim_t0  = time.tick_now()
			pkt_idx = 0
		}
	}
}

sdr_engine_start_simulation :: proc(eng: ^SDR_Engine, rx: ^Receiver_Context) -> bool {
	eng.rx_ctx    = rx
	eng.sim_stop  = false
	eng.streaming = true

	sync.mutex_lock(&rx.tel_mutex)
	rx.tel.streaming       = true
	rx.tel.simulation_mode = true
	rx.tel.device_name     = "6 MSPS 14-Bit Multi-Channel CB + House EMI Sim"
	rx.tel.device_serial   = "SIM-14BIT-6MSPS"
	rx.tel.max_lna_state   = 19
	sync.mutex_unlock(&rx.tel_mutex)

	eng.sim_thread = thread.create(sim_worker_proc)
	eng.sim_thread.data = rawptr(eng)
	thread.start(eng.sim_thread)

	screen_printfln("  [SIM] Running 6.000 MSamples/s 14-Bit Multi-Channel CB Spectrum (Ch 19 AM, Ch 31 FM, Ch 34 USB, Ch 38 LSB @ %.1f dB)", rx.sim_snr_db)
	return true
}

sdr_engine_stop :: proc(eng: ^SDR_Engine) {
	if eng.sim_thread != nil {
		eng.sim_stop = true
		thread.join(eng.sim_thread)
		thread.destroy(eng.sim_thread)
		eng.sim_thread = nil
		eng.streaming  = false
	}

	if eng.streaming && eng.device_selected {
		err := sdr.sdrplay_api_Uninit(eng.dev.dev)
		if err == .StopPending {
			time.sleep(500 * time.Millisecond)
			sdr.sdrplay_api_Uninit(eng.dev.dev)
		}
		eng.streaming = false
	}

	if eng.device_selected {
		sdr.sdrplay_api_ReleaseDevice(&eng.dev)
		eng.device_selected = false
	}

	if eng.api_opened {
		sdr.sdrplay_api_Close()
		eng.api_opened = false
	}
}
