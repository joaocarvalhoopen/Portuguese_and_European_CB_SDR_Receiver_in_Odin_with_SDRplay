// Portuguese (ANACOM) & European (CEPT EN 300 433) 40-Channel CB SDR Receiver in Odin
//   - 6.0 MSamples/s Hardware ADC Rate -> Maximum 14-Bit SDRplay Depth
//   - 100% 64-Bit Floating-Point (f64) DSP Pipeline & Custom Radix-2 FFT from Scratch
//   - AM (Synchronous PLL), FM (Threshold-Extension PLL), SSB (LSB & USB Analytic FFT)
//   - 40-Channel × 3-Band (AM / FM / SSB) Auto-Scanner, Auto-LNA & Auto-RF-Gain, Antenna Prober
//   - 4 Progressive Band-Specific Indoor House & Channel Noise Filters (Recovering SSB -20..-30 dB)
//   - Audio Output via Odin vendor:miniaudio, Plain Text-Mode CLI

package main

import "core:os"
import "core:strconv"
import "core:strings"

import sdr "../SDRplay_API_Odin_bindings/sdrplay_api_bindings"

print_startup_usage :: proc(prog: string) {
	frame : Screen_Frame
	frame_reset(&frame)
	frame_println(&frame, "================================================================================")
	frame_println(&frame, "  Portuguese & European Norm 40-Channel CB SDR Receiver (Odin + SDRplay 14-Bit)")
	frame_println(&frame, "================================================================================")
	frame_printfln(&frame, "  Usage: %s [channel 1..40] [options]", prog)
	frame_println(&frame)
	frame_println(&frame, "  Options:")
	frame_println(&frame, "    -ch <1..40>          Initial CB channel (1..40, e.g. 19 or 38)")
	frame_println(&frame, "    -mode <am|fm|lsb|usb> Initial modulation (default: am)")
	frame_println(&frame, "    -scan [all|am|fm|ssb] Start 40-ch Auto-Scanner across AM, FM & SSB on launch")
	frame_println(&frame, "    -sp [on|off]         Enable live 40-ch Braille Spectrum & 60s Waterfall (-spetrum)")
	frame_println(&frame, "    -save [dir|file]     Start saving 6.0 MSPS 14-Bit raw spectrum -> IQ_file_xxxx.samples")
	frame_println(&frame, "    -play [id|file]      Open & play saved IQ_file_xxxx.samples from disk on launch")
	frame_println(&frame, "    -freqcheck           Sweep all 40 ch & print exact RF frequency check on launch")
	frame_println(&frame, "    -diag <1..40>        Run deep live RF & 4-filter diagnostic on channel & exit")
	frame_println(&frame, "    -filter <0..4>       Audio noise filter 0..4 (default: 2; 3 = -15dB, 4 = -20..-30dB)")
	frame_println(&frame, "    -notch [on|off]      Enable RF/DAB + DSP Narrowband Heterodyne Auto-Notch")
	frame_println(&frame, "    -vol <0..200>        Initial audio volume % (default: 90)")
	frame_println(&frame, "    -sq <auto|dBFS|off>  Initial squelch ('auto', manual dBFS, or 'off'; default: off)")
	frame_println(&frame, "    -autosq [on|off]     Enable/disable Adaptive Auto-Squelch on startup")
	frame_println(&frame, "    -clar <auto|off|Hz>  Auto-Clarifier ('auto', default), 'off', or manual Hz (-5000..+5000)")
	frame_println(&frame, "    -gain <auto|20..59>  Auto RF Gain ('auto', default) or manual IF gRdB (20..59)")
	frame_println(&frame, "    -lna <auto|0..19>    Auto LNA ('auto', default) or manual 27 MHz LNA state")
	frame_println(&frame, "    -ant <auto|a|b|c>    Antenna port A, B, C or 'auto' probe (default: auto on RSPdx/RSP2)")
	frame_println(&frame, "    -ppm <value>         Crystal frequency calibration in PPM (default: 0.0)")
	frame_println(&frame, "    -sim                 Run with 6.0 MSPS 14-Bit multi-channel CB + house EMI simulator")
	frame_println(&frame, "    -simsnr <dB>         Simulated voice SNR in dB when using -sim (default: -24)")
	frame_println(&frame, "    -selftest            Run built-in f64 DSP, FFT, Freq, Scanner & -30dB SSB test suite")
	frame_println(&frame, "    -help                Show this help screen")
	frame_println(&frame, "================================================================================")
	screen_submit_frame(&frame)
	screen_flush()
}

main :: proc() {
	screen_init()
	defer screen_shutdown()

	rx := new(Receiver_Context)
	defer free(rx)
	receiver_context_init(rx)
	defer receiver_context_shutdown(rx)

	args          := os.args
	channel_set   := false
	run_selftest  := false
	start_scan    := false
	init_scan_bf  := Scan_Band_Filter.All_Bands
	run_freqcheck := false
	diag_ch       := 0
	ant_explicit  := false
	ant_auto_req  := false
	init_save_iq  := false
	init_save_arg := ""
	init_play_iq  := false
	init_play_arg := ""

	i := 1
	for i < len(args) {
		arg := args[i]

		if arg == "-h" || arg == "-help" || arg == "--help" {
			print_startup_usage(args[0])
			return
		}
		if arg == "-selftest" || arg == "--selftest" {
			run_selftest = true
			i += 1
			continue
		}
		if arg == "-sim" || arg == "--sim" {
			rx.sim_mode = true
			i += 1
			continue
		}
		if arg == "-save" || arg == "--save" {
			init_save_iq = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				if _, ch_ok := strconv.parse_int(args[i + 1]); !ch_ok {
					init_save_arg = args[i + 1]
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if arg == "-play" || arg == "--play" || arg == "-read" || arg == "--read" {
			init_play_iq = true
			channel_set  = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				init_play_arg = args[i + 1]
				i += 2
				continue
			}
			i += 1
			continue
		}
		if arg == "-freqcheck" || arg == "--freqcheck" {
			run_freqcheck = true
			i += 1
			continue
		}
		if arg == "-diag" || arg == "--diag" {
			diag_ch     = CB_DEFAULT_CHANNEL
			channel_set = true
			if i + 1 < len(args) {
				if dch, ok := strconv.parse_int(args[i + 1]); ok && is_valid_channel(dch) {
					diag_ch          = dch
					rx.cfg.channel   = dch
					rx.last_channel  = dch
					rx.sim_custom_ch = dch
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if arg == "-scan" || arg == "--scan" {
			start_scan  = true
			channel_set = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				if bf, ok := parse_scan_band_arg(args[i + 1]); ok {
					init_scan_bf = bf
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if (arg == "-ch" || arg == "--ch") && i + 1 < len(args) {
			if ch, ok := strconv.parse_int(args[i + 1]); ok && is_valid_channel(ch) {
				rx.cfg.channel   = ch
				rx.last_channel  = ch
				rx.sim_custom_ch = ch
				channel_set      = true
			} else {
				screen_eprintfln("Invalid channel '%s' (must be 1..40).", args[i + 1])
				screen_shutdown()
				os.exit(1)
			}
			i += 2
			continue
		}
		if (arg == "-mode" || arg == "--mode") && i + 1 < len(args) {
			if m, ok := parse_modulation_mode(args[i + 1]); ok {
				rx.cfg.mode        = m
				rx.last_mode       = m
				rx.sim_custom_mode = m
			} else {
				screen_eprintfln("Invalid mode '%s' (choose am, fm, lsb, usb).", args[i + 1])
				screen_shutdown()
				os.exit(1)
			}
			i += 2
			continue
		}
		if (arg == "-filter" || arg == "--filter") && i + 1 < len(args) {
			if fl, ok := strconv.parse_int(args[i + 1]); ok && fl >= 0 && fl <= 4 {
				rx.cfg.filter_level = Filter_Level(fl)
			} else {
				screen_eprintfln("Invalid filter level '%s' (choose 0..4).", args[i + 1])
				screen_shutdown()
				os.exit(1)
			}
			i += 2
			continue
		}
		if (arg == "-vol" || arg == "--vol") && i + 1 < len(args) {
			if v, ok := strconv.parse_f64(args[i + 1]); ok {
				rx.cfg.volume_pct = clamp(v, 0.0, 200.0)
			}
			i += 2
			continue
		}
		if arg == "-sp" || arg == "--sp" || arg == "-spetrum" || arg == "--spetrum" || arg == "-spectrum" || arg == "--spectrum" {
			rx.cfg.waterfall_enabled = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				sub := args[i + 1]
				if strings.equal_fold(sub, "off") || sub == "0" {
					rx.cfg.waterfall_enabled = false
					i += 2
					continue
				} else if strings.equal_fold(sub, "on") || sub == "1" {
					rx.cfg.waterfall_enabled = true
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if arg == "-autosq" || arg == "--autosq" || arg == "-autosquesh" || arg == "--autosquesh" || arg == "-autosquelch" || arg == "--autosquelch" {
			rx.cfg.squelch_enabled = true
			rx.cfg.auto_squelch    = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				sub := args[i + 1]
				if strings.equal_fold(sub, "off") || sub == "0" {
					rx.cfg.auto_squelch    = false
					rx.cfg.squelch_enabled = false
					i += 2
					continue
				} else if strings.equal_fold(sub, "on") || strings.equal_fold(sub, "auto") || sub == "1" {
					rx.cfg.squelch_enabled = true
					rx.cfg.auto_squelch    = true
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if (arg == "-sq" || arg == "--sq" || arg == "-squesh" || arg == "--squesh") && i + 1 < len(args) {
			sq_arg := args[i + 1]
			if strings.equal_fold(sq_arg, "off") {
				rx.cfg.squelch_enabled = false
				rx.cfg.auto_squelch    = false
			} else if strings.equal_fold(sq_arg, "auto") {
				rx.cfg.squelch_enabled = true
				rx.cfg.auto_squelch    = true
			} else if sqv, ok := strconv.parse_f64(sq_arg); ok {
				rx.cfg.squelch_dbfs    = clamp(sqv, -135.0, 0.0)
				rx.cfg.squelch_enabled = true
				rx.cfg.auto_squelch    = false
			}
			i += 2
			continue
		}
		if arg == "-autoclar" || arg == "--autoclar" {
			rx.cfg.auto_clarifier = true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				sub := args[i + 1]
				if strings.equal_fold(sub, "off") || sub == "0" {
					rx.cfg.auto_clarifier = false
					i += 2
					continue
				} else if strings.equal_fold(sub, "on") || strings.equal_fold(sub, "auto") || sub == "1" {
					rx.cfg.auto_clarifier = true
					i += 2
					continue
				}
			}
			i += 1
			continue
		}
		if (arg == "-clar" || arg == "--clar") && i + 1 < len(args) {
			c_arg := args[i + 1]
			if strings.equal_fold(c_arg, "auto") || strings.equal_fold(c_arg, "on") {
				rx.cfg.auto_clarifier = true
				rx.cfg.clarifier_hz   = 0.0
			} else if strings.equal_fold(c_arg, "off") {
				rx.cfg.auto_clarifier = false
			} else if clv, ok := strconv.parse_f64(c_arg); ok {
				rx.cfg.auto_clarifier = false
				rx.cfg.clarifier_hz   = clamp(clv, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
			}
			i += 2
			continue
		}
		if (arg == "-gain" || arg == "--gain") && i + 1 < len(args) {
			if strings.equal_fold(args[i + 1], "auto") {
				rx.cfg.auto_rf_gain   = true
				rx.cfg.hw_agc_enabled = true
			} else if gr, ok := strconv.parse_int(args[i + 1]); ok {
				rx.cfg.auto_rf_gain   = false
				rx.cfg.hw_agc_enabled = false
				rx.cfg.rf_gr_db       = i32(clamp(gr, 20, 59))
			}
			i += 2
			continue
		}
		if (arg == "-lna" || arg == "--lna") && i + 1 < len(args) {
			if strings.equal_fold(args[i + 1], "auto") {
				rx.cfg.auto_lna = true
			} else if st, ok := strconv.parse_int(args[i + 1]); ok {
				rx.cfg.auto_lna  = false
				rx.cfg.lna_state = u8(clamp(st, 0, 27))
			}
			i += 2
			continue
		}
		if (arg == "-ant" || arg == "--ant") && i + 1 < len(args) {
			a := args[i + 1]
			if strings.equal_fold(a, "auto") {
				ant_auto_req = true
				ant_explicit = true
			} else if strings.equal_fold(a, "a") {
				rx.cfg.antenna_port = 0
				ant_explicit        = true
			} else if strings.equal_fold(a, "b") {
				rx.cfg.antenna_port = 1
				ant_explicit        = true
			} else if strings.equal_fold(a, "c") {
				rx.cfg.antenna_port = 2
				ant_explicit        = true
			}
			i += 2
			continue
		}
		if arg == "-notch" || arg == "--notch" {
			en := true
			if i + 1 < len(args) && !strings.has_prefix(args[i + 1], "-") {
				if strings.equal_fold(args[i + 1], "on") || args[i + 1] == "1" {
					en = true
					i += 2
					rx.cfg.rf_notch       = en
					rx.cfg.hw_dirty_notch = true
					continue
				} else if strings.equal_fold(args[i + 1], "off") || args[i + 1] == "0" {
					en = false
					i += 2
					rx.cfg.rf_notch       = en
					rx.cfg.hw_dirty_notch = true
					continue
				}
			}
			rx.cfg.rf_notch       = en
			rx.cfg.hw_dirty_notch = true
			i += 1
			continue
		}
		if (arg == "-ppm" || arg == "--ppm") && i + 1 < len(args) {
			if pv, ok := strconv.parse_f64(args[i + 1]); ok {
				rx.cfg.ppm_cal = clamp(pv, -200.0, 200.0)
			}
			i += 2
			continue
		}
		if (arg == "-simsnr" || arg == "--simsnr") && i + 1 < len(args) {
			if sv, ok := strconv.parse_f64(args[i + 1]); ok {
				rx.sim_snr_db = clamp(sv, -35.0, +40.0)
			}
			i += 2
			continue
		}

		// Bare positional CB channel number (e.g. ./cb_receiver.exe 19)
		if ch_pos, ok := strconv.parse_int(arg); ok && is_valid_channel(ch_pos) {
			rx.cfg.channel   = ch_pos
			rx.last_channel  = ch_pos
			rx.sim_custom_ch = ch_pos
			channel_set      = true
			i += 1
			continue
		}

		screen_eprintfln("Unknown option '%s'. Run with -help for usage.", arg)
		screen_shutdown()
		os.exit(1)
	}

	if run_selftest {
		if !run_complete_dsp_selftest() {
			screen_shutdown()
			os.exit(1)
		}
		return
	}

	{
		banner : Screen_Frame
		frame_reset(&banner)
		frame_println(&banner, "================================================================================")
		frame_println(&banner, "  PORTUGUESE & EUROPEAN NORM 40-CHANNEL CB SDR RECEIVER")
		frame_println(&banner, "  6.000 MSamples/s @ 14-Bit ADC | Pure f64 DSP | AM / FM / LSB / USB")
		frame_println(&banner, "  Double-Buffered Async Screen Thread | Lock-Free SPSC Audio Ring | 40-Ch Scanner")
		frame_println(&banner, "================================================================================")
		screen_submit_frame(&banner)
	}

	// Prompt in plain text for initial CB channel (or 'scan') if not passed via CLI argument
	if !channel_set && !run_freqcheck {
		screen_print("  Enter CB Channel (1-40) or 'scan' [Press Enter for Ch 19 - 27.185 MHz]: ")
		screen_flush()
		in_buf : [64]u8
		if line, ok := read_stdin_line(in_buf[:]); ok {
			trimmed := strings.trim_space(line)
			if len(trimmed) > 0 {
				if strings.equal_fold(trimmed, "scan") {
					start_scan = true
				} else if ch_in, cok := strconv.parse_int(trimmed); cok && is_valid_channel(ch_in) {
					rx.cfg.channel   = ch_in
					rx.last_channel  = ch_in
					rx.sim_custom_ch = ch_in
				} else {
					screen_printfln("  [Note] Using default Channel %d (27.185 MHz).", CB_DEFAULT_CHANNEL)
				}
			}
		}
	}

	// Start SDRplay hardware (or 6 MSPS 14-bit simulator when -sim is specified)
	sdr_eng : SDR_Engine
	started := false
	if rx.sim_mode {
		started = sdr_engine_start_simulation(&sdr_eng, rx)
	} else {
		started = sdr_engine_start_hardware(&sdr_eng, rx)
		if !started && init_play_iq {
			rx.sim_mode = true
			started     = sdr_engine_start_simulation(&sdr_eng, rx)
		}
	}
	if !started {
		screen_shutdown()
		os.exit(1)
	}
	defer sdr_engine_stop(&sdr_eng)

	// Automatically probe antenna ports (A, B, C BNC) on multi-port RSPdx/RSPdx-R2/RSP2 hardware
	// when user did not manually pin -ant a|b|c, OR when -ant auto was explicitly requested!
	hw_has_multi_ant := !rx.sim_mode && !init_play_iq && (
		sdr_eng.dev.hwVer == sdr.SDRPLAY_RSPdx_ID ||
		sdr_eng.dev.hwVer == sdr.SDRPLAY_RSPdxR2_ID ||
		sdr_eng.dev.hwVer == sdr.SDRPLAY_RSP2_ID
	)
	if (ant_auto_req && !init_play_iq) || (!ant_explicit && hw_has_multi_ant) {
		scanner_probe_antennas(rx, &sdr_eng, true)
	}

	if init_play_iq {
		iq_play_cmd(&rx.iq_eng, init_play_arg, true)
	} else if init_save_iq {
		iq_save_start(&rx.iq_eng, init_save_arg, true)
	}

	if diag_ch > 0 {
		run_channel_deep_diagnostic(rx, &sdr_eng, diag_ch)
		return
	}

	if run_freqcheck {
		rep := scanner_run_oneshot_sweep(rx, &sdr_eng, .All_Bands, 110)
		print_spectrum_scan_report(rx, &rep, .All_Bands)
	}

	// Start miniaudio playback
	aud_eng : Audio_Engine
	audio_engine_start(&aud_eng, rx)
	defer audio_engine_stop(&aud_eng)

	// Enter plain text interactive CLI loop
	run_plain_cli_loop(rx, &sdr_eng, start_scan, init_scan_bf)
	screen_println("  CB SDR Receiver stopped cleanly.")
}
