// Plain Text-Mode Command Line Interface (Zero complex TTY / raw mode / ANSI escape codes).
// Reads standard text lines from stdin, displays CB channel, frequency, signal dB, S-meter,
// noise floor, SNR, filter voice-recovery metrics, 40-channel 3-band Auto-Scanner,
// RF Frequency Calibration check, Antenna Port Auto-Prober, and Auto-LNA / Auto-RF-Gain controls.

package main

import "core:fmt"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:thread"
import "core:time"

CLI_MAX_LINE_LEN :: 240
CLI_HISTORY_CAP  :: 128

CLI_Command_History :: struct {
	entries : [CLI_HISTORY_CAP][CLI_MAX_LINE_LEN]u8,
	lengths : [CLI_HISTORY_CAP]int,
	count   : int,
}

Line_Edit_Action :: enum i32 {
	Continue  = 0,
	Redraw    = 1,
	Submit    = 2,
	Abort_EOF = 3,
}

CLI_Line_Editor :: struct {
	buf       : [CLI_MAX_LINE_LEN]u8,
	length    : int,
	cursor    : int,
	prev_draw : int,
	draft_buf : [CLI_MAX_LINE_LEN]u8,
	draft_len : int,
	hist_nav  : int,
	esc_state : int,
	esc_param : u8,
}

@(private)
g_cli_prompt_mutex  : sync.Mutex
@(private)
g_cli_active_editor : ^CLI_Line_Editor
@(private)
g_cli_editing_now   : bool
@(private)
g_cli_history       : CLI_Command_History

cli_history_push :: proc(hist: ^CLI_Command_History, raw_cmd: string) {
	trimmed := strings.trim_space(raw_cmd)
	if len(trimmed) == 0 do return
	n := min(len(trimmed), CLI_MAX_LINE_LEN)

	// Avoid storing consecutive duplicate commands
	if hist.count > 0 {
		last_i := hist.count - 1
		if hist.lengths[last_i] == n && string(hist.entries[last_i][:n]) == trimmed[:n] {
			return
		}
	}

	slot := hist.count
	if slot >= CLI_HISTORY_CAP {
		for i in 0 ..< CLI_HISTORY_CAP - 1 {
			hist.entries[i] = hist.entries[i + 1]
			hist.lengths[i] = hist.lengths[i + 1]
		}
		slot = CLI_HISTORY_CAP - 1
	} else {
		hist.count += 1
	}
	copy(hist.entries[slot][:n], trimmed[:n])
	hist.lengths[slot] = n
}

line_editor_reset :: proc(ed: ^CLI_Line_Editor, hist: ^CLI_Command_History) {
	ed.length    = 0
	ed.cursor    = 0
	ed.prev_draw = 0
	ed.draft_len = 0
	ed.hist_nav  = hist.count if hist != nil else 0
	ed.esc_state = 0
	ed.esc_param = 0
}

// Feed one keystroke byte into the pure-Odin line editor (Up/Down history, Left/Right, Backspace/Delete, Insert, Enter)
line_editor_feed_byte :: proc(ed: ^CLI_Line_Editor, hist: ^CLI_Command_History, b: u8) -> Line_Edit_Action {
	// Handle standard terminal arrow/key escape sequences (\x1b[A, \x1b[B, \x1b[C, \x1b[D, \x1b[3~, Home/End)
	if ed.esc_state == 1 {
		if b == '[' || b == 'O' {
			ed.esc_state = 2
			return .Continue
		}
		ed.esc_state = 0
		return .Continue
	}
	if ed.esc_state == 2 {
		switch b {
		case 'A': // Arrow Up -> older command in history
			ed.esc_state = 0
			if hist != nil && hist.count > 0 && ed.hist_nav > 0 {
				if ed.hist_nav == hist.count {
					copy(ed.draft_buf[:ed.length], ed.buf[:ed.length])
					ed.draft_len = ed.length
				}
				ed.hist_nav -= 1
				hlen := hist.lengths[ed.hist_nav]
				copy(ed.buf[:hlen], hist.entries[ed.hist_nav][:hlen])
				ed.length = hlen
				ed.cursor = hlen
				return .Redraw
			}
			return .Continue

		case 'B': // Arrow Down -> newer command / draft in history
			ed.esc_state = 0
			if hist != nil && ed.hist_nav < hist.count {
				ed.hist_nav += 1
				if ed.hist_nav == hist.count {
					copy(ed.buf[:ed.draft_len], ed.draft_buf[:ed.draft_len])
					ed.length = ed.draft_len
					ed.cursor = ed.draft_len
				} else {
					hlen := hist.lengths[ed.hist_nav]
					copy(ed.buf[:hlen], hist.entries[ed.hist_nav][:hlen])
					ed.length = hlen
					ed.cursor = hlen
				}
				return .Redraw
			}
			return .Continue

		case 'D': // Arrow Left
			ed.esc_state = 0
			if ed.cursor > 0 {
				ed.cursor -= 1
				return .Redraw
			}
			return .Continue

		case 'C': // Arrow Right
			ed.esc_state = 0
			if ed.cursor < ed.length {
				ed.cursor += 1
				return .Redraw
			}
			return .Continue

		case 'H': // Home
			ed.esc_state = 0
			ed.cursor    = 0
			return .Redraw

		case 'F': // End
			ed.esc_state = 0
			ed.cursor    = ed.length
			return .Redraw
		}

		if b >= '0' && b <= '9' {
			ed.esc_param = b
			ed.esc_state = 3
			return .Continue
		}
		ed.esc_state = 0
		return .Continue
	}
	if ed.esc_state == 3 {
		if b == '~' {
			ed.esc_state = 0
			switch ed.esc_param {
			case '3': // Delete key (erase character under cursor)
				if ed.cursor < ed.length {
					for k := ed.cursor; k < ed.length - 1; k += 1 {
						ed.buf[k] = ed.buf[k + 1]
					}
					ed.length -= 1
					return .Redraw
				}
			case '1', '7': // Home
				ed.cursor = 0
				return .Redraw
			case '4', '8': // End
				ed.cursor = ed.length
				return .Redraw
			}
			return .Continue
		}
		if (b >= '0' && b <= '9') || b == ';' {
			return .Continue
		}
		ed.esc_state = 0
		return .Continue
	}

	if b == 0x1B {
		ed.esc_state = 1
		return .Continue
	}
	if b == '\r' || b == '\n' {
		return .Submit
	}
	if b == 0x03 { // Ctrl+C -> clean shutdown
		return .Abort_EOF
	}
	if b == 0x04 { // Ctrl+D -> EOF on empty line, forward delete on non-empty line
		if ed.length == 0 {
			return .Abort_EOF
		}
		if ed.cursor < ed.length {
			for k := ed.cursor; k < ed.length - 1; k += 1 {
				ed.buf[k] = ed.buf[k + 1]
			}
			ed.length -= 1
			return .Redraw
		}
		return .Continue
	}
	if b == 0x7F || b == 0x08 { // Backspace
		if ed.cursor > 0 {
			for k := ed.cursor - 1; k < ed.length - 1; k += 1 {
				ed.buf[k] = ed.buf[k + 1]
			}
			ed.length -= 1
			ed.cursor -= 1
			return .Redraw
		}
		return .Continue
	}
	if b == 0x15 { // Ctrl+U -> erase entire line
		ed.length = 0
		ed.cursor = 0
		return .Redraw
	}
	if b == 0x0B { // Ctrl+K -> erase from cursor to end
		ed.length = ed.cursor
		return .Redraw
	}
	if b == 0x01 { // Ctrl+A -> start of line
		ed.cursor = 0
		return .Redraw
	}
	if b == 0x05 { // Ctrl+E -> end of line
		ed.cursor = ed.length
		return .Redraw
	}
	if b == 0x17 { // Ctrl+W -> erase previous word
		orig := ed.cursor
		for ed.cursor > 0 && ed.buf[ed.cursor - 1] == ' ' {
			for k := ed.cursor - 1; k < ed.length - 1; k += 1 do ed.buf[k] = ed.buf[k + 1]
			ed.length -= 1
			ed.cursor -= 1
		}
		for ed.cursor > 0 && ed.buf[ed.cursor - 1] != ' ' {
			for k := ed.cursor - 1; k < ed.length - 1; k += 1 do ed.buf[k] = ed.buf[k + 1]
			ed.length -= 1
			ed.cursor -= 1
		}
		return .Redraw if ed.cursor != orig else .Continue
	}
	if b >= 32 && b < 127 { // Printable ASCII insert at cursor
		if ed.length < CLI_MAX_LINE_LEN {
			for k := ed.length; k > ed.cursor; k -= 1 {
				ed.buf[k] = ed.buf[k - 1]
			}
			ed.buf[ed.cursor] = b
			ed.length += 1
			ed.cursor += 1
			return .Redraw
		}
	}
	return .Continue
}

// Route prompt redraws through the dedicated Double-Buffered Screen Printer Thread
@(private)
line_editor_redraw_unlocked :: proc(ed: ^CLI_Line_Editor) {
	ed.prev_draw = ed.length
	screen_prompt_update(ed)
}

cli_async_output_begin :: proc() {
	// Handled atomically by Double_Buffered_Screen on its dedicated printer thread
}

cli_async_output_end :: proc() {
	// Handled atomically by Double_Buffered_Screen on its dedicated printer thread
}

// Configure terminal via simple 'stty' command (no Linux/POSIX termios APIs!)
@(private)
run_stty_command :: proc(args: []string) -> bool {
	desc := os.Process_Desc{
		command = args,
		stdin   = os.stdin,
	}
	proc_h, err := os.process_start(desc)
	if err != nil do return false
	state, werr := os.process_wait(proc_h)
	return werr == nil && state.exit_code == 0
}

@(private)
save_stty_mode :: proc(save_buf: []u8) -> string {
	desc := os.Process_Desc{
		command = {"stty", "-g"},
		stdin   = os.stdin,
	}
	state, out_b, err_b, err := os.process_exec(desc, context.allocator)
	defer delete(out_b)
	defer delete(err_b)
	if err != nil || state.exit_code != 0 || len(out_b) == 0 {
		return ""
	}
	trimmed := strings.trim_space(string(out_b))
	if len(trimmed) == 0 || len(trimmed) > len(save_buf) {
		return ""
	}
	copy(save_buf[:len(trimmed)], trimmed)
	return string(save_buf[:len(trimmed)])
}

// Read a single line from non-TTY stdin (pipes / scripts) using plain standard I/O
read_stdin_line :: proc(buf: []u8) -> (line: string, ok: bool) {
	pos := 0
	one : [1]u8
	for pos < len(buf) {
		n, err := os.read(os.stdin, one[:])
		if err != nil || n == 0 {
			if pos > 0 {
				return strings.trim_right(string(buf[:pos]), "\r\n"), true
			}
			return "", false
		}
		b := one[0]
		if b == '\n' {
			return strings.trim_right(string(buf[:pos]), "\r\n"), true
		}
		buf[pos] = b
		pos += 1
	}
	return strings.trim_right(string(buf[:pos]), "\r\n"), true
}

// Read an editable command line with Up/Down history, Left/Right, Backspace/Delete & Enter
// All prompt drawing is delegated to the dedicated Double-Buffered Screen Printer Thread!
read_editable_cli_line :: proc(ed: ^CLI_Line_Editor, hist: ^CLI_Command_History, out_buf: []u8) -> (line: string, ok: bool) {
	sync.mutex_lock(&g_cli_prompt_mutex)
	line_editor_reset(ed, hist)
	g_cli_active_editor = ed
	g_cli_editing_now   = true
	screen_prompt_begin(ed)
	sync.mutex_unlock(&g_cli_prompt_mutex)

	defer {
		sync.mutex_lock(&g_cli_prompt_mutex)
		g_cli_editing_now   = false
		g_cli_active_editor = nil
		screen_prompt_end()
		sync.mutex_unlock(&g_cli_prompt_mutex)
	}

	one : [1]u8
	for {
		n, err := os.read(os.stdin, one[:])
		if err != nil || n == 0 {
			sync.mutex_lock(&g_cli_prompt_mutex)
			screen_prompt_cancel(ed, false)
			has_len := ed.length
			n_cp    := min(has_len, len(out_buf))
			if n_cp > 0 do copy(out_buf[:n_cp], ed.buf[:n_cp])
			sync.mutex_unlock(&g_cli_prompt_mutex)
			if n_cp > 0 {
				res := string(out_buf[:n_cp])
				cli_history_push(hist, res)
				return res, true
			}
			return "", false
		}

		sync.mutex_lock(&g_cli_prompt_mutex)
		act := line_editor_feed_byte(ed, hist, one[0])
		switch act {
		case .Continue:
			sync.mutex_unlock(&g_cli_prompt_mutex)

		case .Redraw:
			line_editor_redraw_unlocked(ed)
			sync.mutex_unlock(&g_cli_prompt_mutex)

		case .Submit:
			screen_prompt_submit(ed)
			n_cp := min(ed.length, len(out_buf))
			if n_cp > 0 do copy(out_buf[:n_cp], ed.buf[:n_cp])
			res := string(out_buf[:n_cp])
			cli_history_push(hist, res)
			sync.mutex_unlock(&g_cli_prompt_mutex)
			return res, true

		case .Abort_EOF:
			screen_prompt_cancel(ed, true)
			sync.mutex_unlock(&g_cli_prompt_mutex)
			return "", false
		}
	}
}

// Print the complete 40-channel Portuguese (ANACOM) & European (CEPT) CB frequency table
print_cb_channel_table :: proc() {
	frame : Screen_Frame
	frame_println(&frame, "================================================================================")
	frame_println(&frame, "  PORTUGUESE (ANACOM QNAF) & EUROPEAN (CEPT EN 300 433) 40 CB CHANNELS")
	frame_println(&frame, "================================================================================")
	for i in 0 ..< 20 {
		c1 := CB_CHANNELS[i]
		c2 := CB_CHANNELS[i + 20]
		frame_printfln(
			&frame,
			"  Ch %02d: %.3f MHz  %-22s |  Ch %02d: %.3f MHz  %s",
			c1.channel, c1.freq_mhz, truncate_note(c1.note_pt, 22),
			c2.channel, c2.freq_mhz, truncate_note(c2.note_pt, 24),
		)
	}
	frame_println(&frame, "================================================================================")
	screen_submit_frame(&frame)
}

@(private)
truncate_note :: proc(s: string, max_len: int) -> string {
	if len(s) <= max_len do return s
	return s[:max_len]
}

// Print plain-text help reference
print_cli_help :: proc() {
	frame : Screen_Frame
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_println(&frame, "  PLAIN TEXT COMMANDS (type command and press Enter):")
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_println(&frame, "  <1..40> or ch <1..40>  Select CB channel 1 to 40 (e.g. '19' or 'ch 38')")
	frame_println(&frame, "  n / p                  Next channel (+1) / Previous channel (-1)")
	frame_println(&frame, "  am | fm | lsb | usb    Change modulation mode (or 'mode usb')")
	frame_println(&frame, "  scan [all|am|fm|ssb]   Start 40-ch Auto-Scanner (all 3 bands or am/fm/ssb/lsb/usb)")
	frame_println(&frame, "  scan once              Sweep all 40 ch once, print report & tune strongest voice")
	frame_println(&frame, "  scan next / scan stop  Skip current locked channel / Stop scanner & hold channel")
	frame_println(&frame, "  scan thresh <dB>       Set scanner SNR threshold in dB (default: 4.5)")
	frame_println(&frame, "  sp [on|off]            Show 40-ch Braille Spectrum & 60s Waterfall (also 'spetrum on|off')")
	frame_println(&frame, "  save / save stop       Start / End saving raw 6.0 MSPS spectrum -> IQ_file_xxxx.samples")
	frame_println(&frame, "  files                  List saved IQ_file_xxxx.samples with Date/Time, Duration & Size")
	frame_println(&frame, "  read [id|file]         Read/load IQ_file_xxxx.samples paused at 00:00 (e.g. 'read 0000')")
	frame_println(&frame, "  play [id|file] | stop  Play/resume IQ file (e.g. 'play 0000' or 'play') / Pause playback")
	frame_println(&frame, "  ff [sec] | rew [sec]   Go forward / backward in IQ file (also 'forward 10', 'backward 5')")
	frame_println(&frame, "  goto <timestamp>       Go to timestamp ('12.5', '01:30', 'HH:MM:SS', 'YYYY-MM-DD HH:MM:SS')")
	frame_println(&frame, "  live                   Close IQ file playback and return to LIVE SDR stream")
	frame_println(&frame, "  freqcheck              Sweep 40 ch & verify exact measured RF frequencies (Hz)")
	frame_println(&frame, "  diag [1..40]           Deep RF spectrum & 4-filter live benchmark on channel")
	frame_println(&frame, "  ant <a|b|c|auto>       Select antenna A/B/C or auto-detect connected port ('ant auto')")
	frame_println(&frame, "  gain <auto | 20..59>   Enable Auto RF Gain ('gain auto') or manual IF gRdB (20..59)")
	frame_println(&frame, "  lna <auto | 0..19>     Enable Auto LNA ('lna auto') or manual LNA state (0=max sens)")
	frame_println(&frame, "  filter <0..4>          Select audio noise filter complexity (or 'f 4'):")
	frame_println(&frame, "                           0 = OFF (raw demodulator)")
	frame_println(&frame, "                           1 = Simple (f64 Biquad Voice BP + 50Hz House Hum Comb)")
	frame_println(&frame, "                           2 = Moderate (De-Clicker + Dual 64-tap NLMS + Expander)")
	frame_println(&frame, "                           3 = Advanced (-15 dB OM-LSA + Pitch Comb + Zero-Silence Gate)")
	frame_println(&frame, "                           4 = Ultra Deep Voice (-20 to -30 dB MMSE-LSA + 3-Scale SHS Comb")
	frame_println(&frame, "                               + Cepstrum + Zero-Silence Gate)")
	frame_println(&frame, "  vol <0..200>           Set volume % (e.g. 'vol 90', 'vol +10', 'vol -10')")
	frame_println(&frame, "  sq <auto|-135..0|off>  Enable Auto-Squelch ('sq auto' / 'autosq'), manual dBFS, or 'off'")
	frame_println(&frame, "  clar <auto|off|Hz>     Enable Auto-Clarifier ('clar auto'), 'clar off', or manual Hz")
	frame_println(&frame, "  clar step <Hz> | reset Nudge manual clarifier (e.g. 'clar step +10') or reset to 0 Hz")
	frame_println(&frame, "  notch <on|off>         Toggle RF/DAB hardware + DSP narrowband heterodyne auto-notch")
	frame_println(&frame, "  ppm <value>            Set crystal frequency correction in PPM")
	frame_println(&frame, "  mon <0..60>            Periodic plain-text signal line every N sec (0 = off)")
	frame_println(&frame, "  channels               Display all 40 Portuguese/European CB channels")
	frame_println(&frame, "  history / hist         Show recent command history (use Up/Down arrows to recall & edit)")
	frame_println(&frame, "  Up/Down/Left/Right     Navigate command history, move cursor, Backspace/Delete, Enter")
	frame_println(&frame, "  s  (or empty Enter)    Refresh and display signal dB & full receiver statistics")
	frame_println(&frame, "  help / ?               Show this command list")
	frame_println(&frame, "  q / quit               Stop receiver and exit")
	frame_println(&frame, "--------------------------------------------------------------------------------")
	screen_submit_frame(&frame)
}

// Print plain-text status block with Channel, Frequency, Signal dB, S-meter, Scanner & Gain statistics
print_receiver_status :: proc(rx: ^Receiver_Context) {
	t     := receiver_get_telemetry(rx)
	info  := get_channel_info(t.channel)
	sp_st := screen_get_stats()

	s_buf : [64]u8
	gr_eff := f64(t.rf_gr_db) + f64(t.lna_state) * 3.0
	s_label, bar_fill := format_s_meter(t.sig_dbfs, gr_eff, s_buf[:])

	bar : [16]u8
	for i in 0 ..< 16 {
		bar[i] = '#' if i < bar_fill else '.'
	}

	ant_char := "A (SMA)"
	if t.antenna_port == 1 do ant_char = "B (SMA)"
	if t.antenna_port == 2 do ant_char = "C (BNC)"

	sq_state := "OPEN" if t.squelch_open else "MUTED"
	sq_cfg   := "OFF"
	sq_cfg_buf : [56]u8
	if t.squelch_enabled {
		if t.auto_squelch {
			sq_cfg = fmt.bprintf(sq_cfg_buf[:], "AUTO %.1f dBFS (%s)", t.squelch_dbfs, sq_state)
		} else {
			sq_cfg = fmt.bprintf(sq_cfg_buf[:], "%.1f dBFS (%s)", t.squelch_dbfs, sq_state)
		}
	} else {
		sq_cfg = fmt.bprintf(sq_cfg_buf[:], "OFF (Audio %s)", sq_state)
	}

	gain_mode_str := "AUTO" if t.auto_rf_gain else ("HW-AGC" if t.hw_agc_enabled else "MANUAL")
	lna_mode_str  := "AUTO" if t.auto_lna else "MANUAL"
	clar_mode_str := "MANUAL"
	if t.auto_clarifier {
		clar_mode_str = "AUTO-LOCKED" if t.clar_locked else "AUTO-TRACK"
	}

	dt_buf : [48]u8
	dt_str := format_datetime_utc(t.stream_timestamp_ns, dt_buf[:], true)
	dt_lbl := "Recorded Time" if t.iq_file_mode else "Date & Time  "
	dt_tag := "IQ File Recorded Timestamp" if t.iq_file_mode else "Live UTC Clock"

	iq_line_buf : [160]u8
	iq_line_str := ""
	if t.iq_file_mode {
		b_p, b_t : [32]u8
		fn    := string(t.iq_play_name_buf[:t.iq_play_name_len])
		p_str := format_duration_mmss(t.iq_play_pos_sec, b_p[:])
		t_str := format_duration_mmss(t.iq_play_tot_sec, b_t[:])
		pct   := (t.iq_play_pos_sec / max(t.iq_play_tot_sec, 0.001)) * 100.0
		iq_line_str = fmt.bprintf(
			iq_line_buf[:],
			"IQ FILE [%s] %s @ %s / %s (%.1f%%)",
			"PLAYING" if t.iq_playing else "PAUSED", fn, p_str, t_str, pct,
		)
	} else if t.iq_saving {
		b_r : [32]u8
		rn    := string(t.iq_save_name_buf[:t.iq_save_name_len])
		r_str := format_duration_mmss(t.iq_save_sec, b_r[:])
		iq_line_str = fmt.bprintf(
			iq_line_buf[:],
			"LIVE + SAVING -> %s (Elapsed %s | %.2f MiB | Drops:%d)",
			rn, r_str, t.iq_save_mb, t.iq_save_drops,
		)
	} else {
		iq_line_str = "LIVE 6.000 MSPS STREAM  ('save' to record IQ_file_xxxx.samples, 'files'/'play' to replay)"
	}

	frame : Screen_Frame
	frame_println(&frame, "================================================================================")
	frame_printfln(&frame, "  CB PORTUGAL / CEPT RECEIVER  |  Device: %s (%s)", t.device_name, t.device_serial)
	frame_printfln(&frame, "  %s: %s  (%s)", dt_lbl, dt_str, dt_tag)
	frame_printfln(&frame, "  IQ Transport : %s", iq_line_str)
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_printfln(&frame, "  Channel      : Ch %02d  (%s)  |  Scanner: %s", t.channel, info.note_pt, t.scan_band_label)
	frame_printfln(&frame, "  Channel Freq : %.6f MHz  (%.0f Hz)  |  HW LO: %.6f MHz",
		t.nominal_freq_hz / 1e6, t.nominal_freq_hz, t.lo_freq_hz / 1e6)
	frame_printfln(&frame, "  Clarifier    : %-11s (%+.1f Hz) -> Effective RX: %.6f MHz (%.1f Hz)",
		clar_mode_str, t.clarifier_hz, t.effective_hz / 1e6, t.effective_hz)
	frame_printfln(&frame, "  Modulation   : %s", mode_norm_desc(t.mode))
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_printfln(&frame, "  Signal Power : %+.1f dBFS    |  S-Meter : [%s] %s", t.sig_dbfs, string(bar[:]), s_label)
	frame_printfln(&frame, "  Noise Floor  : %+.1f dBFS    |  Ch SNR  : %+.1f dB    |  Carrier Off: %+.1f Hz",
		t.noise_dbfs, t.snr_db, t.carrier_off_hz)
	frame_printfln(&frame, "  ADC Wideband : %+.1f dBFS (Peak %.1f%%) @ 6.000 MSps 14-Bit -> 48.0 kHz f64",
		t.adc_dbfs, t.adc_peak_pct)
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_printfln(&frame, "  Audio Filter : %s", filter_level_name(t.filter_level, t.mode))
	frame_printfln(&frame, "  Filter Stats : Noise Reduc: %.1f dB | Voice Conf: %.1f%% | Pitch F0: %.1f Hz | Blanked: %d",
		t.filter_nr_db, t.voice_conf_pct, t.voice_pitch_hz, t.blanked_impulses)
	frame_printfln(&frame, "  Volume & Sq  : Volume: %.0f%%  |  Squelch: %s  |  Speech AGC: %+.1f dB",
		t.volume_pct, sq_cfg, t.agc_gain_db)
	frame_printfln(&frame, "  Audio Out    : RMS: %+.1f dBFS (Peak %.1f%%)  |  SPSC Ring: %.1f%% (Und:%d Ovf:%d) | ScreenSwaps:%d",
		t.audio_rms_dbfs, t.audio_peak_pct, t.ring_fill_pct, t.audio_underruns, t.audio_overflows, sp_st.swaps_count)
	frame_printfln(&frame, "  RF Gain/LNA  : RFGain=%s (gRdB=%d dB) | LNA=%s (State=%d/%d) | Ant=%s | Notch=%s | Ovl=%d",
		gain_mode_str, t.rf_gr_db, lna_mode_str, t.lna_state, t.max_lna_state,
		ant_char, "ON" if t.rf_notch else "OFF", t.overload_events)
	frame_println(&frame, "================================================================================")
	screen_submit_frame(&frame)
}

// Print compact single-line plain-text telemetry (used by 'mon <sec>')
print_compact_telemetry_line :: proc(rx: ^Receiver_Context) {
	t := receiver_get_telemetry(rx)
	s_buf   : [48]u8
	hms_buf : [16]u8
	hms_str := format_hms_utc(t.stream_timestamp_ns, hms_buf[:])
	src_tag := " PLAY" if t.iq_file_mode else (" REC" if t.iq_saving else "")
	gr_eff  := f64(t.rf_gr_db) + f64(t.lna_state) * 3.0
	s_label, _ := format_s_meter(t.sig_dbfs, gr_eff, s_buf[:])
	sq_tag := "OPEN" if t.squelch_open else "MUTE"
	screen_printfln(
		"  [MON %s%s] Ch %02d (%.4f MHz %+.0fHz) %-3s | Sig %+.1f dBFS | Flr %+.1f | SNR %+.1f dB | %-6s | gR=%d LNA=%d | F%d NR %.1fdB | %s",
		hms_str, src_tag, t.channel, t.nominal_freq_hz / 1e6, t.clarifier_hz, mode_name(t.mode),
		t.sig_dbfs, t.noise_dbfs, t.snr_db, s_label[:min(len(s_label), 6)],
		t.rf_gr_db, t.lna_state, int(t.filter_level), t.filter_nr_db, sq_tag,
	)
}

@(private)
Monitor_Worker_Ctx :: struct {
	rx  : ^Receiver_Context,
	eng : ^SDR_Engine,
}

// Background 100 ms coordinator for Auto-Scanner, Auto-LNA + Auto-RF-Gain, and periodic plain-text telemetry
@(private)
monitor_worker_proc :: proc(thr: ^thread.Thread) {
	mctx := (^Monitor_Worker_Ctx)(thr.data)
	rx   := mctx.rx
	eng  := mctx.eng

	mon_elapsed_ms  := 0
	wf_elapsed_ms   := 0
	gain_elapsed_ms := 0

	for rx.running {
		time.sleep(100 * time.Millisecond)
		if !rx.running do break

		// 1. Drive 40-channel 3-band Auto-Scanner state machine
		scanner_tick_state_machine(rx, eng, 100)

		// 2. Drive closed-loop Auto-LNA & Auto-RF-Gain every 300 ms
		gain_elapsed_ms += 100
		if gain_elapsed_ms >= 300 {
			gain_elapsed_ms = 0
			sdr_engine_auto_gain_tick(eng)
		}

		// 3. Periodic plain-text telemetry ('mon') & live Braille 60s Waterfall ('sp on' / 'spetrum on')
		sync.mutex_lock(&rx.cfg_mutex)
		mon_s       := rx.cfg.monitor_sec
		wf_on       := rx.cfg.waterfall_enabled
		wf_s        := max(rx.cfg.waterfall_sec, 1)
		scan_muting := rx.cfg.mute_for_scan
		sync.mutex_unlock(&rx.cfg_mutex)

		if mon_s > 0 && !scan_muting {
			mon_elapsed_ms += 100
			if mon_elapsed_ms >= mon_s * 1000 {
				mon_elapsed_ms = 0
				print_compact_telemetry_line(rx)
			}
		} else if mon_s <= 0 {
			mon_elapsed_ms = 0
		}

		if wf_on {
			wf_elapsed_ms += 100
			if wf_elapsed_ms >= wf_s * 1000 {
				wf_elapsed_ms = 0
				print_braille_waterfall(rx)
			}
		} else {
			wf_elapsed_ms = 0
		}
	}
}

parse_scan_band_arg :: proc(arg: string) -> (Scan_Band_Filter, bool) {
	if strings.equal_fold(arg, "all") || strings.equal_fold(arg, "3bands") do return .All_Bands, true
	if strings.equal_fold(arg, "am")  do return .AM_Only, true
	if strings.equal_fold(arg, "fm")  do return .FM_Only, true
	if strings.equal_fold(arg, "ssb") do return .SSB_Only, true
	if strings.equal_fold(arg, "lsb") do return .LSB_Only, true
	if strings.equal_fold(arg, "usb") do return .USB_Only, true
	return .All_Bands, false
}

@(private)
parse_relative_step_seconds :: proc(spec_raw: string, default_sec: f64) -> (step_sec: f64, ok: bool) {
	s := strings.trim_space(spec_raw)
	if len(s) == 0 do return default_sec, true
	if strings.index_byte(s, ':') >= 0 {
		if ts_s, tok := parse_seek_timestamp(s, 0, 86_400.0); tok && ts_s > 0.0 {
			return ts_s, true
		}
		return 0.0, false
	}
	if len(s) > 1 && (s[len(s) - 1] == 's' || s[len(s) - 1] == 'S') {
		s = s[:len(s) - 1]
	}
	if v, vok := strconv.parse_f64(s); vok {
		av := abs(v)
		if av > 0.0 do return av, true
	}
	return 0.0, false
}

// Execute a single plain-text user command; returns false when user requests quit
execute_cli_command :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, raw_line: string) -> bool {
	line := strings.trim_space(raw_line)

	// Empty Enter refreshes statistics immediately
	if len(line) == 0 {
		print_receiver_status(rx)
		return true
	}

	// Tokenize on whitespace
	parts_buf : [8]string
	n_parts   := 0
	rem       := line
	for len(rem) > 0 && n_parts < len(parts_buf) {
		rem = strings.trim_left_space(rem)
		if len(rem) == 0 do break
		sp := strings.index_any(rem, " \t")
		if sp < 0 {
			parts_buf[n_parts] = rem
			n_parts += 1
			break
		}
		parts_buf[n_parts] = rem[:sp]
		n_parts += 1
		rem = rem[sp + 1:]
	}
	if n_parts == 0 {
		print_receiver_status(rx)
		return true
	}

	cmd := parts_buf[0]

	// Direct bare number 1..40 switches channel immediately (and stops continuous scan)
	if ch_direct, ok := strconv.parse_int(cmd); ok && n_parts == 1 {
		if is_valid_channel(ch_direct) {
			scanner_stop_continuous(rx, eng, false)
			set_channel_cmd(rx, eng, ch_direct)
		} else {
			screen_printfln("  [!] Invalid CB channel %d (valid range: 1 to 40).", ch_direct)
		}
		return true
	}

	// Quit
	if strings.equal_fold(cmd, "q") || strings.equal_fold(cmd, "quit") || strings.equal_fold(cmd, "exit") {
		scanner_stop_continuous(rx, eng, false)
		rx.running = false
		return false
	}

	// Help
	if strings.equal_fold(cmd, "help") || strings.equal_fold(cmd, "h") || cmd == "?" {
		print_cli_help()
		return true
	}

	// Status
	if strings.equal_fold(cmd, "s") || strings.equal_fold(cmd, "stat") || strings.equal_fold(cmd, "status") {
		print_receiver_status(rx)
		return true
	}

	// Channels list
	if strings.equal_fold(cmd, "channels") || strings.equal_fold(cmd, "list") || strings.equal_fold(cmd, "chs") {
		print_cb_channel_table()
		return true
	}

	// Command history list
	if strings.equal_fold(cmd, "history") || strings.equal_fold(cmd, "hist") {
		h_frame : Screen_Frame
		sync.mutex_lock(&g_cli_prompt_mutex)
		cnt := g_cli_history.count
		frame_println(&h_frame, "--------------------------------------------------------------------------------")
		frame_printfln(&h_frame, "  COMMAND HISTORY (%d saved — use Up/Down arrows to recall, edit & press Enter):", cnt)
		if cnt == 0 {
			frame_println(&h_frame, "  (empty)")
		} else {
			for i in 0 ..< cnt {
				hlen := g_cli_history.lengths[i]
				frame_printfln(&h_frame, "  % 3d: %s", i + 1, string(g_cli_history.entries[i][:hlen]))
			}
		}
		frame_println(&h_frame, "--------------------------------------------------------------------------------")
		sync.mutex_unlock(&g_cli_prompt_mutex)
		screen_submit_frame(&h_frame)
		return true
	}

	// 40-channel Braille Spectrum & 60-second Waterfall ('sp [on|off]', 'spetrum [on|off]', 'spectrum [on|off]')
	if strings.equal_fold(cmd, "sp") || strings.equal_fold(cmd, "spetrum") || strings.equal_fold(cmd, "spectrum") {
		if n_parts == 1 {
			print_braille_waterfall(rx)
			return true
		}
		sub_sp := parts_buf[1]
		if strings.equal_fold(sub_sp, "on") || sub_sp == "1" {
			sec := 2
			if n_parts >= 3 {
				if parsed_s, ok := strconv.parse_int(parts_buf[2]); ok && parsed_s >= 1 && parsed_s <= 60 {
					sec = parsed_s
				}
			}
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.waterfall_enabled = true
			rx.cfg.waterfall_sec     = sec
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Live 40-Ch Braille Spectrum & 60s Waterfall ENABLED (refresh every %d s; type 'sp off' to stop)", sec)
			print_braille_waterfall(rx)
			return true
		}
		if strings.equal_fold(sub_sp, "off") || sub_sp == "0" {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.waterfall_enabled = false
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_println("  -> Live 40-Ch Braille Spectrum & 60s Waterfall DISABLED")
			return true
		}
		if strings.equal_fold(sub_sp, "table") || strings.equal_fold(sub_sp, "check") {
			scanner_stop_continuous(rx, eng, false)
			screen_println("  [SPECTRUM] Sweeping all 40 CB channels (26.965 - 27.405 MHz @ 6.0 MSPS 14-Bit f64)...")
			rep := scanner_run_oneshot_sweep(rx, eng, .All_Bands, 110)
			print_spectrum_scan_report(rx, &rep, .All_Bands)
			return true
		}
		if iv, ok := strconv.parse_int(sub_sp); ok && iv >= 0 && iv <= 60 {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.waterfall_enabled = (iv > 0)
			if iv > 0 do rx.cfg.waterfall_sec = iv
			sync.mutex_unlock(&rx.cfg_mutex)
			if iv > 0 {
				screen_printfln("  -> Live 40-Ch Braille Spectrum & 60s Waterfall ENABLED (refresh every %d s)", iv)
				print_braille_waterfall(rx)
			} else {
				screen_println("  -> Live 40-Ch Braille Spectrum & 60s Waterfall DISABLED")
			}
			return true
		}
		screen_println("  Usage: sp [on [sec] | off | table]  (or 'spetrum on' / 'spetrum off')")
		return true
	}

	// Exact 40-channel RF frequency calibration sweep
	if strings.equal_fold(cmd, "freqcheck") || strings.equal_fold(cmd, "freqtest") {
		scanner_stop_continuous(rx, eng, false)
		bf := Scan_Band_Filter.All_Bands
		if n_parts >= 2 {
			if parsed_bf, ok := parse_scan_band_arg(parts_buf[1]); ok do bf = parsed_bf
		}
		screen_println("  [FREQCHECK] Sweeping all 40 CB channels (26.965 - 27.405 MHz @ 6.0 MSPS 14-Bit f64)...")
		rep := scanner_run_oneshot_sweep(rx, eng, bf, 110)
		print_spectrum_scan_report(rx, &rep, bf)
		return true
	}

	// Deep live channel & 4-filter benchmark
	if strings.equal_fold(cmd, "diag") || strings.equal_fold(cmd, "testch") {
		dch := rx.cfg.channel
		if n_parts >= 2 {
			if parsed_ch, ok := strconv.parse_int(parts_buf[1]); ok && is_valid_channel(parsed_ch) {
				dch = parsed_ch
			}
		}
		run_channel_deep_diagnostic(rx, eng, dch)
		return true
	}

	// Antenna port auto-probe shortcut
	if strings.equal_fold(cmd, "antscan") {
		scanner_stop_continuous(rx, eng, false)
		scanner_probe_antennas(rx, eng, true)
		return true
	}

	// Stop scanner / Pause IQ playback / Stop IQ recording shortcut
	if strings.equal_fold(cmd, "stop") {
		sync.mutex_lock(&rx.scanner.mutex)
		scan_on := rx.scanner.enabled
		sync.mutex_unlock(&rx.scanner.mutex)
		if scan_on {
			scanner_stop_continuous(rx, eng, true)
			return true
		}
		if iq_is_file_mode(&rx.iq_eng) {
			iq_stop_pause_cmd(&rx.iq_eng, true)
			return true
		}
		if iq_is_saving(&rx.iq_eng) {
			iq_save_stop(&rx.iq_eng, true)
			return true
		}
		screen_println("  [STOP] Nothing active to stop (not scanning, playing IQ file, or recording).")
		return true
	}

	// Raw 6.0 MSPS 14-Bit IQ Disk Save ('save', 'save start', 'save stop', 'save end', 'endsave', 'rec')
	if strings.equal_fold(cmd, "endsave") || strings.equal_fold(cmd, "stopsave") {
		iq_save_stop(&rx.iq_eng, true)
		return true
	}
	if strings.equal_fold(cmd, "startsave") {
		rest_arg := strings.trim_space(line[len(cmd):])
		iq_save_start(&rx.iq_eng, rest_arg, true)
		return true
	}
	if strings.equal_fold(cmd, "save") || strings.equal_fold(cmd, "rec") || strings.equal_fold(cmd, "record") {
		if n_parts == 1 {
			iq_save_start(&rx.iq_eng, "", true)
			return true
		}
		sub_sv := parts_buf[1]
		if strings.equal_fold(sub_sv, "stop") || strings.equal_fold(sub_sv, "end") || strings.equal_fold(sub_sv, "off") || sub_sv == "0" {
			iq_save_stop(&rx.iq_eng, true)
			return true
		}
		if strings.equal_fold(sub_sv, "start") || strings.equal_fold(sub_sv, "on") || sub_sv == "1" {
			custom_p := ""
			if n_parts >= 3 {
				after_cmd := strings.trim_space(line[len(cmd):])
				custom_p  = strings.trim_space(after_cmd[len(sub_sv):])
			}
			iq_save_start(&rx.iq_eng, custom_p, true)
			return true
		}
		rest_arg := strings.trim_space(line[len(cmd):])
		iq_save_start(&rx.iq_eng, rest_arg, true)
		return true
	}

	// List saved IQ_file_xxxx.samples ('files', 'iqfiles', 'lsiq')
	if strings.equal_fold(cmd, "files") || strings.equal_fold(cmd, "iqfiles") || strings.equal_fold(cmd, "lsiq") {
		dir_arg := "."
		if n_parts >= 2 {
			dir_arg = strings.trim_space(line[len(cmd):])
		}
		iq_print_saved_files_list(dir_arg)
		return true
	}

	// Read / Open IQ_file_xxxx.samples from disk paused at 00:00 ('read [0000|file]', 'load', 'open')
	if strings.equal_fold(cmd, "read") || strings.equal_fold(cmd, "load") || strings.equal_fold(cmd, "open") {
		scanner_stop_continuous(rx, eng, false)
		rest_arg  := strings.trim_space(line[len(cmd):])
		auto_play := false
		if n_parts >= 3 && strings.equal_fold(parts_buf[n_parts - 1], "play") {
			auto_play = true
			rest_arg  = strings.trim_space(rest_arg[:len(rest_arg) - 4])
		}
		iq_file_open(&rx.iq_eng, rest_arg, auto_play, true)
		return true
	}

	// Play / Resume IQ_file_xxxx.samples ('play [0000|file]', 'resume')
	if strings.equal_fold(cmd, "play") || strings.equal_fold(cmd, "resume") || strings.equal_fold(cmd, "startplay") {
		scanner_stop_continuous(rx, eng, false)
		rest_arg := ""
		if n_parts >= 2 && !strings.equal_fold(cmd, "resume") {
			rest_arg = strings.trim_space(line[len(cmd):])
		}
		iq_play_cmd(&rx.iq_eng, rest_arg, true)
		return true
	}

	// Pause IQ playback ('pause')
	if strings.equal_fold(cmd, "pause") {
		if !iq_stop_pause_cmd(&rx.iq_eng, true) {
			screen_println("  [!] No IQ file is currently loaded (use 'play' or 'read' to open an IQ_file_xxxx.samples file).")
		}
		return true
	}

	// Return from IQ file playback to LIVE SDR stream ('live', 'close', 'closeiq')
	if strings.equal_fold(cmd, "live") || strings.equal_fold(cmd, "close") || strings.equal_fold(cmd, "closeiq") {
		scanner_stop_continuous(rx, eng, false)
		iq_close_return_to_live(&rx.iq_eng, eng, true)
		return true
	}

	// Go forward in IQ file ('forward [s]', 'forword [s]', 'fwd [s]', 'ff [s]')
	if strings.equal_fold(cmd, "forward") || strings.equal_fold(cmd, "forword") || strings.equal_fold(cmd, "forw") || strings.equal_fold(cmd, "fwd") || strings.equal_fold(cmd, "ff") {
		rest_arg := strings.trim_space(line[len(cmd):])
		if step_s, sok := parse_relative_step_seconds(rest_arg, 5.0); sok {
			iq_seek_relative(&rx.iq_eng, +step_s, true)
		} else {
			screen_println("  Usage: forward [seconds | MM:SS]  (e.g. 'ff', 'ff 10', 'forward 30')")
		}
		return true
	}

	// Go backward in IQ file ('backward [s]', 'backword [s]', 'back [s]', 'bwd [s]', 'rew [s]', 'rw [s]')
	if strings.equal_fold(cmd, "backward") || strings.equal_fold(cmd, "backword") || strings.equal_fold(cmd, "back") || strings.equal_fold(cmd, "bwd") || strings.equal_fold(cmd, "rew") || strings.equal_fold(cmd, "rw") {
		rest_arg := strings.trim_space(line[len(cmd):])
		if step_s, sok := parse_relative_step_seconds(rest_arg, 5.0); sok {
			iq_seek_relative(&rx.iq_eng, -step_s, true)
		} else {
			screen_println("  Usage: backward [seconds | MM:SS]  (e.g. 'rew', 'back 10', 'backward 30')")
		}
		return true
	}

	// Natural two-word 'go to <ts>' / 'go forward [s]' / 'go backward [s]' + 'goto <ts>' / 'seek <ts>' / 'ts <ts>'
	if strings.equal_fold(cmd, "go") && n_parts >= 2 {
		sub_go    := parts_buf[1]
		after_cmd := strings.trim_space(line[len(cmd):])
		after_sub := strings.trim_space(after_cmd[len(sub_go):])
		if strings.equal_fold(sub_go, "forward") || strings.equal_fold(sub_go, "forword") || strings.equal_fold(sub_go, "fwd") {
			if step_s, sok := parse_relative_step_seconds(after_sub, 5.0); sok {
				iq_seek_relative(&rx.iq_eng, +step_s, true)
			} else {
				screen_println("  Usage: go forward [seconds]")
			}
			return true
		}
		if strings.equal_fold(sub_go, "backward") || strings.equal_fold(sub_go, "backword") || strings.equal_fold(sub_go, "back") {
			if step_s, sok := parse_relative_step_seconds(after_sub, 5.0); sok {
				iq_seek_relative(&rx.iq_eng, -step_s, true)
			} else {
				screen_println("  Usage: go backward [seconds]")
			}
			return true
		}
		ts_arg := after_sub if strings.equal_fold(sub_go, "to") else after_cmd
		if !iq_is_file_mode(&rx.iq_eng) {
			screen_println("  [!] No IQ file is currently loaded. Use 'read' or 'play' first.")
			return true
		}
		if target_s, tok := parse_seek_timestamp(ts_arg, rx.iq_eng.play_info.start_ns, rx.iq_eng.play_info.duration_sec); tok {
			iq_seek_to_seconds(&rx.iq_eng, target_s, true)
		} else {
			screen_println("  Usage: go to <sec | MM:SS | HH:MM:SS | YYYY-MM-DD HH:MM:SS | start | end>")
		}
		return true
	}

	if strings.equal_fold(cmd, "goto") || strings.equal_fold(cmd, "seek") || strings.equal_fold(cmd, "ts") || strings.equal_fold(cmd, "jump") {
		if n_parts < 2 {
			screen_println("  Usage: goto <sec | MM:SS | HH:MM:SS | YYYY-MM-DD HH:MM:SS | start | end>")
			return true
		}
		if !iq_is_file_mode(&rx.iq_eng) {
			screen_println("  [!] No IQ file is currently loaded. Use 'read' or 'play' first.")
			return true
		}
		ts_arg := strings.trim_space(line[len(cmd):])
		if strings.has_prefix(ts_arg, "+") || (strings.has_prefix(ts_arg, "-") && strings.index_byte(ts_arg[1:], '-') < 0) {
			sign := -1.0 if strings.has_prefix(ts_arg, "-") else +1.0
			if step_s, sok := parse_relative_step_seconds(ts_arg[1:], 0.0); sok && step_s > 0.0 {
				iq_seek_relative(&rx.iq_eng, sign * step_s, true)
				return true
			}
		}
		if target_s, tok := parse_seek_timestamp(ts_arg, rx.iq_eng.play_info.start_ns, rx.iq_eng.play_info.duration_sec); tok {
			iq_seek_to_seconds(&rx.iq_eng, target_s, true)
		} else {
			screen_println("  [!] Invalid timestamp. Examples: 'goto 15.5', 'goto 01:20', 'goto 21:30:05', 'goto 2026-11-14 21:30:05'")
		}
		return true
	}

	// 40-Channel 3-Band Auto-Scanner commands:
	//   scan | scan all | scan am | scan fm | scan ssb | scan lsb | scan usb
	//   scan once | scan next | scan stop | scan thresh <dB>
	if strings.equal_fold(cmd, "scan") || strings.equal_fold(cmd, "autoscan") {
		if n_parts == 1 {
			scanner_start_continuous(rx, eng, .All_Bands)
			return true
		}
		sub := parts_buf[1]
		if strings.equal_fold(sub, "stop") || strings.equal_fold(sub, "off") {
			scanner_stop_continuous(rx, eng, true)
			return true
		}
		if strings.equal_fold(sub, "next") || strings.equal_fold(sub, "skip") {
			scanner_skip_to_next(rx, eng)
			return true
		}
		if strings.equal_fold(sub, "table") {
			scanner_stop_continuous(rx, eng, false)
			rep := scanner_run_oneshot_sweep(rx, eng, .All_Bands, 110)
			print_spectrum_scan_report(rx, &rep, .All_Bands)
			return true
		}
		if strings.equal_fold(sub, "once") {
			scanner_stop_continuous(rx, eng, false)
			bf := Scan_Band_Filter.All_Bands
			if n_parts >= 3 {
				if pbf, ok := parse_scan_band_arg(parts_buf[2]); ok do bf = pbf
			}
			screen_println("  [SCAN ONCE] Sweeping all 40 CB channels across AM, FM, and SSB...")
			rep := scanner_run_oneshot_sweep(rx, eng, bf, 110)
			print_spectrum_scan_report(rx, &rep, bf)

			// Lock onto strongest modulated / active channel automatically
			best_i   := -1
			best_snr := -999.0
			for idx in 0 ..< 40 {
				e := rep.entries[idx]
				if e.active {
					score := e.snr_db + (4.0 if e.is_modulated else 0.0)
					if score > best_snr {
						best_snr = score
						best_i   = idx
					}
				}
			}
			if best_i >= 0 {
				hit := rep.entries[best_i]
				sync.mutex_lock(&rx.cfg_mutex)
				rx.cfg.channel     = hit.channel
				rx.cfg.mode        = hit.best_mode
				rx.cfg.hw_dirty_rf = true
				sync.mutex_unlock(&rx.cfg_mutex)
				sdr_engine_apply_pending_updates(eng)
				time.sleep(90 * time.Millisecond)
				screen_printfln(
					"  -> Auto-Tuned Best Hit: Ch %02d (%.6f MHz, Meas %.6f MHz) Mode=%s | SNR=%+.1f dB",
					hit.channel, hit.nominal_hz / 1e6, hit.measured_peak_hz / 1e6, mode_name(hit.best_mode), hit.snr_db,
				)
			}
			return true
		}
		if strings.equal_fold(sub, "thresh") || strings.equal_fold(sub, "th") {
			if n_parts >= 3 {
				if tv, ok := strconv.parse_f64(parts_buf[2]); ok {
					sync.mutex_lock(&rx.scanner.mutex)
					rx.scanner.threshold_db = clamp(tv, 1.0, 50.0)
					new_th := rx.scanner.threshold_db
					sync.mutex_unlock(&rx.scanner.mutex)
					screen_printfln("  -> Scanner SNR threshold set to %+.1f dB", new_th)
					return true
				}
			}
			screen_println("  Usage: scan thresh <1.0 .. 50.0>  (e.g. 'scan thresh 4.5')")
			return true
		}
		if bf, ok := parse_scan_band_arg(sub); ok {
			scanner_start_continuous(rx, eng, bf)
			return true
		}
		screen_println("  Usage: scan [all | am | fm | ssb | lsb | usb | once | next | stop | thresh <dB>]")
		return true
	}

	// Next / Previous channel
	if strings.equal_fold(cmd, "n") || strings.equal_fold(cmd, "next") || cmd == "+ch" {
		scanner_stop_continuous(rx, eng, false)
		sync.mutex_lock(&rx.cfg_mutex)
		next_ch := rx.cfg.channel + 1
		if next_ch > CB_MAX_CHANNEL do next_ch = CB_MIN_CHANNEL
		sync.mutex_unlock(&rx.cfg_mutex)
		set_channel_cmd(rx, eng, next_ch)
		return true
	}
	if strings.equal_fold(cmd, "p") || strings.equal_fold(cmd, "prev") || cmd == "-ch" {
		scanner_stop_continuous(rx, eng, false)
		sync.mutex_lock(&rx.cfg_mutex)
		prev_ch := rx.cfg.channel - 1
		if prev_ch < CB_MIN_CHANNEL do prev_ch = CB_MAX_CHANNEL
		sync.mutex_unlock(&rx.cfg_mutex)
		set_channel_cmd(rx, eng, prev_ch)
		return true
	}

	// Direct modulation shortcuts: am, fm, lsb, usb
	if m_direct, m_ok := parse_modulation_mode(cmd); m_ok && n_parts == 1 {
		set_mode_cmd(rx, m_direct)
		return true
	}

	// ch <1..40>
	if strings.equal_fold(cmd, "ch") || strings.equal_fold(cmd, "c") || strings.equal_fold(cmd, "chan") {
		if n_parts < 2 {
			screen_println("  Usage: ch <1..40>")
			return true
		}
		if v, ok := strconv.parse_int(parts_buf[1]); ok && is_valid_channel(v) {
			scanner_stop_continuous(rx, eng, false)
			set_channel_cmd(rx, eng, v)
		} else {
			screen_println("  [!] Channel must be an integer between 1 and 40.")
		}
		return true
	}

	// mode <am|fm|lsb|usb>
	if strings.equal_fold(cmd, "mode") || strings.equal_fold(cmd, "m") {
		if n_parts < 2 {
			screen_println("  Usage: mode <am | fm | lsb | usb>")
			return true
		}
		if m, ok := parse_modulation_mode(parts_buf[1]); ok {
			set_mode_cmd(rx, m)
		} else {
			screen_println("  [!] Unknown mode. Choose: am, fm, lsb, usb")
		}
		return true
	}

	// filter <0..4>
	if strings.equal_fold(cmd, "filter") || strings.equal_fold(cmd, "f") || strings.equal_fold(cmd, "flt") {
		if n_parts < 2 {
			screen_println("  Usage: filter <0..4>  (0=Off, 1=Simple, 2=Moderate, 3=Adv -15dB, 4=Ultra -20..-30dB)")
			return true
		}
		if strings.equal_fold(parts_buf[1], "off") {
			set_filter_cmd(rx, .Off)
			return true
		}
		if lvl, ok := strconv.parse_int(parts_buf[1]); ok && lvl >= 0 && lvl <= 4 {
			set_filter_cmd(rx, Filter_Level(lvl))
		} else {
			screen_println("  [!] Filter complexity level must be 0, 1, 2, 3, or 4.")
		}
		return true
	}

	// vol <0..200 | +N | -N>
	if strings.equal_fold(cmd, "vol") || strings.equal_fold(cmd, "v") || strings.equal_fold(cmd, "volume") {
		if n_parts < 2 {
			screen_println("  Usage: vol <0..200>  (or 'vol +10', 'vol -10')")
			return true
		}
		arg := parts_buf[1]
		is_rel := strings.has_prefix(arg, "+") || strings.has_prefix(arg, "-")
		if val, ok := strconv.parse_f64(arg); ok {
			sync.mutex_lock(&rx.cfg_mutex)
			if is_rel {
				rx.cfg.volume_pct = clamp(rx.cfg.volume_pct + val, 0.0, 200.0)
			} else {
				rx.cfg.volume_pct = clamp(val, 0.0, 200.0)
			}
			new_v := rx.cfg.volume_pct
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Volume set to %.0f%%", new_v)
		} else {
			screen_println("  [!] Invalid volume number.")
		}
		return true
	}

	// autosq [on|off] / autosquesh [on|off] / sq <auto | dB | off | on> / squesh <auto | dB | off | on>
	if strings.equal_fold(cmd, "autosq") || strings.equal_fold(cmd, "autosquesh") || strings.equal_fold(cmd, "autosquelch") {
		en := true
		if n_parts >= 2 && (strings.equal_fold(parts_buf[1], "off") || parts_buf[1] == "0") {
			en = false
		}
		sync.mutex_lock(&rx.cfg_mutex)
		if en {
			rx.cfg.squelch_enabled = true
			rx.cfg.auto_squelch    = true
		} else {
			rx.cfg.auto_squelch    = false
			rx.cfg.squelch_enabled = false
		}
		sync.mutex_unlock(&rx.cfg_mutex)
		t_now := receiver_get_telemetry(rx)
		if en {
			screen_printfln("  -> Auto-Squelch ENABLED (Adaptive noise-floor + voice/PLL coherence; live Flr=%+.1f dBFS)", t_now.noise_dbfs)
		} else {
			screen_println("  -> Auto-Squelch DISABLED (audio open)")
		}
		return true
	}

	if strings.equal_fold(cmd, "sq") || strings.equal_fold(cmd, "squelch") || strings.equal_fold(cmd, "squesh") {
		if n_parts < 2 {
			screen_println("  Usage: sq <auto | -135..0 | off | on>  (e.g. 'sq auto', 'sq -95', or 'sq off')")
			return true
		}
		arg := parts_buf[1]
		if strings.equal_fold(arg, "auto") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.squelch_enabled = true
			rx.cfg.auto_squelch    = true
			sync.mutex_unlock(&rx.cfg_mutex)
			t_now := receiver_get_telemetry(rx)
			screen_printfln("  -> Auto-Squelch ENABLED (Adaptive multi-mode f64 tracker; live Flr=%+.1f dBFS)", t_now.noise_dbfs)
			return true
		}
		if strings.equal_fold(arg, "off") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.squelch_enabled = false
			rx.cfg.auto_squelch    = false
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_println("  -> Squelch DISABLED (audio open)")
			return true
		}
		if strings.equal_fold(arg, "on") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.squelch_enabled = true
			is_auto := rx.cfg.auto_squelch
			th      := rx.cfg.squelch_dbfs
			sync.mutex_unlock(&rx.cfg_mutex)
			if is_auto {
				screen_println("  -> Auto-Squelch ENABLED")
			} else {
				screen_printfln("  -> Manual Squelch ENABLED at %.1f dBFS", th)
			}
			return true
		}
		if val, ok := strconv.parse_f64(arg); ok {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.squelch_dbfs    = clamp(val, -135.0, 0.0)
			rx.cfg.squelch_enabled = true
			rx.cfg.auto_squelch    = false
			th := rx.cfg.squelch_dbfs
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Manual Squelch ENABLED at %.1f dBFS", th)
		} else {
			screen_println("  [!] Invalid squelch value. Use 'sq auto', dBFS (e.g. -95), or 'sq off'.")
		}
		return true
	}

	// clar <auto | off | reset | -5000..+5000 | step <Hz>>  or  autoclar [on|off]
	if strings.equal_fold(cmd, "autoclar") {
		en := true
		if n_parts >= 2 && (strings.equal_fold(parts_buf[1], "off") || parts_buf[1] == "0") {
			en = false
		}
		sync.mutex_lock(&rx.cfg_mutex)
		rx.cfg.auto_clarifier = en
		clar := rx.cfg.clarifier_hz
		sync.mutex_unlock(&rx.cfg_mutex)
		if en {
			screen_printfln("  -> Auto-Clarifier ENABLED (AM SAM PLL / FM DC / SSB Harmonic Comb; current %+.1f Hz)", clar)
		} else {
			screen_printfln("  -> Auto-Clarifier DISABLED (holding manual offset %+.1f Hz)", clar)
		}
		return true
	}

	if strings.equal_fold(cmd, "clar") || strings.equal_fold(cmd, "cl") || strings.equal_fold(cmd, "rit") {
		if n_parts < 2 {
			screen_println("  Usage: clar <auto | off | reset | -5000..+5000 | step +-Hz>")
			return true
		}
		sub_c := parts_buf[1]
		if strings.equal_fold(sub_c, "auto") || strings.equal_fold(sub_c, "on") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_clarifier = true
			clar := rx.cfg.clarifier_hz
			ch   := rx.cfg.channel
			sync.mutex_unlock(&rx.cfg_mutex)
			eff := effective_rx_freq_hz(ch, clar)
			screen_printfln("  -> Auto-Clarifier ENABLED (current %+.1f Hz -> Effective: %.6f MHz)", clar, eff / 1e6)
			return true
		}
		if strings.equal_fold(sub_c, "off") || strings.equal_fold(sub_c, "manual") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_clarifier = false
			clar := rx.cfg.clarifier_hz
			ch   := rx.cfg.channel
			sync.mutex_unlock(&rx.cfg_mutex)
			eff := effective_rx_freq_hz(ch, clar)
			screen_printfln("  -> Auto-Clarifier DISABLED (Manual Clarifier holding %+.1f Hz -> %.6f MHz)", clar, eff / 1e6)
			return true
		}
		if strings.equal_fold(sub_c, "reset") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.clarifier_hz = 0.0
			ac_on := rx.cfg.auto_clarifier
			ch    := rx.cfg.channel
			sync.mutex_unlock(&rx.cfg_mutex)
			eff := effective_rx_freq_hz(ch, 0.0)
			screen_printfln("  -> Clarifier reset to +0.0 Hz (%s, Effective: %.6f MHz)",
				"AUTO" if ac_on else "MANUAL", eff / 1e6)
			return true
		}
		if strings.equal_fold(sub_c, "step") && n_parts >= 3 {
			if dv, ok := strconv.parse_f64(parts_buf[2]); ok {
				set_clarifier_cmd(rx, dv, true)
			} else {
				screen_println("  [!] Invalid clarifier step in Hz.")
			}
			return true
		}
		if val, ok := strconv.parse_f64(sub_c); ok {
			set_clarifier_cmd(rx, val, false)
		} else {
			screen_println("  [!] Specify 'auto', 'off', 'reset', 'step <Hz>', or manual offset in Hz.")
		}
		return true
	}

	// gain <auto | 20..59>  or  autogain <on|off>
	if strings.equal_fold(cmd, "gain") || strings.equal_fold(cmd, "gr") || strings.equal_fold(cmd, "autogain") {
		if n_parts < 2 {
			screen_println("  Usage: gain <auto | 20..59>   (or 'autogain on|off')")
			return true
		}
		arg := parts_buf[1]
		if strings.equal_fold(arg, "auto") || strings.equal_fold(arg, "on") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_rf_gain   = true
			rx.cfg.hw_agc_enabled = true
			rx.cfg.hw_dirty_agc   = true
			sync.mutex_unlock(&rx.cfg_mutex)
			sdr_engine_apply_pending_updates(eng)
			screen_println("  -> Auto RF Gain ENABLED (Hardware AGC -30 dBFS + closed-loop ADC supervisor)")
			return true
		}
		if strings.equal_fold(arg, "off") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_rf_gain   = false
			rx.cfg.hw_agc_enabled = false
			rx.cfg.hw_dirty_agc   = true
			rx.cfg.hw_dirty_gain  = true
			gr := rx.cfg.rf_gr_db
			sync.mutex_unlock(&rx.cfg_mutex)
			sdr_engine_apply_pending_updates(eng)
			screen_printfln("  -> Auto RF Gain DISABLED (Manual gRdB = %d dB)", gr)
			return true
		}
		if gr, ok := strconv.parse_int(arg); ok && gr >= 20 && gr <= 59 {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_rf_gain   = false
			rx.cfg.hw_agc_enabled = false
			rx.cfg.rf_gr_db       = i32(gr)
			rx.cfg.hw_dirty_agc   = true
			rx.cfg.hw_dirty_gain  = true
			sync.mutex_unlock(&rx.cfg_mutex)
			sdr_engine_apply_pending_updates(eng)
			screen_printfln("  -> Manual IF Gain Reduction set to %d dB (Auto RF Gain OFF)", gr)
		} else {
			screen_println("  [!] Specify 'auto' or manual IF gain reduction 20..59 dB.")
		}
		return true
	}

	// lna <auto | 0..max_lna>  or  autolna <on|off>
	if strings.equal_fold(cmd, "lna") || strings.equal_fold(cmd, "autolna") {
		max_lna := max_lna_state_for_27mhz(eng.dev.hwVer)
		if n_parts < 2 {
			screen_printfln("  Usage: lna <auto | 0..%d>   (or 'autolna on|off')", max_lna)
			return true
		}
		arg := parts_buf[1]
		if strings.equal_fold(arg, "auto") || strings.equal_fold(arg, "on") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_lna = true
			st := rx.cfg.lna_state
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Auto LNA ENABLED (27 MHz range 0..%d, current state=%d)", max_lna, st)
			return true
		}
		if strings.equal_fold(arg, "off") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_lna = false
			st := rx.cfg.lna_state
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Auto LNA DISABLED (holding manual LNA state=%d)", st)
			return true
		}
		if st, ok := strconv.parse_int(arg); ok && st >= 0 && st <= int(max_lna) {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.auto_lna      = false
			rx.cfg.lna_state     = u8(st)
			rx.cfg.hw_dirty_gain = true
			sync.mutex_unlock(&rx.cfg_mutex)
			sdr_engine_apply_pending_updates(eng)
			screen_printfln("  -> Manual LNA state set to %d (valid 27 MHz range: 0..%d, Auto LNA OFF)", st, max_lna)
		} else {
			screen_printfln("  [!] LNA state for this hardware at 27 MHz must be 'auto' or 0..%d.", max_lna)
		}
		return true
	}

	// ant <a|b|c|auto>
	if strings.equal_fold(cmd, "ant") || strings.equal_fold(cmd, "antenna") {
		if n_parts < 2 {
			screen_println("  Usage: ant <a | b | c | auto>")
			return true
		}
		a := parts_buf[1]
		if strings.equal_fold(a, "auto") || strings.equal_fold(a, "scan") {
			scanner_stop_continuous(rx, eng, false)
			scanner_probe_antennas(rx, eng, true)
			return true
		}
		port := -1
		if strings.equal_fold(a, "a") do port = 0
		if strings.equal_fold(a, "b") do port = 1
		if strings.equal_fold(a, "c") do port = 2
		if port >= 0 {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.antenna_port = port
			rx.cfg.hw_dirty_ant = true
			sync.mutex_unlock(&rx.cfg_mutex)
			sdr_engine_apply_pending_updates(eng)
			screen_printfln("  -> Antenna port set to %s", strings.to_upper(a, context.temp_allocator))
		} else {
			screen_println("  [!] Antenna port must be A, B, C, or 'auto'.")
		}
		return true
	}

	// notch <on|off>
	if strings.equal_fold(cmd, "notch") {
		if n_parts < 2 {
			screen_println("  Usage: notch <on | off>")
			return true
		}
		en := strings.equal_fold(parts_buf[1], "on") || parts_buf[1] == "1"
		sync.mutex_lock(&rx.cfg_mutex)
		rx.cfg.rf_notch       = en
		rx.cfg.hw_dirty_notch = true
		sync.mutex_unlock(&rx.cfg_mutex)
		sdr_engine_apply_pending_updates(eng)
		screen_printfln("  -> Hardware RF/DAB + DSP Heterodyne Auto-Notch %s", "ENABLED" if en else "DISABLED")
		return true
	}

	// ppm <val>
	if strings.equal_fold(cmd, "ppm") {
		if n_parts >= 2 {
			if pv, ok := strconv.parse_f64(parts_buf[1]); ok {
				sync.mutex_lock(&rx.cfg_mutex)
				rx.cfg.ppm_cal      = clamp(pv, -200.0, 200.0)
				rx.cfg.hw_dirty_ppm = true
				sync.mutex_unlock(&rx.cfg_mutex)
				sdr_engine_apply_pending_updates(eng)
				screen_printfln("  -> Crystal PPM calibration set to %+.2f ppm", pv)
				return true
			}
		}
		screen_println("  Usage: ppm <-200..+200>")
		return true
	}

	// mon <0..60 | off>
	if strings.equal_fold(cmd, "mon") || strings.equal_fold(cmd, "monitor") {
		if n_parts < 2 {
			screen_println("  Usage: mon <seconds | off>  (e.g. 'mon 2' or 'mon 0')")
			return true
		}
		if strings.equal_fold(parts_buf[1], "off") {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.monitor_sec = 0
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_println("  -> Periodic telemetry monitor OFF")
			return true
		}
		if sec, ok := strconv.parse_int(parts_buf[1]); ok && sec >= 0 && sec <= 60 {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.cfg.monitor_sec = sec
			sync.mutex_unlock(&rx.cfg_mutex)
			if sec == 0 {
				screen_println("  -> Periodic telemetry monitor OFF")
			} else {
				screen_printfln("  -> Periodic plain-text telemetry every %d s ('mon 0' to stop)", sec)
			}
		} else {
			screen_println("  [!] Specify monitor interval 0..60 seconds.")
		}
		return true
	}

	// simsnr <-35..+40> (adjust simulated voice SNR when in -sim mode)
	if strings.equal_fold(cmd, "simsnr") && n_parts >= 2 {
		if sv, ok := strconv.parse_f64(parts_buf[1]); ok {
			sync.mutex_lock(&rx.cfg_mutex)
			rx.sim_snr_db = clamp(sv, -35.0, +40.0)
			cur := rx.sim_snr_db
			sync.mutex_unlock(&rx.cfg_mutex)
			screen_printfln("  -> Simulator voice SNR set to %+.1f dB relative to noise floor", cur)
			return true
		}
	}

	screen_printfln("  [!] Unrecognized command '%s'. Type 'help' for available commands.", cmd)
	return true
}

@(private)
set_channel_cmd :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, ch: int) {
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.channel     = ch
	if rx.cfg.auto_clarifier do rx.cfg.clarifier_hz = 0.0
	rx.cfg.hw_dirty_rf = true
	clar := rx.cfg.clarifier_hz
	sync.mutex_unlock(&rx.cfg_mutex)

	sdr_engine_apply_pending_updates(eng)
	info := get_channel_info(ch)
	eff  := effective_rx_freq_hz(ch, clar)
	time.sleep(80 * time.Millisecond)
	t := receiver_get_telemetry(rx)
	screen_printfln(
		"  -> Tuned CB Ch %02d: %.6f MHz (Eff: %.6f MHz) | %s | Signal: %+.1f dBFS (SNR %+.1f dB)",
		ch, info.freq_mhz, eff / 1e6, info.note_pt, t.sig_dbfs, t.snr_db,
	)
}

@(private)
set_mode_cmd :: proc(rx: ^Receiver_Context, m: Modulation_Mode) {
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.mode = m
	flt := rx.cfg.filter_level
	sync.mutex_unlock(&rx.cfg_mutex)
	screen_printfln("  -> Modulation set to %s | Active Filter: %s", mode_norm_desc(m), filter_level_name(flt, m))
}

@(private)
set_filter_cmd :: proc(rx: ^Receiver_Context, lvl: Filter_Level) {
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.filter_level = lvl
	m := rx.cfg.mode
	sync.mutex_unlock(&rx.cfg_mutex)
	screen_printfln("  -> Audio Filter set to %s", filter_level_name(lvl, m))
}

@(private)
set_clarifier_cmd :: proc(rx: ^Receiver_Context, val: f64, relative: bool) {
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.auto_clarifier = false
	if relative {
		rx.cfg.clarifier_hz = clamp(rx.cfg.clarifier_hz + val, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
	} else {
		rx.cfg.clarifier_hz = clamp(val, CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)
	}
	ch   := rx.cfg.channel
	clar := rx.cfg.clarifier_hz
	sync.mutex_unlock(&rx.cfg_mutex)
	eff := effective_rx_freq_hz(ch, clar)
	screen_printfln("  -> Manual Clarifier: %+.1f Hz  (Auto-Clarifier OFF | Effective RX: %.6f MHz / %.1f Hz)",
		clar, eff / 1e6, eff)
}

// Run the main plain-text interactive loop
run_plain_cli_loop :: proc(rx: ^Receiver_Context, eng: ^SDR_Engine, start_scan: bool, init_scan_bf: Scan_Band_Filter) {
	mctx := Monitor_Worker_Ctx{rx = rx, eng = eng}
	mon_thread := thread.create(monitor_worker_proc)
	mon_thread.data = rawptr(&mctx)
	thread.start(mon_thread)
	defer {
		rx.running = false
		thread.join(mon_thread)
		thread.destroy(mon_thread)
	}

	// Allow ~450 ms of initial 6 MSPS streaming so initial signal dB, Auto-LNA/Gain & noise floor settle
	time.sleep(450 * time.Millisecond)
	print_receiver_status(rx)
	screen_println("  Commands: 'sp on' (Waterfall), 'save'/'save stop', 'play'/'stop'/'ff'/'rew'/'goto', 'scan', 'help', 'q'.")

	if rx.cfg.waterfall_enabled {
		print_braille_waterfall(rx)
	}
	if start_scan {
		scanner_start_continuous(rx, eng, init_scan_bf)
	}
	screen_flush()

	// Enable character-by-character arrow/backspace line editing via simple 'stty' command when stdin is a terminal
	stty_save_buf  : [256]u8
	saved_stty     := ""
	use_tty_editor := os.is_tty(os.stdin)
	if use_tty_editor {
		saved_stty     = save_stty_mode(stty_save_buf[:])
		use_tty_editor = run_stty_command({"stty", "-icanon", "-echo", "-isig", "min", "1", "time", "0"})
	}
	defer if use_tty_editor {
		screen_flush()
		if len(saved_stty) > 0 {
			if !run_stty_command({"stty", saved_stty}) {
				run_stty_command({"stty", "sane"})
			}
		} else {
			run_stty_command({"stty", "sane"})
		}
	}

	editor   : CLI_Line_Editor
	line_buf : [256]u8
	for rx.running {
		line : string
		ok   : bool
		if use_tty_editor {
			line, ok = read_editable_cli_line(&editor, &g_cli_history, line_buf[:])
		} else {
			screen_print("cb> ")
			screen_flush()
			line, ok = read_stdin_line(line_buf[:])
			if ok {
				sync.mutex_lock(&g_cli_prompt_mutex)
				cli_history_push(&g_cli_history, line)
				sync.mutex_unlock(&g_cli_prompt_mutex)
			}
		}
		if !ok {
			screen_println("\n  [EOF] Stopping CB receiver...")
			rx.running = false
			break
		}
		if !execute_cli_command(rx, eng, line) {
			break
		}
		free_all(context.temp_allocator)
	}
}
