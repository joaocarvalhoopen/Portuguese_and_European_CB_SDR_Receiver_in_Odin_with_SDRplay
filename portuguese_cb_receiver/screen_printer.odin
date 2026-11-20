// Dedicated-Thread Double-Buffered Asynchronous Screen Output Engine.
//
// Architecture:
//   - Two pre-allocated 256 KiB ping-pong buffers (buffers[0] & buffers[1], zero heap allocations):
//       * Back Buffer  (buffers[back_idx])     : Accepts ultra-fast non-blocking RAM copies from
//                                                CLI, Monitor, Scanner, Startup, and Diagnostic code.
//       * Front Buffer (buffers[1 - back_idx]) : Owned exclusively by 'screen_printer_worker_proc'
//                                                on its dedicated OS thread during terminal I/O.
//   - Multi-line views (40-ch Braille Spectrum & 60s Waterfall, Receiver Status, 40-ch Scan Report,
//     Channel Table, Help, Deep Filter Diagnostic) are formatted into stack-allocated 'Screen_Frame'
//     buffers in RAM (zero syscalls, zero heap allocations) and committed atomically in one memcpy.
//   - The dedicated Screen Printer Thread swaps Front <-> Back in O(1) (< 50 ns under mutex),
//     composes [Prompt Wipe + Complete Front Buffer + Prompt Restore] into a single contiguous burst,
//     and writes to stdout/stderr completely off the SDR acquisition, DSP, and audio ring threads.

package main

import "base:runtime"
import "core:fmt"
import "core:os"
import "core:sync"
import "core:thread"

SCREEN_BUF_CAPACITY   :: 262_144 // 256 KiB per buffer (512 KiB Front + Back ping-pong pair)
SCREEN_ERR_CAPACITY   :: 16_384  // 16 KiB per buffer for stderr warnings/errors
SCREEN_FRAME_CAPACITY :: 65_536  // 64 KiB atomic stack frame builder for full-screen tables/waterfalls
SCREEN_COMPOSITE_CAP  :: SCREEN_BUF_CAPACITY + 2048

Screen_Printer_Stats :: struct {
	swaps_count    : u64,
	frames_queued  : u64,
	bytes_written  : u64,
	dropped_bytes  : u64,
}

// Stack-allocated atomic frame builder for tear-free multi-line tables, panels, and Braille waterfalls
Screen_Frame :: struct {
	buf : [SCREEN_FRAME_CAPACITY]u8,
	len : int,
}

Double_Buffered_Screen :: struct {
	mutex                : sync.Mutex,
	wake_cond            : sync.Cond,
	done_cond            : sync.Cond,
	worker_thread        : ^thread.Thread,

	init_refs            : int,
	started              : bool,
	stopping             : bool,
	flushing_now         : bool,

	// Ping-pong Front/Back double buffers (stdout & stderr)
	back_idx             : int,
	buffers              : [2][SCREEN_BUF_CAPACITY]u8,
	buf_used             : [2]int,
	err_buffers          : [2][SCREEN_ERR_CAPACITY]u8,
	err_used             : [2]int,

	// Sequence counters for deterministic screen_flush() synchronization on exit / blocking prompts
	submit_seq           : u64,
	flushed_seq          : u64,

	// Interactive 'cb> ' prompt state (updated in RAM by CLI, rendered exclusively by printer thread)
	prompt_active        : bool,
	prompt_dirty         : bool,
	prompt_text          : [CLI_MAX_LINE_LEN]u8,
	prompt_len           : int,
	prompt_cursor        : int,

	// Owned exclusively by screen_printer_worker_proc (no lock needed when accessed on worker thread)
	drawn_prompt_visible : bool,
	drawn_prompt_len     : int,
	composite_buf        : [SCREEN_COMPOSITE_CAP]u8,

	// Diagnostic telemetry counters
	swaps_count          : u64,
	frames_queued        : u64,
	bytes_written        : u64,
	dropped_bytes        : u64,
}

@(private)
g_screen : Double_Buffered_Screen

// Write all bytes in slice to file handle (handles partial writes cleanly on dedicated printer thread)
@(private)
write_all_handle :: proc(h: ^os.File, data: []u8) {
	off := 0
	for off < len(data) {
		n, err := os.write(h, data[off:])
		if err != nil || n <= 0 {
			break
		}
		off += n
	}
}

// Format 'cb> ' + editable command line + trailing space wipe + '\b' cursor placement using ONLY '\r', ' ', '\b'
@(private)
format_prompt_redraw_bytes :: proc(
	dst       : []u8,
	start_pos : int,
	text      : []u8,
	cursor    : int,
	prev_len  : int,
) -> int {
	p := start_pos
	if p + 8 + len(text) * 3 >= len(dst) do return p

	dst[p] = '\r'; p += 1
	copy(dst[p : p + 4], "cb> "); p += 4
	if len(text) > 0 {
		copy(dst[p : p + len(text)], text)
		p += len(text)
	}
	extra := prev_len - len(text)
	if extra > 0 {
		for _ in 0 ..< extra { dst[p] = ' ';  p += 1 }
		for _ in 0 ..< extra { dst[p] = '\b'; p += 1 }
	}
	back := max(len(text) - cursor, 0)
	for _ in 0 ..< back {
		dst[p] = '\b'
		p += 1
	}
	return p
}

// Dedicated background thread that swaps Front <-> Back buffers and performs all terminal OS writes
@(private)
screen_printer_worker_proc :: proc(thr: ^thread.Thread) {
	context = runtime.default_context()
	sp := (^Double_Buffered_Screen)(thr.data)

	for {
		sync.mutex_lock(&sp.mutex)
		for !sp.stopping &&
		    sp.buf_used[sp.back_idx] == 0 &&
		    sp.err_used[sp.back_idx] == 0 &&
		    !sp.prompt_dirty {
			sync.cond_wait(&sp.wake_cond, &sp.mutex)
		}

		out_ready := sp.buf_used[sp.back_idx] > 0
		err_ready := sp.err_used[sp.back_idx] > 0
		p_dirty   := sp.prompt_dirty

		if !out_ready && !err_ready && !p_dirty && sp.stopping {
			sync.mutex_unlock(&sp.mutex)
			break
		}

		// Instantaneous O(1) Double-Buffer Swap:
		// Producer threads immediately get a clean empty Back Buffer while we flush Front Buffer!
		front_idx := sp.back_idx
		out_len   := 0
		err_len   := 0
		if out_ready || err_ready {
			out_len                  = sp.buf_used[front_idx]
			err_len                  = sp.err_used[front_idx]
			sp.back_idx              = 1 - front_idx
			sp.buf_used[sp.back_idx] = 0
			sp.err_used[sp.back_idx] = 0
			sp.swaps_count          += 1
		}

		p_active := sp.prompt_active
		p_len    := sp.prompt_len
		p_cur    := sp.prompt_cursor
		p_txt    : [CLI_MAX_LINE_LEN]u8
		if p_len > 0 {
			copy(p_txt[:p_len], sp.prompt_text[:p_len])
		}
		sp.prompt_dirty = false
		seq_snap        := sp.submit_seq
		sp.flushing_now  = true
		sync.mutex_unlock(&sp.mutex)

		// --- Outside mutex: perform blocking terminal writes on this dedicated thread only ---
		bytes_this_turn : u64 = 0

		if err_len > 0 {
			write_all_handle(os.stderr, sp.err_buffers[front_idx][:err_len])
			bytes_this_turn += u64(err_len)
		}

		if out_len > 0 {
			comp_pos := 0
			// 1. Cleanly blank active 'cb> ...' line if currently visible on terminal
			if sp.drawn_prompt_visible {
				wipe_n := min(4 + sp.drawn_prompt_len, CLI_MAX_LINE_LEN + 8)
				sp.composite_buf[comp_pos] = '\r'; comp_pos += 1
				for i in 0 ..< wipe_n {
					sp.composite_buf[comp_pos] = ' '; comp_pos += 1
				}
				sp.composite_buf[comp_pos] = '\r'; comp_pos += 1
				sp.drawn_prompt_visible = false
				sp.drawn_prompt_len     = 0
			}

			// 2. Copy entire Front Buffer payload
			copy(sp.composite_buf[comp_pos : comp_pos + out_len], sp.buffers[front_idx][:out_len])
			comp_pos += out_len

			// 3. Re-attach active 'cb> ...' line + exact cursor column in the same single burst
			if p_active {
				comp_pos = format_prompt_redraw_bytes(sp.composite_buf[:], comp_pos, p_txt[:p_len], p_cur, 0)
				sp.drawn_prompt_visible = true
				sp.drawn_prompt_len     = p_len
			}

			write_all_handle(os.stdout, sp.composite_buf[:comp_pos])
			bytes_this_turn += u64(comp_pos)
		} else if p_dirty {
			if p_active {
				p_out : [CLI_MAX_LINE_LEN * 3 + 32]u8
				n_p := format_prompt_redraw_bytes(p_out[:], 0, p_txt[:p_len], p_cur, sp.drawn_prompt_len)
				write_all_handle(os.stdout, p_out[:n_p])
				sp.drawn_prompt_visible = true
				sp.drawn_prompt_len     = p_len
				bytes_this_turn        += u64(n_p)
			} else if sp.drawn_prompt_visible {
				wipe   : [CLI_MAX_LINE_LEN + 16]u8
				wipe_n := min(4 + sp.drawn_prompt_len, CLI_MAX_LINE_LEN + 8)
				wipe[0] = '\r'
				for i in 0 ..< wipe_n do wipe[1 + i] = ' '
				wipe[1 + wipe_n] = '\r'
				write_all_handle(os.stdout, wipe[: wipe_n + 2])
				sp.drawn_prompt_visible = false
				sp.drawn_prompt_len     = 0
				bytes_this_turn        += u64(wipe_n + 2)
			}
		}

		sync.mutex_lock(&sp.mutex)
		sp.bytes_written += bytes_this_turn
		sp.flushing_now   = false
		if seq_snap > sp.flushed_seq {
			sp.flushed_seq = seq_snap
		}
		sync.cond_broadcast(&sp.done_cond)
		sync.mutex_unlock(&sp.mutex)
	}
}

// Start the dedicated Double-Buffered Screen Printer Thread (idempotent / reference-counted)
screen_init :: proc() {
	sync.mutex_lock(&g_screen.mutex)
	g_screen.init_refs += 1
	if g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		return
	}
	g_screen.stopping             = false
	g_screen.flushing_now         = false
	g_screen.back_idx             = 0
	g_screen.buf_used             = {}
	g_screen.err_used             = {}
	g_screen.prompt_active        = false
	g_screen.prompt_dirty         = false
	g_screen.prompt_len           = 0
	g_screen.prompt_cursor        = 0
	g_screen.drawn_prompt_visible = false
	g_screen.drawn_prompt_len     = 0
	g_screen.started              = true
	sync.mutex_unlock(&g_screen.mutex)

	thr := thread.create(screen_printer_worker_proc)
	thr.data = rawptr(&g_screen)
	g_screen.worker_thread = thr
	thread.start(thr)
}

// Wait until all currently queued Back/Front buffer bytes have been written to the OS terminal
screen_flush :: proc() {
	sync.mutex_lock(&g_screen.mutex)
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		return
	}
	target_seq := g_screen.submit_seq
	if g_screen.buf_used[g_screen.back_idx] > 0 ||
	   g_screen.err_used[g_screen.back_idx] > 0 ||
	   g_screen.prompt_dirty {
		sync.cond_signal(&g_screen.wake_cond)
	}
	for g_screen.flushed_seq < target_seq ||
	    g_screen.flushing_now ||
	    g_screen.buf_used[g_screen.back_idx] > 0 ||
	    g_screen.err_used[g_screen.back_idx] > 0 ||
	    g_screen.prompt_dirty {
		sync.cond_wait(&g_screen.done_cond, &g_screen.mutex)
	}
	sync.mutex_unlock(&g_screen.mutex)
}

// Drain remaining queued frames and cleanly join/destroy the Screen Printer Thread
screen_shutdown :: proc() {
	sync.mutex_lock(&g_screen.mutex)
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		return
	}
	g_screen.init_refs -= 1
	if g_screen.init_refs > 0 {
		sync.mutex_unlock(&g_screen.mutex)
		screen_flush()
		return
	}
	g_screen.stopping = true
	thr := g_screen.worker_thread
	g_screen.worker_thread = nil
	sync.cond_signal(&g_screen.wake_cond)
	sync.mutex_unlock(&g_screen.mutex)

	if thr != nil {
		thread.join(thr)
		thread.destroy(thr)
	}

	sync.mutex_lock(&g_screen.mutex)
	g_screen.started = false
	sync.mutex_unlock(&g_screen.mutex)
}

screen_get_stats :: proc() -> Screen_Printer_Stats {
	sync.mutex_lock(&g_screen.mutex)
	st := Screen_Printer_Stats{
		swaps_count   = g_screen.swaps_count,
		frames_queued = g_screen.frames_queued,
		bytes_written = g_screen.bytes_written,
		dropped_bytes = g_screen.dropped_bytes,
	}
	sync.mutex_unlock(&g_screen.mutex)
	return st
}

// Non-blocking append into the active Back Buffer; wakes the dedicated Screen Printer Thread
screen_write :: proc(data: []u8) {
	if len(data) == 0 do return

	sync.mutex_lock(&g_screen.mutex)
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		write_all_handle(os.stdout, data)
		return
	}

	idx   := g_screen.back_idx
	used  := g_screen.buf_used[idx]
	avail := SCREEN_BUF_CAPACITY - used
	n_cp  := min(len(data), avail)
	if n_cp > 0 {
		copy(g_screen.buffers[idx][used : used + n_cp], data[:n_cp])
		g_screen.buf_used[idx] = used + n_cp
	}
	if n_cp < len(data) {
		g_screen.dropped_bytes += u64(len(data) - n_cp)
	}
	g_screen.frames_queued += 1
	g_screen.submit_seq    += 1
	sync.cond_signal(&g_screen.wake_cond)
	sync.mutex_unlock(&g_screen.mutex)
}

@(private)
screen_write_err :: proc(data: []u8) {
	if len(data) == 0 do return

	sync.mutex_lock(&g_screen.mutex)
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		write_all_handle(os.stderr, data)
		return
	}

	idx   := g_screen.back_idx
	used  := g_screen.err_used[idx]
	avail := SCREEN_ERR_CAPACITY - used
	n_cp  := min(len(data), avail)
	if n_cp > 0 {
		copy(g_screen.err_buffers[idx][used : used + n_cp], data[:n_cp])
		g_screen.err_used[idx] = used + n_cp
	}
	g_screen.frames_queued += 1
	g_screen.submit_seq    += 1
	sync.cond_signal(&g_screen.wake_cond)
	sync.mutex_unlock(&g_screen.mutex)
}

// ---------------------------------------------------------------------------
// Zero-Allocation Stack Frame Builder API (Atomic Tear-Free Multi-Line Panels)
// ---------------------------------------------------------------------------

frame_reset :: proc(f: ^Screen_Frame) {
	f.len = 0
}

frame_write :: proc(f: ^Screen_Frame, data: []u8) {
	avail := SCREEN_FRAME_CAPACITY - f.len
	n_cp  := min(len(data), avail)
	if n_cp > 0 {
		copy(f.buf[f.len : f.len + n_cp], data[:n_cp])
		f.len += n_cp
	}
}

frame_print :: proc(f: ^Screen_Frame, s: string) {
	frame_write(f, transmute([]u8)s)
}

frame_println :: proc(f: ^Screen_Frame, s: string = "") {
	if len(s) > 0 {
		frame_write(f, transmute([]u8)s)
	}
	if f.len < SCREEN_FRAME_CAPACITY {
		f.buf[f.len] = '\n'
		f.len += 1
	}
}

frame_printf :: proc(f: ^Screen_Frame, format: string, args: ..any) {
	if f.len >= SCREEN_FRAME_CAPACITY do return
	res := fmt.bprintf(f.buf[f.len:], format, ..args)
	f.len += len(res)
}

frame_printfln :: proc(f: ^Screen_Frame, format: string, args: ..any) {
	if f.len >= SCREEN_FRAME_CAPACITY do return
	res := fmt.bprintf(f.buf[f.len:], format, ..args)
	f.len += len(res)
	if f.len < SCREEN_FRAME_CAPACITY {
		f.buf[f.len] = '\n'
		f.len += 1
	}
}

screen_submit_frame :: proc(f: ^Screen_Frame) {
	if f.len > 0 {
		screen_write(f.buf[:f.len])
		f.len = 0
	}
}

// ---------------------------------------------------------------------------
// Zero-Allocation Convenience Print Wrappers (Format on Stack -> Back Buffer)
// ---------------------------------------------------------------------------

screen_print :: proc(s: string) {
	screen_write(transmute([]u8)s)
}

screen_println :: proc(s: string = "") {
	stack_buf : [2048]u8
	if len(s) + 1 <= len(stack_buf) {
		if len(s) > 0 {
			copy(stack_buf[:len(s)], s)
		}
		stack_buf[len(s)] = '\n'
		screen_write(stack_buf[: len(s) + 1])
	} else {
		screen_write(transmute([]u8)s)
		nl := [1]u8{'\n'}
		screen_write(nl[:])
	}
}

screen_printf :: proc(format: string, args: ..any) {
	stack_buf : [4096]u8
	res := fmt.bprintf(stack_buf[:], format, ..args)
	screen_write(transmute([]u8)res)
}

screen_printfln :: proc(format: string, args: ..any) {
	stack_buf : [4096]u8
	res := fmt.bprintf(stack_buf[: len(stack_buf) - 1], format, ..args)
	n   := len(res)
	stack_buf[n] = '\n'
	screen_write(stack_buf[: n + 1])
}

screen_eprintln :: proc(s: string) {
	stack_buf : [2048]u8
	n := min(len(s), len(stack_buf) - 1)
	if n > 0 {
		copy(stack_buf[:n], s[:n])
	}
	stack_buf[n] = '\n'
	screen_write_err(stack_buf[: n + 1])
}

screen_eprintfln :: proc(format: string, args: ..any) {
	stack_buf : [2048]u8
	res := fmt.bprintf(stack_buf[: len(stack_buf) - 1], format, ..args)
	n   := len(res)
	stack_buf[n] = '\n'
	screen_write_err(stack_buf[: n + 1])
}

// ---------------------------------------------------------------------------
// Interactive 'cb> ' Line-Editor Synchronization (Zero Direct stdout Writes)
// ---------------------------------------------------------------------------

screen_prompt_begin :: proc(ed: ^CLI_Line_Editor) {
	sync.mutex_lock(&g_screen.mutex)
	g_screen.prompt_active = true
	g_screen.prompt_len    = clamp(ed.length, 0, CLI_MAX_LINE_LEN)
	g_screen.prompt_cursor = clamp(ed.cursor, 0, g_screen.prompt_len)
	if g_screen.prompt_len > 0 {
		copy(g_screen.prompt_text[:g_screen.prompt_len], ed.buf[:g_screen.prompt_len])
	}
	g_screen.prompt_dirty = true
	g_screen.submit_seq  += 1
	if g_screen.started {
		sync.cond_signal(&g_screen.wake_cond)
	}
	sync.mutex_unlock(&g_screen.mutex)
}

screen_prompt_update :: proc(ed: ^CLI_Line_Editor) {
	sync.mutex_lock(&g_screen.mutex)
	if g_screen.prompt_active {
		g_screen.prompt_len    = clamp(ed.length, 0, CLI_MAX_LINE_LEN)
		g_screen.prompt_cursor = clamp(ed.cursor, 0, g_screen.prompt_len)
		if g_screen.prompt_len > 0 {
			copy(g_screen.prompt_text[:g_screen.prompt_len], ed.buf[:g_screen.prompt_len])
		}
		g_screen.prompt_dirty = true
		g_screen.submit_seq  += 1
		if g_screen.started {
			sync.cond_signal(&g_screen.wake_cond)
		}
	}
	sync.mutex_unlock(&g_screen.mutex)
}

// Atomically commit 'cb> <command>\r\n' to the Back Buffer and deactivate live prompt editing
screen_prompt_submit :: proc(ed: ^CLI_Line_Editor) {
	echo_buf : [CLI_MAX_LINE_LEN + 16]u8
	p := 0
	copy(echo_buf[p : p + 4], "cb> "); p += 4
	n_cmd := clamp(ed.length, 0, CLI_MAX_LINE_LEN)
	if n_cmd > 0 {
		copy(echo_buf[p : p + n_cmd], ed.buf[:n_cmd])
		p += n_cmd
	}
	echo_buf[p] = '\r'; p += 1
	echo_buf[p] = '\n'; p += 1

	sync.mutex_lock(&g_screen.mutex)
	g_screen.prompt_active = false
	g_screen.prompt_dirty  = false
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		write_all_handle(os.stdout, echo_buf[:p])
		return
	}
	idx   := g_screen.back_idx
	used  := g_screen.buf_used[idx]
	n_cp  := min(p, SCREEN_BUF_CAPACITY - used)
	if n_cp > 0 {
		copy(g_screen.buffers[idx][used : used + n_cp], echo_buf[:n_cp])
		g_screen.buf_used[idx] = used + n_cp
	}
	g_screen.frames_queued += 1
	g_screen.submit_seq    += 1
	sync.cond_signal(&g_screen.wake_cond)
	sync.mutex_unlock(&g_screen.mutex)
}

// Atomically commit '^C\r\n' or '\r\n' on Ctrl+C / EOF and deactivate live prompt editing
screen_prompt_cancel :: proc(ed: ^CLI_Line_Editor, ctrl_c: bool) {
	echo_buf : [CLI_MAX_LINE_LEN + 16]u8
	p := 0
	copy(echo_buf[p : p + 4], "cb> "); p += 4
	n_cmd := clamp(ed.length, 0, CLI_MAX_LINE_LEN)
	if n_cmd > 0 {
		copy(echo_buf[p : p + n_cmd], ed.buf[:n_cmd])
		p += n_cmd
	}
	if ctrl_c {
		echo_buf[p] = '^'; p += 1
		echo_buf[p] = 'C'; p += 1
	}
	echo_buf[p] = '\r'; p += 1
	echo_buf[p] = '\n'; p += 1

	sync.mutex_lock(&g_screen.mutex)
	g_screen.prompt_active = false
	g_screen.prompt_dirty  = false
	if !g_screen.started {
		sync.mutex_unlock(&g_screen.mutex)
		write_all_handle(os.stdout, echo_buf[:p])
		return
	}
	idx   := g_screen.back_idx
	used  := g_screen.buf_used[idx]
	n_cp  := min(p, SCREEN_BUF_CAPACITY - used)
	if n_cp > 0 {
		copy(g_screen.buffers[idx][used : used + n_cp], echo_buf[:n_cp])
		g_screen.buf_used[idx] = used + n_cp
	}
	g_screen.frames_queued += 1
	g_screen.submit_seq    += 1
	sync.cond_signal(&g_screen.wake_cond)
	sync.mutex_unlock(&g_screen.mutex)
}

screen_prompt_end :: proc() {
	sync.mutex_lock(&g_screen.mutex)
	if g_screen.prompt_active {
		g_screen.prompt_active = false
		g_screen.prompt_dirty  = true
		g_screen.submit_seq   += 1
		if g_screen.started {
			sync.cond_signal(&g_screen.wake_cond)
		}
	}
	sync.mutex_unlock(&g_screen.mutex)
}
