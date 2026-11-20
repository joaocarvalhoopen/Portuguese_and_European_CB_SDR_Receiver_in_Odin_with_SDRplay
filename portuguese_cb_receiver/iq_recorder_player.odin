// Raw 6.0 MSamples/s 14-Bit Full-Spectrum I/Q Disk Recorder & Time-Stamped File Player
//
// Features:
//   - Auto-incrementing file sequence: IQ_file_0000.samples, IQ_file_0001.samples, IQ_file_0002.samples, ...
//   - Dedicated background disk-writer OS thread + 64-slot (640 ms) lock-free SPSC block ring buffer
//     so 24.0 MB/s disk I/O NEVER blocks the 6.0 MSPS SDR callback or causes audio underruns/overflows.
//   - Self-describing 512-byte header + fixed-stride 240,032-byte 10.000 ms (60,000-sample) raw 14-bit
//     I/Q blocks stamped with exact UTC nanosecond date & time + active hardware LO & gain metadata.
//   - O(1) random-access file reader & crystal-paced playback thread supporting play, stop/pause,
//     forward, backward, and goto/seek (by seconds, MM:SS, HH:MM:SS recorded clock, or YYYY-MM-DD HH:MM:SS),
//     routing the recorded 6.0 MSPS spectrum through the full 40-channel AM/FM/LSB/USB DSP, filters,
//     scanner, and 60-second Braille waterfall!

package main

import "core:fmt"
import "core:math"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:thread"
import "core:time"

IQ_FILE_MAGIC        :: "CBIQ14V1"
IQ_BLOCK_MAGIC       :: u32(0x4B425149) // "IQBK" little-endian
IQ_FILE_VERSION      :: u32(1)
IQ_HEADER_BYTES      :: 512
IQ_BLOCK_SAMPLES     :: 60_000 // 10.000 ms @ 6.000 MSPS (exact 100 blocks/sec, 480 samples @ 48 kHz)
IQ_BLOCK_HDR_BYTES   :: 32
IQ_BLOCK_CHAN_BYTES  :: IQ_BLOCK_SAMPLES * size_of(i16) // 120,000 bytes per I or Q channel
IQ_BLOCK_DISK_BYTES  :: IQ_BLOCK_HDR_BYTES + 2 * IQ_BLOCK_CHAN_BYTES // 240,032 bytes/block
IQ_REC_RING_SLOTS    :: 64 // 64 x 10 ms = 640 ms (~15.36 MB) non-blocking SPSC disk absorb cushion
IQ_REC_RING_MASK     :: u32(IQ_REC_RING_SLOTS - 1)
IQ_MAX_PATH_BYTES    :: 256
IQ_DEFAULT_STEP_SEC  :: 5.0

// Single 10.000 ms (240,032-byte) contiguous disk frame for single-syscall write/read
IQ_Raw_Disk_Slot :: struct {
	raw : [IQ_BLOCK_DISK_BYTES]u8,
}

// Decoded metadata of a single 10 ms block
IQ_Block_Meta :: struct {
	valid_samples : int,
	timestamp_ns  : i64,
	lo_freq_hz    : f64,
	rf_gr_db      : i32,
	lna_state     : u8,
}

// Decoded 512-byte file header
IQ_File_Info :: struct {
	valid            : bool,
	sample_rate_hz   : f64,
	block_samples    : int,
	initial_lo_hz    : f64,
	start_ns         : i64,
	end_ns           : i64,
	total_blocks     : i64,
	total_samples    : u64,
	duration_sec     : f64,
	file_bytes       : i64,
	initial_channel  : int,
	initial_mode     : Modulation_Mode,
	initial_filter   : Filter_Level,
	initial_rf_gr_db : i32,
	initial_lna      : u8,
	initial_clar_hz  : f64,
	iso_utc_buf      : [32]u8,
	iso_utc_len      : int,
	device_buf       : [64]u8,
	device_len       : int,
}

IQ_Recorder_Player :: struct {
	ctrl_mutex         : sync.Mutex,
	rx_owner           : ^Receiver_Context,

	// --- Asynchronous Raw 6.0 MSPS Disk Recorder ---
	rec_allocated      : bool,
	rec_slots          : []IQ_Raw_Disk_Slot,
	rec_staging        : ^IQ_Raw_Disk_Slot,
	rec_stage_fill     : int,
	rec_stage_ts_ns    : i64,
	rec_stage_lo_hz    : f64,
	rec_stage_gr_db    : i32,
	rec_stage_lna      : u8,

	rec_active         : bool, // Atomic: true while SDR callback should enqueue raw 6 MSPS samples
	rec_stop_writer    : bool, // Atomic: signals background writer thread to drain & exit
	rec_ring_write     : u32,  // SPSC producer index (SDR callback thread)
	rec_ring_read      : u32,  // SPSC consumer index (disk writer thread)
	rec_dropped_blocks : u64,
	rec_written_blocks : u64,
	rec_written_samples: u64,
	rec_written_bytes  : u64,
	rec_start_ns       : i64,
	rec_last_ns        : i64,
	rec_init_lo_hz     : f64,
	rec_file           : ^os.File,
	rec_writer_thread  : ^thread.Thread,
	rec_path_buf       : [IQ_MAX_PATH_BYTES]u8,
	rec_path_len       : int,
	rec_name_buf       : [64]u8,
	rec_name_len       : int,

	// --- Asynchronous O(1) Seekable File Player ---
	play_file_mode     : bool, // Atomic: when true, receiver processes file instead of live SDR/sim
	play_active        : bool, // Atomic: true when playing, false when paused/stopped on a loaded file
	play_in_block      : bool, // Atomic: true while player thread is actively inside 10ms DSP block
	play_worker_stop   : bool, // Atomic: signals player worker thread to exit
	play_seek_Generation: u64, // Incremented on seek so worker re-anchors monotonic pacing clock
	play_file          : ^os.File,
	play_thread        : ^thread.Thread,
	play_read_slot     : ^IQ_Raw_Disk_Slot,
	play_preview_slot  : ^IQ_Raw_Disk_Slot,
	play_suppress_aud  : bool, // Atomic: mutes audio ring push during instant paused seek preview
	play_info          : IQ_File_Info,
	play_cur_block     : i64,  // Atomic: current block index [0 .. total_blocks]
	play_cur_ts_ns     : i64,  // Atomic: exact recorded UTC timestamp (ns) of current block
	play_cur_lo_hz     : f64,
	play_path_buf      : [IQ_MAX_PATH_BYTES]u8,
	play_path_len      : int,
	play_name_buf      : [64]u8,
	play_name_len      : int,
}

// ---------------------------------------------------------------------------
// Endian-Safe Binary Serialization Helpers (Zero Struct-Padding Ambiguity)
// ---------------------------------------------------------------------------

@(private)
put_u32_le :: #force_inline proc(dst: []u8, off: int, v: u32) {
	dst[off + 0] = u8(v)
	dst[off + 1] = u8(v >> 8)
	dst[off + 2] = u8(v >> 16)
	dst[off + 3] = u8(v >> 24)
}

@(private)
get_u32_le :: #force_inline proc(src: []u8, off: int) -> u32 {
	return u32(src[off + 0]) |
	       (u32(src[off + 1]) << 8) |
	       (u32(src[off + 2]) << 16) |
	       (u32(src[off + 3]) << 24)
}

@(private)
put_u64_le :: #force_inline proc(dst: []u8, off: int, v: u64) {
	put_u32_le(dst, off,     u32(v))
	put_u32_le(dst, off + 4, u32(v >> 32))
}

@(private)
get_u64_le :: #force_inline proc(src: []u8, off: int) -> u64 {
	return u64(get_u32_le(src, off)) | (u64(get_u32_le(src, off + 4)) << 32)
}

@(private)
put_i64_le :: #force_inline proc(dst: []u8, off: int, v: i64) {
	put_u64_le(dst, off, transmute(u64)v)
}

@(private)
get_i64_le :: #force_inline proc(src: []u8, off: int) -> i64 {
	return transmute(i64)get_u64_le(src, off)
}

@(private)
put_f64_le :: #force_inline proc(dst: []u8, off: int, v: f64) {
	put_u64_le(dst, off, transmute(u64)v)
}

@(private)
get_f64_le :: #force_inline proc(src: []u8, off: int) -> f64 {
	return transmute(f64)get_u64_le(src, off)
}

// ---------------------------------------------------------------------------
// Zero-Allocation Date, Time & Seek Timestamp Formatting / Parsing Helpers
// ---------------------------------------------------------------------------

// Format UTC nanosecond timestamp as "YYYY-MM-DD HH:MM:SS" or "YYYY-MM-DD HH:MM:SS.mmm"
format_datetime_utc :: proc(ns: i64, buf: []u8, with_ms: bool = false) -> string {
	eff_ns := ns if ns > 0 else time.now()._nsec
	t      := time.Time{_nsec = eff_ns}
	yr, mo, dy          := time.date(t)
	hr, mn, sc, rem_ns  := time.precise_clock_from_time(t)
	ms                  := clamp(rem_ns / 1_000_000, 0, 999)
	if with_ms {
		return fmt.bprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d.%03d UTC", yr, int(mo), dy, hr, mn, sc, ms)
	}
	return fmt.bprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d UTC", yr, int(mo), dy, hr, mn, sc)
}

// Format UTC nanosecond timestamp as compact "HH:MM:SS" for waterfall row labels
format_hms_utc :: proc(ns: i64, buf: []u8) -> string {
	if ns <= 0 {
		return fmt.bprintf(buf, "--:--:--")
	}
	t := time.Time{_nsec = ns}
	hr, mn, sc, _ := time.precise_clock_from_time(t)
	return fmt.bprintf(buf, "%02d:%02d:%02d", hr, mn, sc)
}

// Format relative file position / duration as "MM:SS.cc" (or "HH:MM:SS.cc" when >= 1 hour)
format_duration_mmss :: proc(sec_in: f64, buf: []u8) -> string {
	s_clamped := math.max(sec_in, 0.0)
	tot_cs    := int(math.round(s_clamped * 100.0))
	cs        := tot_cs % 100
	tot_s     := tot_cs / 100
	ss        := tot_s % 60
	tot_m     := tot_s / 60
	mm        := tot_m % 60
	hh        := tot_m / 60
	if hh > 0 {
		return fmt.bprintf(buf, "%02d:%02d:%02d.%02d", hh, mm, ss, cs)
	}
	return fmt.bprintf(buf, "%02d:%02d.%02d", mm, ss, cs)
}

// Exact proleptic Gregorian calendar -> Unix UTC nanoseconds (Howard Hinnant civil algorithm)
civil_to_unix_ns :: proc(year, month, day, hour, min, sec, nanos: int) -> i64 {
	y := i64(year)
	m := i64(month)
	d := i64(day)
	if m <= 2 do y -= 1
	era  := (y if y >= 0 else y - 399) / 400
	yoe  := y - era * 400
	doy  := (153 * (m + (-3 if m > 2 else 9)) + 2) / 5 + d - 1
	doe  := yoe * 365 + yoe / 4 - yoe / 100 + doy
	days := era * 146097 + doe - 719468
	secs := days * 86_400 + i64(hour) * 3_600 + i64(min) * 60 + i64(sec)
	return secs * 1_000_000_000 + i64(nanos)
}

@(private)
parse_sec_fraction :: proc(s: string) -> (sec: int, nanos: int, ok: bool) {
	dot := strings.index_byte(s, '.')
	if dot < 0 {
		sv, sok := strconv.parse_int(s)
		if !sok || sv < 0 || sv > 59 do return 0, 0, false
		return sv, 0, true
	}
	sv, sok := strconv.parse_int(s[:dot])
	if !sok || sv < 0 || sv > 59 do return 0, 0, false
	frac_s := s[dot + 1:]
	if len(frac_s) == 0 do return sv, 0, true
	ns_acc := 0
	mult   := 100_000_000
	for i in 0 ..< len(frac_s) {
		ch := frac_s[i]
		if ch < '0' || ch > '9' do return 0, 0, false
		if mult > 0 {
			ns_acc += int(ch - '0') * mult
			mult   /= 10
		}
	}
	return sv, ns_acc, true
}

// Parse flexible seek target into relative seconds [0 .. total_sec]:
//   - "start" / "begin" -> 0.0
//   - "end"             -> total_sec
//   - "12.5" / "12.5s"  -> 12.5 s
//   - "01:25" / "01:25.4" -> MM:SS(.ff)
//   - "22:05:30"        -> Matches recorded wall-clock HH:MM:SS if inside [start_ns .. end_ns], else HH*3600+MM*60+SS
//   - "2026-11-14 22:05:30" or "2026-11-14T22:05:30.250" -> Exact recorded UTC date & time!
parse_seek_timestamp :: proc(spec_raw: string, start_ns: i64, total_sec: f64) -> (target_sec: f64, ok: bool) {
	spec := strings.trim_space(spec_raw)
	if len(spec) > 3 && strings.has_suffix(spec, "UTC") {
		spec = strings.trim_space(spec[:len(spec) - 3])
	}
	if len(spec) == 0 do return 0.0, false

	if strings.equal_fold(spec, "start") || strings.equal_fold(spec, "begin") || strings.equal_fold(spec, "zero") {
		return 0.0, true
	}
	if strings.equal_fold(spec, "end") {
		return math.max(total_sec, 0.0), true
	}

	// Plain seconds with optional trailing 's' (e.g. "8.5" or "8.5s")
	if strings.index_byte(spec, ':') < 0 && strings.index_byte(spec, '-') < 0 {
		num_s := spec
		if len(num_s) > 1 && (num_s[len(num_s) - 1] == 's' || num_s[len(num_s) - 1] == 'S') {
			num_s = num_s[:len(num_s) - 1]
		}
		if v, vok := strconv.parse_f64(num_s); vok && v >= 0.0 {
			return clamp(v, 0.0, math.max(total_sec, 0.0)), true
		}
		return 0.0, false
	}

	// Check for full "YYYY-MM-DD HH:MM:SS(.mmm)" or "YYYY-MM-DDTHH:MM:SS(.mmm)"
	date_part := ""
	time_part := spec
	if len(spec) >= 13 && spec[4] == '-' && spec[7] == '-' {
		sep_i := strings.index_any(spec, " Tt_")
		if sep_i == 10 {
			date_part = spec[:10]
			time_part = strings.trim_space(spec[11:])
		}
	}

	if len(date_part) == 10 {
		yr, yok := strconv.parse_int(date_part[0:4])
		mo, mok := strconv.parse_int(date_part[5:7])
		dy, dok := strconv.parse_int(date_part[8:10])
		if !yok || !mok || !dok || mo < 1 || mo > 12 || dy < 1 || dy > 31 {
			return 0.0, false
		}
		c1 := strings.index_byte(time_part, ':')
		if c1 <= 0 do return 0.0, false
		rem := time_part[c1 + 1:]
		c2  := strings.index_byte(rem, ':')
		if c2 <= 0 do return 0.0, false
		hh, hok := strconv.parse_int(time_part[:c1])
		mm, mmok := strconv.parse_int(rem[:c2])
		ss, nns, sok := parse_sec_fraction(rem[c2 + 1:])
		if !hok || !mmok || !sok || hh < 0 || hh > 23 || mm < 0 || mm > 59 {
			return 0.0, false
		}
		if start_ns <= 0 do return 0.0, false
		target_ns := civil_to_unix_ns(yr, mo, dy, hh, mm, ss, nns)
		rel_s     := f64(target_ns - start_ns) * 1e-9
		return clamp(rel_s, 0.0, math.max(total_sec, 0.0)), true
	}

	// Otherwise "MM:SS(.ff)" or "HH:MM:SS(.ff)"
	c1 := strings.index_byte(time_part, ':')
	if c1 <= 0 do return 0.0, false
	p0_s := time_part[:c1]
	rem  := time_part[c1 + 1:]
	c2   := strings.index_byte(rem, ':')

	if c2 < 0 {
		// "MM:SS(.ff)" -> relative minutes : seconds
		mm, mok := strconv.parse_int(p0_s)
		ss, nns, sok := parse_sec_fraction(rem)
		if !mok || !sok || mm < 0 do return 0.0, false
		rel_s := f64(mm * 60 + ss) + f64(nns) * 1e-9
		return clamp(rel_s, 0.0, math.max(total_sec, 0.0)), true
	}

	// "HH:MM:SS(.ff)" -> check recorded UTC clock first, then fall back to relative elapsed HH:MM:SS
	hh, hok := strconv.parse_int(p0_s)
	mm, mok := strconv.parse_int(rem[:c2])
	ss, nns, sok := parse_sec_fraction(rem[c2 + 1:])
	if !hok || !mok || !sok || hh < 0 || hh > 99 || mm < 0 || mm > 59 {
		return 0.0, false
	}

	if start_ns > 0 && hh <= 23 {
		yr, mo, dy := time.date(time.Time{_nsec = start_ns})
		for day_add in 0 ..= 1 {
			cand_ns := civil_to_unix_ns(yr, int(mo), dy, hh, mm, ss, nns) + i64(day_add) * 86_400_000_000_000
			diff_s  := f64(cand_ns - start_ns) * 1e-9
			if diff_s >= -0.5 && diff_s <= total_sec + 0.5 {
				return clamp(diff_s, 0.0, math.max(total_sec, 0.0)), true
			}
		}
	}

	rel_s := f64(hh * 3600 + mm * 60 + ss) + f64(nns) * 1e-9
	return clamp(rel_s, 0.0, math.max(total_sec, 0.0)), true
}

// ---------------------------------------------------------------------------
// File Naming & Header / Block Serialization
// ---------------------------------------------------------------------------

@(private)
iq_file_exists_noalloc :: proc(path: string) -> bool {
	fi, err := os.stat(path, context.allocator)
	if err == nil {
		is_reg := fi.type == .Regular
		os.file_info_delete(fi, context.allocator)
		return is_reg
	}
	return false
}

@(private)
iq_dir_exists :: proc(path: string) -> bool {
	fi, err := os.stat(path, context.allocator)
	if err == nil {
		is_d := fi.type == .Directory
		os.file_info_delete(fi, context.allocator)
		return is_d
	}
	return false
}

@(private)
iq_extract_basename :: proc(path: string) -> string {
	last_sep := -1
	for i in 0 ..< len(path) {
		if path[i] == '/' || path[i] == '\\' {
			last_sep = i
		}
	}
	return path[last_sep + 1:] if last_sep >= 0 else path
}

// Parse "IQ_file_XXXX.samples" -> (XXXX, true)
@(private)
iq_parse_sequence_number :: proc(name: string) -> (idx: int, ok: bool) {
	PREFIX :: "IQ_file_"
	SUFFIX :: ".samples"
	if len(name) != len(PREFIX) + 4 + len(SUFFIX) do return 0, false
	if !strings.has_prefix(name, PREFIX) || !strings.has_suffix(name, SUFFIX) do return 0, false
	digits := name[len(PREFIX) : len(PREFIX) + 4]
	for i in 0 ..< 4 {
		if digits[i] < '0' || digits[i] > '9' do return 0, false
	}
	return strconv.parse_int(digits)
}

// Find next auto-incrementing IQ_file_xxxx.samples in `dir` (0000, 0001, 0002, ...)
iq_next_available_filename :: proc(dir: string, full_buf: []u8, name_buf: []u8) -> (full_path: string, base_name: string, seq_idx: int) {
	eff_dir := dir if len(dir) > 0 else "."
	next_id := 0

	if infos, err := os.read_all_directory_by_path(eff_dir, context.allocator); err == nil {
		defer os.file_info_slice_delete(infos, context.allocator)
		for fi in infos {
			if idx, ok := iq_parse_sequence_number(fi.name); ok {
				if idx + 1 > next_id {
					next_id = idx + 1
				}
			}
		}
	}

	for cand := next_id; cand <= 99999; cand += 1 {
		bname := fmt.bprintf(name_buf, "IQ_file_%04d.samples", cand)
		fpath := bname if eff_dir == "." else fmt.bprintf(full_buf, "%s/%s", strings.trim_right(eff_dir, "/\\"), bname)
		if eff_dir == "." {
			copy(full_buf[:len(bname)], bname)
			fpath = string(full_buf[:len(bname)])
		}
		if !iq_file_exists_noalloc(fpath) {
			return fpath, bname, cand
		}
	}

	bname := fmt.bprintf(name_buf, "IQ_file_0000.samples")
	copy(full_buf[:len(bname)], bname)
	return string(full_buf[:len(bname)]), bname, 0
}

// Resolve user argument ("", "0", "0001", "IQ_file_0001", or full path) to an existing .samples file
iq_resolve_target_file :: proc(arg_raw: string, search_dir: string, out_buf: []u8) -> (resolved: string, ok: bool) {
	arg     := strings.trim_space(arg_raw)
	eff_dir := search_dir if len(search_dir) > 0 else "."

	// Empty arg -> pick highest-numbered IQ_file_xxxx.samples in eff_dir
	if len(arg) == 0 {
		best_idx  := -1
		best_name : [128]u8
		best_len  := 0
		if infos, err := os.read_all_directory_by_path(eff_dir, context.allocator); err == nil {
			defer os.file_info_slice_delete(infos, context.allocator)
			for fi in infos {
				if idx, iok := iq_parse_sequence_number(fi.name); iok && idx > best_idx {
					best_idx = idx
					best_len = min(len(fi.name), len(best_name))
					copy(best_name[:best_len], fi.name[:best_len])
				}
			}
		}
		if best_idx < 0 do return "", false
		nm := string(best_name[:best_len])
		res := fmt.bprintf(out_buf, "%s", nm) if eff_dir == "." else fmt.bprintf(out_buf, "%s/%s", strings.trim_right(eff_dir, "/\\"), nm)
		return res, true
	}

	// Direct regular file path check
	if iq_file_exists_noalloc(arg) {
		res := fmt.bprintf(out_buf, "%s", arg)
		return res, true
	}

	// Directory path -> pick highest-numbered IQ_file_xxxx.samples inside that directory
	if iq_dir_exists(arg) {
		return iq_resolve_target_file("", arg, out_buf)
	}

	// Pure integer index (e.g. "0", "0000", "2") -> IQ_file_0002.samples
	all_digits := len(arg) > 0
	for i in 0 ..< len(arg) {
		if arg[i] < '0' || arg[i] > '9' {
			all_digits = false
			break
		}
	}
	if all_digits {
		if seq, sok := strconv.parse_int(arg); sok && seq >= 0 {
			cand := fmt.bprintf(out_buf, "IQ_file_%04d.samples", seq) if eff_dir == "." else fmt.bprintf(out_buf, "%s/IQ_file_%04d.samples", strings.trim_right(eff_dir, "/\\"), seq)
			if iq_file_exists_noalloc(cand) do return cand, true
		}
	}

	// Append .samples if omitted
	if !strings.has_suffix(arg, ".samples") {
		cand := fmt.bprintf(out_buf, "%s.samples", arg)
		if iq_file_exists_noalloc(cand) do return cand, true
		if eff_dir != "." {
			cand2 := fmt.bprintf(out_buf, "%s/%s.samples", strings.trim_right(eff_dir, "/\\"), arg)
			if iq_file_exists_noalloc(cand2) do return cand2, true
		}
	} else if eff_dir != "." {
		cand := fmt.bprintf(out_buf, "%s/%s", strings.trim_right(eff_dir, "/\\"), arg)
		if iq_file_exists_noalloc(cand) do return cand, true
	}

	return "", false
}

@(private)
iq_encode_header :: proc(
	hdr          : ^[IQ_HEADER_BYTES]u8,
	lo_hz        : f64,
	start_ns     : i64,
	end_ns       : i64,
	total_blocks : u64,
	total_samples: u64,
	cfg          : ^Receiver_Config,
	dev_name     : string,
	dev_serial   : string,
) {
	hdr^ = {}
	copy(hdr[0:8], IQ_FILE_MAGIC)
	put_u32_le(hdr[:], 8,  IQ_FILE_VERSION)
	put_u32_le(hdr[:], 12, u32(IQ_BLOCK_SAMPLES))
	put_f64_le(hdr[:], 16, SDR_SAMPLE_RATE_HZ)
	put_f64_le(hdr[:], 24, lo_hz)
	put_i64_le(hdr[:], 32, start_ns)
	put_i64_le(hdr[:], 40, end_ns)
	put_u64_le(hdr[:], 48, total_blocks)
	put_u64_le(hdr[:], 56, total_samples)
	put_u32_le(hdr[:], 64, u32(cfg.channel))
	put_u32_le(hdr[:], 68, u32(int(cfg.mode)))
	put_u32_le(hdr[:], 72, u32(int(cfg.filter_level)))
	packed_gain := (u32(cfg.lna_state) << 16) | (u32(cfg.rf_gr_db) & 0xFFFF)
	put_u32_le(hdr[:], 76, packed_gain)
	put_f64_le(hdr[:], 80, cfg.clarifier_hz)

	iso_buf : [40]u8
	iso_str := format_datetime_utc(start_ns, iso_buf[:], true)
	copy(hdr[88 : 88 + min(len(iso_str), 32)], iso_str)

	dev_buf : [80]u8
	dev_str := fmt.bprintf(dev_buf[:], "%s (%s)", dev_name, dev_serial)
	copy(hdr[120 : 120 + min(len(dev_str), 64)], dev_str)
}

iq_read_file_info :: proc(f: ^os.File) -> (info: IQ_File_Info, ok: bool) {
	fsize, serr := os.file_size(f)
	if serr != nil || fsize < IQ_HEADER_BYTES {
		return {}, false
	}

	hdr : [IQ_HEADER_BYTES]u8
	n_hdr, rerr := os.read_at(f, hdr[:], 0)
	if rerr != nil || n_hdr != IQ_HEADER_BYTES {
		return {}, false
	}
	if string(hdr[0:8]) != IQ_FILE_MAGIC {
		return {}, false
	}
	ver := get_u32_le(hdr[:], 8)
	if ver != IQ_FILE_VERSION {
		return {}, false
	}

	blk_samps := int(get_u32_le(hdr[:], 12))
	if blk_samps != IQ_BLOCK_SAMPLES {
		return {}, false
	}

	disk_blocks := (fsize - IQ_HEADER_BYTES) / IQ_BLOCK_DISK_BYTES
	if disk_blocks <= 0 {
		return {}, false
	}

	hdr_blocks  := i64(get_u64_le(hdr[:], 48))
	tot_blocks  := disk_blocks
	if hdr_blocks > 0 && hdr_blocks <= disk_blocks {
		tot_blocks = hdr_blocks
	}

	tot_samples := get_u64_le(hdr[:], 56)
	start_ns    := get_i64_le(hdr[:], 32)
	end_ns      := get_i64_le(hdr[:], 40)

	// Inspect last block header so even an unfinalized recording has exact sample count & end timestamp!
	last_hdr : [IQ_BLOCK_HDR_BYTES]u8
	last_off := i64(IQ_HEADER_BYTES) + (tot_blocks - 1) * i64(IQ_BLOCK_DISK_BYTES)
	if nr, lerr := os.read_at(f, last_hdr[:], last_off); lerr == nil && nr == IQ_BLOCK_HDR_BYTES {
		if get_u32_le(last_hdr[:], 0) == IQ_BLOCK_MAGIC {
			last_valid := clamp(int(get_u32_le(last_hdr[:], 4)), 1, IQ_BLOCK_SAMPLES)
			calc_samps := u64(tot_blocks - 1) * u64(IQ_BLOCK_SAMPLES) + u64(last_valid)
			if tot_samples == 0 || tot_samples > calc_samps {
				tot_samples = calc_samps
			}
			last_ts := get_i64_le(last_hdr[:], 8)
			calc_end := last_ts + i64(math.round((f64(last_valid) / SDR_SAMPLE_RATE_HZ) * 1e9))
			if end_ns <= start_ns {
				end_ns = calc_end
			}
		}
	}
	if tot_samples == 0 {
		tot_samples = u64(tot_blocks) * u64(IQ_BLOCK_SAMPLES)
	}

	fs_hz := get_f64_le(hdr[:], 16)
	if fs_hz <= 0.0 do fs_hz = SDR_SAMPLE_RATE_HZ

	packed_gain := get_u32_le(hdr[:], 76)
	ch_raw      := int(get_u32_le(hdr[:], 64))
	mode_raw    := int(get_u32_le(hdr[:], 68))
	flt_raw     := int(get_u32_le(hdr[:], 72))

	info.valid            = true
	info.sample_rate_hz   = fs_hz
	info.block_samples    = blk_samps
	info.initial_lo_hz    = get_f64_le(hdr[:], 24)
	info.start_ns         = start_ns
	info.end_ns           = end_ns
	info.total_blocks     = tot_blocks
	info.total_samples    = tot_samples
	info.duration_sec     = f64(tot_samples) / fs_hz
	info.file_bytes       = fsize
	info.initial_channel  = ch_raw if is_valid_channel(ch_raw) else CB_DEFAULT_CHANNEL
	info.initial_mode     = Modulation_Mode(clamp(mode_raw, 0, 3))
	info.initial_filter   = Filter_Level(clamp(flt_raw, 0, 4))
	info.initial_rf_gr_db = clamp(i32(packed_gain & 0xFFFF), 20, 59)
	info.initial_lna      = u8((packed_gain >> 16) & 0xFF)
	info.initial_clar_hz  = clamp(get_f64_le(hdr[:], 80), CLARIFIER_MIN_HZ, CLARIFIER_MAX_HZ)

	for i in 0 ..< 32 {
		b := hdr[88 + i]
		if b == 0 do break
		info.iso_utc_buf[i] = b
		info.iso_utc_len    = i + 1
	}
	for i in 0 ..< 64 {
		b := hdr[120 + i]
		if b == 0 do break
		info.device_buf[i] = b
		info.device_len    = i + 1
	}
	return info, true
}

@(private)
iq_slot_xi_slice :: #force_inline proc(slot: ^IQ_Raw_Disk_Slot) -> []i16 {
	ptr := ([^]i16)(&slot.raw[IQ_BLOCK_HDR_BYTES])
	return ptr[:IQ_BLOCK_SAMPLES]
}

@(private)
iq_slot_xq_slice :: #force_inline proc(slot: ^IQ_Raw_Disk_Slot) -> []i16 {
	ptr := ([^]i16)(&slot.raw[IQ_BLOCK_HDR_BYTES + IQ_BLOCK_CHAN_BYTES])
	return ptr[:IQ_BLOCK_SAMPLES]
}

@(private)
iq_slot_pack_header :: #force_inline proc(slot: ^IQ_Raw_Disk_Slot, valid_samples: int, ts_ns: i64, lo_hz: f64, rf_gr_db: i32, lna: u8) {
	put_u32_le(slot.raw[:], 0,  IQ_BLOCK_MAGIC)
	put_u32_le(slot.raw[:], 4,  u32(valid_samples))
	put_i64_le(slot.raw[:], 8,  ts_ns)
	put_f64_le(slot.raw[:], 16, lo_hz)
	put_u32_le(slot.raw[:], 24, transmute(u32)rf_gr_db)
	put_u32_le(slot.raw[:], 28, u32(lna))
}

@(private)
iq_slot_unpack_header :: #force_inline proc(slot: ^IQ_Raw_Disk_Slot) -> (meta: IQ_Block_Meta, ok: bool) {
	if get_u32_le(slot.raw[:], 0) != IQ_BLOCK_MAGIC {
		return {}, false
	}
	vs := int(get_u32_le(slot.raw[:], 4))
	if vs <= 0 || vs > IQ_BLOCK_SAMPLES {
		return {}, false
	}
	meta.valid_samples = vs
	meta.timestamp_ns  = get_i64_le(slot.raw[:], 8)
	meta.lo_freq_hz    = get_f64_le(slot.raw[:], 16)
	meta.rf_gr_db      = transmute(i32)get_u32_le(slot.raw[:], 24)
	meta.lna_state     = u8(get_u32_le(slot.raw[:], 28) & 0xFF)
	return meta, true
}

// ---------------------------------------------------------------------------
// Lifecycle & Lock-Free SPSC Disk Recorder Implementation
// ---------------------------------------------------------------------------

iq_engine_init :: proc(iq: ^IQ_Recorder_Player, rx: ^Receiver_Context) {
	iq^          = {}
	iq.rx_owner  = rx
}

iq_engine_shutdown :: proc(iq: ^IQ_Recorder_Player) {
	if iq == nil do return
	if iq_is_saving(iq) {
		iq_save_stop(iq, false)
	}
	if iq_is_file_mode(iq) {
		iq_close_return_to_live(iq, nil, false)
	}
	sync.mutex_lock(&iq.ctrl_mutex)
	if iq.rec_allocated {
		delete(iq.rec_slots)
		free(iq.rec_staging)
		iq.rec_slots     = nil
		iq.rec_staging   = nil
		iq.rec_allocated = false
	}
	if iq.play_read_slot != nil {
		free(iq.play_read_slot)
		iq.play_read_slot = nil
	}
	if iq.play_preview_slot != nil {
		free(iq.play_preview_slot)
		iq.play_preview_slot = nil
	}
	sync.mutex_unlock(&iq.ctrl_mutex)
}

iq_is_saving :: #force_inline proc(iq: ^IQ_Recorder_Player) -> bool {
	return sync.atomic_load_explicit(&iq.rec_active, .Acquire)
}

iq_is_file_mode :: #force_inline proc(iq: ^IQ_Recorder_Player) -> bool {
	return sync.atomic_load_explicit(&iq.play_file_mode, .Acquire)
}

iq_is_playing :: #force_inline proc(iq: ^IQ_Recorder_Player) -> bool {
	return sync.atomic_load_explicit(&iq.play_active, .Acquire)
}

// Dedicated background OS thread draining the 64-slot SPSC ring to disk (240,032 bytes/write)
@(private)
iq_writer_worker_proc :: proc(t: ^thread.Thread) {
	iq := (^IQ_Recorder_Player)(t.data)

	for {
		r := sync.atomic_load_explicit(&iq.rec_ring_read,  .Relaxed)
		w := sync.atomic_load_explicit(&iq.rec_ring_write, .Acquire)

		if r == w {
			if sync.atomic_load_explicit(&iq.rec_stop_writer, .Acquire) {
				// Re-check once after stop flag to guarantee 100% drain of final enqueued blocks
				w_final := sync.atomic_load_explicit(&iq.rec_ring_write, .Acquire)
				if r == w_final do break
				w = w_final
			} else {
				time.sleep(2 * time.Millisecond)
				continue
			}
		}

		for r != w {
			slot := &iq.rec_slots[r & IQ_REC_RING_MASK]
			vs   := int(get_u32_le(slot.raw[:], 4))
			ts   := get_i64_le(slot.raw[:], 8)
			nw, werr := os.write(iq.rec_file, slot.raw[:])
			if werr == nil && nw == IQ_BLOCK_DISK_BYTES {
				_ = sync.atomic_add_explicit(&iq.rec_written_blocks,  1,                      .Relaxed)
				_ = sync.atomic_add_explicit(&iq.rec_written_samples, u64(max(vs, 0)),        .Relaxed)
				_ = sync.atomic_add_explicit(&iq.rec_written_bytes,   u64(IQ_BLOCK_DISK_BYTES), .Relaxed)
				end_blk_ns := ts + i64(math.round((f64(vs) / SDR_SAMPLE_RATE_HZ) * 1e9))
				sync.atomic_store_explicit(&iq.rec_last_ns, end_blk_ns, .Relaxed)
			} else {
				_ = sync.atomic_add_explicit(&iq.rec_dropped_blocks, 1, .Relaxed)
			}
			r += 1
			sync.atomic_store_explicit(&iq.rec_ring_read, r, .Release)
		}
	}
}

@(private)
iq_recorder_enqueue_staged_slot :: #force_inline proc(iq: ^IQ_Recorder_Player) {
	fill := iq.rec_stage_fill
	if fill <= 0 do return

	xi_st := iq_slot_xi_slice(iq.rec_staging)
	xq_st := iq_slot_xq_slice(iq.rec_staging)
	for k in fill ..< IQ_BLOCK_SAMPLES {
		xi_st[k] = 0
		xq_st[k] = 0
	}
	iq_slot_pack_header(
		iq.rec_staging,
		fill,
		iq.rec_stage_ts_ns,
		iq.rec_stage_lo_hz,
		iq.rec_stage_gr_db,
		iq.rec_stage_lna,
	)

	w := sync.atomic_load_explicit(&iq.rec_ring_write, .Relaxed)
	r := sync.atomic_load_explicit(&iq.rec_ring_read,  .Acquire)
	if int(w - r) < IQ_REC_RING_SLOTS {
		iq.rec_slots[w & IQ_REC_RING_MASK] = iq.rec_staging^
		sync.atomic_store_explicit(&iq.rec_ring_write, w + 1, .Release)
	} else {
		_ = sync.atomic_add_explicit(&iq.rec_dropped_blocks, 1, .Relaxed)
	}
	iq.rec_stage_fill = 0
}

// Non-blocking lock-free SPSC feed called directly from the 6.0 MSPS stream callback
iq_recorder_feed_6m :: proc(
	iq           : ^IQ_Recorder_Player,
	xi           : [^]i16,
	xq           : [^]i16,
	num_samples  : int,
	lo_hz        : f64,
	rf_gr_db     : i32,
	lna_state    : u8,
	timestamp_ns : i64,
) {
	if num_samples <= 0 || !iq_is_saving(iq) || iq.rec_staging == nil {
		return
	}

	idx := 0
	for idx < num_samples {
		if iq.rec_stage_fill == 0 {
			// Exact nanosecond timestamp at sample offset `idx` within this callback packet
			off_ns := i64(math.round((f64(idx) / SDR_SAMPLE_RATE_HZ) * 1e9))
			iq.rec_stage_ts_ns = timestamp_ns + off_ns
			iq.rec_stage_lo_hz = lo_hz
			iq.rec_stage_gr_db = rf_gr_db
			iq.rec_stage_lna   = lna_state
		}

		need  := IQ_BLOCK_SAMPLES - iq.rec_stage_fill
		chunk := min(num_samples - idx, need)

		dst_i := iq_slot_xi_slice(iq.rec_staging)
		dst_q := iq_slot_xq_slice(iq.rec_staging)
		copy(dst_i[iq.rec_stage_fill : iq.rec_stage_fill + chunk], xi[idx : idx + chunk])
		copy(dst_q[iq.rec_stage_fill : iq.rec_stage_fill + chunk], xq[idx : idx + chunk])

		iq.rec_stage_fill += chunk
		idx               += chunk

		if iq.rec_stage_fill == IQ_BLOCK_SAMPLES {
			iq_recorder_enqueue_staged_slot(iq)
		}
	}
}

// Start saving raw 6.0 MSPS 14-bit spectrum to auto-incremented IQ_file_xxxx.samples
iq_save_start :: proc(iq: ^IQ_Recorder_Player, dir_or_custom: string = "", announce: bool = true, override_start_ns: i64 = 0) -> (saved_path: string, ok: bool) {
	rx := iq.rx_owner
	if rx == nil do return "", false

	if iq_is_file_mode(iq) {
		if announce {
			screen_println("  [!] Currently in IQ file playback mode. Type 'live' first before recording a new IQ file.")
		}
		return "", false
	}

	sync.mutex_lock(&iq.ctrl_mutex)
	defer sync.mutex_unlock(&iq.ctrl_mutex)

	if sync.atomic_load_explicit(&iq.rec_active, .Acquire) {
		cur_p := string(iq.rec_path_buf[:iq.rec_path_len])
		if announce {
			screen_printfln("  [!] Already saving raw 6.0 MSPS spectrum to '%s' ('save stop' to finish).", cur_p)
		}
		return cur_p, false
	}

	if !iq.rec_allocated {
		iq.rec_slots     = make([]IQ_Raw_Disk_Slot, IQ_REC_RING_SLOTS)
		iq.rec_staging   = new(IQ_Raw_Disk_Slot)
		iq.rec_allocated = true
	}

	// Determine output path: default auto-incrementing IQ_file_0000.samples, IQ_file_0001.samples, ...
	arg := strings.trim_space(dir_or_custom)
	full_path : string
	base_name : string
	if len(arg) == 0 {
		full_path, base_name, _ = iq_next_available_filename(".", iq.rec_path_buf[:], iq.rec_name_buf[:])
	} else if strings.has_suffix(arg, ".samples") {
		bn := iq_extract_basename(arg)
		copy(iq.rec_path_buf[:min(len(arg), len(iq.rec_path_buf))], arg)
		iq.rec_path_len = min(len(arg), len(iq.rec_path_buf))
		copy(iq.rec_name_buf[:min(len(bn), len(iq.rec_name_buf))], bn)
		iq.rec_name_len = min(len(bn), len(iq.rec_name_buf))
		full_path = string(iq.rec_path_buf[:iq.rec_path_len])
		base_name = string(iq.rec_name_buf[:iq.rec_name_len])
	} else {
		_ = os.make_directory_all(arg)
		full_path, base_name, _ = iq_next_available_filename(arg, iq.rec_path_buf[:], iq.rec_name_buf[:])
	}
	iq.rec_path_len = len(full_path)
	iq.rec_name_len = len(base_name)

	f, ferr := os.open(full_path, {.Read, .Write, .Create, .Trunc}, os.Permissions_Default_File)
	if ferr != nil {
		if announce {
			screen_printfln("  [!] Failed to create IQ file '%s' (disk error).", full_path)
		}
		return "", false
	}

	sync.mutex_lock(&rx.cfg_mutex)
	cfg := rx.cfg
	sync.mutex_unlock(&rx.cfg_mutex)

	sync.mutex_lock(&rx.tel_mutex)
	dev_name   := rx.tel.device_name
	dev_serial := rx.tel.device_serial
	sync.mutex_unlock(&rx.tel_mutex)

	now_ns  := override_start_ns if override_start_ns > 0 else time.now()._nsec
	init_lo := cfg.lo_override_hz if cfg.lo_override_hz > 0.0 else hardware_lo_freq_hz(cfg.channel)

	hdr : [IQ_HEADER_BYTES]u8
	iq_encode_header(&hdr, init_lo, now_ns, now_ns, 0, 0, &cfg, dev_name, dev_serial)
	nw, werr := os.write(f, hdr[:])
	if werr != nil || nw != IQ_HEADER_BYTES {
		os.close(f)
		if announce {
			screen_printfln("  [!] Failed writing 512-byte header to '%s'.", full_path)
		}
		return "", false
	}

	iq.rec_file            = f
	iq.rec_stage_fill      = 0
	iq.rec_ring_write      = 0
	iq.rec_ring_read       = 0
	iq.rec_dropped_blocks  = 0
	iq.rec_written_blocks  = 0
	iq.rec_written_samples = 0
	iq.rec_written_bytes   = u64(IQ_HEADER_BYTES)
	iq.rec_start_ns        = now_ns
	iq.rec_last_ns         = now_ns
	iq.rec_init_lo_hz      = init_lo
	sync.atomic_store_explicit(&iq.rec_stop_writer, false, .Release)

	thr := thread.create(iq_writer_worker_proc)
	thr.data = rawptr(iq)
	iq.rec_writer_thread = thr
	thread.start(thr)

	// Enable SPSC producer feed from 6.0 MSPS callback
	sync.atomic_store_explicit(&iq.rec_active, true, .Release)

	if announce {
		dt_buf : [48]u8
		dt_str := format_datetime_utc(now_ns, dt_buf[:], true)
		f_msg : Screen_Frame
		frame_println(&f_msg, "--------------------------------------------------------------------------------")
		frame_printfln(&f_msg, "  [IQ SAVE START] Recording raw 6.000 MSPS 14-Bit full CB spectrum -> %s", full_path)
		frame_printfln(&f_msg, "    Date & Time : %s  |  LO: %.6f MHz (All 40 CB ch captured!)", dt_str, init_lo / 1e6)
		frame_println(&f_msg, "    Type 'save stop' (or 'endsave') to finalize and close the IQ file.")
		frame_println(&f_msg, "--------------------------------------------------------------------------------")
		screen_submit_frame(&f_msg)
	}

	return full_path, true
}

// Stop saving, flush any partial staging block + SPSC queue, patch 512-byte header with final counts & end timestamp
iq_save_stop :: proc(iq: ^IQ_Recorder_Player, announce: bool = true) -> bool {
	rx := iq.rx_owner
	if rx == nil do return false

	sync.mutex_lock(&iq.ctrl_mutex)
	defer sync.mutex_unlock(&iq.ctrl_mutex)

	if !sync.atomic_load_explicit(&iq.rec_active, .Acquire) {
		if announce {
			screen_println("  [!] No active IQ recording in progress (type 'save' to start saving).")
		}
		return false
	}

	// Stop new callback pushes and allow any in-flight callback to finish (< 5 ms)
	sync.atomic_store_explicit(&iq.rec_active, false, .Release)
	time.sleep(12 * time.Millisecond)

	// Flush partial tail block (< 10 ms) so not a single sample is lost
	if iq.rec_stage_fill > 0 {
		iq_recorder_enqueue_staged_slot(iq)
	}

	sync.atomic_store_explicit(&iq.rec_stop_writer, true, .Release)
	if iq.rec_writer_thread != nil {
		thread.join(iq.rec_writer_thread)
		thread.destroy(iq.rec_writer_thread)
		iq.rec_writer_thread = nil
	}

	tot_blks  := sync.atomic_load_explicit(&iq.rec_written_blocks,  .Relaxed)
	tot_samps := sync.atomic_load_explicit(&iq.rec_written_samples, .Relaxed)
	tot_bytes := sync.atomic_load_explicit(&iq.rec_written_bytes,   .Relaxed)
	drops     := sync.atomic_load_explicit(&iq.rec_dropped_blocks,  .Relaxed)
	end_ns    := sync.atomic_load_explicit(&iq.rec_last_ns,         .Relaxed)
	if end_ns <= iq.rec_start_ns {
		end_ns = time.now()._nsec
	}

	// Patch finalized end_ns, total_blocks, and total_samples into the 512-byte file header
	if iq.rec_file != nil {
		sync.mutex_lock(&rx.cfg_mutex)
		cfg := rx.cfg
		sync.mutex_unlock(&rx.cfg_mutex)
		sync.mutex_lock(&rx.tel_mutex)
		dev_name   := rx.tel.device_name
		dev_serial := rx.tel.device_serial
		sync.mutex_unlock(&rx.tel_mutex)

		hdr : [IQ_HEADER_BYTES]u8
		iq_encode_header(&hdr, iq.rec_init_lo_hz, iq.rec_start_ns, end_ns, tot_blks, tot_samps, &cfg, dev_name, dev_serial)
		_, _ = os.write_at(iq.rec_file, hdr[:], 0)
		_ = os.sync(iq.rec_file)
		_ = os.close(iq.rec_file)
		iq.rec_file = nil
	}

	saved_path := string(iq.rec_path_buf[:iq.rec_path_len])
	saved_name := string(iq.rec_name_buf[:iq.rec_name_len])
	dur_sec    := f64(tot_samps) / SDR_SAMPLE_RATE_HZ
	mb_written := f64(tot_bytes) / (1024.0 * 1024.0)

	if announce {
		b_st, b_en, b_du : [48]u8
		st_s := format_datetime_utc(iq.rec_start_ns, b_st[:], true)
		en_s := format_datetime_utc(end_ns,          b_en[:], true)
		du_s := format_duration_mmss(dur_sec,        b_du[:])
		f_msg : Screen_Frame
		frame_println(&f_msg, "--------------------------------------------------------------------------------")
		frame_printfln(&f_msg, "  [IQ SAVE COMPLETE] Saved '%s' (%.2f MiB, %d blocks, Drops: %d)",
			saved_path, mb_written, tot_blks, drops)
		frame_printfln(&f_msg, "    Recorded Span : %s  ->  %s  (Duration: %s)", st_s, en_s, du_s)
		frame_printfln(&f_msg, "    Play anytime  : 'play %s' (or 'read %s', 'goto <ts>', 'ff', 'rew', 'stop')",
			saved_name, saved_name)
		frame_println(&f_msg, "--------------------------------------------------------------------------------")
		screen_submit_frame(&f_msg)
	}
	return true
}

// ---------------------------------------------------------------------------
// Seekable File Reader & Crystal-Paced 6.0 MSPS Playback Engine
// ---------------------------------------------------------------------------

@(private)
iq_flush_audio_ring_clean :: proc(rx: ^Receiver_Context) {
	rx.ring_primed = false
	w := sync.atomic_load_explicit(&rx.ring_write, .Relaxed)
	sync.atomic_store_explicit(&rx.ring_read, w, .Release)
}

@(private)
iq_reset_waterfall_history :: proc(wf: ^Braille_Waterfall_State) {
	sync.mutex_lock(&wf.mutex)
	wf.history       = {}
	wf.head          = 0
	wf.history_count = 0
	wf.live_slice    = {}
	sync.mutex_unlock(&wf.mutex)
	wf.cur_sub_pow    = {}
	wf.cur_frames     = 0
	wf.sec_samples_6m = 0
	wf.burst_active   = false
	wf.since_burst_6m = WF_BURST_PERIOD_6M
}

// Feed up to 4 blocks (40 ms) silently when seeking/loading in paused mode so 'sp' & 's' immediately update!
@(private)
iq_preview_paused_position_locked :: proc(iq: ^IQ_Recorder_Player, target_blk: i64) {
	rx := iq.rx_owner
	if rx == nil || iq.play_file == nil || iq.play_preview_slot == nil do return

	n_preview := min(i64(4), iq.play_info.total_blocks - target_blk)
	if n_preview <= 0 do return

	channel_filter_reset(&rx.chan_filter)
	sync.atomic_store_explicit(&iq.play_suppress_aud, true, .Release)
	defer sync.atomic_store_explicit(&iq.play_suppress_aud, false, .Release)

	first_ts := iq.play_info.start_ns + target_blk * 10_000_000
	first_lo := iq.play_info.initial_lo_hz

	for b in 0 ..< n_preview {
		off := i64(IQ_HEADER_BYTES) + (target_blk + b) * i64(IQ_BLOCK_DISK_BYTES)
		nr, rerr := os.read_at(iq.play_file, iq.play_preview_slot.raw[:], off)
		if rerr != nil || nr != IQ_BLOCK_DISK_BYTES do break
		meta, mok := iq_slot_unpack_header(iq.play_preview_slot)
		if !mok do break
		if b == 0 {
			first_ts = meta.timestamp_ns
			first_lo = meta.lo_freq_hz
		}
		xi_s := iq_slot_xi_slice(iq.play_preview_slot)
		xq_s := iq_slot_xq_slice(iq.play_preview_slot)
		receiver_process_sdr_samples(
			rx,
			&xi_s[0],
			&xq_s[0],
			meta.valid_samples,
			meta.lo_freq_hz,
			meta.timestamp_ns,
			true,
		)
	}

	sync.atomic_store_explicit(&iq.play_cur_ts_ns, first_ts, .Release)
	iq.play_cur_lo_hz = first_lo
	sync.mutex_lock(&rx.tel_mutex)
	rx.tel.stream_timestamp_ns = first_ts
	sync.mutex_unlock(&rx.tel_mutex)
}

// Background crystal-accurate 10.000 ms block playback worker thread
@(private)
iq_player_worker_proc :: proc(t: ^thread.Thread) {
	iq := (^IQ_Recorder_Player)(t.data)
	rx := iq.rx_owner

	pace_t0    := time.tick_now()
	paced_blks : i64 = 0
	last_gen   : u64 = 0
	was_active := false

	for !sync.atomic_load_explicit(&iq.play_worker_stop, .Acquire) && rx.running {
		if !sync.atomic_load_explicit(&iq.play_active, .Acquire) {
			was_active = false
			time.sleep(10 * time.Millisecond)
			continue
		}

		cur_gen := sync.atomic_load_explicit(&iq.play_seek_Generation, .Acquire)
		if !was_active || cur_gen != last_gen {
			pace_t0    = time.tick_now()
			paced_blks = 0
			last_gen   = cur_gen
			was_active = true
		}

		blk_idx := sync.atomic_load_explicit(&iq.play_cur_block, .Acquire)
		if blk_idx >= iq.play_info.total_blocks {
			sync.atomic_store_explicit(&iq.play_active, false, .Release)
			iq_flush_audio_ring_clean(rx)
			was_active = false
			b_du, b_dt : [48]u8
			du_s := format_duration_mmss(iq.play_info.duration_sec, b_du[:])
			dt_s := format_datetime_utc(iq.play_info.end_ns, b_dt[:], true)
			nm_s := string(iq.play_name_buf[:iq.play_name_len])
			screen_printfln(
				"  [IQ PLAY END] Reached end of '%s' (%s | %s) — type 'play' to replay, 'back 5', 'goto <ts>', or 'live'.",
				nm_s, du_s, dt_s,
			)
			continue
		}

		off := i64(IQ_HEADER_BYTES) + blk_idx * i64(IQ_BLOCK_DISK_BYTES)
		nr, rerr := os.read_at(iq.play_file, iq.play_read_slot.raw[:], off)
		if rerr != nil || nr != IQ_BLOCK_DISK_BYTES {
			sync.atomic_store_explicit(&iq.play_active, false, .Release)
			iq_flush_audio_ring_clean(rx)
			was_active = false
			continue
		}

		meta, mok := iq_slot_unpack_header(iq.play_read_slot)
		if !mok {
			_ = sync.atomic_add_explicit(&iq.play_cur_block, 1, .Acq_Rel)
			continue
		}

		sync.atomic_store_explicit(&iq.play_cur_ts_ns, meta.timestamp_ns, .Release)
		iq.play_cur_lo_hz = meta.lo_freq_hz

		sync.mutex_lock(&rx.tel_mutex)
		rx.tel.hw_curr_gr_db = meta.rf_gr_db
		sync.mutex_unlock(&rx.tel_mutex)

		xi_s := iq_slot_xi_slice(iq.play_read_slot)
		xq_s := iq_slot_xq_slice(iq.play_read_slot)
		sync.atomic_store_explicit(&iq.play_in_block, true, .Release)
		if sync.atomic_load_explicit(&iq.play_active, .Acquire) {
			receiver_process_sdr_samples(
				rx,
				&xi_s[0],
				&xq_s[0],
				meta.valid_samples,
				meta.lo_freq_hz,
				meta.timestamp_ns,
				true,
			)
		}
		sync.atomic_store_explicit(&iq.play_in_block, false, .Release)

		// Advance block pointer ONLY if user didn't seek mid-block
		if sync.atomic_load_explicit(&iq.play_seek_Generation, .Acquire) == cur_gen {
			_ = sync.atomic_add_explicit(&iq.play_cur_block, 1, .Acq_Rel)
		}

		// Monotonic 10.000 ms/block pacing (eliminates cumulative sleep drift -> 0 audio underruns/overflows!)
		paced_blks += 1
		target_ns  := time.Duration(paced_blks * 10_000_000)
		elapsed_ns := time.tick_since(pace_t0)
		if elapsed_ns < target_ns {
			time.sleep(target_ns - elapsed_ns)
		} else if elapsed_ns - target_ns > 250 * time.Millisecond {
			pace_t0    = time.tick_now()
			paced_blks = 0
		}
	}
}

@(private)
iq_stop_player_thread_locked :: proc(iq: ^IQ_Recorder_Player) {
	sync.atomic_store_explicit(&iq.play_active,      false, .Release)
	sync.atomic_store_explicit(&iq.play_worker_stop, true,  .Release)
	if iq.play_thread != nil {
		thread.join(iq.play_thread)
		thread.destroy(iq.play_thread)
		iq.play_thread = nil
	}
	if iq.play_file != nil {
		_ = os.close(iq.play_file)
		iq.play_file = nil
	}
}

// Open/read an IQ_file_xxxx.samples file from disk and either start playing immediately or pause at 00:00
iq_file_open :: proc(iq: ^IQ_Recorder_Player, target_arg: string, auto_play: bool, announce: bool = true) -> bool {
	rx := iq.rx_owner
	if rx == nil do return false

	// If currently recording to disk, finalize and flush the recording first so user can immediately play what they just saved!
	if iq_is_saving(iq) {
		iq_save_stop(iq, announce)
	}

	sync.mutex_lock(&iq.ctrl_mutex)
	defer sync.mutex_unlock(&iq.ctrl_mutex)

	res_buf : [IQ_MAX_PATH_BYTES]u8
	resolved, rok := iq_resolve_target_file(target_arg, ".", res_buf[:])
	if !rok {
		if announce {
			if len(strings.trim_space(target_arg)) == 0 {
				screen_println("  [!] No 'IQ_file_xxxx.samples' files found in current directory (use 'save' to record one).")
			} else {
				screen_printfln("  [!] Could not find IQ file '%s' (type 'files' to list saved IQ_file_xxxx.samples).", target_arg)
			}
		}
		return false
	}

	f, ferr := os.open(resolved)
	if ferr != nil {
		if announce {
			screen_printfln("  [!] Unable to open IQ file '%s'.", resolved)
		}
		return false
	}

	info, iok := iq_read_file_info(f)
	if !iok {
		_ = os.close(f)
		if announce {
			screen_printfln("  [!] '%s' is not a valid 6.0 MSPS 14-Bit CBIQ14V1 .samples file (or contains 0 blocks).", resolved)
		}
		return false
	}

	// Shut down any previously open playback stream cleanly
	iq_stop_player_thread_locked(iq)

	if iq.play_read_slot == nil {
		iq.play_read_slot = new(IQ_Raw_Disk_Slot)
	}
	if iq.play_preview_slot == nil {
		iq.play_preview_slot = new(IQ_Raw_Disk_Slot)
	}

	bn := iq_extract_basename(resolved)
	iq.play_path_len = min(len(resolved), len(iq.play_path_buf))
	copy(iq.play_path_buf[:iq.play_path_len], resolved[:iq.play_path_len])
	iq.play_name_len = min(len(bn), len(iq.play_name_buf))
	copy(iq.play_name_buf[:iq.play_name_len], bn[:iq.play_name_len])

	iq.play_file      = f
	iq.play_info      = info
	iq.play_cur_lo_hz = info.initial_lo_hz
	sync.atomic_store_explicit(&iq.play_cur_block,  0,             .Release)
	sync.atomic_store_explicit(&iq.play_cur_ts_ns,  info.start_ns, .Release)
	sync.atomic_store_explicit(&iq.play_file_mode,  true,          .Release)
	sync.atomic_store_explicit(&iq.play_worker_stop, false,        .Release)
	_ = sync.atomic_add_explicit(&iq.play_seek_Generation, 1, .Acq_Rel)

	// Reset waterfall & audio ring for clean switch into recorded spectrum
	iq_flush_audio_ring_clean(rx)
	iq_reset_waterfall_history(&rx.waterfall)
	channel_filter_reset(&rx.chan_filter)
	demod_reset_mode(&rx.demod)
	auto_clarifier_reset(&rx.auto_clar)

	// Adopt initial recorded channel & modulation if user hasn't changed them, while keeping all 40 ch tunable!
	sync.mutex_lock(&rx.cfg_mutex)
	rx.cfg.channel = info.initial_channel
	rx.cfg.mode    = info.initial_mode
	if rx.cfg.auto_clarifier {
		rx.cfg.clarifier_hz = 0.0
	}
	sync.mutex_unlock(&rx.cfg_mutex)

	if !auto_play {
		iq_preview_paused_position_locked(iq, 0)
	}

	sync.atomic_store_explicit(&iq.play_active, auto_play, .Release)
	thr := thread.create(iq_player_worker_proc)
	thr.data = rawptr(iq)
	iq.play_thread = thr
	thread.start(thr)

	if announce {
		b_st, b_en, b_du : [48]u8
		st_s := format_datetime_utc(info.start_ns,     b_st[:], true)
		en_s := format_datetime_utc(info.end_ns,       b_en[:], true)
		du_s := format_duration_mmss(info.duration_sec, b_du[:])
		mb   := f64(info.file_bytes) / (1024.0 * 1024.0)
		state_lbl := "PLAYING" if auto_play else "LOADED & PAUSED AT 00:00.00"
		f_msg : Screen_Frame
		frame_println(&f_msg, "================================================================================")
		frame_printfln(&f_msg, "  [IQ FILE %s] %s (%.2f MiB)", state_lbl, resolved, mb)
		frame_printfln(&f_msg, "    Recorded Date/Time : %s  ->  %s", st_s, en_s)
		frame_printfln(&f_msg, "    Duration & Format  : %s (%d blocks @ 6.000 MSPS 14-Bit, LO %.6f MHz)",
			du_s, info.total_blocks, info.initial_lo_hz / 1e6)
		frame_printfln(&f_msg, "    Initial Tuning     : Ch %02d (%s) — All 40 CB channels, AM/FM/LSB/USB, Filters 0..4 & 'sp' active!",
			info.initial_channel, mode_name(info.initial_mode))
		frame_println(&f_msg, "    Transport Keywords : 'play', 'stop', 'ff [s]', 'rew [s]', 'goto <ts>', 'live'")
		frame_println(&f_msg, "================================================================================")
		screen_submit_frame(&f_msg)
	}
	return true
}

// Start or resume playback (or open & play `target_arg` if provided or no file is loaded yet)
iq_play_cmd :: proc(iq: ^IQ_Recorder_Player, target_arg: string = "", announce: bool = true) -> bool {
	arg := strings.trim_space(target_arg)
	if len(arg) > 0 || !iq_is_file_mode(iq) {
		return iq_file_open(iq, arg, true, announce)
	}

	rx := iq.rx_owner
	sync.mutex_lock(&iq.ctrl_mutex)
	defer sync.mutex_unlock(&iq.ctrl_mutex)

	cur_blk := sync.atomic_load_explicit(&iq.play_cur_block, .Acquire)
	if cur_blk >= iq.play_info.total_blocks {
		sync.atomic_store_explicit(&iq.play_cur_block, 0, .Release)
		sync.atomic_store_explicit(&iq.play_cur_ts_ns, iq.play_info.start_ns, .Release)
		iq_reset_waterfall_history(&rx.waterfall)
		cur_blk = 0
	}
	iq_flush_audio_ring_clean(rx)
	_ = sync.atomic_add_explicit(&iq.play_seek_Generation, 1, .Acq_Rel)
	sync.atomic_store_explicit(&iq.play_active, true, .Release)

	if announce {
		pos_s := f64(cur_blk) * (f64(IQ_BLOCK_SAMPLES) / SDR_SAMPLE_RATE_HZ)
		b_p, b_t, b_dt : [48]u8
		ts_ns := sync.atomic_load_explicit(&iq.play_cur_ts_ns, .Acquire)
		screen_printfln(
			"  [IQ PLAY] Playing '%s' from %s / %s  (Recorded: %s)",
			string(iq.play_name_buf[:iq.play_name_len]),
			format_duration_mmss(pos_s, b_p[:]),
			format_duration_mmss(iq.play_info.duration_sec, b_t[:]),
			format_datetime_utc(ts_ns, b_dt[:], true),
		)
	}
	return true
}

// Pause/stop file playback at current timestamp
iq_stop_pause_cmd :: proc(iq: ^IQ_Recorder_Player, announce: bool = true) -> bool {
	if !iq_is_file_mode(iq) {
		return false
	}
	rx := iq.rx_owner
	sync.mutex_lock(&iq.ctrl_mutex)
	sync.atomic_store_explicit(&iq.play_active, false, .Release)
	for wait_i := 0; wait_i < 25 && sync.atomic_load_explicit(&iq.play_in_block, .Acquire); wait_i += 1 {
		time.sleep(1 * time.Millisecond)
	}
	if rx != nil do iq_flush_audio_ring_clean(rx)
	cur_blk := sync.atomic_load_explicit(&iq.play_cur_block, .Acquire)
	ts_ns   := sync.atomic_load_explicit(&iq.play_cur_ts_ns, .Acquire)
	nm      := string(iq.play_name_buf[:iq.play_name_len])
	tot_s   := iq.play_info.duration_sec
	sync.mutex_unlock(&iq.ctrl_mutex)

	if announce {
		pos_s := math.min(f64(cur_blk) * 0.01, tot_s)
		b_p, b_t, b_dt : [48]u8
		screen_printfln(
			"  [IQ STOP/PAUSE] Paused '%s' at %s / %s  (Recorded: %s) — 'play' to resume, 'goto <ts>', or 'live'.",
			nm,
			format_duration_mmss(pos_s, b_p[:]),
			format_duration_mmss(tot_s, b_t[:]),
			format_datetime_utc(ts_ns, b_dt[:], true),
		)
	}
	return true
}

// O(1) random-access seek to exact relative second [0 .. duration_sec]
iq_seek_to_seconds :: proc(iq: ^IQ_Recorder_Player, target_sec: f64, announce: bool = true) -> bool {
	if !iq_is_file_mode(iq) {
		if announce {
			screen_println("  [!] No IQ file is currently loaded. Use 'read' or 'play' to load an IQ_file_xxxx.samples file first.")
		}
		return false
	}

	rx := iq.rx_owner
	sync.mutex_lock(&iq.ctrl_mutex)
	tot_blks := iq.play_info.total_blocks
	tot_sec  := iq.play_info.duration_sec
	clamped  := clamp(target_sec, 0.0, math.max(tot_sec, 0.0))
	blk_idx  := clamp(i64(math.floor(clamped * 100.0 + 1e-6)), 0, max(tot_blks - 1, 0))
	old_blk  := sync.atomic_load_explicit(&iq.play_cur_block, .Acquire)

	sync.atomic_store_explicit(&iq.play_cur_block, blk_idx, .Release)
	_ = sync.atomic_add_explicit(&iq.play_seek_Generation, 1, .Acq_Rel)

	if rx != nil {
		iq_flush_audio_ring_clean(rx)
		if abs(blk_idx - old_blk) >= 50 {
			iq_reset_waterfall_history(&rx.waterfall)
		}
	}

	is_playing := sync.atomic_load_explicit(&iq.play_active, .Acquire)
	est_ts_ns  := iq.play_info.start_ns + blk_idx * 10_000_000
	if !is_playing {
		iq_preview_paused_position_locked(iq, blk_idx)
		est_ts_ns = sync.atomic_load_explicit(&iq.play_cur_ts_ns, .Acquire)
	} else {
		// Read 32-byte block header at target block for exact recorded timestamp announcement
		hdr_b : [IQ_BLOCK_HDR_BYTES]u8
		off   := i64(IQ_HEADER_BYTES) + blk_idx * i64(IQ_BLOCK_DISK_BYTES)
		if nr, rerr := os.read_at(iq.play_file, hdr_b[:], off); rerr == nil && nr == IQ_BLOCK_HDR_BYTES {
			if get_u32_le(hdr_b[:], 0) == IQ_BLOCK_MAGIC {
				est_ts_ns = get_i64_le(hdr_b[:], 8)
			}
		}
		sync.atomic_store_explicit(&iq.play_cur_ts_ns, est_ts_ns, .Release)
	}
	nm := string(iq.play_name_buf[:iq.play_name_len])
	sync.mutex_unlock(&iq.ctrl_mutex)

	if announce {
		actual_s := math.min(f64(blk_idx) * 0.01, tot_sec)
		b_p, b_t, b_dt : [48]u8
		screen_printfln(
			"  [IQ SEEK] '%s' -> Position %s / %s  |  Recorded Time: %s  (%s)",
			nm,
			format_duration_mmss(actual_s, b_p[:]),
			format_duration_mmss(tot_sec,  b_t[:]),
			format_datetime_utc(est_ts_ns, b_dt[:], true),
			"PLAYING" if is_playing else "PAUSED",
		)
	}
	return true
}

// Step forward (+delta_sec) or backward (-delta_sec) in the currently loaded IQ file
iq_seek_relative :: proc(iq: ^IQ_Recorder_Player, delta_sec: f64, announce: bool = true) -> bool {
	if !iq_is_file_mode(iq) {
		if announce {
			screen_println("  [!] No IQ file is currently loaded. Use 'read' or 'play' first.")
		}
		return false
	}
	cur_blk := sync.atomic_load_explicit(&iq.play_cur_block, .Acquire)
	cur_sec := f64(cur_blk) * 0.01
	return iq_seek_to_seconds(iq, cur_sec + delta_sec, announce)
}

// Close IQ file playback and return receiver immediately to live SDR hardware / simulator stream
iq_close_return_to_live :: proc(iq: ^IQ_Recorder_Player, eng: ^SDR_Engine = nil, announce: bool = true) -> bool {
	if !iq_is_file_mode(iq) {
		if announce {
			screen_println("  [IQ] Receiver is already running on the LIVE stream.")
		}
		return false
	}

	rx := iq.rx_owner
	sync.mutex_lock(&iq.ctrl_mutex)
	nm_buf : [64]u8
	nm_len := iq.play_name_len
	copy(nm_buf[:nm_len], iq.play_name_buf[:nm_len])
	iq_stop_player_thread_locked(iq)
	sync.atomic_store_explicit(&iq.play_file_mode, false, .Release)
	iq.play_info = {}
	sync.mutex_unlock(&iq.ctrl_mutex)

	if rx != nil {
		iq_flush_audio_ring_clean(rx)
		iq_reset_waterfall_history(&rx.waterfall)
		channel_filter_reset(&rx.chan_filter)
		demod_reset_mode(&rx.demod)
		auto_clarifier_reset(&rx.auto_clar)
		sync.mutex_lock(&rx.cfg_mutex)
		rx.cfg.hw_dirty_rf = true
		sync.mutex_unlock(&rx.cfg_mutex)
		if eng != nil {
			sdr_engine_apply_pending_updates(eng)
		}
	}

	if announce {
		screen_printfln("  [IQ LIVE] Closed '%s' -> Returned to LIVE 6.000 MSPS SDR stream.", string(nm_buf[:nm_len]))
	}
	return true
}

// List all saved IQ_file_xxxx.samples files in directory with date/time stamps, duration & size
iq_print_saved_files_list :: proc(dir: string = ".") {
	eff_dir := dir if len(dir) > 0 else "."
	frame   : Screen_Frame
	frame_println(&frame, "================================================================================")
	frame_printfln(&frame, "  SAVED 6.0 MSPS 14-BIT RAW SPECTRUM FILES (%s/IQ_file_xxxx.samples)", eff_dir)
	frame_println(&frame, "--------------------------------------------------------------------------------")

	infos, err := os.read_all_directory_by_path(eff_dir, context.allocator)
	if err != nil {
		frame_println(&frame, "  [!] Unable to read directory.")
		frame_println(&frame, "================================================================================")
		screen_submit_frame(&frame)
		return
	}
	defer os.file_info_slice_delete(infos, context.allocator)

	// Collect up to 128 IQ_file_xxxx.samples sorted by sequence number
	Entry :: struct {
		seq  : int,
		idx  : int,
	}
	entries : [128]Entry
	n_ent   := 0
	for fi, i in infos {
		if fi.type == .Directory do continue
		if strings.has_suffix(fi.name, ".samples") && n_ent < len(entries) {
			seq, ok := iq_parse_sequence_number(fi.name)
			entries[n_ent] = Entry{seq = seq if ok else 999_999, idx = i}
			n_ent += 1
		}
	}
	for i in 1 ..< n_ent {
		key := entries[i]
		j   := i - 1
		for j >= 0 && entries[j].seq > key.seq {
			entries[j + 1] = entries[j]
			j -= 1
		}
		entries[j + 1] = key
	}

	if n_ent == 0 {
		frame_println(&frame, "  (No IQ_file_xxxx.samples files found yet — type 'save' to start recording!)")
	} else {
		frame_println(&frame, "  File Name              Size (MiB)   Duration    Ch/Mode   Recorded Date & Time (UTC)")
		frame_println(&frame, "  --------------------   ----------   ---------   -------   ---------------------------")
		for k in 0 ..< n_ent {
			fi := infos[entries[k].idx]
			f, ferr := os.open(fi.fullpath)
			if ferr == nil {
				hdr_info, hok := iq_read_file_info(f)
				_ = os.close(f)
				if hok {
					b_du, b_dt, b_mb : [48]u8
					du_s := format_duration_mmss(hdr_info.duration_sec, b_du[:])
					dt_s := format_datetime_utc(hdr_info.start_ns,      b_dt[:], true)
					mb   := f64(hdr_info.file_bytes) / (1024.0 * 1024.0)
					mb_s := fmt.bprintf(b_mb[:], "%.2f", mb)
					frame_printfln(
						&frame,
						"  %-20s   %10s   %9s   Ch%02d %-3s  %s",
						fi.name, mb_s, du_s, hdr_info.initial_channel, mode_name(hdr_info.initial_mode), dt_s,
					)
					continue
				}
			}
			b_mb2 : [32]u8
			mb_s2 := fmt.bprintf(b_mb2[:], "%.2f", f64(fi.size) / (1024.0 * 1024.0))
			frame_printfln(&frame, "  %-20s   %10s   (raw/open)", fi.name, mb_s2)
		}
	}
	frame_println(&frame, "--------------------------------------------------------------------------------")
	frame_println(&frame, "  Commands: 'save' | 'save stop' | 'read 0000' | 'play 0000' | 'stop' | 'ff' | 'rew' | 'goto <ts>' | 'live'")
	frame_println(&frame, "================================================================================")
	screen_submit_frame(&frame)
}

// Snapshot current recorder/player state into Telemetry_Snapshot (zero heap allocations)
iq_fill_telemetry_snapshot :: proc(iq: ^IQ_Recorder_Player, snap: ^Telemetry_Snapshot) {
	saving := iq_is_saving(iq)
	snap.iq_saving = saving
	if saving {
		samps := sync.atomic_load_explicit(&iq.rec_written_samples, .Relaxed)
		bytes := sync.atomic_load_explicit(&iq.rec_written_bytes,   .Relaxed)
		drops := sync.atomic_load_explicit(&iq.rec_dropped_blocks,  .Relaxed)
		snap.iq_save_sec   = f64(samps) / SDR_SAMPLE_RATE_HZ
		snap.iq_save_mb    = f64(bytes) / (1024.0 * 1024.0)
		snap.iq_save_drops = drops
		nlen := min(iq.rec_name_len, len(snap.iq_save_name_buf))
		copy(snap.iq_save_name_buf[:nlen], iq.rec_name_buf[:nlen])
		snap.iq_save_name_len = nlen
	}

	fmode := iq_is_file_mode(iq)
	snap.iq_file_mode = fmode
	if fmode {
		snap.iq_playing       = iq_is_playing(iq)
		cur_blk               := sync.atomic_load_explicit(&iq.play_cur_block, .Relaxed)
		snap.iq_play_tot_sec  = iq.play_info.duration_sec
		snap.iq_play_pos_sec  = math.min(f64(cur_blk) * 0.01, iq.play_info.duration_sec)
		snap.iq_file_start_ns = iq.play_info.start_ns
		snap.iq_file_end_ns   = iq.play_info.end_ns
		cur_ts                := sync.atomic_load_explicit(&iq.play_cur_ts_ns, .Relaxed)
		if cur_ts > 0 {
			snap.stream_timestamp_ns = cur_ts
		}
		if iq.play_cur_lo_hz > 0.0 {
			snap.lo_freq_hz = iq.play_cur_lo_hz
		}
		nlen := min(iq.play_name_len, len(snap.iq_play_name_buf))
		copy(snap.iq_play_name_buf[:nlen], iq.play_name_buf[:nlen])
		snap.iq_play_name_len = nlen
	}
}
