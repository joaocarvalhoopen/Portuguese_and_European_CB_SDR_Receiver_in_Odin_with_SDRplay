// Audio Playback Engine using Odin's built-in vendor:miniaudio API.
// Pulls 64-bit f64 audio frames from the lock-free SPSC receiver ring buffer at 48,000 Hz
// and outputs dual-mono stereo frames to the system audio device with zero mutexes.

package main

import "base:runtime"
import "core:sync"
import "vendor:miniaudio"

Audio_Engine :: struct {
	device      : miniaudio.device,
	initialized : bool,
	started     : bool,
	rx_ctx      : ^Receiver_Context,
}

cb_miniaudio_data_callback :: proc "c" (
	p_device    : ^miniaudio.device,
	p_output    : rawptr,
	p_input     : rawptr,
	frame_count : u32,
) {
	context = runtime.default_context()

	out := ([^]f32)(p_output)
	if p_device == nil || p_device.pUserData == nil {
		for i in 0 ..< int(frame_count * 2) {
			out[i] = 0.0
		}
		return
	}

	rx  := (^Receiver_Context)(p_device.pUserData)
	req := int(frame_count)
	w   := sync.atomic_load_explicit(&rx.ring_write, .Acquire)
	r   := sync.atomic_load_explicit(&rx.ring_read,  .Relaxed)
	avail := int(w - r)

	// Pre-cushion 64 ms (3072 samples) on startup or after wideband scan mute so USB jitter never underruns
	if !rx.ring_primed {
		if avail < AUDIO_PREFILL_SAMPLES {
			for i in 0 ..< (req * 2) {
				out[i] = 0.0
			}
			return
		}
		rx.ring_primed = true
	}

	// Elastic clock-drift protection: if multi-hour crystal mismatch accumulates > 95% ring fill,
	// smoothly drop 1 sample via 50/50 blend so the SPSC ring never reaches overflow!
	if avail > (AUDIO_RING_CAPACITY - 8192) && avail >= req + 2 {
		s0 := rx.audio_ring[r & AUDIO_RING_MASK]
		s1 := rx.audio_ring[(r + 1) & AUDIO_RING_MASK]
		rx.audio_ring[(r + 1) & AUDIO_RING_MASK] = 0.5 * (s0 + s1)
		r     += 1
		avail -= 1
	}

	n_take := min(req, max(avail, 0))
	for i in 0 ..< n_take {
		s_f32 := f32(rx.audio_ring[(r + u32(i)) & AUDIO_RING_MASK])
		out[i * 2]     = s_f32
		out[i * 2 + 1] = s_f32
	}
	if n_take > 0 {
		sync.atomic_store_explicit(&rx.ring_read, r + u32(n_take), .Release)
	}

	if n_take < req {
		for i in n_take ..< req {
			out[i * 2]     = 0.0
			out[i * 2 + 1] = 0.0
		}
		rx.ring_primed = false
		if !rx.cfg.mute_for_scan && rx.running {
			_ = sync.atomic_add_explicit(&rx.underrun_cnt, 1, .Relaxed)
		}
	}
}

audio_engine_start :: proc(ae: ^Audio_Engine, rx: ^Receiver_Context) -> bool {
	ae.rx_ctx      = rx
	ae.initialized = false
	ae.started     = false

	dev_cfg                    := miniaudio.device_config_init(.playback)
	dev_cfg.sampleRate          = u32(AUDIO_SAMPLE_RATE_HZ)
	dev_cfg.periodSizeInFrames  = 1024
	dev_cfg.periods             = 3
	dev_cfg.playback.format     = .f32
	dev_cfg.playback.channels   = 2
	dev_cfg.dataCallback        = cb_miniaudio_data_callback
	dev_cfg.pUserData           = rawptr(rx)

	res := miniaudio.device_init(nil, &dev_cfg, &ae.device)
	if res != .SUCCESS {
		screen_eprintfln("  [WARN] miniaudio device_init failed (code=%v) - continuing without audio hardware.", res)
		return false
	}
	ae.initialized = true

	rx.ring_primed = false
	sync.atomic_store_explicit(&rx.ring_read, sync.atomic_load_explicit(&rx.ring_write, .Acquire), .Release)
	sync.atomic_store_explicit(&rx.audio_consumer_on, true, .Release)

	res = miniaudio.device_start(&ae.device)
	if res != .SUCCESS {
		sync.atomic_store_explicit(&rx.audio_consumer_on, false, .Release)
		screen_eprintfln("  [WARN] miniaudio device_start failed (code=%v) - continuing without audio playback.", res)
		miniaudio.device_uninit(&ae.device)
		ae.initialized = false
		return false
	}

	ae.started = true
	return true
}

audio_engine_stop :: proc(ae: ^Audio_Engine) {
	if ae.rx_ctx != nil {
		sync.atomic_store_explicit(&ae.rx_ctx.audio_consumer_on, false, .Release)
	}
	if ae.started {
		miniaudio.device_stop(&ae.device)
		ae.started = false
	}
	if ae.initialized {
		miniaudio.device_uninit(&ae.device)
		ae.initialized = false
	}
}
