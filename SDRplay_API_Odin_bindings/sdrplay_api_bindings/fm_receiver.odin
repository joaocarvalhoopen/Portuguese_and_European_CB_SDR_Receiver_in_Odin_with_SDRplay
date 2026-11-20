// FM Radio Receiver using SDRplay API and Odin vendor:miniaudio
//
// Pipeline:
//   SDR (2 MHz, zero-IF, 200 kHz BW, decimation=4) → 500 kS/s complex I/Q
//   → FM demodulation (atan2 phase discriminator)
//   → DC blocker
//   → Deemphasis filter (75 µs) — mono (L+R) path
//   → 19 kHz pilot extraction → PLL → 38 kHz coherent subcarrier
//   → L-R demodulation (multiply demodulated signal by 38 kHz, low-pass)
//   → Stereo combination: L = (L+R)+(L-R), R = (L+R)-(L-R)
//   → Software decimation by 10 (boxcar average)
//   → 50 kS/s stereo audio → miniaudio (resampled to device native rate)
//
// AGC is enabled by default on the SDR.
// Automatic audio normalization prevents saturation.
// Press + / - to adjust volume, Enter to quit.
//
// Tunes in Portugal Lisbon to the Radio FM Comercial 97.4 MHz
//
// Usage: ./sdrplay_bindings_test -fm 97.4

package sdrplay_api_bindings

import "core:c"
import "core:fmt"
import "core:math"
import "core:os"
import "base:runtime"
import "core:sync"
import "core:time"
import "vendor:miniaudio"

foreign import libc "system:c"

foreign libc {

	system :: proc ( cmd : cstring ) -> c.int ---
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

RING_SIZE          :: 1 << 17 // 131072 stereo frame slots (~2.6 s buffer at 50 kHz)

FM_SDR_FS          :: f64(2_000_000.0) // SDR internal sample rate
FM_SDR_DECIM       :: 4                // SDR hardware decimation → 500 kS/s output
FM_OUTPUT_RATE     :: f64( FM_SDR_FS / f64( FM_SDR_DECIM ) ) // 500,000 Hz

AUDIO_DECIM_FACTOR :: 10              // software decimation
AUDIO_RATE         :: f64( FM_OUTPUT_RATE / f64( AUDIO_DECIM_FACTOR ) ) // 50,000 Hz

DEEMPHASIS_TAU     :: f64(75e-6) // 75 µs (North America / Japan)
deemphasis_alpha : f32           // computed at startup

DC_ALPHA           :: f32(0.0002) // DC blocker (slow tracking)

// FM stereo pilot and subcarrier frequencies (at 500 kS/s rate)
PILOT_FREQ         :: f64( 19_000.0 )
SUBCARRIER_FREQ    :: f64( 38_000.0 )

// PLL for pilot recovery — loop filter coefficients
// The PLL locks a local oscillator to the 19 kHz pilot tone
PLL_LOOP_BW        :: f64( 0.001 )   // loop bandwidth (narrow for clean lock)
PILOT_THRESHOLD    :: f32( 0.02 )    // minimum pilot amplitude for stereo

// Audio normalization — track peak and apply slow AGC
NORM_ATTACK        :: f32( 0.001 )    // fast attack (reduce gain quickly)
NORM_DECAY         :: f32( 0.00002 )  // slow decay (restore gain gradually)
NORM_TARGET        :: f32( 0.75 )     // target peak level
NORM_MIN_GAIN      :: f32( 0.05 )     // don't go below this gain
NORM_MAX_GAIN      :: f32( 4.0 )      // don't go above this gain

// Volume control (adjusted by + / - keys)
volume         : f32 = 1.0
volume_changed : bool = false

// ---------------------------------------------------------------------------
// Ring buffer — stereo interleaved (L, R, L, R, ...)
// SDR callback writes, audio callback reads
// ---------------------------------------------------------------------------

fm_ring_data  : [ RING_SIZE * 2 ]f32 // *2 for stereo interleaved
fm_ring_write : u32 = 0
fm_ring_read  : u32 = 0
fm_ring_mutex : sync.Mutex

// ---------------------------------------------------------------------------
// FM demodulation state (only accessed from SDR callback thread)
// ---------------------------------------------------------------------------

fm_prev_i : f32 = 0
fm_prev_q : f32 = 0
fm_dc_avg : f32 = 0

// Deemphasis (applied to L+R and L-R separately)
fm_deemph_lr  : f32 = 0
fm_deemph_lmr : f32 = 0

// Software decimation accumulators
fm_decim_accum_l : f32 = 0
fm_decim_accum_r : f32 = 0
fm_decim_count   : u32 = 0

// Pilot PLL state
pilot_cos       : f32 = 1.0   // local oscillator cosine (19 kHz)
pilot_sin       : f32 = 0.0   // local oscillator sine (19 kHz)
pilot_phase_err : f32 = 0
pilot_amplitude : f32 = 0     // detected pilot strength (smoothed)

// 38 kHz subcarrier (2x pilot frequency, phase-locked)
sub_cos : f32 = 1.0
sub_sin : f32 = 0.0

// Audio normalization
norm_peak : f32 = 0.1
norm_gain : f32 = 1.0

// Device state
fm_dev    : DeviceT
fm_params : ^DeviceParamsT

// ---------------------------------------------------------------------------
// Oscillator update — direct digital synthesis (NCO)
// ---------------------------------------------------------------------------

nco_step :: proc ( cos_val, sin_val : ^f32, phase_inc : f32 ) {

	// Rotate vector by phase_inc using:
	//   cos' = cos*cos(d) - sin*sin(d)
	//   sin' = sin*cos(d) + cos*sin(d)
	// For small d, cos(d) ≈ 1 - d²/2, sin(d) ≈ d
	// But for accuracy at 19kHz/500kHz, use full trig
	d_cos   := math.cos( phase_inc )
	d_sin   := math.sin( phase_inc )
	new_cos := cos_val^ * d_cos - sin_val^ * d_sin
	new_sin := sin_val^ * d_sin + cos_val^ * d_cos
	cos_val^ = new_cos
	sin_val^ = new_sin
}

// ---------------------------------------------------------------------------
// SDR stream callback — FM stereo demodulation pipeline
// ---------------------------------------------------------------------------

fm_stream_callback :: proc "c" (
	xi         : [^]i16,
	xq         : [^]i16,
	params     : ^StreamCbParamsT,
	numSamples : u32,
	reset      : u32,
	cbContext  : rawptr,
) {

	context = runtime.default_context( )

	pilot_inc := f32( 2.0 * math.PI * PILOT_FREQ / FM_OUTPUT_RATE )
	sub_inc   := f32( 2.0 * math.PI * SUBCARRIER_FREQ / FM_OUTPUT_RATE )

	for i in 0 ..< int( numSamples ) {

		curr_i := f32( xi[ i ] )
		curr_q := f32( xq[ i ] )

		// --- FM demodulate: phase difference via atan2 ---
		di    := f64( fm_prev_i * curr_i + fm_prev_q * curr_q )
		dq    := f64( fm_prev_i * curr_q - fm_prev_q * curr_i )
		demod := f32( math.atan2( dq, di ) )

		fm_prev_i = curr_i
		fm_prev_q = curr_q

		// --- DC blocker (remove tuner LO offset) ---
		fm_dc_avg = DC_ALPHA * demod + ( 1.0 - DC_ALPHA ) * fm_dc_avg
		demod -= fm_dc_avg

		// --- Pilot PLL: lock local 19 kHz oscillator to pilot ---
		// Mix demodulated signal with local oscillator
		pilot_i := demod * pilot_cos
		pilot_q := demod * pilot_sin

		// Phase detector: error = -atan2(Q, I) for Costas-like loop
		// For small angles, error ≈ pilot_q / amplitude
		pilot_amp_inst := math.sqrt( pilot_i * pilot_i + pilot_q * pilot_q )
		if pilot_amp_inst > 1e-8 {

			pilot_phase_err = -pilot_q / pilot_amp_inst
		}

		// Smooth pilot amplitude estimate
		pilot_amplitude = 0.999 * pilot_amplitude + 0.001 * pilot_amp_inst

		// Loop filter: adjust NCO phase by error * loop bandwidth
		phase_adj := pilot_phase_err * f32( PLL_LOOP_BW )
		nco_step( & pilot_cos, & pilot_sin, pilot_inc + phase_adj )

		// 38 kHz subcarrier: frequency = 2x pilot, phase-locked
		// Advance subcarrier by 2x the (corrected) pilot phase
		nco_step( & sub_cos, & sub_sin, sub_inc + 2.0 * phase_adj )

		// --- Stereo demodulation ---
		// L+R (baseband mono) = demod (low-pass filtered)
		// L-R = demod * 38 kHz subcarrier (DSB-SC), then low-pass
		l_plus_r  := demod
		l_minus_r := demod * sub_cos

		// Deemphasis on both paths (75 µs)
		fm_deemph_lr  = deemphasis_alpha * l_plus_r  + ( 1.0 - deemphasis_alpha ) * fm_deemph_lr
		fm_deemph_lmr = deemphasis_alpha * l_minus_r + ( 1.0 - deemphasis_alpha ) * fm_deemph_lmr

		// Stereo combination (only if pilot is strong enough)
		left, right : f32
		if pilot_amplitude > PILOT_THRESHOLD {

			// Full stereo
			left  = fm_deemph_lr + fm_deemph_lmr
			right = fm_deemph_lr - fm_deemph_lmr
		} else {

			// Pilot too weak — fall back to mono
			left  = fm_deemph_lr
			right = fm_deemph_lr
		}

		// --- Software decimation by AUDIO_DECIM_FACTOR (boxcar average) ---
		fm_decim_accum_l += left
		fm_decim_accum_r += right
		fm_decim_count   += 1
		if fm_decim_count >= AUDIO_DECIM_FACTOR {

			fm_decim_count = 0
			s_l := fm_decim_accum_l / f32( AUDIO_DECIM_FACTOR )
			s_r := fm_decim_accum_r / f32( AUDIO_DECIM_FACTOR )
			fm_decim_accum_l = 0
			fm_decim_accum_r = 0

			// --- Audio normalization (anti-saturation) ---
			// Track peak and adjust gain slowly
			peak := math.max( math.abs( s_l ), math.abs( s_r ) )
			if peak > norm_peak {

				// Fast attack
				norm_peak = NORM_ATTACK * peak + ( 1.0 - NORM_ATTACK ) * norm_peak
			} else {

				// Slow decay
				norm_peak = NORM_DECAY * peak + ( 1.0 - NORM_DECAY ) * norm_peak
			}

			// Compute target gain
			if norm_peak > 1e-6 {

				target_gain := NORM_TARGET / norm_peak
				// Smoothly approach target gain
				norm_gain = 0.5 * norm_gain + 0.5 * target_gain
				if norm_gain < NORM_MIN_GAIN { norm_gain = NORM_MIN_GAIN }
				if norm_gain > NORM_MAX_GAIN { norm_gain = NORM_MAX_GAIN }
			}

			// Apply normalization + user volume
			s_l *= norm_gain * volume
			s_r *= norm_gain * volume

			// Hard clip to [-1, 1] as final safety
			if s_l > 1.0       { s_l = 1.0 }
			else if s_l < -1.0 { s_l = -1.0 }
			if s_r > 1.0       { s_r = 1.0 }
			else if s_r < -1.0 { s_r = -1.0 }

			// Push to ring buffer (interleaved L, R)
			sync.mutex_lock( & fm_ring_mutex )
			next_write := ( fm_ring_write + 1 ) % RING_SIZE
			if next_write != fm_ring_read {

				idx                    := fm_ring_write * 2
				fm_ring_data[ idx ]     = s_l
				fm_ring_data[ idx + 1 ] = s_r
				fm_ring_write           = next_write
			}
			sync.mutex_unlock( & fm_ring_mutex )
		}
	}
}

// ---------------------------------------------------------------------------
// SDR event callback — handle power overload
// ---------------------------------------------------------------------------

fm_event_callback :: proc "c" (
	eventId   : EventT,
	tuner     : TunerSelectT,
	params    : ^EventParamsT,
	cbContext : rawptr,
) {

	#partial switch eventId {
	case .PowerOverloadChange:
		sdrplay_api_Update( fm_dev.dev, fm_dev.tuner, .Ctrl_OverloadMsgAck, .None )
	case .DeviceRemoved:
		// Acknowledge silently
	case: // ignore other events
	}
}

// ---------------------------------------------------------------------------
// Miniaudio data callback — read from ring buffer (stereo)
// ---------------------------------------------------------------------------

fm_audio_callback :: proc "c" (
	pDevice    : ^miniaudio.device,
	pOutput    : rawptr,
	pInput     : rawptr,
	frameCount : u32,
) {

	out := ( [^]f32 )( pOutput )

	sync.mutex_lock( & fm_ring_mutex )
	for i in 0 ..< int( frameCount ) {

		if fm_ring_read != fm_ring_write {

			idx             := fm_ring_read * 2
			out[ i * 2 ]     = fm_ring_data[ idx ]      // Left
			out[ i * 2 + 1 ] = fm_ring_data[ idx + 1 ]  // Right
			fm_ring_read     = ( fm_ring_read + 1 ) % RING_SIZE
		} else {

			out[ i * 2 ]     = 0.0 // underrun
			out[ i * 2 + 1 ] = 0.0
		}
	}
	sync.mutex_unlock( & fm_ring_mutex )
}

// ---------------------------------------------------------------------------
// Terminal raw mode — read single keypresses without Enter
// ---------------------------------------------------------------------------

term_state_saved : bool = false
term_orig        : [ 32 ]u8

term_set_raw :: proc ( ) -> bool {

	// Use C system() to set raw mode on the terminal
	// This lets us read single keypresses (+/-) without waiting for Enter
	ret := system( "stty -echo -icanon min 1 time 0 < /dev/tty" )
	if ret != 0 {

		return false
	}
	term_state_saved = true
	return true
}

term_restore :: proc ( ) {

	if term_state_saved {

		system( "stty echo icanon < /dev/tty" )
		term_state_saved = false
	}
}

// ---------------------------------------------------------------------------
// FM receiver entry point
// ---------------------------------------------------------------------------

start_fm_receiver :: proc(freq_mhz: f64) {

	fmt.println( "==============================================" )
	fmt.println( "  SDRplay in Odin FM Radio Receiver (Stereo)" )
	fmt.println( "==============================================" )
	fmt.printfln( "  Frequency: %.1f MHz", freq_mhz )
	fmt.println( )

	// Compute deemphasis filter coefficient (needs math.exp at runtime)
	deemphasis_alpha = f32( 1.0 - math.exp( -1.0 / ( FM_OUTPUT_RATE * DEEMPHASIS_TAU ) ) )

	// Validate frequency (FM broadcast band)
	if freq_mhz < 64.0 || freq_mhz > 110.0 {

		fmt.eprintfln( "  ERROR: Frequency %.1f MHz is outside FM band (64-110 MHz)", freq_mhz )
		return
	}

	// State tracking for cleanup
	api_opened      := false
	device_selected := false
	streaming       := false
	audio_init      := false
	ma_dev : miniaudio.device

	defer {
		if term_state_saved { term_restore( ) }
		if audio_init {

			miniaudio.device_stop( & ma_dev )
			miniaudio.device_uninit( & ma_dev )
		}
		if streaming {

			sdrplay_api_Uninit( fm_dev.dev )
		}
		if device_selected {

			sdrplay_api_ReleaseDevice( & fm_dev )
		}
		if api_opened {

			sdrplay_api_Close( )
		}

		fmt.println( "\n  FM receiver stopped." )
	}

	// --- Open SDR API ---
	err := sdrplay_api_Open( )
	if !check_err( err, "sdrplay_api_Open" ) { return }
	api_opened = true

	ver : f32
	err = sdrplay_api_ApiVersion( & ver )
	if !check_err( err, "sdrplay_api_ApiVersion" ) { return }
	fmt.printfln( "  API version: %.2f", ver )

	// --- Enumerate devices ---
	devices : [ SDRPLAY_MAX_DEVICES ]DeviceT
	numDevs : u32
	err = sdrplay_api_GetDevices( & devices[ 0 ], &numDevs, SDRPLAY_MAX_DEVICES )
	if !check_err( err, "sdrplay_api_GetDevices" ) { return }

	if numDevs == 0 {

		fmt.eprintln( "  ERROR: No SDRplay devices found" )
		return
	}

	found := false
	for i in 0 ..< int( numDevs ) {

		d := & devices[ i ]
		fmt.printf( "  Found: %s (hwVer=%d, serial=%s)\n",
			hw_version_name( d.hwVer ), d.hwVer, serial_string( d ) )
		if d.valid != 0 && !found {

			fm_dev = devices[ i ]
			found  = true
		}
	}
	if !found {

		fmt.eprintln( "  ERROR: No valid device found" )
		return
	}

	// --- Select device ---
	err = sdrplay_api_SelectDevice( & fm_dev )
	if !check_err( err, "sdrplay_api_SelectDevice" ) { return }
	device_selected = true

	sdrplay_api_DebugEnable( fm_dev.dev, .Disable )

	err = sdrplay_api_GetDeviceParams( fm_dev.dev, & fm_params )
	if !check_err( err, "sdrplay_api_GetDeviceParams" ) { return }

	if fm_params.devParams == nil || fm_params.rxChannelA == nil {
		fmt.eprintln( "  ERROR: Null device parameters" )
		return
	}

	// --- Configure SDR for FM reception ---
	dp := fm_params.devParams
	ch := fm_params.rxChannelA

	// Device: 2 MHz internal sample rate
	dp.fsFreq.fsHz = 2_000_000.0
	dp.ppm = 0.0

	// RSPdx params: Antenna A, no bias-T, no notches, no HDR
	dp.rspDxParams.hdrEnable        = 0
	dp.rspDxParams.biasTEnable      = 0
	dp.rspDxParams.antennaSel       = .RspDx_ANTENNA_A
	dp.rspDxParams.rfNotchEnable    = 0
	dp.rspDxParams.rfDabNotchEnable = 0

	// Tuner: target frequency, zero-IF, 200 kHz BW, auto LO
	ch.tunerParams.rfFreq.rfHz = freq_mhz * 1_000_000.0
	ch.tunerParams.bwType = .BW_0_200
	ch.tunerParams.ifType = .IF_Zero
	ch.tunerParams.loMode = .LO_Auto

	// Gain: start at moderate level, AGC will adjust automatically
	ch.tunerParams.gain.gRdB     = 50
	ch.tunerParams.gain.LNAstate = 0
	ch.tunerParams.gain.minGr    = .NORMAL_MIN_GR

	// DC offset tuner calibration
	ch.tunerParams.dcOffsetTuner.dcCal           = 3
	ch.tunerParams.dcOffsetTuner.speedUp         = 0
	ch.tunerParams.dcOffsetTuner.trackTime       = 1
	ch.tunerParams.dcOffsetTuner.refreshRateTime = 2048

	// Control: DC/IQ correction on, AGC enabled by default, decimation = 4
	ch.ctrlParams.dcOffset.DCenable           = 1
	ch.ctrlParams.dcOffset.IQenable           = 1
	ch.ctrlParams.decimation.enable           = 1
	ch.ctrlParams.decimation.decimationFactor = FM_SDR_DECIM
	ch.ctrlParams.decimation.wideBandSignal   = 1
	ch.ctrlParams.agc.enable                  = .AGC_CTRL_EN  // AGC enabled by default
	ch.ctrlParams.agc.setPoint_dBfs           = -30    // conservative setpoint to avoid saturation
	ch.ctrlParams.adsbMode                    = .DECIMATION

	// RSPdx HDR bandwidth (not used, but set to default)
	ch.rspDxTunerParams.hdrBw = .HDRMODE_BW_1_700

	fmt.printfln( "  SDR config: fs=2MHz, decim=%d → %d kS/s, bw=200kHz, IF=Zero",
		FM_SDR_DECIM, u32( FM_OUTPUT_RATE / 1000.0 ) )
	fmt.println( "  AGC: enabled (setpoint=-30 dBfs)" )
	fmt.printf( "  Audio: %d Hz stereo, deemphasis=%.0fµs\n",
		u32( AUDIO_RATE ), DEEMPHASIS_TAU * 1e6 )
	fmt.println( "  Auto-normalization: enabled (anti-saturation)" )

	// --- Init SDR streaming ---
	cbFns := CallbackFnsT{

		StreamACbFn = fm_stream_callback,
		StreamBCbFn = fm_stream_callback,
		EventCbFn   = fm_event_callback,
	}

	err = sdrplay_api_Init( fm_dev.dev, &cbFns, nil )
	if !check_err( err, "sdrplay_api_Init" ) { return }
	streaming = true

	// Give the SDR a moment to start producing samples
	fmt.println( "  Waiting for samples..." )
	{

		waited: u32 = 0
		for waited < 3000 {

			sync.mutex_lock( & fm_ring_mutex )
			has_data := fm_ring_write != fm_ring_read
			sync.mutex_unlock( & fm_ring_mutex )
			if has_data { break }
			time.sleep( time.Millisecond * 50 )
			waited += 50
		}
	}

	// --- Initialize miniaudio playback device (stereo) ---
	audio_config                  := miniaudio.device_config_init( .playback )
	audio_config.sampleRate        = u32( AUDIO_RATE )
	audio_config.playback.format   = .f32
	audio_config.playback.channels = 2  // stereo
	audio_config.dataCallback      = fm_audio_callback
	audio_config.pUserData         = nil

	res := miniaudio.device_init( nil, & audio_config, & ma_dev )
	if res != .SUCCESS {

		fmt.eprintfln( "  ERROR: miniaudio device_init failed (result=%d)", res )
		return
	}
	audio_init = true

	res = miniaudio.device_start( & ma_dev )
	if res != .SUCCESS {

		fmt.eprintfln( "  ERROR: miniaudio device_start failed (result=%d)", res )
		return
	}

	// --- Set terminal to raw mode for keypress reading ---
	raw_ok := term_set_raw( )

	fmt.println( )
	fmt.println(  "  ╔══════════════════════════════════════════╗" )
	fmt.printfln( "  ║  FM Radio: %.1f MHz  │  Stereo  │  AGC   ║", freq_mhz )
	fmt.println(  "  ╚══════════════════════════════════════════╝" )
	fmt.println( )
	fmt.println( "  Controls:" )
	fmt.println( "    + / =   Increase volume" )
	fmt.println( "    -       Decrease volume" )
	fmt.println( "    Enter   Quit" )
	fmt.println( )
	fmt.printfln( "    Volume: %d%%", int( volume * 100 ) )

	// --- Main loop: read keypresses ---
	if raw_ok {

		// Raw mode: read single characters from /dev/tty
		tty, open_err := os.open( "/dev/tty", os.O_RDONLY )
		if open_err == nil {

			defer os.close( tty )
			buf : [ 1 ]u8
			for {

				n, _ := os.read( tty, buf[ : ] )
				if n > 0 {

					c := buf[ 0 ]
					switch c {

					case '+', '=':
						volume += 0.1
						if volume > 4.0 { volume = 4.0 }
						fmt.printf( "\r    Volume: %d%%   ", int( volume * 100 ) )

					case '-':
						volume -= 0.1
						if volume < 0.0 { volume = 0.0 }
						fmt.printf( "\r    Volume: %d%%   ", int( volume * 100 ) )
					case '\n', '\r':
						break

					}
				} else {

					time.sleep( time.Millisecond * 10 )
				}
			}
		} else {

			// Fallback: wait for Enter on stdin
			buf: [ 1 ]u8
			for {

				n, _ := os.read( os.stdin, buf[ : ] )
				if n > 0 { break }
				time.sleep( time.Millisecond * 10 )
			}
		}
	} else {

		// Fallback: wait for Enter on stdin
		buf : [ 1 ]u8
		for {

			n, _ := os.read( os.stdin, buf[ : ] )
			if n > 0 { break }
			time.sleep( time.Millisecond * 10 )
			// Check if volume was changed from another path (unlikely in fallback)
		}
	}
}
