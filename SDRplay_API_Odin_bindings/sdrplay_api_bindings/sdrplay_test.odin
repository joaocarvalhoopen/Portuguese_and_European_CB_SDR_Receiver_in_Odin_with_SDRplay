// SDRplay API Odin bindings — hardware integration tests
// Tests are designed to run against a real RSPdx-R2 device.
// They exercise every major API function and verify struct layouts
// by reading/writing device parameters and streaming real samples.
//
// SDRplay API flow: Open → GetDevices → SelectDevice → GetDeviceParams
// → set params → Init → (stream + Update) → Uninit → ReleaseDevice → Close

package sdrplay_api_bindings

import "core:fmt"
import "core:time"
import "core:os"

// ---------------------------------------------------------------------------
// Shared test state (accessed from callback thread)
// ---------------------------------------------------------------------------

TestState :: struct {
	totalSamples:   u64,
	numCallbacks:   u32,
	overloadEvents: u32,
	gainEvents:     u32,
	minI:           i16,
	maxI:           i16,
	minQ:           i16,
	maxQ:           i16,
	streamReady:    bool,
	deviceRemoved:  bool,
}

test_state: TestState
test_dev: DeviceT
test_params: ^DeviceParamsT
streaming_active: bool

// ---------------------------------------------------------------------------
// Callback implementations (C calling convention, called from API thread)
// ---------------------------------------------------------------------------

stream_callback :: proc "c" (
	xi:         [^]i16,
	xq:         [^]i16,
	params:     ^StreamCbParamsT,
	numSamples: u32,
	reset:      u32,
	cbContext:  rawptr,
) {
	s := (^TestState)(cbContext)
	s.totalSamples += u64(numSamples)
	s.numCallbacks += 1

	if !s.streamReady do s.streamReady = true

	// Track min/max of I and Q for signal presence verification
	for i in 0..<int(numSamples) {
		vi := xi[i]
		vq := xq[i]
		if vi < s.minI do s.minI = vi
		if vi > s.maxI do s.maxI = vi
		if vq < s.minQ do s.minQ = vq
		if vq > s.maxQ do s.maxQ = vq
	}
}

event_callback :: proc "c" (
	eventId:   EventT,
	tuner:     TunerSelectT,
	params:    ^EventParamsT,
	cbContext:  rawptr,
) {
	s := (^TestState)(cbContext)

	switch eventId {
	case .PowerOverloadChange:
		s.overloadEvents += 1
		// Acknowledge the overload message
		sdrplay_api_Update(test_dev.dev, test_dev.tuner,
			.Ctrl_OverloadMsgAck, .None)

	case .GainChange:
		s.gainEvents += 1

	case .DeviceRemoved:
		s.deviceRemoved = true

	case .RspDuoModeChange:
		// Not relevant for RSPdx-R2

	case .DeviceFailure:
		// Not expected in normal operation
	}
}

// ---------------------------------------------------------------------------
// Individual test functions
// ---------------------------------------------------------------------------

// Test 1: Open the API and verify version
test_open_and_version :: proc() -> bool {
	err := sdrplay_api_Open()
	if !check_err(err, "sdrplay_api_Open") do return false

	ver: f32
	err = sdrplay_api_ApiVersion(&ver)
	if !check_err(err, "sdrplay_api_ApiVersion") {
		sdrplay_api_Close()
		return false
	}

	fmt.printf("  API version: %.2f (expected %.2f)\n", ver, SDRPLAY_API_VERSION)
	if f64(ver) < 3.0 {
		fmt.eprintf("  ERROR: API version too old\n")
		sdrplay_api_Close()
		return false
	}

	return true
}

// Test 2: Enumerate devices and find the RSPdx-R2
test_enumerate_devices :: proc() -> bool {
	devices: [SDRPLAY_MAX_DEVICES]DeviceT
	numDevs: u32

	err := sdrplay_api_GetDevices(&devices[0], &numDevs, SDRPLAY_MAX_DEVICES)
	if !check_err(err, "sdrplay_api_GetDevices") do return false

	fmt.printf("  Found %d device(s)\n", numDevs)
	if numDevs == 0 {
		fmt.eprintf("  ERROR: No SDRplay devices found\n")
		return false
	}

	// Find the RSPdx-R2 (hwVer == 7)
	found := false
	for i in 0..<int(numDevs) {
		d := &devices[i]
		fmt.printf("  Device %d: %s (hwVer=%d, serial=%s, valid=%d)\n",
			i, hw_version_name(d.hwVer), d.hwVer, serial_string(d), d.valid)
		if d.hwVer == SDRPLAY_RSPdxR2_ID && d.valid != 0 {
			test_dev = devices[i]
			found = true
		}
	}

	if !found {
		// Fall back to any valid device
		for i in 0..<int(numDevs) {
			if devices[i].valid != 0 {
				test_dev = devices[i]
				found = true
				fmt.printf("  (Using first valid device: %s)\n", hw_version_name(test_dev.hwVer))
				break
			}
		}
	}

	if !found {
		fmt.eprintf("  ERROR: No valid RSPdx-R2 device found\n")
		return false
	}

	fmt.printf("  Selected device: %s (hwVer=%d)\n",
		hw_version_name(test_dev.hwVer), test_dev.hwVer)
	return true
}

// Test 3: Select the device and get device parameters
test_select_and_params :: proc() -> bool {
	err := sdrplay_api_SelectDevice(&test_dev)
	if !check_err(err, "sdrplay_api_SelectDevice") do return false

	err = sdrplay_api_DebugEnable(test_dev.dev, .Disable)
	if !check_err(err, "sdrplay_api_DebugEnable") do return false

	err = sdrplay_api_GetDeviceParams(test_dev.dev, &test_params)
	if !check_err(err, "sdrplay_api_GetDeviceParams") do return false

	if test_params == nil {
		fmt.eprintf("  ERROR: GetDeviceParams returned null\n")
		return false
	}

	if test_params.devParams == nil {
		fmt.eprintf("  ERROR: devParams is null\n")
		return false
	}
	if test_params.rxChannelA == nil {
		fmt.eprintf("  ERROR: rxChannelA is null\n")
		return false
	}

	fmt.printf("  devParams: %p, rxChannelA: %p, rxChannelB: %p\n",
		test_params.devParams, test_params.rxChannelA, test_params.rxChannelB)
	fmt.printf("  samplesPerPkt (initial): %d\n", test_params.devParams.samplesPerPkt)

	return true
}

// Test 4: Configure device parameters (set struct values before Init)
// Note: sdrplay_api_Update must NOT be called before Init.
// The initial parameters are set by modifying the structs directly,
// and they are applied when sdrplay_api_Init is called.
test_configure :: proc() -> bool {
	dp := test_params.devParams
	ch := test_params.rxChannelA

	// Device-level: sample rate = 2 MHz, ppm correction = 0
	dp.fsFreq.fsHz = 2_000_000.0
	dp.ppm = 0.0

	// RSPdx-specific device params: Antenna A, no bias-T, no notches, no HDR
	dp.rspDxParams.hdrEnable = 0
	dp.rspDxParams.biasTEnable = 0
	dp.rspDxParams.antennaSel = .RspDx_ANTENNA_A
	dp.rspDxParams.rfNotchEnable = 0
	dp.rspDxParams.rfDabNotchEnable = 0

	// Tuner params: 100 MHz (FM broadcast), zero-IF, 200 kHz BW, auto LO
	ch.tunerParams.rfFreq.rfHz = 100_000_000.0
	ch.tunerParams.bwType = .BW_0_200
	ch.tunerParams.ifType = .IF_Zero
	ch.tunerParams.loMode = .LO_Auto

	// Gain: 50 dB gain reduction, LNA state 0, normal min GR
	ch.tunerParams.gain.gRdB = 50
	ch.tunerParams.gain.LNAstate = 0
	ch.tunerParams.gain.minGr = .NORMAL_MIN_GR

	// DC offset tuner calibration
	ch.tunerParams.dcOffsetTuner.dcCal = 3
	ch.tunerParams.dcOffsetTuner.speedUp = 0
	ch.tunerParams.dcOffsetTuner.trackTime = 1
	ch.tunerParams.dcOffsetTuner.refreshRateTime = 2048

	// Control params: DC offset on, IQ balance on, AGC disabled, no decimation
	ch.ctrlParams.dcOffset.DCenable = 1
	ch.ctrlParams.dcOffset.IQenable = 1
	ch.ctrlParams.decimation.enable = 0
	ch.ctrlParams.decimation.decimationFactor = 1
	ch.ctrlParams.decimation.wideBandSignal = 0
	ch.ctrlParams.agc.enable = .AGC_DISABLE
	ch.ctrlParams.agc.setPoint_dBfs = -30
	ch.ctrlParams.adsbMode = .DECIMATION

	// RSPdx tuner params: HDR bandwidth = 1.7 MHz (default)
	ch.rspDxTunerParams.hdrBw = .HDRMODE_BW_1_700

	// Verify struct fields were written correctly (struct layout test)
	ok := true
	if dp.fsFreq.fsHz != 2_000_000.0 { fmt.eprintf("  ERROR: fsHz mismatch\n"); ok = false }
	if ch.tunerParams.rfFreq.rfHz != 100_000_000.0 { fmt.eprintf("  ERROR: rfHz mismatch\n"); ok = false }
	if ch.tunerParams.gain.gRdB != 50 { fmt.eprintf("  ERROR: gRdB mismatch\n"); ok = false }
	if dp.rspDxParams.antennaSel != .RspDx_ANTENNA_A { fmt.eprintf("  ERROR: antennaSel mismatch\n"); ok = false }
	if ch.ctrlParams.agc.enable != .AGC_DISABLE { fmt.eprintf("  ERROR: agc mismatch\n"); ok = false }
	if ch.tunerParams.bwType != .BW_0_200 { fmt.eprintf("  ERROR: bwType mismatch\n"); ok = false }
	if ch.tunerParams.ifType != .IF_Zero { fmt.eprintf("  ERROR: ifType mismatch\n"); ok = false }
	if !ok do return false

	fmt.printf("  Configured: fs=2MHz, rf=100MHz, bw=200kHz, IF=Zero, gain=50dB, LNA=0, AGC=off\n")
	fmt.printf("  Antenna=A, BiasT=off, RFNotch=off, DABNotch=off, HDR=off\n")
	fmt.printf("  Struct field verification passed\n")
	return true
}

// Test 5: Init streaming and receive samples
// Calls sdrplay_api_Init with callbacks, verifies samples arrive,
// then keeps streaming active for runtime update tests.
test_streaming :: proc() -> bool {
	// Reset test state
	test_state = TestState{
		minI = 32767,
		maxI = -32768,
		minQ = 32767,
		maxQ = -32768,
	}

	// Set up callbacks
	cbFns := CallbackFnsT{
		StreamACbFn = stream_callback,
		StreamBCbFn = stream_callback,
		EventCbFn   = event_callback,
	}

	// Init the device (starts streaming with the configured parameters)
	err := sdrplay_api_Init(test_dev.dev, &cbFns, &test_state)
	if !check_err(err, "sdrplay_api_Init") do return false

	streaming_active = true

	fmt.printf("  Streaming initialized, waiting for samples...\n")

	// Wait for the first callback to fire
	waited_ms: u32 = 0
	for !test_state.streamReady && waited_ms < 5000 {
		time.sleep(time.Millisecond * 10)
		waited_ms += 10
	}

	if !test_state.streamReady {
		fmt.eprintf("  ERROR: No stream callback received within 5 seconds\n")
		sdrplay_api_Uninit(test_dev.dev)
		streaming_active = false
		return false
	}

	fmt.printf("  First callback received after %d ms\n", waited_ms)

	// Collect samples for 2 seconds
	time.sleep(time.Second * 2)

	// Print interim statistics
	fmt.printf("  Samples so far: %d (callbacks: %d)\n",
		test_state.totalSamples, test_state.numCallbacks)
	fmt.printf("  I range: [%d, %d], Q range: [%d, %d]\n",
		test_state.minI, test_state.maxI, test_state.minQ, test_state.maxQ)

	// Verify we got a reasonable number of samples
	// At 2 MHz sample rate, 2 seconds = ~4M samples
	if test_state.totalSamples < 500_000 {
		fmt.eprintf("  ERROR: Too few samples (%d, expected >500K)\n", test_state.totalSamples)
		return false
	}

	// Verify signal is not all zeros
	if test_state.maxI == 0 && test_state.minI == 0 &&
		test_state.maxQ == 0 && test_state.minQ == 0 {
		fmt.eprintf("  ERROR: All-zero samples received\n")
		return false
	}

	fmt.printf("  Signal presence verified (non-zero I/Q samples)\n")
	return true
}

// Test 6: Runtime parameter updates (change freq, gain, antenna during streaming)
// These call sdrplay_api_Update while the device is actively streaming.
test_runtime_update :: proc() -> bool {
	if !streaming_active {
		fmt.eprintf("  ERROR: Streaming not active\n")
		return false
	}

	ch := test_params.rxChannelA
	dp := test_params.devParams
	all_ok := true

	// Change frequency to 98.5 MHz
	ch.tunerParams.rfFreq.rfHz = 98_500_000.0
	err := sdrplay_api_Update(test_dev.dev, test_dev.tuner, .Tuner_Frf, .None)
	if !check_err(err, "Update(Tuner_Frf)") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change gain to 40 dB
	ch.tunerParams.gain.gRdB = 40
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .Tuner_Gr, .None)
	if !check_err(err, "Update(Tuner_Gr)") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change gain back to 50 dB
	ch.tunerParams.gain.gRdB = 50
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .Tuner_Gr, .None)
	if !check_err(err, "Update(Tuner_Gr) revert") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change antenna to B (RSPdx has 3 antennas)
	dp.rspDxParams.antennaSel = .RspDx_ANTENNA_B
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .None, .RspDx_AntennaControl)
	if !check_err(err, "Update(RspDx_AntennaControl)") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change back to Antenna A
	dp.rspDxParams.antennaSel = .RspDx_ANTENNA_A
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .None, .RspDx_AntennaControl)
	if !check_err(err, "Update(RspDx_AntennaControl) revert") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change bandwidth to 600 kHz
	ch.tunerParams.bwType = .BW_0_600
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .Tuner_BwType, .None)
	if !check_err(err, "Update(Tuner_BwType)") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	// Change bandwidth back to 200 kHz
	ch.tunerParams.bwType = .BW_0_200
	err = sdrplay_api_Update(test_dev.dev, test_dev.tuner, .Tuner_BwType, .None)
	if !check_err(err, "Update(Tuner_BwType) revert") { all_ok = false }
	time.sleep(time.Millisecond * 200)

	if all_ok {
		fmt.printf("  Updated: freq 100->98.5MHz, gain 50<->40, antenna A<->B<->A, bw 200<->600kHz\n")
	}
	return all_ok
}

// Test 7: Test error string function
test_error_strings :: proc() -> bool {
	s_success := error_string(.Success)
	s_fail := error_string(.Fail)
	s_invalid := error_string(.InvalidParam)

	if len(s_success) == 0 || len(s_fail) == 0 || len(s_invalid) == 0 {
		fmt.eprintf("  ERROR: Error strings are empty\n")
		return false
	}

	fmt.printf("  Success -> '%s'\n", s_success)
	fmt.printf("  Fail    -> '%s'\n", s_fail)
	fmt.printf("  Invalid -> '%s'\n", s_invalid)
	return true
}

// Test 8: Test GetLastError function
test_last_error :: proc() -> bool {
	errInfo := sdrplay_api_GetLastError(&test_dev)
	if errInfo == nil {
		fmt.printf("  GetLastError: no error stored (OK)\n")
		return true
	}

	// Read the error info fields (null-terminated strings)
	file_len := 0
	for c in errInfo.file { if c == 0 do break; file_len += 1 }
	func_len := 0
	for c in errInfo.function { if c == 0 do break; func_len += 1 }
	msg_len := 0
	for c in errInfo.message { if c == 0 do break; msg_len += 1 }

	fmt.printf("  GetLastError: file='%s', function='%s', line=%d, message='%s'\n",
		string(errInfo.file[:file_len]),
		string(errInfo.function[:func_len]),
		errInfo.line,
		string(errInfo.message[:msg_len]))
	return true
}

// Test 9: Test lock/unlock device API
test_lock_unlock :: proc() -> bool {
	err := sdrplay_api_LockDeviceApi()
	if !check_err(err, "sdrplay_api_LockDeviceApi") do return false

	err = sdrplay_api_UnlockDeviceApi()
	if !check_err(err, "sdrplay_api_UnlockDeviceApi") do return false

	fmt.printf("  Lock/Unlock API OK\n")
	return true
}

// Test 10: Uninit streaming, release device, and close API
test_cleanup :: proc() -> bool {
	all_ok := true

	// Uninit (stops streaming) — must be done before ReleaseDevice
	if streaming_active {
		err := sdrplay_api_Uninit(test_dev.dev)
		if !check_err(err, "sdrplay_api_Uninit") {
			if err == .StopPending {
				fmt.printf("  (StopPending — retrying in 1s)\n")
				time.sleep(time.Second)
				err = sdrplay_api_Uninit(test_dev.dev)
				if !check_err(err, "sdrplay_api_Uninit (retry)") {
					all_ok = false
				}
			} else {
				all_ok = false
			}
		}
		streaming_active = false

		// Print final streaming statistics
		fmt.printf("  Total samples: %d, callbacks: %d, overloads: %d, gain events: %d\n",
			test_state.totalSamples, test_state.numCallbacks,
			test_state.overloadEvents, test_state.gainEvents)
	}

	// Release device
	err := sdrplay_api_ReleaseDevice(&test_dev)
	if !check_err(err, "sdrplay_api_ReleaseDevice") { all_ok = false }

	// Close API
	err = sdrplay_api_Close()
	if !check_err(err, "sdrplay_api_Close") { all_ok = false }

	if all_ok {
		fmt.printf("  Streaming stopped, device released, API closed\n")
	}
	return all_ok
}

// ---------------------------------------------------------------------------
// Test runner — single entry point called from main.odin
// ---------------------------------------------------------------------------

run_tests :: proc() {
	fmt.println("============================================")
	fmt.println("  SDRplay API Odin Bindings — Hardware Test")
	fmt.println("============================================")

	all_passed := true
	tests_passed: u32 = 0
	tests_total: u32 = 0
	api_opened := false
	device_selected := false

	// --- Test 1: Open API & version ---
	fmt.println("\n[Test 1] Open API and check version")
	tests_total += 1
	if test_open_and_version() {
		fmt.println("  -> PASS")
		tests_passed += 1
		api_opened = true
	} else {
		fmt.println("  -> FAIL")
		all_passed = false
	}

	// --- Test 2: Enumerate devices ---
	if api_opened {
		fmt.println("\n[Test 2] Enumerate devices and find RSPdx-R2")
		tests_total += 1
		if test_enumerate_devices() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Test 3: Select device & get params ---
	if all_passed {
		fmt.println("\n[Test 3] Select device and get device parameters")
		tests_total += 1
		if test_select_and_params() {
			fmt.println("  -> PASS")
			tests_passed += 1
			device_selected = true
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Test 4: Configure device (set params before Init) ---
	if all_passed {
		fmt.println("\n[Test 4] Configure device parameters (pre-Init)")
		tests_total += 1
		if test_configure() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Test 5: Init streaming and receive samples ---
	if all_passed {
		fmt.println("\n[Test 5] Init streaming and receive samples (2s)")
		tests_total += 1
		if test_streaming() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Test 6: Runtime parameter updates (during streaming) ---
	if all_passed && streaming_active {
		fmt.println("\n[Test 6] Runtime parameter updates (freq, gain, antenna, bw)")
		tests_total += 1
		if test_runtime_update() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Tests 7-9: run while device is still active ---
	if all_passed {
		// Test 7: Error strings
		fmt.println("\n[Test 7] Error string function")
		tests_total += 1
		if test_error_strings() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}

		// Test 8: GetLastError
		fmt.println("\n[Test 8] GetLastError function")
		tests_total += 1
		if test_last_error() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}

		// Test 9: Lock/Unlock API
		fmt.println("\n[Test 9] Lock/Unlock Device API")
		tests_total += 1
		if test_lock_unlock() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Test 10: Cleanup (Uninit + Release + Close) ---
	if device_selected {
		fmt.println("\n[Test 10] Uninit, release device, and close API")
		tests_total += 1
		if test_cleanup() {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	} else if api_opened {
		fmt.println("\n[Test 10] Close API (device not selected)")
		tests_total += 1
		if check_err(sdrplay_api_Close(), "sdrplay_api_Close") {
			fmt.println("  -> PASS")
			tests_passed += 1
		} else {
			fmt.println("  -> FAIL")
			all_passed = false
		}
	}

	// --- Summary ---
	fmt.println("\n============================================")
	fmt.printf("  Results: %d/%d tests passed\n", tests_passed, tests_total)
	if all_passed {
		fmt.println("  *** ALL TESTS PASSED ***")
	} else {
		fmt.println("  *** SOME TESTS FAILED ***")
	}
	fmt.println("============================================")

	if !all_passed {
		os.exit(1)
	}
}
