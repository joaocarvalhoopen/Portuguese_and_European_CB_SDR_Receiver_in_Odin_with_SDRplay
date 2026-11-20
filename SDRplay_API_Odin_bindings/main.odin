// SDRplay API Odin bindings — main entry point
// Usage:
//   ./sdrplay_bindings_test            — run hardware tests
//   ./sdrplay_bindings_test -fm 97.4   — FM radio receiver at 97.4 MHz  Radio Comercial

package main

import "core:fmt"
import "core:os"
import "core:strconv"

import sdr_api "./sdrplay_api_bindings"


main :: proc() {

	args := os.args

	// Check for -fm <frequency> argument
	if len( args ) >= 3 && args[ 1 ] == "-fm" {

		freq, ok := strconv.parse_f64( args[ 2 ] )
		if !ok {

			fmt.eprintfln( "Invalid frequency: %s", args[ 2 ] )
			os.exit( 1 )
		}

		sdr_api.start_fm_receiver( freq )
		return
	}

	// Default: run hardware tests
	sdr_api.run_tests( )
}
