#ifndef SOUND_LUTS_H
#define SOUND_LUTS_H

#include <stdint.h>
#include <arm_math.h>

// NOTE: if changing this number, need to redo the bandID LUTs
#define SOUND_FREQ_BANDS 6 // using the 8 standard octave bands, excluding the 1st and last

/* The 3 objects in this file are:
 * sqWsc_FsX_Y[] = scaled, squared A-weightings (scaled to fit all in uint16s)
 * To reverse the scaling, multiply by scaleFactor, SF, which is provided as the value 10*log10(SF).
 * tenlog10SF_FsX_Y = the scaling factor for the above
 * bandIDs_FsX_Y[] = gives the octave band index number from 0 -> (SOUND_FREQ_BANDS-1)
 * 					 or a value of SOUND_FREQ_BANDS to indicate not in a band.
 * Where X = actual audio Fs in Hz and Y = FFT length (Po2)

// bandIDs array stores the octave freq band index from 0->(SOUND_FREQ_BANDS-1) for each FFT bin
// OR if the value "SOUND_FREQ_BANDS" is stored, this indicates that the FFT bin does not
// correspond to any freq band. Note that there are 8 standard bands (not dependent on Fs or FFT-N)
// but the first and last are rejected due to being too low/high in frequency.
// (note that Fs and FFT-N do dictate the low freq detection cutoff, e.g. Fs=16000, N=128, fc = 125 Hz)
 *  The bands are:
//  band_edges = [88.388    176.776    353.553    707.105   1414.212   2828.425   5656.852]
//   band_mids = [125.00    250.00    500.00   1000.00   2000.00   4000.00]
 */

extern const float32_t tenlog10SF_Fs31250_1024;
extern const int32_t tenlog10SF_int_Fs31250_1024;
extern const int32_t tenlog10SF_frac_Fs31250_1024;
extern const uint16_t sqWsc_Fs31250_1024[];
extern const uint8_t bandIDs_Fs31250_1024[];

extern const float32_t tenlog10SF_Fs31250_512;
extern const int32_t tenlog10SF_int_Fs31250_512;
extern const int32_t tenlog10SF_frac_Fs31250_512;
extern const uint16_t sqWsc_Fs31250_512[];
extern const uint8_t bandIDs_Fs31250_512[];

extern const float32_t tenlog10SF_Fs31250_256;
extern const int32_t tenlog10SF_int_Fs31250_256;
extern const int32_t tenlog10SF_frac_Fs31250_256;
extern const uint16_t sqWsc_Fs31250_256[];
extern const uint8_t bandIDs_Fs31250_256[];

extern const float32_t tenlog10SF_Fs31250_128;
extern const int32_t tenlog10SF_int_Fs31250_128;
extern const int32_t tenlog10SF_frac_Fs31250_128;
extern const uint16_t sqWsc_Fs31250_128[];
extern const uint8_t bandIDs_Fs31250_128[];

extern const float32_t tenlog10SF_Fs15625_128;
extern const int32_t tenlog10SF_int_Fs15625_128;
extern const int32_t tenlog10SF_frac_Fs15625_128;
extern const uint16_t sqWsc_Fs15625_128[];
extern const uint8_t bandIDs_Fs15625_128[];

extern const float32_t tenlog10SF_Fs15625_256;
extern const int32_t tenlog10SF_int_Fs15625_256;
extern const int32_t tenlog10SF_frac_Fs15625_256;
extern const uint16_t sqWsc_Fs15625_256[];
extern const uint8_t bandIDs_Fs15625_256[];

extern const float32_t tenlog10SF_Fs15625_512;
extern const int32_t tenlog10SF_int_Fs15625_512;
extern const int32_t tenlog10SF_frac_Fs15625_512;
extern const uint16_t sqWsc_Fs15625_512[];
extern const uint8_t bandIDs_Fs15625_512[];

extern const float32_t tenlog10SF_Fs50000_256;
extern const int32_t tenlog10SF_int_Fs50000_256;
extern const int32_t tenlog10SF_frac_Fs50000_256;
extern const uint16_t sqWsc_Fs50000_256[];
extern const uint8_t bandIDs_Fs50000_256[];

extern const float32_t tenlog10SF_Fs50000_512;
extern const int32_t tenlog10SF_int_Fs50000_512;
extern const int32_t tenlog10SF_frac_Fs50000_512;
extern const uint16_t sqWsc_Fs50000_512[];
extern const uint8_t bandIDs_Fs50000_512[];

#endif
