//----------------------------------//
// LFO class for Arduino
// by mo-thunderz
// modified by felipegaspari
// version 1.3
//----------------------------------//

#include "Arduino.h"
#include <stdint.h>

// -------------------------------------------------
// Configuration macros
// -------------------------------------------------
// Sine lookup table resolution: LFO_SINE_TABLE_BITS = log2(table_size)
#ifndef LFO_SINE_TABLE_BITS
#define LFO_SINE_TABLE_BITS 9
#endif

#define LFO_SINE_TABLE_SIZE (1u << LFO_SINE_TABLE_BITS)
#define LFO_SINE_FRAC_BITS  (16 - LFO_SINE_TABLE_BITS)

// Preferred-path hint (does NOT change the public API — both DAC and Q15
// methods are always available). Used for compile messages / future size strips.
//   0 = examples lean on getWave() (DAC int)
//   1 = synth engines lean on getWaveQ15() (bipolar)
#ifndef MO_LFO_USE_Q15
#define MO_LFO_USE_Q15 0
#endif

#ifndef MO_LFO_CONFIG_REPORTED
#define MO_LFO_CONFIG_REPORTED
#if MO_LFO_USE_Q15
#pragma message("MO-LFO: preferred path=Q15 (MO_LFO_USE_Q15=1); API always has getWave+getWaveQ15")
#else
#pragma message("MO-LFO: preferred path=DAC (MO_LFO_USE_Q15=0); API always has getWave+getWaveQ15")
#endif
#endif

#ifndef mo_lfo_h
#define mo_lfo_h

// Q15 full scale in int16_t (±32767 ≈ ±1.0)
static const int16_t MO_LFO_Q15_ONE = 32767;

class lfo
{
    public:
        // dacSize: vertical range for getWave() → [0, dacSize-1]
        lfo(int dacSize);

        // Amplitude: two input methods, one cached state. Last call wins.
        // setAmpl: DAC counts 0..dacSize-1 (also updates _ampl_q15)
        // setAmplQ15: 0..32767 (also mirrors _ampl for getAmpl)
        void setAmpl(int l_ampl);
        void setAmplQ15(int16_t l_ampl_q15);
        int16_t getAmplQ15() const { return _ampl_q15; }

        // DC offset for getWave() only (unipolar DAC). Ignored by getWaveQ15().
        void setAmplOffset(int l_ampl_offset);

        // 0=off, 1=saw, 2=triangle, 3=sin, 4=square.
        // Phase-aligned (sine-native: 0 at phase 0, rising).
        void setWaveForm(int l_waveForm);
        void setMode(bool l_freq_sync);
        void setMode0Freq(float l_mode0_freq);
        void setMode0Freq(float l_mode0_freq, unsigned long l_t);
        void setMode1Bpm(float l_mode1_bpm);
        void setMode1Rate(float l_mode1_rate);
        void setMode1Phase(float l_mode1_phase_offset);
        void sync(unsigned long l_t);

        int getWaveForm();
        int getAmpl();
        int getAmplOffset();
        bool getMode();
        float getMode0Freq();
        float getMode1Rate();
        float getPhase();

        // Shared engine, two outputs (fixed signatures — always available):
        int getWave(unsigned long l_t);              // unipolar [0, dacSize-1]
        int16_t getWaveQ15(unsigned long l_t);       // bipolar Q15 ±32767

        // (wave_q15 * depth_q24) >> 15
        static inline int32_t applyDepthQ24(int16_t wave_q15, int32_t depth_q24)
        {
            return (int32_t)(((int64_t)wave_q15 * (int64_t)depth_q24) >> 15);
        }

    private:
        int             _dacSize;
        int             _waveForm = 1;
        int             _ampl = 0;
        int             _ampl_offset = 0;
        int16_t         _ampl_q15 = MO_LFO_Q15_ONE;
        bool            _mode = 0;
        float           _mode0_freq = 30;
        float           _mode1_bpm = 120;
        float           _mode1_rate = 1;

        uint32_t        _phase = 0;
        uint32_t        _phase_inc_free = 0;
        uint32_t        _phase_inc_sync = 0;
        unsigned long   _t_last = 0;
        bool            _initialized = false;

        void            _updatePhaseIncFree();
        void            _updatePhaseIncSync();
        void            _updateAmplQ15FromDac();
        // Advance phase; return clamped unit shape in Q15 (0 if waveform off).
        int32_t         _advanceUnitQ15(unsigned long l_t);
};

#endif

// -----------------------------------------------------
// setMode1Rate table (mode 1 only):
//
//l_mode1_rate | lfo cycle duration
//---------------------------------
//         .125|   2 bars
//         .25 |   1 bar
//         .5  |   half note
//         1   |   quarter note
//         2   |   1/8 note
//         3   |   1/12 note
//         4   |   1/16 note
//         5   |   1/20 note
//         6   |   1/24 note
//         7   |   1/28 note
//         8   |   1/32 note
//         9   |   1/36 note
//        10   |   1/40 note
//        11   |   1/44 note
//        12   |   1/48 note
//        13   |   1/52 note
//        14   |   1/56 note
//        15   |   1/60 note
//        16   |   1/64 note
//
// Big THANKS to othmar52 for providing the table :-)
