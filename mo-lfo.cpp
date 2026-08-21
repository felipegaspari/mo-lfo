/**
 * @file mo-lfo.cpp
 * @brief Low Frequency Oscillator implementation with fixed-point math and SRAM optimization.
 * @author mo-thunderz
 * @author felipegaspari (modified & optimized)
 * @version 1.5
 */

 #include "Arduino.h"
 #include "mo-lfo.h"
 #include <math.h>
 
 // -----------------------------------------------
 // Internal helpers (file-local)
 // -----------------------------------------------
 
 /**
  * @brief Computes 32-bit phase increment per microsecond given a frequency in Hz.
  * @param freq_hz Frequency in Hertz.
  * @return uint32_t Phase increment for 32-bit accumulator.
  */
 static uint32_t lfo_compute_phase_inc_from_freq(float freq_hz)
 {
     if (freq_hz <= 0.0f)
         return 0;
 
     const double scale = 4294.967296; // 2^32 / 1e6
     double v = (double)freq_hz * scale;
     if (v < 0.0)
         v = 0.0;
     if (v > 4294967295.0)
         v = 4294967295.0;
 
     return (uint32_t)(v + 0.5);
 }
 
 /** @brief Sine lookup table storage in RAM (.bss). */
 static int16_t s_sineTable[LFO_SINE_TABLE_SIZE];
 static bool s_sineTableInitialized = false;
 
 /**
  * @brief Generates sine lookup table samples using standard math library on first boot.
  */
 static void lfo_initSineTable()
 {
     if (s_sineTableInitialized)
         return;
 
     for (int i = 0; i < (int)LFO_SINE_TABLE_SIZE; ++i)
     {
         double angle = (2.0 * 3.14159265358979323846 * (double)i) / (double)LFO_SINE_TABLE_SIZE;
         s_sineTable[i] = (int16_t)lrint(sin(angle) * 32767.0);
     }
 
     s_sineTableInitialized = true;
 }
 
 /**
  * @brief Linearly interpolates Q15 sine wave from 16-bit phase ramp.
  * @param ramp16 Upper 16 bits of the phase accumulator.
  * @return int32_t Sine value in Q15 range [-32767, +32767].
  */
 MO_LFO_ALWAYS_INLINE int32_t lfo_sine_q15_from_ramp16(uint16_t ramp16)
 {
     const uint16_t idx  = (uint16_t)(ramp16 >> LFO_SINE_FRAC_BITS);
     const uint16_t frac = (uint16_t)(ramp16 & ((1u << LFO_SINE_FRAC_BITS) - 1u));
     const int16_t y0 = s_sineTable[idx];
     const int16_t y1 = s_sineTable[(idx + 1u) & (LFO_SINE_TABLE_SIZE - 1u)];
     return (int32_t)y0 + ((((int32_t)y1 - (int32_t)y0) * (int32_t)frac) >> LFO_SINE_FRAC_BITS);
 }
 
 /**
  * @brief Branchless-friendly saturation clamp to valid Q15 boundaries.
  * @param v Input 32-bit sample.
  * @return int16_t Clamped value in [-32767, +32767].
  */
 MO_LFO_ALWAYS_INLINE int16_t lfo_clamp_q15(int32_t v)
 {
     return v < -(int32_t)MO_LFO_Q15_ONE ? -(int16_t)MO_LFO_Q15_ONE : (v > (int32_t)MO_LFO_Q15_ONE ? MO_LFO_Q15_ONE : (int16_t)v);
 }
 
/**
 * @brief Computes normalized full-scale Q15 unit output for all 10 waveform types.
 * @param ramp16 Upper 16 bits of phase accumulator (0..65535).
 * @param waveForm Waveform selector (0..9).
 * @return int32_t Normalized bipolar shape [-32767, +32767].
 */
 MO_LFO_ALWAYS_INLINE int32_t lfo_unit_q15_from_ramp(uint16_t ramp16, int waveForm)
 {
     switch (waveForm)
     {
         case 1: // Linear Saw: 0 → +peak → jump → -peak → 0
         {
             if (ramp16 < 0x8000u)
                 return (int32_t)ramp16;
             return (int32_t)ramp16 - 65536;
         }
 
         case 2: // Tri Slewed (Real Analog RC-Curved Triangle)
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
 
             // RISING HALF (r < 0x8000): Parabolic capacitor charge curve (shoots up fast, eases into peak)
             if (!(r & 0x8000u)) {
                 const uint32_t inv = 32767u - tri16;
                 const uint32_t curve = (inv * inv) >> 15; // 32767 -> 0
                 const int32_t val = 32767 - (int32_t)curve; // 0 -> 32767
                 return (val * 2) - 32768;
             }
             // FALLING HALF (r >= 0x8000): Parabolic capacitor discharge curve (drops fast, eases into trough)
             else {
                 const uint32_t curve = ((uint32_t)tri16 * (uint32_t)tri16) >> 15; // 32767 -> 0
                 return ((int32_t)curve * 2) - 32768;
             }
         }
 
         case 3: // Sine
             return lfo_sine_q15_from_ramp16(ramp16);
             
         case 4: // Linear Square (Instant edge)
             return (ramp16 & 0x8000u) ? -(int32_t)MO_LFO_Q15_ONE
                                       : (int32_t)MO_LFO_Q15_ONE;
                                       
         case 5: // Sharktooth (90% rise, 10% fall discharge)
         {
             if (ramp16 < 58982u) {
                 return (int32_t)((ramp16 * 72818u) >> 16) - 32767;
             } else {
                 const uint32_t inv = 65535u - (uint32_t)ramp16;
                 return (int32_t)((inv * 655360u) >> 16) - 32767;
             }
         }
         
         case 6: // Trapezoid (Pronounced Slewed Square: 6x Overdrive)
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             
             // 6x Overdrive: Holds solid flat rails with a crisp 10% transition ramp
             const int32_t trap = ((int32_t)tri16 * 6) - 98304;
             return trap < -32767 ? -32767 : (trap > 32767 ? 32767 : trap);
         }
 
         case 7: // Linear Tri (Pure sharp linear peaks)
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)tri16 * 2) - 32768;
         }
         
         case 8: // Staircase (Quantized Triangle, 8 levels)
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             const uint16_t step = tri16 >> 12; // 0..7 discrete steps
             return ((int32_t)step * 9362) - 32767;
         }
         
         case 9: // Folded Sine (Buchla-style overdriven fold)
         {
             const int32_t sine = lfo_sine_q15_from_ramp16(ramp16);
             int32_t folded = (sine * 3) >> 1; // +50% gain drive
             if (folded > 32767) folded = 65534 - folded;
             else if (folded < -32767) folded = -65534 - folded;
             return folded;
         }
         
         default:
             return 0;
     }
 }
 
 // -----------------------------------------------
 // lfo class implementation
 // -----------------------------------------------
 
 lfo::lfo(int dacSize)
 {
     _dacSize     = (dacSize > 1) ? dacSize : 2;
     _ampl        = _dacSize - 1;
     _ampl_offset = 0;
     _ampl_q15    = MO_LFO_Q15_ONE;
 
     lfo_initSineTable();
     _updatePhaseIncFree();
     _updatePhaseIncSync();
 }
 
 void lfo::setWaveForm(int l_waveForm)
 {
     if (l_waveForm < 0)
         l_waveForm = 0;
     if (l_waveForm > 9)
         l_waveForm = 9;
     _waveForm = l_waveForm;
 }
 
 void lfo::setAmpl(int l_ampl)
 {
     if (l_ampl < 0)
         l_ampl = 0;
     if (l_ampl >= _dacSize)
         l_ampl = _dacSize - 1;
     _ampl = l_ampl;
     _updateAmplQ15FromDac();
 }
 
 void lfo::setAmplQ15(int16_t l_ampl_q15)
 {
     if (l_ampl_q15 < 0)
         l_ampl_q15 = 0;
     if (l_ampl_q15 > MO_LFO_Q15_ONE)
         l_ampl_q15 = MO_LFO_Q15_ONE;
     _ampl_q15 = l_ampl_q15;
 
     const int ampl_max = _dacSize - 1;
     if (ampl_max > 0)
         _ampl = (int)(((int32_t)_ampl_q15 * ampl_max + (MO_LFO_Q15_ONE / 2)) / MO_LFO_Q15_ONE);
     else
         _ampl = 0;
 }
 
 void lfo::_updateAmplQ15FromDac()
 {
     const int ampl_max = _dacSize - 1;
     if (ampl_max <= 0)
     {
         _ampl_q15 = 0;
         return;
     }
     if (_ampl >= ampl_max)
     {
         _ampl_q15 = MO_LFO_Q15_ONE;
         return;
     }
     _ampl_q15 = (int16_t)(((int32_t)_ampl * (int32_t)MO_LFO_Q15_ONE) / ampl_max);
 }
 
 void lfo::setAmplOffset(int l_ampl_offset)
 {
     if (l_ampl_offset < 0)
         l_ampl_offset = 0;
     if (l_ampl_offset >= _dacSize)
         l_ampl_offset = _dacSize - 1;
     _ampl_offset = l_ampl_offset;
 }
 
 void lfo::setMode(bool l_mode)
 {
     _mode = l_mode;
 }
 
 void lfo::setMode0Freq(float l_mode0_freq)
 {
     if (l_mode0_freq < 0)
         l_mode0_freq = 0;
     _mode0_freq = l_mode0_freq;
     _updatePhaseIncFree();
 }
 
 void lfo::setMode0Freq(float l_mode0_freq, unsigned long l_t)
 {
     (void)l_t;
     if (l_mode0_freq < 0)
         l_mode0_freq = 0;
     _mode0_freq = l_mode0_freq;
     _updatePhaseIncFree();
 }
 
 void lfo::setMode1Bpm(float l_mode1_bpm)
 {
     if (l_mode1_bpm < 0)
         l_mode1_bpm = 0;
     _mode1_bpm = l_mode1_bpm;
     _updatePhaseIncSync();
 }
 
 void lfo::setMode1Rate(float l_mode1_rate)
 {
     if (l_mode1_rate < 0)
         l_mode1_rate = 0;
     _mode1_rate = l_mode1_rate;
     _updatePhaseIncSync();
 }
 
 void lfo::setMode1Phase(float l_mode1_phase_offset)
 {
     (void)l_mode1_phase_offset;
 }
 
 void lfo::sync(unsigned long l_t)
 {
     _t_last      = l_t;
     _initialized = true;
     _phase       = 0;
 }
 
 int lfo::getWaveForm() { return _waveForm; }
 int lfo::getAmpl() { return _ampl; }
 int lfo::getAmplOffset() { return _ampl_offset; }
 bool lfo::getMode() { return _mode; }
 float lfo::getMode0Freq() { return _mode0_freq; }
 float lfo::getMode1Rate() { return _mode1_rate; }
 
 float lfo::getPhase()
 {
     return (float)_phase * (1.0f / 4294967296.0f);
 }
 
 int32_t MO_LFO_HOT(lfo::_advanceUnitQ15)(unsigned long l_t)
 {
     if (!_initialized)
     {
         _t_last = l_t;
         _initialized = true;
     }
 
     const uint32_t dt = (uint32_t)(l_t - _t_last);
     _t_last = l_t;
 
     const uint32_t phase_inc = _mode ? _phase_inc_sync : _phase_inc_free;
     _phase += phase_inc * dt;
 
     if (_waveForm == 0)
         return 0;
 
     const uint16_t ramp16 = (uint16_t)(_phase >> 16);
     return lfo_unit_q15_from_ramp(ramp16, _waveForm);
 }
 
 int16_t MO_LFO_HOT(lfo::getWaveQ15)(unsigned long l_t)
 {
     if (_ampl_q15 == 0)
     {
         (void)_advanceUnitQ15(l_t); // Keep accumulator moving even at zero amplitude
         return 0;
     }
 
     const int32_t unit_q15 = _advanceUnitQ15(l_t);
     if (_waveForm == 0)
         return 0;
 
     if (_ampl_q15 == MO_LFO_Q15_ONE)
         return (int16_t)unit_q15;
 
     const int32_t out = (int32_t)(((int64_t)unit_q15 * (int64_t)_ampl_q15) >> 15);
     return lfo_clamp_q15(out);
 }
 
 int lfo::getWave(unsigned long l_t)
 {
     const int l_ampl = _ampl;
     const int l_ampl_half = (int)l_ampl / 2;
 
     int l_ampl_offset = 0;
     if (_ampl_offset < _dacSize / 2)
         l_ampl_offset = (_ampl_offset > l_ampl_half) ? _ampl_offset : l_ampl_half;
     else
         l_ampl_offset = (_dacSize - _ampl_offset > l_ampl_half) ? _ampl_offset
                                                                 : _dacSize - l_ampl_half - 1;
 
     if (_waveForm == 0)
     {
         (void)_advanceUnitQ15(l_t);
         return _ampl_offset;
     }
 
     const int32_t unit_q15 = _advanceUnitQ15(l_t);
     const int32_t scaled = ((int32_t)l_ampl_half * unit_q15) >> 15;
     return (int)scaled + l_ampl_offset;
 }
 
 void lfo::_updatePhaseIncFree()
 {
     _phase_inc_free = lfo_compute_phase_inc_from_freq(_mode0_freq);
 }
 
 void lfo::_updatePhaseIncSync()
 {
     float freq_hz = 0.0f;
     if (_mode1_rate > 0.0f && _mode1_bpm > 0.0f)
         freq_hz = (_mode1_rate * _mode1_bpm) / 60.0f;
 
     _phase_inc_sync = lfo_compute_phase_inc_from_freq(freq_hz);
 }