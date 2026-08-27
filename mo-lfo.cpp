/**
 * @file mo-lfo.cpp
 * @brief Low Frequency Oscillator implementation with fixed-point math and SRAM optimization.
 * @author mo-thunderz
 * @author felipegaspari (modified & optimized)
 * @version 2.0 (RP2350 / RP2040 DSP Optimized)
 */

 #include "Arduino.h"
 #include "mo-lfo.h"
 #include <math.h>
 
 // -----------------------------------------------
 // Internal helpers (file-local)
 // -----------------------------------------------
 
 static uint32_t MO_LFO_HOT(lfo_compute_phase_inc_from_freq)(float freq_hz)
 {
     if (freq_hz <= 0.0f) return 0;
     // OPTIMIZATION: Removed 'double' precision literal trap. 
     // Forces RP2040/RP2350 to use single-precision FPU math.
     const float scale = 4294.967296f; 
     float v = freq_hz * scale;
     if (v < 0.0f) v = 0.0f;
     if (v > 4294967295.0f) v = 4294967295.0f;
     return (uint32_t)(v + 0.5f);
 }
 
 static int16_t s_sineTable[LFO_SINE_TABLE_SIZE];
 static bool s_sineTableInitialized = false;
 
 static void lfo_initSineTable()
 {
     if (s_sineTableInitialized) return;
     const float inv_size = 1.0f / (float)LFO_SINE_TABLE_SIZE;
     for (int i = 0; i < (int)LFO_SINE_TABLE_SIZE; ++i) {
         float phase = (float)i * inv_size;
         float sign = 1.0f;
         if (phase >= 0.5f) { phase -= 0.5f; sign = -1.0f; }
         float w = phase * 2.0f;
         float x = w * (1.0f - w);
         float s = (16.0f * x) / (5.0f - 4.0f * x);
         float val = sign * s * 32767.0f;
         s_sineTable[i] = (int16_t)(val + (val >= 0.0f ? 0.5f : -0.5f));
     }
     s_sineTableInitialized = true;
 }
 
 MO_LFO_ALWAYS_INLINE int32_t MO_LFO_HOT(lfo_sine_q15_from_ramp16)(uint16_t ramp16)
 {
     const uint16_t idx  = (uint16_t)(ramp16 >> LFO_SINE_FRAC_BITS);
     const uint16_t frac = (uint16_t)(ramp16 & ((1u << LFO_SINE_FRAC_BITS) - 1u));
     const int16_t y0 = s_sineTable[idx];
     const int16_t y1 = s_sineTable[(idx + 1u) & (LFO_SINE_TABLE_SIZE - 1u)];
     return (int32_t)y0 + ((((int32_t)y1 - (int32_t)y0) * (int32_t)frac) >> LFO_SINE_FRAC_BITS);
 }
 
 MO_LFO_ALWAYS_INLINE int16_t MO_LFO_HOT(lfo_clamp_q15)(int32_t v)
 {
 // OPTIMIZATION: 1-Cycle DSP Saturation. 
 // Cortex-M33 (RP2350) will compile this to exactly 1 instruction (SSAT).
 #if defined(__ARM_FEATURE_SAT) || defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_7EM__)
     return (int16_t)__builtin_arm_ssat(v, 16);
 #else
     // RP2040 branchless fallback
     return v < -32767 ? -32767 : (v > 32767 ? 32767 : (int16_t)v);
 #endif
 }
 
 MO_LFO_ALWAYS_INLINE int32_t MO_LFO_HOT(lfo_unit_q15_from_ramp)(uint16_t ramp16, int waveForm)
 {
     // OPTIMIZATION: Refactored wave generation to use branchless Conditional Moves (IT blocks)
     switch (waveForm)
     {
         case 1: // Saw
             return (int32_t)ramp16 - ((ramp16 & 0x8000u) ? 65536 : 0);
 
         case 2: case 10: case 11: case 12: 
             return 0; // Handled directly in _advanceUnitQ15()
 
         case 3: // Sine
             return lfo_sine_q15_from_ramp16(ramp16);
             
         case 4: // Square
             return (ramp16 & 0x8000u) ? -32767 : 32767;
                                       
         case 5: // Sharktooth
         {
             const uint32_t is_fall = (ramp16 >= 58982u);
             const int32_t rise_val = (int32_t)((ramp16 * 72818u) >> 16);
             const int32_t fall_val = (int32_t)(((65535u - ramp16) * 655360u) >> 16);
             return (is_fall ? fall_val : rise_val) - 32767;
         }
         
         case 6: // Trapezoid
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             const int32_t trap = ((int32_t)tri16 * 6) - 98304;
             return lfo_clamp_q15(trap);
         }
 
         case 7: // Linear Tri
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)tri16 * 2) - 32768;
         }
         
         case 8: // Staircase
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)(tri16 >> 12) * 9362) - 32767;
         }
         
         case 9: // Folded Sine
         {
             const int32_t sine = lfo_sine_q15_from_ramp16(ramp16);
             int32_t folded = (sine * 3) >> 1;
             return folded > 32767 ? 65534 - folded : (folded < -32767 ? -65534 - folded : folded);
         }
         default: return 0;
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
 
     _ana_phase_drift  = 0x12345678;
     _ana_phase_wobble = 0x87654321;
     _ana_phase_mod2   = 0xABCDEF01;
     _ana_phase_mod3   = 0x10203040;
     _ana_phase_bias   = 0x98765432;
     
     _ana_counter = 0;
     _ana_drift_val  = 0.0f;
     _ana_wobble_val = _ana_profile.wobble_base;
     _ana_mod2_val   = _ana_profile.mod2_base;
     _ana_mod3_val   = _ana_profile.mod3_base;
     _ana_bias_val   = _ana_profile.bias_base;

     _ana_mod2_q15 = 0;
     _ana_mod3_q15 = 0;
     _ana_bias_q15 = 0;
     _ana_wobble_drive_float_scalar = 0.0f;
 
     lfo_initSineTable();
     _updatePhaseIncFree();
     _updatePhaseIncSync();
     _updateAnalogIncrements();
 }
 
 void MO_LFO_HOT(lfo::setAnalogProfile)(const lfo_analog_profile_t& profile) {
     _ana_profile = profile;
     _updateAnalogIncrements();
 }
 
 void MO_LFO_HOT(lfo::setAnalogPreset)(LfoAnalogPreset preset) {
     lfo_analog_profile_t p;
     switch (preset) {
         case LfoAnalogPreset::TapeWarble:
             p.drift_freq_hz = 0.08f; p.drift_depth = 0.085f;
             p.wobble_freq_hz = 0.15f; p.wobble_base = 0.90f; p.wobble_depth = 0.10f;
             p.mod2_ratio = 0.04f; p.mod2_base = 0.05f; p.mod2_depth = 0.2f;   
             p.mod3_ratio = 0.12f; p.mod3_base = 0.02f; p.mod3_depth = 0.01f;
             p.bias_ratio = 0.02f; p.bias_base = 0.1f; p.bias_depth = 0.2f;
             p.drive = 1.00f; p.makeup_gain = 1.05f;
             break;
         case LfoAnalogPreset::ClassATube:
             p.drift_freq_hz = 0.10f; p.drift_depth = 0.005f;
             p.wobble_freq_hz = 0.10f; p.wobble_base = 0.95f; p.wobble_depth = 0.05f;
             p.mod2_ratio = 0.25f; p.mod2_base = 0.15f; p.mod2_depth = 0.05f;   
             p.mod3_ratio = 0.05f; p.mod3_base = 0.01f; p.mod3_depth = 0.01f;
             p.bias_ratio = 0.12f; p.bias_base = 0.08f; p.bias_depth = 0.05f;
             p.drive = 0.95f; p.makeup_gain = 1.02f;   
             break;
         case LfoAnalogPreset::BrokenVintage:
             p.drift_freq_hz = 1.20f; p.drift_depth = 0.06f;
             p.wobble_freq_hz = 0.80f; p.wobble_base = 0.85f; p.wobble_depth = 0.15f;
             p.mod2_ratio = 0.43f; p.mod2_base = 0.12f; p.mod2_depth = 0.08f;
             p.mod3_ratio = 0.31f; p.mod3_base = 0.05f; p.mod3_depth = 0.03f;
             p.bias_ratio = 0.27f; p.bias_base = 0.05f; p.bias_depth = 0.08f;
             p.drive = 0.80f; p.makeup_gain = 1.05f;   
             break;
         case LfoAnalogPreset::SubtleWarmth:
         default:
             break;
     }
     setAnalogProfile(p);
 }
 
 void MO_LFO_HOT(lfo::_updateAnalogIncrements)() {
     _ana_inc_drift  = lfo_compute_phase_inc_from_freq(_ana_profile.drift_freq_hz);
     _ana_inc_wobble = lfo_compute_phase_inc_from_freq(_ana_profile.wobble_freq_hz);
     
     // OPTIMIZATION: Avoid double casting
     float active_inc = (float)(_mode ? _phase_inc_sync : _phase_inc_free);
     _ana_inc_mod2 = (uint32_t)(active_inc * _ana_profile.mod2_ratio);
     _ana_inc_mod3 = (uint32_t)(active_inc * _ana_profile.mod3_ratio);
     _ana_inc_bias = (uint32_t)(active_inc * _ana_profile.bias_ratio);
 }
 
 void MO_LFO_HOT(lfo::setWaveForm)(int l_waveForm) {
     if (l_waveForm < 0) l_waveForm = 0;
     if (l_waveForm > 12) l_waveForm = 12;
     _waveForm = l_waveForm;
 
     switch (_waveForm) {
         case 2:  setAnalogPreset(LfoAnalogPreset::SubtleWarmth);  break;
         case 10: setAnalogPreset(LfoAnalogPreset::TapeWarble);    break;
         case 11: setAnalogPreset(LfoAnalogPreset::ClassATube);    break;
         case 12: setAnalogPreset(LfoAnalogPreset::BrokenVintage); break;
         default: break;
     }
 }
 
 void MO_LFO_HOT(lfo::setAmpl)(int l_ampl) {
     if (l_ampl < 0) l_ampl = 0;
     if (l_ampl >= _dacSize) l_ampl = _dacSize - 1;
     _ampl = l_ampl;
     _updateAmplQ15FromDac();
 }
 
 void MO_LFO_HOT(lfo::setAmplQ15)(int16_t l_ampl_q15) {
     if (l_ampl_q15 < 0) l_ampl_q15 = 0;
     if (l_ampl_q15 > MO_LFO_Q15_ONE) l_ampl_q15 = MO_LFO_Q15_ONE;
     _ampl_q15 = l_ampl_q15;
     const int ampl_max = _dacSize - 1;
     if (ampl_max > 0) _ampl = (int)(((int32_t)_ampl_q15 * ampl_max + (MO_LFO_Q15_ONE / 2)) / MO_LFO_Q15_ONE);
     else _ampl = 0;
 }
 
 void MO_LFO_HOT(lfo::_updateAmplQ15FromDac)() {
     const int ampl_max = _dacSize - 1;
     if (ampl_max <= 0) { _ampl_q15 = 0; return; }
     if (_ampl >= ampl_max) { _ampl_q15 = MO_LFO_Q15_ONE; return; }
     _ampl_q15 = (int16_t)(((int32_t)_ampl * (int32_t)MO_LFO_Q15_ONE) / ampl_max);
 }
 
 void MO_LFO_HOT(lfo::setAmplOffset)(int l_ampl_offset) {
     if (l_ampl_offset < 0) l_ampl_offset = 0;
     if (l_ampl_offset >= _dacSize) l_ampl_offset = _dacSize - 1;
     _ampl_offset = l_ampl_offset;
 }
 
 void MO_LFO_HOT(lfo::setMode)(bool l_mode) { 
     _mode = l_mode; 
     _updateAnalogIncrements();
 }
 
 void MO_LFO_HOT(lfo::setMode0Freq)(float l_mode0_freq) {
     if (l_mode0_freq < 0) l_mode0_freq = 0;
     _mode0_freq = l_mode0_freq;
     _updatePhaseIncFree();
 }
 
 void MO_LFO_HOT(lfo::setMode0Freq)(float l_mode0_freq, unsigned long l_t) {
     (void)l_t; setMode0Freq(l_mode0_freq);
 }
 
 void lfo::setMode1Bpm(float l_mode1_bpm) {
     if (l_mode1_bpm < 0) l_mode1_bpm = 0;
     _mode1_bpm = l_mode1_bpm;
     _updatePhaseIncSync();
 }
 
 void MO_LFO_HOT(lfo::setMode1Rate)(float l_mode1_rate) {
     if (l_mode1_rate < 0) l_mode1_rate = 0;
     _mode1_rate = l_mode1_rate;
     _updatePhaseIncSync();
 }
 
 void MO_LFO_HOT(lfo::setMode1Phase)(float l_mode1_phase_offset) { (void)l_mode1_phase_offset; }
 void MO_LFO_HOT(lfo::sync)(unsigned long l_t) {
     _t_last = l_t;
     _initialized = true;
     _phase = 0;
 }
 
 int lfo::getWaveForm() { return _waveForm; }
 int lfo::getAmpl() { return _ampl; }
 int lfo::getAmplOffset() { return _ampl_offset; }
 bool lfo::getMode() { return _mode; }
 float lfo::getMode0Freq() { return _mode0_freq; }
 float lfo::getMode1Rate() { return _mode1_rate; }
 float lfo::getPhase() { return (float)_phase * (1.0f / 4294967296.0f); }
 
 int32_t MO_LFO_HOT(lfo::_advanceUnitQ15)(unsigned long l_t)
 {
     if (!_initialized) {
         _t_last = l_t;
         _initialized = true;
     }
 
     const uint32_t dt = (uint32_t)(l_t - _t_last);
     _t_last = l_t;
     uint32_t phase_inc = _mode ? _phase_inc_sync : _phase_inc_free;
 
     // =========================================================
     // 1. ANALOG SINE PATH (Waveforms 2, 10, 11, 12)
     // =========================================================
     if (_waveForm == 2 || (_waveForm >= 10 && _waveForm <= 12)) 
     {
         _ana_phase_drift  += _ana_inc_drift * dt;
         _ana_phase_wobble += _ana_inc_wobble * dt;
         _ana_phase_mod2   += _ana_inc_mod2 * dt;
         _ana_phase_mod3   += _ana_inc_mod3 * dt;
         _ana_phase_bias   += _ana_inc_bias * dt;
 
         const float to_float = 3.0518509476e-5f; // 1.0f / 32767.0f
 
         // Decimated modulation updates
         if ((_ana_counter & 0x1F) == 0) 
         {
             _ana_drift_val  = (float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_drift >> 16)) * to_float * _ana_profile.drift_depth;
             _ana_wobble_val = _ana_profile.wobble_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_wobble >> 16)) * to_float * _ana_profile.wobble_depth);
             _ana_mod2_val   = _ana_profile.mod2_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod2 >> 16)) * to_float * _ana_profile.mod2_depth);
             _ana_mod3_val   = _ana_profile.mod3_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod3 >> 16)) * to_float * _ana_profile.mod3_depth);
             _ana_bias_val   = _ana_profile.bias_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_bias >> 16)) * to_float * _ana_profile.bias_depth);
         }
         _ana_counter++;
 
         // Drift phase injection
         int32_t drift_offset = (int32_t)((float)phase_inc * _ana_drift_val);
         phase_inc = (uint32_t)((int32_t)phase_inc + drift_offset);
         _phase += phase_inc * dt;
 
 #if defined(PICO_RP2350) || defined(ARDUINO_ARCH_RP2350) || defined(__ARM_FP)
         // -----------------------------------------------------
         // RP2350 / FPU PATH: Pure 32-bit Hardware Float
         // -----------------------------------------------------
         float fund = (float)lfo_sine_q15_from_ramp16((uint16_t)(_phase >> 16)) * to_float;
         float h2   = (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 2) >> 16)) * to_float * _ana_mod2_val;
         float h3   = (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 3) >> 16)) * to_float * _ana_mod3_val;
 
         float mixed = (fund + h2 + h3 + _ana_bias_val) * _ana_wobble_val * _ana_profile.drive;
 
         if (mixed > 1.5f) mixed = 1.5f;
         else if (mixed < -1.5f) mixed = -1.5f;
 
         float sat = mixed * (1.0f - 0.1666667f * mixed * mixed) * _ana_profile.makeup_gain;
         int32_t out = (int32_t)(sat * 32767.0f);
         return lfo_clamp_q15(out);
 
 #else
         // -----------------------------------------------------
         // RP2040 PATH: Optimized Fixed-Point Q15 Integer Math
         // -----------------------------------------------------
         int32_t mod2_q15 = (int32_t)(_ana_mod2_val * 32767.0f);
         int32_t mod3_q15 = (int32_t)(_ana_mod3_val * 32767.0f);
         int32_t bias_q15 = (int32_t)(_ana_bias_val * 32767.0f);
 
         int32_t fund_q15 = lfo_sine_q15_from_ramp16((uint16_t)(_phase >> 16));
         int32_t h2_q15   = (lfo_sine_q15_from_ramp16((uint16_t)((_phase * 2) >> 16)) * mod2_q15) >> 15;
         int32_t h3_q15   = (lfo_sine_q15_from_ramp16((uint16_t)((_phase * 3) >> 16)) * mod3_q15) >> 15;
 
         int32_t mixed_q15 = fund_q15 + h2_q15 + h3_q15 + bias_q15;
         float mixed = (float)mixed_q15 * to_float * _ana_wobble_val * _ana_profile.drive;
 
         if (mixed > 1.5f) mixed = 1.5f;
         else if (mixed < -1.5f) mixed = -1.5f;
 
         float sat = mixed * (1.0f - 0.1666667f * mixed * mixed) * _ana_profile.makeup_gain;
         int32_t out = (int32_t)(sat * 32767.0f);
         return lfo_clamp_q15(out);
 #endif
     }
 
     // =========================================================
     // 2. STANDARD PATH (Waveforms 0, 1, 3..9)
     // =========================================================
     _phase += phase_inc * dt;
 
     if (_waveForm == 0) return 0;
     const uint16_t ramp16 = (uint16_t)(_phase >> 16);
     return lfo_unit_q15_from_ramp(ramp16, _waveForm);
 }
 
 int16_t MO_LFO_HOT(lfo::getWaveQ15)(unsigned long l_t)
 {
     if (_ampl_q15 == 0) { (void)_advanceUnitQ15(l_t); return 0; }
     const int32_t unit_q15 = _advanceUnitQ15(l_t);
     if (_waveForm == 0) return 0;
     if (_ampl_q15 == MO_LFO_Q15_ONE) return (int16_t)unit_q15;
     
     // OPTIMIZATION: Eliminated int64_t cast! 
     // Since max bounds are 32768 x 32768, the result fits beautifully inside int32_t.
     // This entirely bypasses __aeabi_lmul 64-bit library calls on the Cortex-M0+ (RP2040).
     const int32_t out = (unit_q15 * _ampl_q15) >> 15;
     return lfo_clamp_q15(out);
 }
 
 int MO_LFO_HOT(lfo::getWave)(unsigned long l_t)
 {
     const int l_ampl = _ampl;
     // OPTIMIZATION: Replaced '/ 2' with bitwise shift '>> 1'.
     const int l_ampl_half = l_ampl >> 1;
     int l_ampl_offset = 0;
     
     if (_ampl_offset < _dacSize / 2) l_ampl_offset = (_ampl_offset > l_ampl_half) ? _ampl_offset : l_ampl_half;
     else l_ampl_offset = (_dacSize - _ampl_offset > l_ampl_half) ? _ampl_offset : _dacSize - l_ampl_half - 1;
 
     if (_waveForm == 0) { (void)_advanceUnitQ15(l_t); return _ampl_offset; }
 
     const int32_t unit_q15 = _advanceUnitQ15(l_t);
     const int32_t scaled = ((int32_t)l_ampl_half * unit_q15) >> 15;
     return (int)scaled + l_ampl_offset;
 }
 
 void MO_LFO_HOT(lfo::_updatePhaseIncFree)() { 
     _phase_inc_free = lfo_compute_phase_inc_from_freq(_mode0_freq); 
     if (!_mode) _updateAnalogIncrements();
 }
 
 void MO_LFO_HOT(lfo::_updatePhaseIncSync)() {
     float freq_hz = 0.0f;
     if (_mode1_rate > 0.0f && _mode1_bpm > 0.0f) freq_hz = (_mode1_rate * _mode1_bpm) / 60.0f;
     _phase_inc_sync = lfo_compute_phase_inc_from_freq(freq_hz);
     if (_mode) _updateAnalogIncrements();
 }