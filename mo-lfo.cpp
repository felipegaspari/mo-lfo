/**
 * @file mo-lfo.cpp
 * @brief Dual-Engine LFO optimized for both RP2040 (Fixed-Point) and RP2350 (Hardware Float FMA)
 */

 #include "Arduino.h"
 #include "mo-lfo.h"
 #include <math.h>
 
 // -----------------------------------------------
 // Internal helpers (file-local)
 // -----------------------------------------------
 
 static uint32_t lfo_compute_phase_inc_from_freq(float freq_hz)
 {
     if (freq_hz <= 0.0f) return 0;
     const float scale = 4294.967296f; 
     float v = freq_hz * scale;
     if (v < 0.0f) v = 0.0f;
     if (v > 4294967295.0f) v = 4294967295.0f;
     return (uint32_t)(v + 0.5f);
 }
 
 // OPTIMIZATION: +1 Array padding for Zero-Cost wrap-around!
 static int16_t s_sineTable[LFO_SINE_TABLE_SIZE + 1];
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
     // Seamless boundary wrapping to eliminate bitwise AND in the interpolator
     s_sineTable[LFO_SINE_TABLE_SIZE] = s_sineTable[0];
     s_sineTableInitialized = true;
 }
 
 MO_LFO_ALWAYS_INLINE int32_t lfo_sine_q15_from_ramp16(uint16_t ramp16)
 {
     const uint16_t idx  = (uint16_t)(ramp16 >> LFO_SINE_FRAC_BITS);
     const uint16_t frac = (uint16_t)(ramp16 & ((1u << LFO_SINE_FRAC_BITS) - 1u));
     
     // OPTIMIZATION: Array padding allows reading idx + 1 directly without '&' mask
     const int32_t y0 = (int32_t)s_sineTable[idx];
     const int32_t y1 = (int32_t)s_sineTable[idx + 1u];
     
     // Encourages 1-cycle MLA (Multiply-Accumulate) on Cortex cores
     return y0 + (((y1 - y0) * (int32_t)frac) >> LFO_SINE_FRAC_BITS);
 }
 
 MO_LFO_ALWAYS_INLINE int16_t lfo_clamp_q15(int32_t v)
 {
 #if defined(__ARM_FEATURE_SAT) || defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_7EM__)
     return (int16_t)__builtin_arm_ssat(v, 16);
 #else
     return v < -32767 ? -32767 : (v > 32767 ? 32767 : (int16_t)v);
 #endif
 }
 
 MO_LFO_ALWAYS_INLINE int32_t lfo_unit_q15_from_ramp(uint16_t ramp16, int waveForm)
 {
     switch (waveForm) {
         case 1:  return (int32_t)ramp16 - ((ramp16 & 0x8000u) ? 65536 : 0);
         case 2: case 10: case 11: case 12: return 0;
         case 3:  return lfo_sine_q15_from_ramp16(ramp16);
         case 4:  return (ramp16 & 0x8000u) ? -32767 : 32767;
         case 5: {
             const uint32_t is_fall = (ramp16 >= 58982u);
             const int32_t rise_val = (int32_t)((ramp16 * 72818u) >> 16);
             const int32_t fall_val = (int32_t)(((65535u - ramp16) * 655360u) >> 16);
             return (is_fall ? fall_val : rise_val) - 32767;
         }
         case 6: {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             const int32_t trap = ((int32_t)tri16 * 6) - 98304;
             return lfo_clamp_q15(trap);
         }
         case 7: {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)tri16 * 2) - 32768;
         }
         case 8: {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)(tri16 >> 12) * 9362) - 32767;
         }
         case 9: {
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
     _ana_counter      = 0;
 
     lfo_initSineTable();
     _updatePhaseIncFree();
     _updatePhaseIncSync();
     _updateAnalogIncrements();
 }
 
 void lfo::setAnalogProfile(const lfo_analog_profile_t& profile) {
     _ana_profile = profile;
     _updateAnalogIncrements();
 }
 
 void lfo::setAnalogPreset(LfoAnalogPreset preset) {
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
     
     float active_inc = (float)(_mode ? _phase_inc_sync : _phase_inc_free);
     _ana_inc_mod2 = (uint32_t)(active_inc * _ana_profile.mod2_ratio);
     _ana_inc_mod3 = (uint32_t)(active_inc * _ana_profile.mod3_ratio);
     _ana_inc_bias = (uint32_t)(active_inc * _ana_profile.bias_ratio);
     
 #if !FLOAT_ENGINE
     _ana_drift_depth_q15 = (int32_t)(_ana_profile.drift_depth * 32768.0f);
 #endif
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
 
 void MO_LFO_HOT(lfo::setMode1Bpm)(float l_mode1_bpm) {
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
 
 int MO_LFO_HOT(lfo::getWaveForm)() { return _waveForm; }
 int MO_LFO_HOT(lfo::getAmpl)() { return _ampl; }
 int MO_LFO_HOT(lfo::getAmplOffset)() { return _ampl_offset; }
 bool MO_LFO_HOT(lfo::getMode)() { return _mode; }
 float MO_LFO_HOT(lfo::getMode0Freq)() { return _mode0_freq; }
 float MO_LFO_HOT(lfo::getMode1Rate)() { return _mode1_rate; }
 float MO_LFO_HOT(lfo::getPhase)() { return (float)_phase * (1.0f / 4294967296.0f); }
 
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
     // 1. ANALOG SINE MODELS (Waveforms 2, 10, 11, 12)
     // =========================================================
     if (_waveForm == 2 || (_waveForm >= 10 && _waveForm <= 12)) 
     {
         const lfo_analog_profile_t* __restrict prof = &_ana_profile;
 
         _ana_phase_drift  += _ana_inc_drift * dt;
         _ana_phase_wobble += _ana_inc_wobble * dt;
         _ana_phase_mod2   += _ana_inc_mod2 * dt;
         _ana_phase_mod3   += _ana_inc_mod3 * dt;
         _ana_phase_bias   += _ana_inc_bias * dt;
 
         // -----------------------------------------------------
         // Decimation Block (Runs once every 32 samples)
         // -----------------------------------------------------
         if ((_ana_counter & 0x1F) == 0) 
         {
             const float to_float = 3.0518509476e-5f; // 1.0f / 32767.0f
             
 #if FLOAT_ENGINE
             // OPTIMIZATION: RP2350 FMA (Fused Multiply-Add) Constant Pre-Baking
             _ana_drift_val_f = (float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_drift >> 16)) * to_float * prof->drift_depth;
             
             float wobble = prof->wobble_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_wobble >> 16)) * to_float * prof->wobble_depth);
             float mod2   = prof->mod2_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod2 >> 16)) * to_float * prof->mod2_depth);
             float mod3   = prof->mod3_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod3 >> 16)) * to_float * prof->mod3_depth);
             float bias   = prof->bias_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_bias >> 16)) * to_float * prof->bias_depth);
 
             float scale_global = wobble * prof->drive;
             float scale_norm = scale_global * to_float; // Bakes Q15 scaling into the FMA multiplier
 
             _ana_fma_fund = scale_norm;
             _ana_fma_h2   = mod2 * scale_norm;
             _ana_fma_h3   = mod3 * scale_norm;
             _ana_fma_bias = bias * scale_global; 
 
             // Bake soft-clipper normalization directly into polynomial coefficients
             _ana_sat_c1 = prof->makeup_gain * 32767.0f;
             _ana_sat_c2 = 0.1666667f * prof->makeup_gain * 32767.0f;
 #else
             // Caches for RP2040 (Pure Fixed-Point path)
             float wobble_f = prof->wobble_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_wobble >> 16)) * to_float * prof->wobble_depth);
             float mod2_f   = prof->mod2_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod2 >> 16)) * to_float * prof->mod2_depth);
             float mod3_f   = prof->mod3_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod3 >> 16)) * to_float * prof->mod3_depth);
             float bias_f   = prof->bias_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_bias >> 16)) * to_float * prof->bias_depth);
             
             _ana_mod2_q15 = (int32_t)(mod2_f * 32768.0f);
             _ana_mod3_q15 = (int32_t)(mod3_f * 32768.0f);
             _ana_bias_q15 = (int32_t)(bias_f * 32768.0f);
             
             _ana_drive_wobble_q14 = (int32_t)(wobble_f * prof->drive * 16384.0f);
             _ana_makeup_q15 = (int32_t)(prof->makeup_gain * 32768.0f);
 #endif
         }
         _ana_counter++;
 
 #if FLOAT_ENGINE
         // -----------------------------------------------------
         // RP2350 (Cortex-M33) - 1-Cycle FMA Float Engine
         // -----------------------------------------------------
         
         // Fast float phase injection (preserves precision at extreme low speeds)
         float drift_offset_f = (float)phase_inc * _ana_drift_val_f;
         phase_inc += (int32_t)drift_offset_f;
         _phase += phase_inc * dt;
 
         // FMA Engine: Executes as VFMA.F32 (1 cycle per line)
         float mixed = _ana_fma_bias;
         mixed += (float)lfo_sine_q15_from_ramp16((uint16_t)(_phase >> 16))       * _ana_fma_fund;
         mixed += (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 2) >> 16)) * _ana_fma_h2;
         mixed += (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 3) >> 16)) * _ana_fma_h3;
 
         // Hardware FPU Bounds Clamping (VMAXNM.F32 / VMINNM.F32)
 #if defined(__ARM_FP)
         mixed = __builtin_fmaxf(-1.5f, __builtin_fminf(mixed, 1.5f));
 #else
         mixed = mixed > 1.5f ? 1.5f : (mixed < -1.5f ? -1.5f : mixed);
 #endif
 
         // Factored Cubic Soft Clipper: x * (C1 - C2 * x^2)
         float mixed_sq = mixed * mixed;
         float sat = mixed * (_ana_sat_c1 - _ana_sat_c2 * mixed_sq);
 
         // Int-cast and saturate 
         return lfo_clamp_q15((int32_t)sat);
 
 #else
         // -----------------------------------------------------
         // RP2040 (Cortex-M0+) - Pure Fixed-Point DSP
         // -----------------------------------------------------
         int32_t drift_sine_q15 = lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_drift >> 16));
         int32_t drift_factor_q15 = (drift_sine_q15 * _ana_drift_depth_q15) >> 15;
         int32_t drift_offset = (int32_t)( ((phase_inc >> 8) * drift_factor_q15) >> 7 );
         
         phase_inc = (uint32_t)((int32_t)phase_inc + drift_offset);
         _phase += phase_inc * dt;
         
         int32_t fund_q15 = lfo_sine_q15_from_ramp16((uint16_t)(_phase >> 16));
         int32_t h2_q15   = (lfo_sine_q15_from_ramp16((uint16_t)((_phase * 2) >> 16)) * _ana_mod2_q15) >> 15;
         int32_t h3_q15   = (lfo_sine_q15_from_ramp16((uint16_t)((_phase * 3) >> 16)) * _ana_mod3_q15) >> 15;
 
         int32_t mixed_q15 = fund_q15 + h2_q15 + h3_q15 + _ana_bias_q15;
 
         int32_t x_q14 = (mixed_q15 * _ana_drive_wobble_q14) >> 15;
 
         if (x_q14 > 24576) x_q14 = 24576;
         else if (x_q14 < -24576) x_q14 = -24576;
 
         int32_t x2_q14 = (x_q14 * x_q14) >> 14;
         int32_t x3_q14 = (x2_q14 * x_q14) >> 14;
         
         int32_t x3_over_6_q14 = (x3_q14 * 2731) >> 14; 
         int32_t sat_q14 = x_q14 - x3_over_6_q14;
 
         int32_t out = (sat_q14 * _ana_makeup_q15) >> 14;
         
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
     
     const int32_t out = (unit_q15 * _ampl_q15) >> 15;
     return lfo_clamp_q15(out);
 }
 
 int MO_LFO_HOT(lfo::getWave)(unsigned long l_t)
 {
     const int l_ampl = _ampl;
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