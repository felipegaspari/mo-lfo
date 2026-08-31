/**
 * @file mo-lfo.cpp
 * @brief Ultra-Optimized Dual-Engine LFO
 */

 #include "Arduino.h"
 #include "mo-lfo.h"
 
 static uint32_t lfo_compute_phase_inc_from_freq(float freq_hz) {
     if (freq_hz <= 0.0f) return 0;
     float v = freq_hz * 4294.967296f; 
     return (uint32_t)(__builtin_fmaxf(0.0f, __builtin_fminf(v, 4294967295.0f)) + 0.5f);
 }
 
 // -----------------------------------------------
 // Engine Specific Math Helpers
 // -----------------------------------------------
 #if FLOAT_ENGINE
 
 // 1. RP2350: Pure FPU Polynomial Sine (Branchless, Zero Memory Access)
 // Evaluates exactly sin(pi * x) mapped to 32-bit integer boundaries in ~9 CPU Cycles
 MO_LFO_ALWAYS_INLINE float lfo_fast_sine_f32(uint32_t phase) {
     float x = (float)(int32_t)phase * 4.65661287307739e-10f; 
     float abs_x = __builtin_fabsf(x);
     
     // Parabolic approximation core: 4x(1 - |x|)
     float y = x * (1.0f - abs_x) * 4.0f; 
     
     // Smooth refinement curve to perfect sine via FMA mapping
     return y * __builtin_fmaf(0.225f, __builtin_fabsf(y), 0.775f);
 }
 
 #else
 
 // 2. RP2040: Packed 32-bit Memory Array (Halves SRAM LDR Overhead)
 static uint32_t s_sineTablePacked[LFO_SINE_TABLE_SIZE];
 static bool s_sineTableInitialized = false;
 
 static void lfo_initSineTable() {
     if (s_sineTableInitialized) return;
     int16_t temp[LFO_SINE_TABLE_SIZE + 1];
     
     const float inv_size = 1.0f / (float)LFO_SINE_TABLE_SIZE;
     for (int i = 0; i <= (int)LFO_SINE_TABLE_SIZE; ++i) {
         float phase = (float)i * inv_size;
         float sign = 1.0f;
         if (phase >= 0.5f) { phase -= 0.5f; sign = -1.0f; }
         float w = phase * 2.0f;
         float x = w * (1.0f - w);
         float s = (16.0f * x) / (5.0f - 4.0f * x);
         temp[i] = (int16_t)(sign * s * 32767.0f);
     }
     temp[LFO_SINE_TABLE_SIZE] = temp[0];
     
     // Pre-Pack into uint32_t [y1 | y0] for single-instruction LDR fetching
     for (int i = 0; i < LFO_SINE_TABLE_SIZE; ++i) {
         s_sineTablePacked[i] = ((uint32_t)(uint16_t)temp[i+1] << 16) | (uint16_t)temp[i];
     }
     s_sineTableInitialized = true;
 }
 
 MO_LFO_ALWAYS_INLINE int32_t lfo_sine_q15_packed(uint16_t ramp16) {
     const uint32_t pair = s_sineTablePacked[ramp16 >> LFO_SINE_FRAC_BITS];
     const int32_t frac = (int32_t)(ramp16 & ((1u << LFO_SINE_FRAC_BITS) - 1u));
     
     const int32_t y0 = (int16_t)(pair & 0xFFFF);
     const int32_t y1 = (int16_t)(pair >> 16);
     return y0 + (((y1 - y0) * frac) >> LFO_SINE_FRAC_BITS);
 }
 #endif
 
 MO_LFO_ALWAYS_INLINE int16_t lfo_clamp_q15(int32_t v) {
 #if defined(__ARM_FEATURE_SAT) || defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_7EM__)
     return (int16_t)__builtin_arm_ssat(v, 16);
 #else
     int32_t c = (v > 32767) ? 32767 : v;
     return (c < -32767) ? -32767 : (int16_t)c;
 #endif
 }
 
 // -----------------------------------------------
 // Class Implementation
 // -----------------------------------------------
 
 lfo::lfo(int dacSize) {
     _dacSize     = (dacSize > 1) ? dacSize : 2;
     _ampl        = _dacSize - 1;
     _ampl_offset = 0;
     _ampl_q15    = MO_LFO_Q15_ONE;
     _waveForm    = 1;
     _mode        = 0;
     _mode0_freq  = 30.0f;
     _mode1_bpm   = 120.0f;
     _mode1_rate  = 1.0f;
     _phase       = 0;
     _ana_dt_accum = 0;
     _ana_counter = 1;
     _t_last      = 0;
     _initialized = false;
 
     _ana_phases[0] = 0x12345678;
     _ana_phases[1] = 0x87654321;
     _ana_phases[2] = 0xABCDEF01;
     _ana_phases[3] = 0x10203040;
     _ana_phases[4] = 0x98765432;
 
 #if !FLOAT_ENGINE
     lfo_initSineTable();
 #endif
 
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
     _ana_incs[0] = lfo_compute_phase_inc_from_freq(_ana_profile.drift_freq_hz);
     _ana_incs[1] = lfo_compute_phase_inc_from_freq(_ana_profile.wobble_freq_hz);
     
     float inc_f = (float)(_mode ? _phase_inc_sync : _phase_inc_free);
     _ana_incs[2] = (uint32_t)(inc_f * _ana_profile.mod2_ratio);
     _ana_incs[3] = (uint32_t)(inc_f * _ana_profile.mod3_ratio);
     _ana_incs[4] = (uint32_t)(inc_f * _ana_profile.bias_ratio);
     
 #if !FLOAT_ENGINE
     _ana_drift_depth_q15 = (int32_t)(_ana_profile.drift_depth * 32768.0f);
 #endif
     
     _ana_counter = 1; 
 }
 
 void MO_LFO_HOT(lfo::setWaveForm)(int l_waveForm) {
     if (l_waveForm < 0) l_waveForm = 0;
     if (l_waveForm > 12) l_waveForm = 12;
     _waveForm = (uint8_t)l_waveForm;
 
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
     (void)l_t; 
     setMode0Freq(l_mode0_freq);
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
 
 void MO_LFO_HOT(lfo::setMode1Phase)(float l_mode1_phase_offset) { 
     (void)l_mode1_phase_offset; 
 }
 
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
 
 // -----------------------------------------------------
 // HOT PATH: Decimated Analog Execution
 // -----------------------------------------------------
 int32_t MO_LFO_HOT(lfo::_advanceAnalog)(uint32_t dt)
 {
     // Accumulate time internally. Bypasses math ops in the audio loop.
     _ana_dt_accum += dt;
     
     // Decimation: Executes every 32 audio frames
     if (--_ana_counter == 0) 
     {
         _ana_counter = 32;
         const uint32_t agg_dt = _ana_dt_accum;
         _ana_dt_accum = 0;
         
         // Vectorized Phase Accumulation (LDMIA/STMIA generated natively)
         #pragma GCC unroll 5
         for (int i = 0; i < 5; ++i) {
             _ana_phases[i] += _ana_incs[i] * agg_dt;
         }
 
         uint32_t base_phase_inc = _mode ? _phase_inc_sync : _phase_inc_free;
 
 #if FLOAT_ENGINE
         const lfo_analog_profile_t* __restrict p = &_ana_profile;
         
         // Fast Polynomial Sine Evaluation
         float drift = lfo_fast_sine_f32(_ana_phases[0]) * p->drift_depth;
         
         // Algorithmic Lift: Bake drift directly into the phase_inc for the next 32 samples!
         _ana_active_phase_inc = base_phase_inc + (int32_t)((float)base_phase_inc * drift);
 
         float wobble = __builtin_fmaf(lfo_fast_sine_f32(_ana_phases[1]), p->wobble_depth, p->wobble_base);
         float mod2   = __builtin_fmaf(lfo_fast_sine_f32(_ana_phases[2]), p->mod2_depth, p->mod2_base);
         float mod3   = __builtin_fmaf(lfo_fast_sine_f32(_ana_phases[3]), p->mod3_depth, p->mod3_base);
         float bias   = __builtin_fmaf(lfo_fast_sine_f32(_ana_phases[4]), p->bias_depth, p->bias_base);
 
         float scale_g = wobble * p->drive;
         _ana_fma_fund = scale_g;
         _ana_fma_h2   = mod2 * scale_g;
         _ana_fma_h3   = mod3 * scale_g;
         _ana_fma_bias = bias * scale_g; 
 
         _ana_sat_c1 = p->makeup_gain * 32767.0f;
         _ana_sat_c2 = 0.1666667f * p->makeup_gain * 32767.0f;
 #else
         const float to_float = 3.0518509476e-5f; 
         int32_t drift_q15 = lfo_sine_q15_packed((uint16_t)(_ana_phases[0] >> 16));
         int32_t drift_factor_q15 = (drift_q15 * _ana_drift_depth_q15) >> 15;
         
         // Algorithmic Lift (Fixed-Point)
         _ana_active_phase_inc = base_phase_inc + (uint32_t)( ((base_phase_inc >> 8) * drift_factor_q15) >> 7 );
 
         float wobble_f = _ana_profile.wobble_base + ((float)lfo_sine_q15_packed((uint16_t)(_ana_phases[1] >> 16)) * to_float * _ana_profile.wobble_depth);
         float mod2_f   = _ana_profile.mod2_base + ((float)lfo_sine_q15_packed((uint16_t)(_ana_phases[2] >> 16)) * to_float * _ana_profile.mod2_depth);
         float mod3_f   = _ana_profile.mod3_base + ((float)lfo_sine_q15_packed((uint16_t)(_ana_phases[3] >> 16)) * to_float * _ana_profile.mod3_depth);
         float bias_f   = _ana_profile.bias_base + ((float)lfo_sine_q15_packed((uint16_t)(_ana_phases[4] >> 16)) * to_float * _ana_profile.bias_depth);
         
         _ana_mod2_q15 = (int32_t)(mod2_f * 32768.0f);
         _ana_mod3_q15 = (int32_t)(mod3_f * 32768.0f);
         _ana_bias_q15 = (int32_t)(bias_f * 32768.0f);
         
         _ana_drive_wobble_q14 = (int32_t)(wobble_f * _ana_profile.drive * 16384.0f);
         _ana_makeup_q15 = (int32_t)(_ana_profile.makeup_gain * 32768.0f);
 #endif
     }
 
     // Step the main oscillator using the pre-baked active increment
     uint32_t current_phase = _phase;
     current_phase += _ana_active_phase_inc * dt;
     _phase = current_phase;
 
 #if FLOAT_ENGINE
     // FMA Hot Engine Synthesis
     float mixed = _ana_fma_bias;
     mixed = __builtin_fmaf(lfo_fast_sine_f32(current_phase), _ana_fma_fund, mixed);
     mixed = __builtin_fmaf(lfo_fast_sine_f32(current_phase * 2), _ana_fma_h2, mixed);
     mixed = __builtin_fmaf(lfo_fast_sine_f32(current_phase * 3), _ana_fma_h3, mixed);
 
 #if defined(__ARM_FP)
     mixed = __builtin_fmaxf(-1.5f, __builtin_fminf(mixed, 1.5f));
 #else
     mixed = mixed > 1.5f ? 1.5f : (mixed < -1.5f ? -1.5f : mixed);
 #endif
 
     // Cubic Soft Clipper (Factored to 3 FPU Cycles)
     float mixed_sq = mixed * mixed;
     float sat = mixed * __builtin_fmaf(-_ana_sat_c2, mixed_sq, _ana_sat_c1);
 
     return lfo_clamp_q15((int32_t)sat);
 
 #else
     // Pure RP2040 Packed LUT Engine Synthesis
     int32_t fund_q15 = lfo_sine_q15_packed((uint16_t)(current_phase >> 16));
     int32_t h2_q15   = (lfo_sine_q15_packed((uint16_t)(current_phase >> 15)) * _ana_mod2_q15) >> 15;
     int32_t h3_q15   = (lfo_sine_q15_packed((uint16_t)((current_phase * 3) >> 16)) * _ana_mod3_q15) >> 15;
 
     int32_t mixed_q15 = fund_q15 + h2_q15 + h3_q15 + _ana_bias_q15;
     int32_t x_q14 = (mixed_q15 * _ana_drive_wobble_q14) >> 15;
 
     int32_t clamp_pos = (x_q14 > 24576) ? 24576 : x_q14;
     x_q14 = (clamp_pos < -24576) ? -24576 : clamp_pos;
 
     int32_t x2_q14 = (x_q14 * x_q14) >> 14;
     int32_t sat_q14 = x_q14 - ((x2_q14 * x_q14 * 2731) >> 28); 
     
     return lfo_clamp_q15((sat_q14 * _ana_makeup_q15) >> 14);
 #endif
 }
 
 int32_t MO_LFO_HOT(lfo::_advanceUnitQ15)(unsigned long l_t)
 {
     if (!_initialized) { 
         _t_last = l_t; 
         _initialized = true; 
     }
 
     const uint32_t dt = (uint32_t)(l_t - _t_last);
     _t_last = l_t;
 
     if (_waveForm == 2 || (_waveForm >= 10 && _waveForm <= 12)) {
         return _advanceAnalog(dt);
     }
     
     if (_waveForm == 0) return 0;
 
     _phase += (_mode ? _phase_inc_sync : _phase_inc_free) * dt;
     const uint16_t ramp16 = (uint16_t)(_phase >> 16);
 
     switch (_waveForm) {
         case 1:  return (int32_t)ramp16 - ((ramp16 & 0x8000u) ? 65536 : 0);
 #if FLOAT_ENGINE
         case 3:  return (int32_t)(lfo_fast_sine_f32(_phase) * 32767.0f);
 #else
         case 3:  return lfo_sine_q15_packed(ramp16);
 #endif
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
 #if FLOAT_ENGINE
             int32_t sine = (int32_t)(lfo_fast_sine_f32(_phase) * 32767.0f);
 #else
             int32_t sine = lfo_sine_q15_packed(ramp16);
 #endif
             int32_t folded = (sine * 3) >> 1;
             return folded > 32767 ? 65534 - folded : (folded < -32767 ? -65534 - folded : folded);
         }
         default: return 0;
     }
 }
 
 int16_t MO_LFO_HOT(lfo::getWaveQ15)(unsigned long l_t) {
     if (_ampl_q15 == 0) { (void)_advanceUnitQ15(l_t); return 0; }
     int32_t unit = _advanceUnitQ15(l_t);
     return (_ampl_q15 == MO_LFO_Q15_ONE) ? (int16_t)unit : lfo_clamp_q15((unit * _ampl_q15) >> 15);
 }
 
 int MO_LFO_HOT(lfo::getWave)(unsigned long l_t) {
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