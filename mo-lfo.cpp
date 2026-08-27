/**
 * @file mo-lfo.cpp
 * @brief Low Frequency Oscillator implementation with fixed-point math and SRAM optimization.
 * @author mo-thunderz
 * @author felipegaspari (modified & optimized)
 * @version 1.9
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
     const double scale = 4294.967296; // 2^32 / 1e6
     double v = (double)freq_hz * scale;
     if (v < 0.0) v = 0.0;
     if (v > 4294967295.0) v = 4294967295.0;
     return (uint32_t)(v + 0.5);
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
 
 MO_LFO_ALWAYS_INLINE int32_t lfo_sine_q15_from_ramp16(uint16_t ramp16)
 {
     const uint16_t idx  = (uint16_t)(ramp16 >> LFO_SINE_FRAC_BITS);
     const uint16_t frac = (uint16_t)(ramp16 & ((1u << LFO_SINE_FRAC_BITS) - 1u));
     const int16_t y0 = s_sineTable[idx];
     const int16_t y1 = s_sineTable[(idx + 1u) & (LFO_SINE_TABLE_SIZE - 1u)];
     return (int32_t)y0 + ((((int32_t)y1 - (int32_t)y0) * (int32_t)frac) >> LFO_SINE_FRAC_BITS);
 }
 
 MO_LFO_ALWAYS_INLINE int16_t lfo_clamp_q15(int32_t v)
 {
     return v < -(int32_t)MO_LFO_Q15_ONE ? -(int16_t)MO_LFO_Q15_ONE : (v > (int32_t)MO_LFO_Q15_ONE ? MO_LFO_Q15_ONE : (int16_t)v);
 }
 
 MO_LFO_ALWAYS_INLINE int32_t lfo_unit_q15_from_ramp(uint16_t ramp16, int waveForm)
 {
     switch (waveForm)
     {
         case 1: 
             if (ramp16 < 0x8000u) return (int32_t)ramp16;
             return (int32_t)ramp16 - 65536;
 
         case 2: case 10: case 11: case 12: 
             return 0; // Handled directly in _advanceUnitQ15()
 
         case 3: 
             return lfo_sine_q15_from_ramp16(ramp16);
             
         case 4: 
             return (ramp16 & 0x8000u) ? -(int32_t)MO_LFO_Q15_ONE : (int32_t)MO_LFO_Q15_ONE;
                                       
         case 5: 
             if (ramp16 < 58982u) return (int32_t)((ramp16 * 72818u) >> 16) - 32767;
             else {
                 const uint32_t inv = 65535u - (uint32_t)ramp16;
                 return (int32_t)((inv * 655360u) >> 16) - 32767;
             }
         
         case 6: 
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             const int32_t trap = ((int32_t)tri16 * 6) - 98304;
             return trap < -32767 ? -32767 : (trap > 32767 ? 32767 : trap);
         }
 
         case 7: 
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             return ((int32_t)tri16 * 2) - 32768;
         }
         
         case 8: 
         {
             const uint16_t r = (uint16_t)(ramp16 + 0x4000u);
             const uint16_t tri16 = (r & 0x8000u) ? (uint16_t)(0xFFFFu - r) : r;
             const uint16_t step = tri16 >> 12;
             return ((int32_t)step * 9362) - 32767;
         }
         
         case 9: 
         {
             const int32_t sine = lfo_sine_q15_from_ramp16(ramp16);
             int32_t folded = (sine * 3) >> 1;
             if (folded > 32767) folded = 65534 - folded;
             else if (folded < -32767) folded = -65534 - folded;
             return folded;
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
 
     // Seed analog sub-LFO phases
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
 
     lfo_initSineTable();
     _updatePhaseIncFree();
     _updatePhaseIncSync();
     _updateAnalogIncrements();
 }
 
 void lfo::setAnalogProfile(const lfo_analog_profile_t& profile)
 {
     _ana_profile = profile;
     _updateAnalogIncrements();
 }
 
 void lfo::setAnalogPreset(LfoAnalogPreset preset)
 {
     lfo_analog_profile_t p;
     switch (preset)
     {
         case LfoAnalogPreset::TapeWarble:
             // Noticeable Wow & Flutter and tape dropouts, strictly clean saturation
             p.drift_freq_hz  = 0.08f;
             p.drift_depth    = 0.085f;  // +-3.5% pitch flutter (very obvious warble)
             p.wobble_freq_hz = 0.15f;
             p.wobble_base    = 0.90f;   
             p.wobble_depth   = 0.10f;   // +-10% volume tremolo (tape dropouts)
             
             p.mod2_ratio     = 0.04f;    
             p.mod2_base      = 0.05f;
             p.mod2_depth     = 0.2f;   
             
             p.mod3_ratio     = 0.12f;
             p.mod3_base      = 0.02f;   // Kept low! High 3rd harmonic makes a square wave.
             p.mod3_depth     = 0.01f;
             
             p.bias_ratio     = 0.02f;
             p.bias_base      = 0.1f;
             p.bias_depth     = 0.2f;
             
             p.drive          = 1.00f;    // Pushes into the soft curve perfectly
             p.makeup_gain    = 1.05f;   // Max safe gain (0.94 * 1.05 = 0.98 peak)
             break;
 
         case LfoAnalogPreset::ClassATube:
             // Massive lop-sided waveform bending, but strictly clean
             p.drift_freq_hz  = 0.10f;
             p.drift_depth    = 0.005f;
             
             p.wobble_freq_hz = 0.10f;
             p.wobble_base    = 0.95f;
             p.wobble_depth   = 0.05f;
             
             p.mod2_ratio     = 0.25f;   
             p.mod2_base      = 0.15f;   // 2nd harmonic safely bends the wave sideways
             p.mod2_depth     = 0.05f;   
             
             p.mod3_ratio     = 0.05f;
             p.mod3_base      = 0.01f;
             p.mod3_depth     = 0.01f;
             
             p.bias_ratio     = 0.12f;   
             p.bias_base      = 0.08f;   // Gently pushes wave off-center (DC offset)
             p.bias_depth     = 0.05f;   // Breathes up and down safely
             
             p.drive          = 0.95f;   // Gives headroom for the massive 2nd harmonic + bias
             p.makeup_gain    = 1.02f;   
             break;
 
         case LfoAnalogPreset::BrokenVintage:
             // Absolute chaos: drunk pitch, sputtering volume, chaotic shape
             p.drift_freq_hz  = 1.20f;
             p.drift_depth    = 0.06f;   // +-6% pitch warble (seasick)
             
             p.wobble_freq_hz = 0.80f;
             p.wobble_base    = 0.85f;
             p.wobble_depth   = 0.15f;   // Sputtering volume drops
             
             p.mod2_ratio     = 0.43f;
             p.mod2_base      = 0.12f;
             p.mod2_depth     = 0.08f;
             
             p.mod3_ratio     = 0.31f;
             p.mod3_base      = 0.05f;   // Adding bite, but stopping before it squares off
             p.mod3_depth     = 0.03f;
             
             p.bias_ratio     = 0.27f;
             p.bias_base      = 0.05f;
             p.bias_depth     = 0.08f;   // Erratically jumping center point
             
             p.drive          = 0.80f;   
             p.makeup_gain    = 1.05f;   
             break;
 
         case LfoAnalogPreset::SubtleWarmth:
         default:
             // The default struct definition will handle this.
             // (Note: ensure your struct definition in mo-lfo.h has makeup_gain = 1.05f, NOT 1.15f!)
             break;
     }
     setAnalogProfile(p);
 }
 
 void lfo::_updateAnalogIncrements()
 {
     // Absolute Time Sub-LFOs
     _ana_inc_drift  = lfo_compute_phase_inc_from_freq(_ana_profile.drift_freq_hz);
     _ana_inc_wobble = lfo_compute_phase_inc_from_freq(_ana_profile.wobble_freq_hz);
 
     // Proportional Sub-LFOs
     uint32_t active_inc = _mode ? _phase_inc_sync : _phase_inc_free;
     _ana_inc_mod2 = (uint32_t)((double)active_inc * (double)_ana_profile.mod2_ratio);
     _ana_inc_mod3 = (uint32_t)((double)active_inc * (double)_ana_profile.mod3_ratio);
     _ana_inc_bias = (uint32_t)((double)active_inc * (double)_ana_profile.bias_ratio);
 }
 
 void lfo::setWaveForm(int l_waveForm) {
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
 
 void lfo::setAmpl(int l_ampl) {
     if (l_ampl < 0) l_ampl = 0;
     if (l_ampl >= _dacSize) l_ampl = _dacSize - 1;
     _ampl = l_ampl;
     _updateAmplQ15FromDac();
 }
 
 void lfo::setAmplQ15(int16_t l_ampl_q15) {
     if (l_ampl_q15 < 0) l_ampl_q15 = 0;
     if (l_ampl_q15 > MO_LFO_Q15_ONE) l_ampl_q15 = MO_LFO_Q15_ONE;
     _ampl_q15 = l_ampl_q15;
     const int ampl_max = _dacSize - 1;
     if (ampl_max > 0) _ampl = (int)(((int32_t)_ampl_q15 * ampl_max + (MO_LFO_Q15_ONE / 2)) / MO_LFO_Q15_ONE);
     else _ampl = 0;
 }
 
 void lfo::_updateAmplQ15FromDac() {
     const int ampl_max = _dacSize - 1;
     if (ampl_max <= 0) { _ampl_q15 = 0; return; }
     if (_ampl >= ampl_max) { _ampl_q15 = MO_LFO_Q15_ONE; return; }
     _ampl_q15 = (int16_t)(((int32_t)_ampl * (int32_t)MO_LFO_Q15_ONE) / ampl_max);
 }
 
 void lfo::setAmplOffset(int l_ampl_offset) {
     if (l_ampl_offset < 0) l_ampl_offset = 0;
     if (l_ampl_offset >= _dacSize) l_ampl_offset = _dacSize - 1;
     _ampl_offset = l_ampl_offset;
 }
 
 void lfo::setMode(bool l_mode) { 
     _mode = l_mode; 
     _updateAnalogIncrements(); // Active inc changed
 }
 
 void lfo::setMode0Freq(float l_mode0_freq) {
     if (l_mode0_freq < 0) l_mode0_freq = 0;
     _mode0_freq = l_mode0_freq;
     _updatePhaseIncFree();
 }
 
 void lfo::setMode0Freq(float l_mode0_freq, unsigned long l_t) {
     (void)l_t; setMode0Freq(l_mode0_freq);
 }
 
 void lfo::setMode1Bpm(float l_mode1_bpm) {
     if (l_mode1_bpm < 0) l_mode1_bpm = 0;
     _mode1_bpm = l_mode1_bpm;
     _updatePhaseIncSync();
 }
 
 void lfo::setMode1Rate(float l_mode1_rate) {
     if (l_mode1_rate < 0) l_mode1_rate = 0;
     _mode1_rate = l_mode1_rate;
     _updatePhaseIncSync();
 }
 
 void lfo::setMode1Phase(float l_mode1_phase_offset) { (void)l_mode1_phase_offset; }
 void lfo::sync(unsigned long l_t) {
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
 
     // ---------------------------------------------------------
     // SPECIAL PATH: Analog Sine Models (Waveforms 2, 10, 11, 12)
     // ---------------------------------------------------------
     if (_waveForm == 2 || (_waveForm >= 10 && _waveForm <= 12)) 
     {
         // 1. Advance sub-LFO phases using microseconds dt timings
         _ana_phase_drift  += _ana_inc_drift * dt;
         _ana_phase_wobble += _ana_inc_wobble * dt;
         _ana_phase_mod2   += _ana_inc_mod2 * dt;
         _ana_phase_mod3   += _ana_inc_mod3 * dt;
         _ana_phase_bias   += _ana_inc_bias * dt;
 
         // 2. CPU Decimation: Only evaluate heavy math every 32 ticks
         if ((_ana_counter & 0x1F) == 0) 
         {
             const float to_float = 3.0518509476e-5f; // 1.0f / 32767.0f
             
             _ana_drift_val  = (float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_drift >> 16)) * to_float * _ana_profile.drift_depth;
             _ana_wobble_val = _ana_profile.wobble_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_wobble >> 16)) * to_float * _ana_profile.wobble_depth);
             
             _ana_mod2_val   = _ana_profile.mod2_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod2 >> 16)) * to_float * _ana_profile.mod2_depth);
             _ana_mod3_val   = _ana_profile.mod3_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_mod3 >> 16)) * to_float * _ana_profile.mod3_depth);
             _ana_bias_val   = _ana_profile.bias_base + ((float)lfo_sine_q15_from_ramp16((uint16_t)(_ana_phase_bias >> 16)) * to_float * _ana_profile.bias_depth);
         }
         _ana_counter++;
 
         // 3. Inject absolute pitch drift (cast through int32_t to handle negative drift safely)
         int32_t drift_offset = (int32_t)((float)phase_inc * _ana_drift_val);
         phase_inc = (uint32_t)((int32_t)phase_inc + drift_offset);
 
         // 4. Advance main phase
         _phase += phase_inc * dt;
 
         // 5. Build Dynamic Analog Wave
         const float to_float = 3.0518509476e-5f;
         float fund = (float)lfo_sine_q15_from_ramp16((uint16_t)(_phase >> 16)) * to_float;
         float h2   = (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 2) >> 16)) * to_float * _ana_mod2_val;
         float h3   = (float)lfo_sine_q15_from_ramp16((uint16_t)((_phase * 3) >> 16)) * to_float * _ana_mod3_val;
 
         // Apply DC bias shift and overdrive
         float mixed = (fund + h2 + h3 + _ana_bias_val) * _ana_wobble_val * _ana_profile.drive;
 
         // Fast cubic soft-clip bounds protection
         if (mixed > 1.5f) mixed = 1.5f;
         else if (mixed < -1.5f) mixed = -1.5f;
 
         // f(x) = x - x^3 / 6 scaled by makeup gain
         float sat = mixed * (1.0f - 0.1666667f * mixed * mixed) * _ana_profile.makeup_gain;
 
         // Convert back to Q15 Int16
         int32_t out = (int32_t)(sat * 32767.0f);
         return out > 32767 ? 32767 : (out < -32767 ? -32767 : out);
     }
 
     // ---------------------------------------------------------
     // STANDARD PATH: All other waveforms
     // ---------------------------------------------------------
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
     const int32_t out = (int32_t)(((int64_t)unit_q15 * (int64_t)_ampl_q15) >> 15);
     return lfo_clamp_q15(out);
 }
 
 int lfo::getWave(unsigned long l_t)
 {
     const int l_ampl = _ampl;
     const int l_ampl_half = (int)l_ampl / 2;
     int l_ampl_offset = 0;
     
     if (_ampl_offset < _dacSize / 2) l_ampl_offset = (_ampl_offset > l_ampl_half) ? _ampl_offset : l_ampl_half;
     else l_ampl_offset = (_dacSize - _ampl_offset > l_ampl_half) ? _ampl_offset : _dacSize - l_ampl_half - 1;
 
     if (_waveForm == 0) { (void)_advanceUnitQ15(l_t); return _ampl_offset; }
 
     const int32_t unit_q15 = _advanceUnitQ15(l_t);
     const int32_t scaled = ((int32_t)l_ampl_half * unit_q15) >> 15;
     return (int)scaled + l_ampl_offset;
 }
 
 void lfo::_updatePhaseIncFree() { 
     _phase_inc_free = lfo_compute_phase_inc_from_freq(_mode0_freq); 
     if (!_mode) _updateAnalogIncrements(); // Refresh proportions if active
 }
 
 void lfo::_updatePhaseIncSync() {
     float freq_hz = 0.0f;
     if (_mode1_rate > 0.0f && _mode1_bpm > 0.0f) freq_hz = (_mode1_rate * _mode1_bpm) / 60.0f;
     _phase_inc_sync = lfo_compute_phase_inc_from_freq(freq_hz);
     if (_mode) _updateAnalogIncrements(); // Refresh proportions if active
 }