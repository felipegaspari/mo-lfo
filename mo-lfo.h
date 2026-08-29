/**
 * @file mo-lfo.h
 * @brief Low Frequency Oscillator (LFO) class for Arduino and embedded synthesizers.
 */

 #ifndef mo_lfo_h
 #define mo_lfo_h
 
 #include "Arduino.h"
 #include <stdint.h>
 
 // -------------------------------------------------
 // Configuration macros & Dual Engine Toggle
 // -------------------------------------------------
 
 // NEW: Auto-detect Architecture to switch between Pure FPU and Pure Fixed-Point
 #ifndef FLOAT_ENGINE
   #if defined(PICO_RP2350) || defined(ARDUINO_ARCH_RP2350) || defined(__ARM_FP)
     #define FLOAT_ENGINE 1
   #else
     #define FLOAT_ENGINE 0
   #endif
 #endif
 
 #ifndef LFO_SINE_TABLE_BITS
 #define LFO_SINE_TABLE_BITS 9
 #endif
 
 #define LFO_SINE_TABLE_SIZE (1u << LFO_SINE_TABLE_BITS)
 #define LFO_SINE_FRAC_BITS  (16 - LFO_SINE_TABLE_BITS)
 
 #ifndef MO_LFO_USE_Q15
 #define MO_LFO_USE_Q15 0
 #endif
 
 #ifndef MO_LFO_SRAM_HOT
 #define MO_LFO_SRAM_HOT 0
 #endif
    
 #if MO_LFO_SRAM_HOT
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      #ifndef __not_in_flash_func
        #define __not_in_flash_func(fn) fn
      #endif
      #define MO_LFO_HOT(fn) __not_in_flash_func(fn)
    #elif defined(STM32H7) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      #define MO_LFO_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn
    #else
      #define MO_LFO_HOT(fn) fn
    #endif
 #else
    #define MO_LFO_HOT(fn) fn
 #endif
 
 #ifndef MO_LFO_ALWAYS_INLINE
 #if defined(__GNUC__) || defined(__clang__)
 #define MO_LFO_ALWAYS_INLINE static inline __attribute__((always_inline))
 #else
 #define MO_LFO_ALWAYS_INLINE static inline
 #endif
 #endif
 
 static const int16_t MO_LFO_Q15_ONE = 32767;
 
 struct lfo_analog_profile_t {
     float drift_freq_hz  = 0.31f;  
     float drift_depth    = 0.025f; 
     float wobble_freq_hz = 0.13f;  
     float wobble_base    = 0.95f; 
     float wobble_depth   = 0.05f; 
     float mod2_ratio     = 0.23f;  
     float mod2_base      = 0.04f;  
     float mod2_depth     = 0.04f; 
     float mod3_ratio     = 0.37f;  
     float mod3_base      = 0.015f;  
     float mod3_depth     = 0.015f; 
     float bias_ratio     = 0.17f;  
     float bias_base      = 0.01f; 
     float bias_depth     = 0.03f;  
     float drive          = 1.15f;  
     float makeup_gain    = 1.06f;  
 };
 
 enum class LfoAnalogPreset {
     SubtleWarmth = 0,
     TapeWarble   = 1,
     ClassATube   = 2,
     BrokenVintage= 3
 };
 
 class lfo
 {
     public:
         lfo(int dacSize);
 
         void setAmpl(int l_ampl);
         void setAmplQ15(int16_t l_ampl_q15);
         int16_t getAmplQ15() const { return _ampl_q15; }
         void setAmplOffset(int l_ampl_offset);
 
         void setWaveForm(int l_waveForm);
         void setAnalogProfile(const lfo_analog_profile_t& profile);
         lfo_analog_profile_t getAnalogProfile() const { return _ana_profile; }
         void setAnalogPreset(LfoAnalogPreset preset);
 
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
 
         int getWave(unsigned long l_t);
         int16_t getWaveQ15(unsigned long l_t);
 
     private:
         int                  _dacSize;
         int                  _waveForm = 1;
         int                  _ampl = 0;
         int                  _ampl_offset = 0;
         int16_t              _ampl_q15 = MO_LFO_Q15_ONE;
         bool                 _mode = 0;
         float                _mode0_freq = 30;
         float                _mode1_bpm = 120;
         float                _mode1_rate = 1;
 
         uint32_t             _phase = 0;
         uint32_t             _phase_inc_free = 0;
         uint32_t             _phase_inc_sync = 0;
         unsigned long        _t_last = 0;
         bool                 _initialized = false;
         
         lfo_analog_profile_t _ana_profile;
         uint32_t             _ana_phase_drift;
         uint32_t             _ana_phase_wobble;
         uint32_t             _ana_phase_mod2;
         uint32_t             _ana_phase_mod3;
         uint32_t             _ana_phase_bias;
 
         uint32_t             _ana_inc_drift;
         uint32_t             _ana_inc_wobble;
         uint32_t             _ana_inc_mod2;
         uint32_t             _ana_inc_mod3;
         uint32_t             _ana_inc_bias;
 
         uint8_t              _ana_counter;
        
         // ------------------------------------------------
         // Dual Engine State Variables
         // ------------------------------------------------
 #if FLOAT_ENGINE
         // Fast float caches for RP2350 hardware FPU FMA block
         float                _ana_drift_val_f;
         float                _ana_fma_fund;
         float                _ana_fma_h2;
         float                _ana_fma_h3;
         float                _ana_fma_bias;
         float                _ana_sat_c1;
         float                _ana_sat_c2;
 #else
         // Q15/Q14 caches for pure integer RP2040 math
         int32_t              _ana_drift_depth_q15;
         int32_t              _ana_mod2_q15;
         int32_t              _ana_mod3_q15;
         int32_t              _ana_bias_q15;
         int32_t              _ana_drive_wobble_q14;
         int32_t              _ana_makeup_q15;
 #endif
 
         void                 _updatePhaseIncFree();
         void                 _updatePhaseIncSync();
         void                 _updateAmplQ15FromDac();
         void                 _updateAnalogIncrements();
 
         int32_t              _advanceUnitQ15(unsigned long l_t);
 };
 
 #endif