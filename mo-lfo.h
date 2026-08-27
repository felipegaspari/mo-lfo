/**
 * @file mo-lfo.h
 * @brief Low Frequency Oscillator (LFO) class for Arduino and embedded synthesizers.
 * @author mo-thunderz
 * @author felipegaspari (modified & optimized)
 * @version 1.9
 *
 * @details High-performance LFO engine supporting both integer DAC scaling [0, dacSize-1]
 *          and bipolar fixed-point Q15 [-32767, +32767] output. Features analog modeling
 *          (including 4 dedicated parametric analog sine waveforms), hardware slew
 *          emulations, and non-traditional wavefolded/quantized shapes.
 */

 #ifndef mo_lfo_h
 #define mo_lfo_h
 
 #include "Arduino.h"
 #include <stdint.h>
 
 // -------------------------------------------------
 // Configuration macros
 // -------------------------------------------------
 
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
    /* ---------- Raspberry Pi Pico / RP2040 / RP2350 ---------- */
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      #ifndef __not_in_flash_func
        #define __not_in_flash_func(fn) fn
      #endif
      #define MO_LFO_HOT(fn) __not_in_flash_func(fn)
    /* ---------- STM32H7 (ITCM) ---------- */
    #elif defined(STM32H7) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      #define MO_LFO_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn
    /* ---------- Fallback ---------- */
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
 
 /**
  * @struct lfo_analog_profile_t
  * @brief Complete parameter profile controlling the Analog Sine deformation engine.
  */
  struct lfo_analog_profile_t {
    // ---------------------------------------------------------
    // ABSOLUTE IMPERFECTIONS (Static Hz, physical hardware models)
    // ---------------------------------------------------------
    // Shifted to 0.31 Hz / 2.5% depth so cycle periods vary subtly in time
    float drift_freq_hz  = 0.31f;  
    float drift_depth    = 0.025f; 
    
    // Wobble breathes between 0.90 and 1.00
    float wobble_freq_hz = 0.13f;  
    float wobble_base    = 0.95f; 
    float wobble_depth   = 0.05f; 

    // ---------------------------------------------------------
    // PROPORTIONAL SHAPE EVOLUTION (Scales with Main LFO Speed)
    // ---------------------------------------------------------
    // Faster, prime ratios make consecutive cycles look and feel distinct:
    // 2nd Harmonic sweeps every ~4.3 cycles (smooth asymmetrical belly)
    float mod2_ratio     = 0.23f;  
    float mod2_base      = 0.04f;  
    float mod2_depth     = 0.04f; 
    
    // 3rd Harmonic kept LOW so it does NOT sharpen into a triangle!
    // Sweeps every ~2.7 cycles
    float mod3_ratio     = 0.37f;  
    float mod3_base      = 0.015f;  
    float mod3_depth     = 0.015f; 

    // Dynamic DC shift sweeps every ~5.8 cycles (positive vs negative crests alternate height)
    float bias_ratio     = 0.17f;  
    float bias_base      = 0.01f; 
    float bias_depth     = 0.03f;  

    // ---------------------------------------------------------
    // SATURATION & GAIN STAGING
    // ---------------------------------------------------------
    // Drive pushes the wave into the sweet spot (x ≈ 1.32) of the soft clipper
    float drive          = 1.15f;  
    
    // 1.06 scales the soft-clipper output to exactly 99.4% full amplitude (no hard-clipping!)
    float makeup_gain    = 1.06f;  
};

 /**
  * @enum LfoAnalogPreset
  * @brief Factory preset profiles for the analog sine engine.
  */
 enum class LfoAnalogPreset {
     SubtleWarmth = 0,   /**< Gentle hardware drift, subtle shape morphing (Waveform 2). */
     TapeWarble   = 1,   /**< Noticeable tape wow/flutter and magnetic tape saturation (Waveform 10). */
     ClassATube   = 2,   /**< Heavy 2nd harmonic asymmetry, dynamic DC bias shift, round clipping (Waveform 11). */
     BrokenVintage= 3    /**< Severe pitch instability, high harmonic grit, heavy overdrive (Waveform 12). */
 };
 
 class lfo
 {
     public:
         lfo(int dacSize);
 
         void setAmpl(int l_ampl);
         void setAmplQ15(int16_t l_ampl_q15);
         int16_t getAmplQ15() const { return _ampl_q15; }
         void setAmplOffset(int l_ampl_offset);
 
         /**
          * @brief Selects the active LFO waveform shape.
          * 
          * | Index | UI Name          | Type            | Description / Musical Behavior |
          * |:-----:|:-----------------|:----------------|:-------------------------------|
          * | **0** | **Off**          | Static          | Output holds DC offset (0 in Q15, ampl_offset in DAC). |
          * | **1** | **Saw**          | Standard        | Linear ramp rising from -1.0 to +1.0 with instant reset. |
          * | **2** | **Analog Sine**  | Analog Model    | Subtle Warmth: gentle drift, mild morphing harmonics. |
          * | **3** | **Sine**         | Standard        | 512-point table lookup with linear interpolation. |
          * | **4** | **Square**       | Standard        | 50% duty cycle bipolar pulse with instantaneous edge. |
          * | **5** | **Sharktooth**   | Analog Model    | 90% linear rise, 10% slewed discharge. Softens harsh clicks. |
          * | **6** | **Trapezoid**    | Analog Model    | Slewed square wave (3x overdrive). Soft vintage edge. |
          * | **7** | **Linear Tri**   | Standard        | Pure linear triangle with sharp peak turnarounds. |
          * | **8** | **Staircase**    | Non-Traditional | 8-step quantized triangle. Perfect for rhythmic S&H and arps. |
          * | **9** | **Folded Sine**  | Non-Traditional | Buchla-style fold: +50% overdriven sine folding inwards. |
          * | **10**| **Analog Tape**  | Analog Model    | Tape Warble: strong wow/flutter & magnetic saturation. |
          * | **11**| **Analog Tube**  | Analog Model    | Class-A Tube: heavy asymmetry, shifting DC bias, round clipping. |
          * | **12**| **Analog Broken**| Analog Model    | Broken Vintage: severe warble, heavy grit, aggressive overdrive. |
          */
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
         
         // ------------------------------------------------
         // Analog Sine Stateful Variables
         // ------------------------------------------------
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
         float                _ana_drift_val;
         float                _ana_wobble_val;
         float                _ana_mod2_val;
         float                _ana_mod3_val;
         float                _ana_bias_val;
 
         void                 _updatePhaseIncFree();
         void                 _updatePhaseIncSync();
         void                 _updateAmplQ15FromDac();
         void                 _updateAnalogIncrements();
 
         int32_t              _advanceUnitQ15(unsigned long l_t);
 };
 
 #endif