/**
 * @file mo-lfo.h
 * @brief Low Frequency Oscillator (LFO) class for Arduino and embedded synthesizers.
 * @author mo-thunderz
 * @author felipegaspari (modified & optimized)
 * @version 1.5
 *
 * @details High-performance LFO engine supporting both integer DAC scaling [0, dacSize-1]
 *          and bipolar fixed-point Q15 [-32767, +32767] output. Supports hardware slew
 *          emulations and non-traditional wavefolded/quantized shapes.
 */

 #ifndef mo_lfo_h
 #define mo_lfo_h
 
 #include "Arduino.h"
 #include <stdint.h>
 
 // -------------------------------------------------
 // Configuration macros
 // -------------------------------------------------
 
 /**
  * @def LFO_SINE_TABLE_BITS
  * @brief Sine lookup table resolution in bits (table size = 2^LFO_SINE_TABLE_BITS).
  */
 #ifndef LFO_SINE_TABLE_BITS
 #define LFO_SINE_TABLE_BITS 9
 #endif
 
 /** @brief Number of entries in the sine lookup table (default: 512). */
 #define LFO_SINE_TABLE_SIZE (1u << LFO_SINE_TABLE_BITS)
 
 /** @brief Number of fractional interpolation bits. */
 #define LFO_SINE_FRAC_BITS  (16 - LFO_SINE_TABLE_BITS)
 
 /**
  * @def MO_LFO_USE_Q15
  * @brief Preferred engine path hint (0 = DAC int, 1 = bipolar Q15).
  */
 #ifndef MO_LFO_USE_Q15
 #define MO_LFO_USE_Q15 0
 #endif
 
 /**
  * @def MO_LFO_SRAM_HOT
  * @brief Places critical DSP functions into fast SRAM (.time_critical) on supported MCUs (RP2040, RP2350, STM32).
  *        1 = Enabled (RAM execution), 0 = Disabled (Flash execution).
  */

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
      /* Place the function in the .itcmram section (must be defined in the linker script) */
      #define MO_LFO_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn
  
    /* ---------- Fallback ---------- */
    #else
      #define MO_LFO_HOT(fn) fn
    #endif
  
  #else
    /* Portable fallback (no special placement) */
    #define MO_LFO_HOT(fn) fn
  #endif
 
 /**
  * @def MO_LFO_ALWAYS_INLINE
  * @brief Forces inlining of shape generators into the caller to guarantee RAM execution.
  */
 #ifndef MO_LFO_ALWAYS_INLINE
 #if defined(__GNUC__) || defined(__clang__)
 #define MO_LFO_ALWAYS_INLINE static inline __attribute__((always_inline))
 #else
 #define MO_LFO_ALWAYS_INLINE static inline
 #endif
 #endif
 
 #ifndef MO_LFO_CONFIG_REPORTED
 #define MO_LFO_CONFIG_REPORTED
 #if MO_LFO_USE_Q15
 #pragma message("MO-LFO: preferred path=Q15 (MO_LFO_USE_Q15=1); API always has getWave+getWaveQ15")
 #else
 #pragma message("MO-LFO: preferred path=DAC (MO_LFO_USE_Q15=0); API always has getWave+getWaveQ15")
 #endif
 #if MO_LFO_SRAM_HOT
 #pragma message("MO-LFO: SRAM hot path ON (MO_LFO_SRAM_HOT=1) — getWaveQ15/_advanceUnitQ15 .time_critical")
 #else
 #pragma message("MO-LFO: SRAM hot path OFF (MO_LFO_SRAM_HOT=0) — library default")
 #endif
 #endif
 
 /** @brief Q15 full scale unit value in int16_t (±32767 ≈ ±1.0). */
 static const int16_t MO_LFO_Q15_ONE = 32767;
 
 /**
  * @class lfo
  * @brief High-precision phase accumulator LFO with analog and complex waveform generation.
  */
 class lfo
 {
     public:
         /**
          * @brief Constructs an LFO instance.
          * @param dacSize Vertical range for unipolar DAC integer output -> [0, dacSize-1].
          */
         lfo(int dacSize);
 
         /**
          * @brief Sets LFO amplitude using DAC integer counts.
          * @param l_ampl Peak-to-peak amplitude (0 to dacSize-1). Also updates internal Q15 amplitude.
          */
         void setAmpl(int l_ampl);
 
         /**
          * @brief Sets LFO amplitude using standard Q15 representation.
          * @param l_ampl_q15 Amplitude from 0 (silent) to 32767 (full scale ±1.0). Also updates DAC amplitude.
          */
         void setAmplQ15(int16_t l_ampl_q15);
 
         /**
          * @brief Gets current amplitude in Q15 format.
          * @return int16_t Amplitude in range 0..32767.
          */
         int16_t getAmplQ15() const { return _ampl_q15; }
 
         /**
          * @brief Sets DC offset for getWave() unipolar output only. Ignored by getWaveQ15().
          * @param l_ampl_offset Center offset in DAC steps.
          */
         void setAmplOffset(int l_ampl_offset);
 
         /**
          * @brief Selects the active LFO waveform shape.
          * 
          * @param l_waveForm Waveform selector index (0 to 9):
          * 
          * | Index | UI Name       | Type            | Description / Musical Behavior |
          * |:-----:|:--------------|:----------------|:-------------------------------|
          * | **0** | **Off**       | Static          | Output holds DC offset (0 in Q15, ampl_offset in DAC). |
          * | **1** | **Saw**       | Standard        | Linear ramp rising from -1.0 to +1.0 with instant reset. |
          * | **2** | **Tri Slewed**| Analog Model    | Analog triangle with cubic rounded peaks (zero-derivative turnaround). |
          * | **3** | **Sine**      | Standard        | 512-point table lookup with linear interpolation. |
          * | **4** | **Square**    | Standard        | 50% duty cycle bipolar pulse with instantaneous edge. |
          * | **5** | **Sharktooth**| Analog Model    | 90% linear rise, 10% slewed discharge. Softens harsh clicks. |
          * | **6** | **Trapezoid** | Analog Model    | Slewed square wave (3x overdrive). Soft vintage edge. |
          * | **7** | **Linear Tri**| Standard        | Pure linear triangle with sharp peak turnarounds. |
          * | **8** | **Staircase** | Non-Traditional | 8-step quantized triangle. Perfect for rhythmic S&H and arps. |
          * | **9** | **Folded Sine**| Non-Traditional| Buchla-style fold: +50% overdriven sine folding inwards. |
          * 
          * @note All shapes are phase-aligned (sine-native: crossing zero and rising at phase 0).
          */
         void setWaveForm(int l_waveForm);
 
         /**
          * @brief Sets LFO frequency mode.
          * @param l_freq_sync 0 = Free-running Hz mode (Mode 0), 1 = BPM tempo-sync mode (Mode 1).
          */
         void setMode(bool l_freq_sync);
 
         /**
          * @brief Sets frequency in Hertz for free-running mode (Mode 0).
          * @param l_mode0_freq Frequency in Hz (e.g., 0.1 to 50.0).
          */
         void setMode0Freq(float l_mode0_freq);
 
         /**
          * @brief Sets frequency in Hertz for free-running mode (Mode 0) with timestamp.
          * @param l_mode0_freq Frequency in Hz.
          * @param l_t Current timestamp in microseconds.
          */
         void setMode0Freq(float l_mode0_freq, unsigned long l_t);
 
         /**
          * @brief Sets tempo in BPM for sync mode (Mode 1).
          * @param l_mode1_bpm Tempo in beats per minute.
          */
         void setMode1Bpm(float l_mode1_bpm);
 
         /**
          * @brief Sets musical subdivision rate multiplier for sync mode (Mode 1).
          * @param l_mode1_rate Musical note subdivision factor (refer to setMode1Rate table).
          */
         void setMode1Rate(float l_mode1_rate);
 
         /**
          * @brief Sets phase offset for sync mode (Mode 1).
          * @param l_mode1_phase_offset Phase offset value.
          */
         void setMode1Phase(float l_mode1_phase_offset);
 
         /**
          * @brief Resets accumulator phase to 0 immediately (hard-sync / key-sync).
          * @param l_t Current timestamp in microseconds.
          */
         void sync(unsigned long l_t);
 
         /** @brief Returns active waveform ID (0..9). */
         int getWaveForm();
 
         /** @brief Returns current DAC amplitude. */
         int getAmpl();
 
         /** @brief Returns current DAC DC offset. */
         int getAmplOffset();
 
         /** @brief Returns active mode (0 = Free Hz, 1 = Tempo Sync). */
         bool getMode();
 
         /** @brief Returns free-running frequency in Hz. */
         float getMode0Freq();
 
         /** @brief Returns tempo-sync rate multiplier. */
         float getMode1Rate();
 
         /** @brief Returns normalized accumulator phase [0.0, 1.0). */
         float getPhase();
 
         /**
          * @brief Computes unipolar DAC integer output for current timestamp.
          * @param l_t Current timestamp in microseconds (micros()).
          * @return int Output value in range [0, dacSize-1].
          */
         int getWave(unsigned long l_t);
 
         /**
          * @brief Computes bipolar Q15 output for current timestamp (Mainboard DSP Hot Path).
          * @param l_t Current timestamp in microseconds (micros()).
          * @return int16_t Bipolar output scaled to current amplitude [-32767, +32767].
          */
         int16_t getWaveQ15(unsigned long l_t);
 
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
 
         /**
          * @brief Advances phase accumulator and computes normalized unit shape in Q15.
          * @param l_t Current timestamp in microseconds.
          * @return int32_t Full-scale Q15 unit value [-32767, +32767].
          */
         int32_t         _advanceUnitQ15(unsigned long l_t);
 };
 
 #endif
 
 // -----------------------------------------------------
 // setMode1Rate table (mode 1 only):
 //
 // l_mode1_rate | lfo cycle duration
 // ---------------------------------
 //         .125 |   2 bars
 //         .25  |   1 bar
 //         .5   |   half note
 //         1    |   quarter note
 //         2    |   1/8 note
 //         3    |   1/12 note
 //         4    |   1/16 note
 //         5    |   1/20 note
 //         6    |   1/24 note
 //         7    |   1/28 note
 //         8    |   1/32 note
 //         9    |   1/36 note
 //        10    |   1/40 note
 //        11    |   1/44 note
 //        12    |   1/48 note
 //        13    |   1/52 note
 //        14    |   1/56 note
 //        15    |   1/60 note
 //        16    |   1/64 note
 //
 // Big THANKS to othmar52 for providing the table :-)