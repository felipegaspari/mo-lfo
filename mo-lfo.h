/**
 * @file mo-lfo.h
 * @brief High-performance LFO engine for Arduino / Embedded Synthesizers.
 * @author mo-thunderz
 * @author felipegaspari (modified & expanded)
 * @version 1.5
 *
 * @details Provides unipolar DAC (integer) and bipolar Q15 fixed-point LFO outputs.
 * Features 10 distinct waveforms, including analog circuit emulations (slewed, 
 * asymmetrical, wavefolded) and tempo-synced (BPM/Rate) modes.
 */

 #ifndef mo_lfo_h
 #define mo_lfo_h
 
 #include "Arduino.h"
 #include <stdint.h>
 
 // -------------------------------------------------
 // Configuration macros
 // -------------------------------------------------
 
 /** @def LFO_SINE_TABLE_BITS
  *  @brief Sine lookup table resolution in bits: log2(table_size). 
  *  Default = 9 (512 entries).
  */
 #ifndef LFO_SINE_TABLE_BITS
 #define LFO_SINE_TABLE_BITS 9
 #endif
 
 /** @def LFO_SINE_TABLE_SIZE
  *  @brief Total entries in the sine lookup table.
  */
 #define LFO_SINE_TABLE_SIZE (1u << LFO_SINE_TABLE_BITS)
 
 /** @def LFO_SINE_FRAC_BITS
  *  @brief Fractional interpolation bits remaining from 16-bit phase ramp.
  */
 #define LFO_SINE_FRAC_BITS  (16 - LFO_SINE_TABLE_BITS)
 
 /** @def MO_LFO_USE_Q15
  *  @brief Preferred compilation path hint (0 = DAC int default, 1 = Q15 engine).
  */
 #ifndef MO_LFO_USE_Q15
 #define MO_LFO_USE_Q15 0
 #endif
 
 /** @def MO_LFO_SRAM_HOT
  *  @brief Places critical DSP functions into fast SRAM (.time_critical) on supported MCUs.
  */
 #ifndef MO_LFO_SRAM_HOT
 #define MO_LFO_SRAM_HOT 0
 #endif
 
 #if MO_LFO_SRAM_HOT
 #ifndef __not_in_flash_func
 #define __not_in_flash_func(fn) fn
 #endif
 #define MO_LFO_HOT(fn) __not_in_flash_func(fn)
 #else
 #define MO_LFO_HOT(fn) fn
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
 
 /** @brief Full scale value in Q15 integer representation (±32767 ≈ ±1.0). */
 static const int16_t MO_LFO_Q15_ONE = 32767;
 
 /**
  * @class lfo
  * @brief Multi-waveform Low Frequency Oscillator class with dual DAC/Q15 output paths.
  */
 class lfo
 {
     public:
         /**
          * @brief Constructs an LFO instance.
          * @param dacSize Vertical integer range for unipolar getWave() -> [0, dacSize-1].
          */
         lfo(int dacSize);
 
         /**
          * @brief Sets the LFO amplitude using DAC count units.
          * @param l_ampl Peak-to-peak amplitude in DAC steps (0 .. dacSize-1).
          * @note Automatically updates internal Q15 amplitude representation.
          */
         void setAmpl(int l_ampl);
 
         /**
          * @brief Sets the LFO amplitude using standard Q15 scaling.
          * @param l_ampl_q15 Amplitude from 0 (off) to 32767 (100% full scale).
          * @note Automatically mirrors to DAC integer amplitude.
          */
         void setAmplQ15(int16_t l_ampl_q15);
 
         /**
          * @brief Retrieves the cached Q15 amplitude.
          * @return Amplitude value (0 .. 32767).
          */
         int16_t getAmplQ15() const { return _ampl_q15; }
 
         /**
          * @brief Sets the unipolar DC offset (for getWave() DAC output only).
          * @param l_ampl_offset Offset value in DAC counts.
          */
         void setAmplOffset(int l_ampl_offset);
 
         /**
          * @brief Selects the active LFO waveform.
          * @param l_waveForm Waveform identifier index:
          *   - 0: Off (Output 0)
          *   - 1: Saw (Linear Falling Ramp)
          *   - 2: Triangle- slewed (Slewed, Asymmetric, Soft-Clipped)
          *   - 3: Sine (Pure Lookup + Interpolation)
          *   - 4: Square (Phase-aligned 50% Duty Cycle)
          *   - 5: Analog Sharktooth (90% Rise, 10% Fall Asymmetric Saw)
          *   - 6: Analog Trapezoid (Slewed / Overdriven Square)
          *   - 7: Linear Triangle (Pure Mathematical Triangle)
          *   - 8: Stepped Staircase (Quantized Triangle, 8 discrete steps)
          *   - 9: Wavefolded Sine (Buchla-style peak folding)
          */
         void setWaveForm(int l_waveForm);
 
         /**
          * @brief Sets the synchronization mode.
          * @param l_freq_sync False = Free-running (Hz), True = Tempo-synced (BPM/Rate).
          */
         void setMode(bool l_freq_sync);
 
         /**
          * @brief Sets the free-running frequency in Hertz.
          * @param l_mode0_freq Frequency in Hz.
          */
         void setMode0Freq(float l_mode0_freq);
 
         /**
          * @brief Sets the free-running frequency with a timestamp.
          * @param l_mode0_freq Frequency in Hz.
          * @param l_t Current timestamp in microseconds.
          */
         void setMode0Freq(float l_mode0_freq, unsigned long l_t);
 
         /**
          * @brief Sets the tempo for synced mode.
          * @param l_mode1_bpm Tempo in Beats Per Minute (BPM).
          */
         void setMode1Bpm(float l_mode1_bpm);
 
         /**
          * @brief Sets the rhythmic beat division multiplier for synced mode.
          * @param l_mode1_rate Subdivision factor (e.g., 1.0 = quarter note, 0.25 = 1 bar).
          */
         void setMode1Rate(float l_mode1_rate);
 
         /**
          * @brief Sets the phase offset for synced mode.
          * @param l_mode1_phase_offset Phase offset in normalized radians/ratio.
          */
         void setMode1Phase(float l_mode1_phase_offset);
 
         /**
          * @brief Resets LFO phase accumulator to zero.
          * @param l_t Current timestamp in microseconds.
          */
         void sync(unsigned long l_t);
 
         /** @brief Gets the current waveform ID (0..9). */
         int getWaveForm();
 
         /** @brief Gets the current amplitude in DAC counts. */
         int getAmpl();
 
         /** @brief Gets the current DC offset in DAC counts. */
         int getAmplOffset();
 
         /** @brief Gets the sync mode (false = free, true = sync). */
         bool getMode();
 
         /** @brief Gets the free-running frequency in Hz. */
         float getMode0Freq();
 
         /** @brief Gets the synced mode subdivision rate. */
         float getMode1Rate();
 
         /** @brief Gets the normalized phase [0.0 .. 1.0). */
         float getPhase();
 
         /**
          * @brief Generates the next sample formatted for unipolar DACs.
          * @param l_t Current timestamp in microseconds.
          * @return Integer DAC code in the range [0, dacSize-1].
          */
         int getWave(unsigned long l_t);
 
         /**
          * @brief Generates the next sample formatted for bipolar Q15 fixed-point synth engines.
          * @param l_t Current timestamp in microseconds.
          * @return Bipolar Q15 signed value in the range [-32767, +32767].
          */
         int16_t getWaveQ15(unsigned long l_t);
 
     private:
         int             _dacSize;          /**< Unipolar DAC resolution limit. */
         int             _waveForm = 1;     /**< Currently selected waveform index. */
         int             _ampl = 0;         /**< DAC-scale amplitude. */
         int             _ampl_offset = 0;  /**< DAC-scale DC offset. */
         int16_t         _ampl_q15 = MO_LFO_Q15_ONE; /**< Q15-scale amplitude [0..32767]. */
         bool            _mode = 0;         /**< 0 = Free-running, 1 = BPM synced. */
         float           _mode0_freq = 30;  /**< Free frequency in Hz. */
         float           _mode1_bpm = 120;  /**< Sync tempo in BPM. */
         float           _mode1_rate = 1;   /**< Sync subdivision multiplier. */
 
         uint32_t        _phase = 0;          /**< 32-bit phase accumulator. */
         uint32_t        _phase_inc_free = 0; /**< Phase increment per microsecond in free mode. */
         uint32_t        _phase_inc_sync = 0; /**< Phase increment per microsecond in sync mode. */
         unsigned long   _t_last = 0;         /**< Timestamp of previous calculation in microseconds. */
         bool            _initialized = false;/**< Initialization flag. */
 
         /** @brief Computes phase increment for free-running mode. */
         void            _updatePhaseIncFree();
 
         /** @brief Computes phase increment for BPM sync mode. */
         void            _updatePhaseIncSync();
 
         /** @brief Synchronizes Q15 amplitude from current DAC amplitude value. */
         void            _updateAmplQ15FromDac();
 
         /**
          * @brief Advances phase accumulator and computes normalized unit shape.
          * @param l_t Current timestamp in microseconds.
          * @return Normalized Q15 unit wave shape in range [-32767, +32767].
          */
         int32_t         _advanceUnitQ15(unsigned long l_t);
 };
 
 #endif // mo_lfo_h