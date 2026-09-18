// =============================================================================
// MO-LFO — Q15 / bipolar synth example
// =============================================================================
//
// Uses getWaveQ15(). No special build flag required — the public API always
// includes Q15 methods. Optional: -DMO_LFO_USE_Q15=1 only changes the
// compile-time preferred-path message.
// MO_LFO_SRAM_HOT defaults to 0. Uncomment below (or -DMO_LFO_SRAM_HOT=1) on
// RP2040 to pin getWaveQ15/_advanceUnitQ15 into SRAM.
//
// Depth scaling (wave × depth → Q24 mod) is a synth concern, not part of
// mo-lfo. This sketch keeps a local helper to show the usual pattern.
//
// Key API
// -------
//   // #define MO_LFO_SRAM_HOT 1   // RP2040 SRAM pin (library default 0)
#include <mo-lfo.h>
//   lfo myLfo(128);
//   myLfo.setAmplQ15(32767);
//   myLfo.setWaveForm(3);
//   myLfo.setMode(false);
//   myLfo.setMode0Freq(0.5f);
//   int16_t level = myLfo.getWaveQ15(micros());
//   int32_t mod_q24 = applyDepthQ24_local(level, depth_q24);
//
// setAmplOffset() affects getWave() only; getWaveQ15() is bipolar (no DC).
// Do not call getWave() and getWaveQ15() in the same tick (each advances phase).
// =============================================================================

#include <mo-lfo.h>

static lfo g_lfo(128);

static int g_waveform = 3;
static const unsigned long kWaveformPeriodUs = 4000000UL;
static unsigned long g_lastMorphUs = 0;

static const int32_t kDemoDepthQ24 = (int32_t)(1L << 20);

// Example-only: (wave_q15 * depth_q24) >> 15 — 32-bit split (no int64 mul).
static inline int32_t applyDepthQ24_local(int16_t wave_q15, int32_t depth_q24)
{
  const int32_t w = (int32_t)wave_q15;
  const int32_t hi = depth_q24 >> 15;
  const int32_t lo = depth_q24 - (hi << 15);
  return w * hi + ((w * lo) >> 15);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
  }

  Serial.println(F("MO-LFO Q15 example"));
  Serial.println(F("columns: waveform, lfo_q15, depth_mod_q24"));

  g_lfo.setWaveForm(g_waveform);
  g_lfo.setAmplQ15(MO_LFO_Q15_ONE);
  g_lfo.setMode(false);
  g_lfo.setMode0Freq(0.5f);

  g_lastMorphUs = micros();
}

void loop()
{
  const unsigned long t = micros();

  const int16_t level_q15 = g_lfo.getWaveQ15(t);
  const int32_t mod_q24 = applyDepthQ24_local(level_q15, kDemoDepthQ24);

  static unsigned long lastPrintUs = 0;
  if ((unsigned long)(t - lastPrintUs) >= 20000UL) {
    lastPrintUs = t;
    Serial.print(g_waveform);
    Serial.print(',');
    Serial.print(level_q15);
    Serial.print(',');
    Serial.println(mod_q24);
  }

  if ((unsigned long)(t - g_lastMorphUs) >= kWaveformPeriodUs) {
    g_lastMorphUs = t;
    g_waveform++;
    if (g_waveform > 4) {
      g_waveform = 1;
    }
    g_lfo.setWaveForm(g_waveform);
  }
}
