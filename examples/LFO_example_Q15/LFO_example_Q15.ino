// =============================================================================
// MO-LFO — Q15 / bipolar synth example
// =============================================================================
//
// Uses getWaveQ15() + applyDepthQ24(). No special build flag required — the
// public API always includes Q15 methods. Optional: -DMO_LFO_USE_Q15=1 only
// changes the compile-time preferred-path message.
//
// Key API
// -------
//   #include <mo-lfo.h>
//   lfo myLfo(128);
//   myLfo.setAmplQ15(32767);
//   myLfo.setWaveForm(3);
//   myLfo.setMode(false);
//   myLfo.setMode0Freq(0.5f);
//   int16_t level = myLfo.getWaveQ15(micros());
//   int32_t mod_q24 = lfo::applyDepthQ24(level, depth_q24);
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
  const int32_t mod_q24 = lfo::applyDepthQ24(level_q15, kDemoDepthQ24);

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
