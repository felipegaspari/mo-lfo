// =============================================================================
// MO-LFO — DAC / unipolar example  (uses getWave(); Q15 API also always available)
// =============================================================================
//
// What this path is for
// ---------------------
// Drive a hardware DAC, PWM pin, or any control path that wants an integer
// in [0, dacSize-1]. Amplitude and DC offset are in the same units.
//
// Key API (DAC path)
// ------------------
//   lfo myLfo(dacSize);              // getWave() range [0, dacSize-1]
//   myLfo.setAmpl(dacSize - 1);      // peak-to-peak span in DAC counts
//   myLfo.setAmplOffset(0);          // DC center / floor (clamped so wave fits)
//   myLfo.setWaveForm(n);            // 0=off 1=saw 2=tri 3=sin 4=square
//   myLfo.setMode(false);            // free-running (Hz)
//   myLfo.setMode0Freq(2.0f);        // LFO rate in Hz
//   int sample = myLfo.getWave(micros());
//   // Also available: setAmplQ15 / getWaveQ15 — don't call both getters per tick.
//
// Waveforms are phase-aligned (sine-native: 0 at phase 0, rising). Switching
// shape with setWaveForm() does not reset phase — morphs stay continuous.
//
// BPM sync (not used below): setMode(true), setMode1Bpm(), setMode1Rate().
//
// Board notes
// -----------
// - Default DACSIZE is 256 (8-bit PWM friendly). Override before include logic
//   or edit the #ifndef below (e.g. 4096 for Arduino Due 12-bit DAC).
// - If DAC0 exists (Due), samples go to the DAC; otherwise Serial plotter.
//
// Prefer this sketch over the older LFO_example / LED / no_dac demos.
// =============================================================================

// #define MO_LFO_SRAM_HOT 1   // RP2040: pins Q15 engine only, not getWave() (library default 0)
#include <mo-lfo.h>

// Vertical resolution: must match your output path.
#ifndef DACSIZE
#if defined(DAC0)
#define DACSIZE 4096
#else
#define DACSIZE 256
#endif
#endif

// 1=saw, 2=triangle, 3=sine, 4=square (0=off / DC at offset)
static int g_waveform = 3;

static const unsigned long kWaveformPeriodUs = 3000000UL;  // morph every 3 s
static unsigned long g_lastMorphUs = 0;

lfo g_lfo(DACSIZE);

void setup()
{
  delay(50);

#if defined(DAC0)
  // Arduino Due (and similar): 12-bit DAC
  analogWriteResolution(12);
#else
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {
    // wait briefly for USB serial on native-USB boards
  }
  Serial.println(F("MO-LFO DAC example — Serial output (no DAC0 on this board)"));
  Serial.println(F("Open Serial Plotter for a live trace."));
#endif

  g_lfo.setWaveForm(g_waveform);
  g_lfo.setAmpl(DACSIZE - 1);     // full scale
  g_lfo.setAmplOffset(0);         // unipolar swing from low end / centered by library
  g_lfo.setMode(false);           // free-running
  g_lfo.setMode0Freq(2.0f);       // 2 Hz — easy to see on a scope or plotter

  g_lastMorphUs = micros();
}

void loop()
{
  const unsigned long t = micros();
  const int sample = g_lfo.getWave(t);   // always pass micros() (or a fake us clock)

#if defined(DAC0)
  analogWrite(DAC0, sample);
#else
  // Throttle Serial so the plotter stays readable
  static unsigned long lastPrintUs = 0;
  if ((unsigned long)(t - lastPrintUs) >= 2000UL) {  // ~500 Hz print rate
    lastPrintUs = t;
    Serial.println(sample);
  }
#endif

  // Cycle waveforms to demonstrate phase-aligned morphing
  if ((unsigned long)(t - g_lastMorphUs) >= kWaveformPeriodUs) {
    g_lastMorphUs = t;
    g_waveform++;
    if (g_waveform > 4) {
      g_waveform = 1;
    }
    g_lfo.setWaveForm(g_waveform);
#if !defined(DAC0)
    Serial.print(F("# waveform -> "));
    Serial.println(g_waveform);
#endif
  }
}
