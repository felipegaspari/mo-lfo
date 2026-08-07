# MO-LFO – micros() based LFO for Arduino / RP2040 / Teensy / ESP32

Light-weight, sample-rate–agnostic LFO class based on `micros()`.  
Works well on AVR, ARM (e.g. RP2040 / Pico), Teensy, ESP32, etc.

The LFO can be **free-running** or **synced to a master clock (BPM-based)** and supports:

- Saw, triangle, sine (lookup-table + linear interpolation), square and DC (off)
- **Phase-aligned** waveshapes (sine-native: 0 at phase 0, rising) so live waveform switches do not jump phase
- Arbitrary DAC / control resolution, or bipolar **Q15** output for synth engines
- Time-based phase progression using a 32-bit phase accumulator

Original project and video by **mo-thunderz**.  
Video walkthrough: `https://youtu.be/ch03-75Fkuw`

---

## 1. Installation

1. Create a folder called `mo-lfo` in your Arduino libraries folder.
2. Copy `mo-lfo.cpp` and `mo-lfo.h` into that folder.
3. Restart the Arduino IDE. You can now `#include <mo-lfo.h>` in your sketches.

`LFO_SINE_TABLE_BITS` (if overridden) must be defined **before** including `mo-lfo.h` in every TU that sees the header.

---

## 2. Configuration

### 2.1 Public API is always dual (DAC + Q15)

Both output styles are **always** available — no compile flag required for symbols to exist:

| Method | Returns | Use for |
|--------|---------|---------|
| `getWave(micros())` | `int` in `[0, dacSize-1]` | DAC / PWM |
| `getWaveQ15(micros())` | `int16_t` ±32767 | Synth mod bus |
| `setAmpl(dacCounts)` | — | Amplitude in DAC counts (also updates Q15 cache) |
| `setAmplQ15(q15)` | — | Amplitude 0..32767 (also mirrors DAC counts) |
| `applyDepthQ24(wave, depth)` | `int32_t` | `(wave * depth) >> 15` |

Shared engine: one phase accumulator + unit Q15 shape; each getter only scales differently. Call **one** getter per tick (each advances phase).

Amplitude: **last setter wins** (`setAmpl` / `setAmplQ15` keep `_ampl` and `_ampl_q15` in sync). Hot path reads the cache — dual setters do not cost cycles on `getWave*`.

### 2.2 `MO_LFO_USE_Q15` — preferred-path hint only

| Value | Meaning |
|-------|---------|
| `0` (default) | Examples / DAC-oriented projects |
| `1` | Synth-oriented (e.g. DCO); documents intent |

This flag does **not** change the API. It only affects the compile-time `#pragma message`. Optional: `-DMO_LFO_USE_Q15=1` for clearer logs.

### 2.3 DAC / control range

```cpp
#include <mo-lfo.h>

constexpr int MOD_RANGE = 128;
lfo myLFO(MOD_RANGE);   // getWave() in [0, 127]
```

### 2.4 Sine table resolution

Sine uses a Q15 lookup table (±32767) with **integer linear interpolation**.

```cpp
#define LFO_SINE_TABLE_BITS 9   // before #include <mo-lfo.h>
#include <mo-lfo.h>
```

| Bits | Size | RAM |
|------|------|-----|
| 8 | 256 | 512 B |
| 9 | 512 | 1 KB (default) |
| 10 | 1024 | 2 KB |

---

## 3. Public API

### 3.1 Constructor

```cpp
lfo::lfo(int dacSize);  // getWave() range [0, dacSize-1]; also scales setAmpl bridge
```

### 3.2 Parameters

```cpp
void setWaveForm(int waveForm);     // 0=off, 1=saw, 2=tri, 3=sin, 4=square
void setAmpl(int ampl);             // DAC counts; syncs _ampl_q15
void setAmplQ15(int16_t ampl_q15);  // 0..32767; syncs _ampl
void setAmplOffset(int offset);     // getWave() DC only; ignored by getWaveQ15()
void setMode(bool mode);
void setMode0Freq(float freqHz);
void setMode1Bpm(float bpm);
void setMode1Rate(float rate);
void sync(unsigned long t);
```

### 3.3 Reading

```cpp
int getWave(unsigned long t);              // unipolar DAC int
int16_t getWaveQ15(unsigned long t);       // bipolar Q15
static int32_t applyDepthQ24(int16_t w, int32_t d);
```

Hot path is integer-only. Pass `micros()` (or any µs clock).

---

## 4. Examples

| Sketch | Start here if… |
|--------|----------------|
| [`examples/LFO_example_DAC`](examples/LFO_example_DAC/) | DAC/PWM/Serial via `getWave()` |
| [`examples/LFO_example_Q15`](examples/LFO_example_Q15/) | Synth mod via `getWaveQ15` + `applyDepthQ24` |
| `examples/LFO_example` / `_LED` / `_no_dac` | Legacy demos |

Each primary example has its own `README.md`.

---

## 5. Waveform phase alignment

All shapes share one phase reference (same as sine):

| Phase | Sine | Triangle | Saw | Square |
|-------|------|----------|-----|--------|
| 0 | 0, rising | 0, rising | 0, rising | +high (positive lobe) |
| 0.25 | +peak | +peak | mid-rise | +high |
| 0.5 | 0 | 0 | discontinuity (+ → −) | edge → low |
| 0.75 | −peak | −peak | mid-rise from − | −low |

`setWaveForm()` does not reset `_phase`; alignment comes from the shape maps themselves.

---

## 6. Performance notes

- Sine: one extra table load + mul vs nearest-neighbor; removes multi-ms zipper steps at slow LFO rates.
- Dual API (`getWave` + `getWaveQ15`) shares one engine; no per-sample flag branch.
- Full-scale Q15 amp skips the multiply.
- Frequency / BPM setters use float on the cold path only.

---

## 7. Notes and updates

- 29.12.2020: `_mode1_rate` changed from `int` to `float`.
- 16.07.2022: Saw and triangle UNO-compatible (thanks othmar52).
- 2024+: 32-bit phase accumulator + per-µs phase increments; configurable `LFO_SINE_TABLE_BITS`.
- 2026: Linear-interpolated sine; phase-aligned shapes; unified DAC+Q15 API; `MO_LFO_USE_Q15` is a preferred-path hint only.

Have fun :-)

Many thanks to mo-thunderz!
