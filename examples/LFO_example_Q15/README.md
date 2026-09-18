# LFO_example_Q15 — bipolar Q15 output

Primary example for **`getWaveQ15()`**. Depth scaling is shown with a **local** helper (not part of mo-lfo).

## When to use this

- Synth engines / mod matrices
- Fixed-point pitch, filter, or PWM modulation
- Bipolar **±32767 ≈ ±1.0** (`int16_t`)

For DAC / PWM unipolar integers, use **`LFO_example_DAC`** (`getWave()`).

## Build

No special flag required. Upload like any Arduino sketch.

Optional: `-DMO_LFO_USE_Q15=1` only changes the library’s preferred-path `#pragma message`.

`MO_LFO_SRAM_HOT` defaults to **0**. On RP2040, `-DMO_LFO_SRAM_HOT=1` (or uncomment the define in the sketch) pins `getWaveQ15` / `_advanceUnitQ15` into SRAM. DCO shipping enables it; this example stays off for portability.

## What it does

1. 0.5 Hz LFO at full Q15 amplitude (`setAmplQ15(32767)`).
2. Each tick: `getWaveQ15(micros())`, then a sketch-local `(wave * depth) >> 15`.
3. Prints CSV: `waveform,lfo_q15,depth_mod_q24`.
4. Morphs saw → triangle → sine → square every 4 s (phase-aligned).

## API cheat sheet

| Call | Role |
|------|------|
| `setAmplQ15(0..32767)` | Preferred amplitude for synth |
| `getWaveQ15` | Bipolar `int16_t` sample |
| `applyDepthQ24_local` (sketch) | Example `(wave * depth) >> 15` — own this in your synth |
| `setAmpl` / `getWave` | Also available (DAC path); don’t mix getters in one tick |
