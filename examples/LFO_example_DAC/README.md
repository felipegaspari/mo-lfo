# LFO_example_DAC — unipolar / DAC mode

Primary example for **`getWave()`** (unipolar DAC int). Q15 methods also exist; this sketch uses the DAC path.

## When to use this

- Hardware DAC (`DAC0` on Arduino Due, etc.)
- PWM / LED brightness
- Any host that wants an integer in `[0, dacSize - 1]`

For synth modulation buses (bipolar ±1.0 as Q15), use **`LFO_example_Q15`**.

## What it does

1. Builds an LFO at 2 Hz, full amplitude.
2. Writes each sample with `getWave(micros())`.
3. Every 3 seconds cycles saw → triangle → sine → square (phase-aligned morph).
4. If the board has `DAC0`, samples go to the DAC; otherwise values are printed for the Serial Plotter.

## How to run

1. Install the `mo-lfo` library (see main README).
2. Open this sketch in the Arduino IDE / `arduino-cli`.
3. Select your board and upload — **no extra build flags** required.
4. Without a DAC: open **Serial Plotter** at 115200 baud.

### Optional: 12-bit Due-style range on other boards

Edit the sketch (or define before compile):

```cpp
#define DACSIZE 4096
```

Match `analogWriteResolution()` / PWM range to the same size.

## API cheat sheet (DAC mode)

| Call | Role |
|------|------|
| `lfo(dacSize)` | Output range `[0, dacSize-1]` |
| `setAmpl` / `setAmplOffset` | Span and DC in DAC counts |
| `setWaveForm(1..4)` | Saw / tri / sin / square |
| `setMode(false)` + `setMode0Freq(Hz)` | Free-running |
| `getWave(micros())` | Next sample (`int`) |

See also: `LFO_example_Q15` for bipolar Q15 + `applyDepthQ24`.
