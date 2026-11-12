/**
 * @file readKY039.ino
 * @brief Heartbeat detector for the KY-039 finger sensor (IR LED + phototransistor)
 *        read through the UE0177. Prints beats per minute.
 *
 * Put a fingertip between the LED and the phototransistor and keep it still.
 * The blood volume changes the IR light that reaches the phototransistor with
 * every beat, which shows up as a tiny pulsation on top of a large DC level.
 * The sketch removes the DC level, finds the peaks and measures the time
 * between them.
 *
 * Example output:
 *   BPM=72  amplitude=14.2
 *   BPM=73  amplitude=14.0
 *   LPM:72   (beats in 15 s x 4)
 *
 * Tips: the pulse is tiny (a few ADC counts) next to slow waves from breathing
 * and finger pressure, so rest the hand on a table, touch the sensor lightly
 * without pressing (pressing stops the blood flow) and avoid moving. Shade the
 * sensor from strong ambient light. A real finger gives a few seconds of
 * "measuring..." before a stable BPM, and a sudden movement interrupts it for a
 * few seconds. Use PLOT_SIGNAL to see what the detector sees.
 *
 * This is an experimental demo, not a medical device.
 *
 * Compatible with ESP32, RP2040/RP2350, STM32 and AVR.
 *
 * @author Jonathan Mejorado
 * @organization UNIT Electronics MX
 */

#include <Arduino.h>
#include <Wire.h>
#include <DevLab_AnalogInterface.h>

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  #if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL) && \
      PIN_WIRE1_SDA == 24U && PIN_WIRE1_SCL == 25U
    // Pulsar connector pins.
    constexpr int SDA_PIN = 24;
    constexpr int SCL_PIN = 25;
  #else
    // Generic RP2040/RP2350 wiring.
    constexpr int SDA_PIN = 12;
    constexpr int SCL_PIN = 13;
  #endif
  // The RP2040/RP2350 I2C controller is fixed by the pin: SDA on GPIO 0, 4, 8,
  // ... 24 belongs to I2C0 (Wire) and GPIO 2, 6, 10, ... to I2C1 (Wire1).
  // Using the wrong object leaves the controller disconnected from the pins
  // and every transfer times out (GPIO 24/25 on the Pulsar are I2C0).
  #define I2C_BUS (((SDA_PIN / 2) % 2 == 0) ? Wire : Wire1)
#elif defined(ARDUINO_ARCH_ESP32)
  #define I2C_BUS Wire
  constexpr int SDA_PIN = 6;
  constexpr int SCL_PIN = 7;
#elif defined(ARDUINO_ARCH_STM32)
  // STM32duino: default I2C pins of the selected board.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
#elif defined(ARDUINO_ARCH_AVR)
  // AVR has fixed I2C pins (Uno/Nano: A4/A5, Mega: 20/21, Leonardo: 2/3);
  // SDA/SCL come from the board variant and begin() ignores the pin numbers.
  #define I2C_BUS Wire
  constexpr int SDA_PIN = SDA;
  constexpr int SCL_PIN = SCL;
#else
#error "Use an ESP32, RP2040, RP2350, STM32, or AVR master"
#endif

// The UE0177 firmware runs its I2C slave at 100 kHz.
constexpr uint32_t I2C_FREQ = 100000UL;

// ---------------------------------------------------------------------------
// Wiring: S -> SIGIN (JP2 pin 1), + -> VSYS, - -> GND (take VSYS and GND from
// J2, or from the Qwiic connector). Check the JP2 pinout on your board before
// plugging the 3-pin module straight into JP2: only pin 1 is SIGIN, and pin 3
// is the PB5 GPIO, not ground.
// ---------------------------------------------------------------------------

// Set true to print "signal:... threshold:..." at every sample for the Arduino
// Serial Plotter. Use it first to see the pulse and tune the values below.
constexpr bool PLOT_SIGNAL = false;

// Turn the signal upside down if the plot shows the pulses pointing down, or
// if the BPM comes out double (the detector locks on the second bump).
constexpr bool INVERT_SIGNAL = false;

// Blink an output on every beat (the LED of pin 13 in the original sketch).
// PB5 is JP2 pin 3: enable this only with an LED and resistor on PB5, and not
// with a sensor that uses that pin.
constexpr bool USE_BEAT_OUTPUT = false;
constexpr uint16_t BEAT_PULSE_MS = 80U;

constexpr uint16_t SAMPLE_MS = 20U;     // 50 samples per second
constexpr uint8_t OVERSAMPLE = 4U;      // readings averaged per sample
constexpr uint32_t STATUS_MS = 1000UL;  // live BPM line
constexpr uint32_t WINDOW_MS = 15000UL; // "LPM" like the original: beats in 15 s x 4

// ===== BeatDetector begin ==================================================
// Pure signal processing (no Arduino calls), so it can be tested on a PC.
constexpr float SMOOTH = 0.8f;             // EMA low-pass on the raw signal (the original used 0.75)
constexpr float BASELINE_ALPHA = 0.95f;   // two cascaded slow EMAs = second-order high-pass (~0.5 Hz)
constexpr float ENVELOPE_RELAX = 0.01f;    // per sample: the min/max envelope relaxes (~2 s)
constexpr float ENVELOPE_RISE = 0.03f;     // the envelope widens at most 3 % per sample (plus
constexpr float ENVELOPE_RISE_MIN = 0.5f;  //   ENVELOPE_RISE_MIN), so one jolt cannot blind it
constexpr float ARTIFACT_FACTOR = 3.0f;    // a swing this many times the pulse amplitude is a movement,
constexpr float ARTIFACT_FLOOR = 30.0f;    //   not a pulse (and larger than this many counts): restart
constexpr float THRESHOLD_FRACTION = 0.6f; // a beat is the signal crossing this point between min and max
constexpr float REARM_FRACTION = 0.4f;     // after a beat the signal must fall below this point
constexpr float MIN_AMPLITUDE = 4.0f;     // ADC counts, peak to peak; smaller is noise, not a pulse
constexpr uint16_t REFRACTORY_MS = 300U;   // no two beats closer than this (max 200 BPM)
constexpr uint16_t MAX_INTERVAL_MS = 2000U; // slower than 30 BPM: the finger moved, start again
constexpr uint8_t INTERVALS = 8U;          // beat intervals used for the BPM (median)
constexpr uint8_t MIN_INTERVALS = 3U;      // intervals needed before a BPM is reported

class BeatDetector {
 public:
  // Feed one sample (ADC counts) taken every SAMPLE_MS. Returns true on a beat.
  bool update(float sample, uint32_t nowMs) {
    if (!_started) {
      _smooth = sample;
      _baseline = sample;
      _baseline2 = 0.0f;
      _started = true;
    }
    _smooth = SMOOTH * _smooth + (1.0f - SMOOTH) * sample;
    // Second-order high-pass: it removes the DC level and also the slow waves
    // from breathing and finger pressure (0.1-0.3 Hz), which on a real finger
    // are several times larger than the pulse itself.
    _baseline = BASELINE_ALPHA * _baseline + (1.0f - BASELINE_ALPHA) * _smooth;
    float stage1 = _smooth - _baseline;
    _baseline2 = BASELINE_ALPHA * _baseline2 + (1.0f - BASELINE_ALPHA) * stage1;
    float ac = stage1 - _baseline2;   // the pulsation without the DC level

    // Placing or moving the finger shifts the signal by hundreds of counts,
    // far more than a pulse. Treat it as a movement: take the new level as the
    // baseline instead of letting the jolt inflate the envelope.
    float magnitude = ac < 0.0f ? -ac : ac;
    float reference = amplitude() > 2.0f * MIN_AMPLITUDE ? amplitude() : 2.0f * MIN_AMPLITUDE;
    if (magnitude > ARTIFACT_FACTOR * reference && magnitude > ARTIFACT_FLOOR) {
      _baseline = _smooth;
      _baseline2 = 0.0f;
      ac = 0.0f;
      _armed = true;
    }

    // Envelope of the signal: the recent maximum and minimum. Using both makes
    // the threshold independent of any offset left in the signal. They relax
    // slowly toward the signal and widen at a limited speed.
    _hi += (ac - _hi) * ENVELOPE_RELAX;
    _lo += (ac - _lo) * ENVELOPE_RELAX;
    float rise = (_hi - _lo) * ENVELOPE_RISE + ENVELOPE_RISE_MIN;
    if (ac > _hi) _hi = (ac - _hi > rise) ? _hi + rise : ac;
    if (ac < _lo) _lo = (_lo - ac > rise) ? _lo - rise : ac;
    _ac = ac;

    bool beat = false;
    if (amplitude() >= MIN_AMPLITUDE) {
      float level = threshold();
      float rearm = _lo + REARM_FRACTION * amplitude();
      if (!_armed && ac < rearm) _armed = true;
      if (_armed && _prevAc < level && ac >= level &&
          (!_hasBeat || (uint32_t)(nowMs - _lastBeatMs) >= REFRACTORY_MS)) {
        beat = true;
        _armed = false;
        registerBeat(nowMs);
      }
    }
    _prevAc = ac;
    return beat;
  }

  // True if a beat was seen recently enough to trust the numbers.
  bool pulseDetected(uint32_t nowMs) const {
    return _hasBeat && (uint32_t)(nowMs - _lastBeatMs) <= MAX_INTERVAL_MS;
  }

  // Beats per minute from the median of the last intervals, or 0 if there is
  // not enough data. The median ignores the odd missed or doubled beat, which
  // would skew a plain average (a missed beat doubles one interval).
  float bpm(uint32_t nowMs) const {
    if (!pulseDetected(nowMs) || _count < MIN_INTERVALS) return 0.0f;
    uint16_t sorted[INTERVALS];
    for (uint8_t i = 0U; i < _count; ++i) {
      uint16_t value = _intervals[i];
      uint8_t j = i;
      while (j > 0U && sorted[j - 1U] > value) {
        sorted[j] = sorted[j - 1U];
        --j;
      }
      sorted[j] = value;
    }
    float median = (_count % 2U == 1U)
        ? (float)sorted[_count / 2U]
        : 0.5f * ((float)sorted[_count / 2U - 1U] + (float)sorted[_count / 2U]);
    return 60000.0f / median;
  }

  float signal() const { return _ac; }
  float amplitude() const { return _hi - _lo; }   // recent peak-to-peak, in counts
  float threshold() const { return _lo + THRESHOLD_FRACTION * (_hi - _lo); }

 private:
  void registerBeat(uint32_t nowMs) {
    if (_hasBeat) {
      uint32_t interval = (uint32_t)(nowMs - _lastBeatMs);
      if (interval > MAX_INTERVAL_MS) {
        _count = 0U;  // a long gap: the finger was moved, restart the average
      } else {
        _intervals[_head] = (uint16_t)interval;
        _head = (uint8_t)((_head + 1U) % INTERVALS);
        if (_count < INTERVALS) ++_count;
      }
    }
    _lastBeatMs = nowMs;
    _hasBeat = true;
  }

  bool _started = false;
  bool _armed = true;
  bool _hasBeat = false;
  float _smooth = 0.0f;
  float _baseline = 0.0f;
  float _baseline2 = 0.0f;
  float _ac = 0.0f;
  float _prevAc = 0.0f;
  float _hi = 0.0f;
  float _lo = 0.0f;
  uint32_t _lastBeatMs = 0U;
  uint16_t _intervals[INTERVALS] = {0};
  uint8_t _count = 0U;
  uint8_t _head = 0U;
};
// ===== BeatDetector end ====================================================

DevLab_AnalogInterface analog(I2C_BUS, DevLab_AnalogInterface::DEFAULT_ADDRESS, I2C_FREQ);
BeatDetector detector;

uint32_t lastSampleMs = 0UL;
uint32_t lastStatusMs = 0UL;
uint32_t windowStartMs = 0UL;
uint32_t beatOutputOffMs = 0UL;
bool beatOutputOn = false;
uint16_t beatsInWindow = 0U;

void setup() {
  Serial.begin(115200);
  // USB boards (ESP32-S3, RP2040/RP2350): wait up to 3 s for the Serial Monitor.
  while (!Serial && millis() < 3000UL) delay(10);
  delay(200);

  // The optional beat output needs PB5 as an output; otherwise leave it alone.
  DevLab_AnalogGpioMode gpioMode = USE_BEAT_OUTPUT ? DevLab_AnalogGpioMode::OutputLow
                                                   : DevLab_AnalogGpioMode::Input;
  if (!analog.beginRecovered(SDA_PIN, SCL_PIN, gpioMode)) {
    Serial.println("UE0177 module not found or PB5 could not be configured");
    return;
  }
  if (!PLOT_SIGNAL) Serial.println("KY-039: place a fingertip on the sensor and keep it still...");
  windowStartMs = millis();
}

void loop() {
  if (!analog.isConnected()) return;

  uint32_t now = millis();
  if ((uint32_t)(now - lastSampleMs) < SAMPLE_MS) return;
  lastSampleMs = now;

  // Average a few readings per sample to reduce ADC noise.
  uint32_t sum = 0U;
  for (uint8_t i = 0U; i < OVERSAMPLE; ++i) {
    uint16_t raw = 0U;
    if (!analog.readRaw(raw)) {
      Serial.println("ERROR: read failed");
      delay(500U);
      return;
    }
    sum += raw;
  }
  float sample = sum / (float)OVERSAMPLE;
  if (INVERT_SIGNAL) sample = (float)DevLab_AnalogInterface::ADC_MAX - sample;

  if (detector.update(sample, now)) {
    ++beatsInWindow;
    if (USE_BEAT_OUTPUT) {
      analog.gpioWrite(true);
      beatOutputOn = true;
      beatOutputOffMs = now + BEAT_PULSE_MS;
    }
  }
  if (beatOutputOn && (int32_t)(now - beatOutputOffMs) >= 0) {
    analog.gpioWrite(false);
    beatOutputOn = false;
  }

  if (PLOT_SIGNAL) {
    Serial.print("signal:");
    Serial.print(detector.signal(), 1);
    Serial.print(" threshold:");
    Serial.println(detector.threshold(), 1);
    return;
  }

  if ((uint32_t)(now - lastStatusMs) >= STATUS_MS) {
    lastStatusMs = now;
    float bpm = detector.bpm(now);
    if (bpm > 0.0f) {
      Serial.print("BPM=");
      Serial.print(bpm, 0);
      Serial.print("  amplitude=");
      Serial.println(detector.amplitude(), 1);
    } else if (!detector.pulseDetected(now)) {
      Serial.println("no pulse: place a fingertip on the sensor and keep it still");
    } else {
      Serial.println("measuring...");
    }
  }

  // Same idea as the original sketch: beats counted in 15 s, times 4.
  if ((uint32_t)(now - windowStartMs) >= WINDOW_MS) {
    Serial.print("LPM:");
    Serial.print(beatsInWindow * 4U);
    Serial.println("   (beats in 15 s x 4)");
    beatsInWindow = 0U;
    windowStartMs = now;
  }
}
