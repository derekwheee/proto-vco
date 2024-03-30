#include "SAMDTimerInterrupt.h"
#include <Adafruit_DotStar.h>

// Timer config
#define SAMPLE_RATE 6000
#define USING_TIMER_TC3 true
#define SELECTED_TIMER TIMER_TC3
#define TIMER_INTERVAL_US 100000 / SAMPLE_RATE
#define PI_STEP (2 * PI / SAMPLE_RATE)

SAMDTimer ITimer(SELECTED_TIMER);

// DotStar config
#define NUM_LEDS 1
#define DATAPIN 8
#define CLOCKPIN 6

#define GLIDE_PIN A2
#define CV_PIN A3
#define TUNING_PIN A4
#define DAC_1_PIN A0
#define DAC_2_PIN A1
#define OSC_1_DETUNE_PIN A5
#define OSC_1_WAVEFORM_UP_PIN 22
#define OSC_1_WAVEFORM_DOWN_PIN 21
#define OSC_1_OCTAVE_UP_PIN 9
#define OSC_1_OCTAVE_DOWN_PIN 7
#define OSC_2_DETUNE_PIN A6
#define OSC_2_WAVEFORM_UP_PIN 11
#define OSC_2_WAVEFORM_DOWN_PIN 10
#define OSC_2_OCTAVE_UP_PIN 13
#define OSC_2_OCTAVE_DOWN_PIN 12

Adafruit_DotStar strip(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Constants
#define ANALOG_HIGH 1023.0
#define DAC_HIGH 4095.0

// Define parameters
const float baseFrequency = 16.352;
const double voltageRange[2] = {0, ANALOG_HIGH};
const double tuningRange[2] = {0.5, 2};
const double detuneRange[2] = {0.1, 2};
const double glideRange[2] = {0.01, 1};

// Variables
float sineTable[SAMPLE_RATE];

// Input voltage smoothing
float smoothingFactor = 0.1;
float smoothedGlideFactor = 0.1;
float smoothedControlVoltage = 0;
float smoothedTunedFrequency = 0;
float smoothedOsc1DetuneFactor = 1;
float smoothedOsc2DetuneFactor = 1;

struct Oscillator
{
    int outputPin;
    int detunePin;
    int waveformPins[2];
    int octavePins[2];
    float detuneFactor;
    int waveform;
    int waveformLastValue;
    int octave;
    int octaveLastValue;
    float phase;
    float phaseIncrement;
};

Oscillator oscillators[2] = {
    // Oscillator 1
    {
        outputPin : DAC_2_PIN,
        detunePin : OSC_1_DETUNE_PIN,
        waveformPins : {OSC_1_WAVEFORM_UP_PIN, OSC_1_WAVEFORM_DOWN_PIN},
        octavePins : {OSC_1_OCTAVE_UP_PIN, OSC_1_OCTAVE_DOWN_PIN},
        detuneFactor : 1.0,
        waveform : 0,
        waveformLastValue : 0,
        octave : 0,
        octaveLastValue : 0,
        phase : 0.0,
        phaseIncrement : 0.0
    },
    // Oscillator 2
    {
        outputPin : DAC_1_PIN,
        detunePin : OSC_2_DETUNE_PIN,
        waveformPins : {OSC_2_WAVEFORM_UP_PIN, OSC_2_WAVEFORM_DOWN_PIN},
        octavePins : {OSC_2_OCTAVE_UP_PIN, OSC_2_OCTAVE_DOWN_PIN},
        detuneFactor : 1.0,
        waveform : 0,
        waveformLastValue : 0,
        octave : 0,
        octaveLastValue : 0,
        phase : 0.0,
        phaseIncrement : 0.0
    }};

void setup()
{
    pinMode(OSC_1_WAVEFORM_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_WAVEFORM_DOWN_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_OCTAVE_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_OCTAVE_DOWN_PIN, INPUT_PULLDOWN);
    pinMode(OSC_2_WAVEFORM_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_2_WAVEFORM_DOWN_PIN, INPUT_PULLDOWN);
    pinMode(OSC_2_OCTAVE_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_2_OCTAVE_DOWN_PIN, INPUT_PULLDOWN);

    initSineTable();

    ITimer.attachInterruptInterval(TIMER_INTERVAL_US, updateVCO);

    strip.begin();
    strip.show();

    Serial.begin(9600);
}

void loop()
{
    uint32_t inputVoltage = analogRead(CV_PIN);
    uint32_t glideVoltage = analogRead(GLIDE_PIN);
    uint32_t tuningVoltage = analogRead(TUNING_PIN);
    uint32_t osc1Detune = analogRead(oscillators[0].detunePin);
    uint32_t osc2Detune = analogRead(oscillators[1].detunePin);

    // Get smoothed glide factor
    float glideValue = scale(glideVoltage, voltageRange, glideRange);
    smoothedGlideFactor = (smoothingFactor * glideValue) + ((1 - smoothingFactor) * smoothedGlideFactor);

    // Get smoothed tuning frequency
    float tunedFrequency = baseFrequency * scale(tuningVoltage, voltageRange, tuningRange);
    smoothedTunedFrequency = (smoothingFactor * tunedFrequency) + ((1 - smoothingFactor) * smoothedTunedFrequency);

    // Get smoothed oscillator detune factor
    oscillators[0].detuneFactor = scale(osc1Detune, voltageRange, detuneRange);
    smoothedOsc1DetuneFactor = (smoothingFactor * oscillators[0].detuneFactor) + ((1 - smoothingFactor) * smoothedOsc1DetuneFactor);

    oscillators[1].detuneFactor = scale(osc2Detune, voltageRange, detuneRange);
    smoothedOsc2DetuneFactor = (smoothingFactor * oscillators[1].detuneFactor) + ((1 - smoothingFactor) * smoothedOsc2DetuneFactor);

    // Get smoothed control voltage
    float controlVoltage = smoothedTunedFrequency * pow(2, inputVoltage * 3 / ANALOG_HIGH);
    smoothedControlVoltage = (smoothedGlideFactor * controlVoltage) + ((1 - smoothedGlideFactor) * smoothedControlVoltage);

    handleWaveformChange();
    handleOctaveChange();

    oscillators[0].phaseIncrement = (smoothedControlVoltage * smoothedOsc1DetuneFactor * pow(2, oscillators[0].octave)) / SAMPLE_RATE;
    oscillators[1].phaseIncrement = (smoothedControlVoltage * smoothedOsc2DetuneFactor * pow(2, oscillators[1].octave)) / SAMPLE_RATE;

    Serial.println(smoothedTunedFrequency);
    Serial.println(smoothedControlVoltage);
    Serial.println(oscillators[0].waveform);
    Serial.println(oscillators[0].octave);
    Serial.println(oscillators[1].waveform);
    Serial.println(oscillators[1].octave);
    Serial.println(oscillators[0].detuneFactor);
    Serial.println(oscillators[1].detuneFactor);
    Serial.println("--------------------");

    strip.setPixelColor(0, strip.Color(0, 64 * tuningVoltage / ANALOG_HIGH, 64 * inputVoltage / ANALOG_HIGH));
    strip.show();

    // TODO: Eliminate this delay and use millis() instead
    delay(10);
}

float scale(float value, const double r1[2], const double r2[2])
{
    return (value - r1[0]) * (r2[1] - r2[0]) / (r1[1] - r1[0]) + r2[0];
}

void initSineTable()
{
    for (int i = 0; i < SAMPLE_RATE; i++)
    {
        sineTable[i] = sin(i * PI_STEP);
    }
}

float approxSine(float x)
{
    // Map x to the range [0, 2*PI]
    x = fmod(x, 2 * PI);
    if (x < 0)
    {
        x += 2 * PI;
    }

    // Interpolate between table entries
    int index = x / PI_STEP;
    float alpha = x / PI_STEP - index;
    float y0 = sineTable[index];
    float y1 = sineTable[(index + 1) % SAMPLE_RATE];
    return (1 - alpha) * y0 + alpha * y1;
}

void updateVCO()
{
    for (int i = 0; i < 2; i++)
    {
        Oscillator &osc = oscillators[i];
        uint32_t output;

        switch (osc.waveform)
        {
        case 0: // Sawtooth
            output = osc.phase * DAC_HIGH;
            break;
        case 1: // Square
            output = round(osc.phase) * DAC_HIGH;
            break;
        case 2: // Triangle
            output = abs((2 * (osc.phase - 0.5)) * DAC_HIGH);
            break;
        // TODO: Sine wave is currently causing problems        
        // case 3: // Sine
        //     output = (approxSine(2 * PI * osc.phase) + 1) * DAC_HIGH / 2;
        //     break;
        default:
            break;
        }

        analogWrite(osc.outputPin, osc.phase * DAC_HIGH);

        // Update phase accumulator
        osc.phase += osc.phaseIncrement;
        if (osc.phase >= 1.0)
        {
            osc.phase -= 1.0;
        }
    }
}

void handleWaveformChange()
{
    for (int i = 0; i < 2; i++)
    {
        Oscillator &osc = oscillators[i];

        if (digitalRead(oscillators[i].waveformPins[0]) == LOW && oscillators[i].waveformLastValue == 1)
        {
            oscillators[i].waveform = oscillators[i].waveform >= 2 ? 2 : oscillators[i].waveform + 1;
            oscillators[i].waveformLastValue = 0;
        }
        else if (digitalRead(oscillators[i].waveformPins[1]) == LOW && oscillators[i].waveformLastValue == -1)
        {
            oscillators[i].waveform = oscillators[i].waveform <= 0 ? 0 : oscillators[i].waveform - 1;
            oscillators[i].waveformLastValue = 0;
        }
        else if (digitalRead(oscillators[i].waveformPins[0]) == HIGH)
        {
            oscillators[i].waveformLastValue = 1;
        }
        else if (digitalRead(oscillators[i].waveformPins[1]) == HIGH)
        {
            oscillators[i].waveformLastValue = -1;
        }
    }
}
void handleOctaveChange()
{
    for (int i = 0; i < 2; i++)
    {
        Oscillator &osc = oscillators[i];

        if (digitalRead(osc.octavePins[0]) == LOW && osc.octaveLastValue == 1)
        {
            osc.octave = osc.octave >= 3 ? 3 : osc.octave + 1;
            osc.octaveLastValue = 0;
        }
        else if (digitalRead(osc.octavePins[1]) == LOW && osc.octaveLastValue == -1)
        {
            osc.octave = osc.octave <= -3 ? -3 : osc.octave - 1;
            osc.octaveLastValue = 0;
        }
        else if (digitalRead(osc.octavePins[0]) == HIGH)
        {
            osc.octaveLastValue = 1;
        }
        else if (digitalRead(osc.octavePins[1]) == HIGH)
        {
            osc.octaveLastValue = -1;
        }
    }
}