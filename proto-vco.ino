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

#define CV_PIN A3
#define TUNING_PIN A4
#define DAC_PIN A0
#define OSC_1_DETUNE_PIN A5
#define OSC_1_WAVEFORM_UP_PIN 9
#define OSC_1_WAVEFORM_DOWN_PIN 10
#define OSC_1_OCTAVE_UP_PIN 11
#define OSC_1_OCTAVE_DOWN_PIN 7

Adafruit_DotStar strip(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Constants
#define ANALOG_HIGH 1023.0
#define DAC_HIGH 4095.0

// Define parameters
const float baseFrequency = 4.088;
const double voltageRange[2] = {0, ANALOG_HIGH};
const double tuningRange[2] = {0.5, 2};
const double detuneRange[2] = {0.1, 2};

// Variables
float sineTable[SAMPLE_RATE];
float phase = 0.0;
float phaseIncrement = 0.0;
float sawtoothValue = 0;
float squareValue = 0;
float sineValue = 0;
float triangleValue = 0;

int osc1Waveform = 0;
int osc1WaveformLastValue = 0;
int osc1Octave = 0;
int osc1OctaveLastValue = 0;

// Input voltage smoothing
float smoothingFactor = 0.1;
float smoothedControlVoltage = 0;
float smoothedTunedFrequency = 0;
float smoothedOsc1DetuneFactor = 1;

void setup()
{
    pinMode(OSC_1_WAVEFORM_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_WAVEFORM_DOWN_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_OCTAVE_UP_PIN, INPUT_PULLDOWN);
    pinMode(OSC_1_OCTAVE_DOWN_PIN, INPUT_PULLDOWN);

    initSineTable();

    ITimer.attachInterruptInterval(TIMER_INTERVAL_US, updateVCO);

    strip.begin();
    strip.show();

    Serial.begin(9600);
}

void loop()
{
    uint32_t inputVoltage = analogRead(CV_PIN);
    uint32_t tuningVoltage = analogRead(TUNING_PIN);
    uint32_t osc1Detune = analogRead(OSC_1_DETUNE_PIN);

    // Get smoothed tuning frequency
    float tunedFrequency = baseFrequency * scale(tuningVoltage, voltageRange, tuningRange);
    smoothedTunedFrequency = (smoothingFactor * tunedFrequency) + ((1 - smoothingFactor) * smoothedTunedFrequency);

    // Get smoothed oscillator detune factor
    float osc1DetuneFactor = scale(tuningVoltage, voltageRange, detuneRange);
    smoothedOsc1DetuneFactor = (smoothingFactor * osc1DetuneFactor) + ((1 - smoothingFactor) * smoothedOsc1DetuneFactor);

    // Get smoothed control voltage
    float controlVoltage = smoothedTunedFrequency * pow(2, inputVoltage * 3 / ANALOG_HIGH);
    smoothedControlVoltage = (smoothingFactor * controlVoltage) + ((1 - smoothingFactor) * smoothedControlVoltage);

    // Update waveform state if switch position has changed
    if (digitalRead(OSC_1_WAVEFORM_UP_PIN) == LOW && osc1WaveformLastValue == 1)
    {
        osc1Waveform = osc1Waveform >= 3 ? 3 : osc1Waveform + 1;
        osc1WaveformLastValue = 0;
    }
    else if (digitalRead(OSC_1_WAVEFORM_DOWN_PIN) == LOW && osc1WaveformLastValue == -1)
    {
        osc1Waveform = osc1Waveform <= 0 ? 0 : osc1Waveform - 1;
        osc1WaveformLastValue = 0;
    }
    else if (digitalRead(OSC_1_WAVEFORM_UP_PIN) == HIGH)
    {
        osc1WaveformLastValue = 1;
    }
    else if (digitalRead(OSC_1_WAVEFORM_DOWN_PIN) == HIGH)
    {
        osc1WaveformLastValue = -1;
    }

    // Update octave state if switch position has changed
    if (digitalRead(OSC_1_OCTAVE_UP_PIN) == LOW && osc1OctaveLastValue == 1)
    {
        osc1Octave = osc1Octave >= 3 ? 3 : osc1Octave + 1;
        osc1OctaveLastValue = 0;
    }
    else if (digitalRead(OSC_1_OCTAVE_DOWN_PIN) == LOW && osc1OctaveLastValue == -1)
    {
        osc1Octave = osc1Octave <= -3 ? -3 : osc1Octave - 1;
        osc1OctaveLastValue = 0;
    }
    else if (digitalRead(OSC_1_OCTAVE_UP_PIN) == HIGH)
    {
        osc1OctaveLastValue = 1;
    }
    else if (digitalRead(OSC_1_OCTAVE_DOWN_PIN) == HIGH)
    {
        osc1OctaveLastValue = -1;
    }

    phaseIncrement = (smoothedControlVoltage * smoothedOsc1DetuneFactor * pow(2, osc1Octave)) / SAMPLE_RATE;

    Serial.println(inputVoltage);
    Serial.println(smoothedTunedFrequency);
    Serial.println(smoothedControlVoltage);
    Serial.println(osc1Waveform);
    Serial.println(osc1Octave);
    Serial.println("--------------------");

    strip.setPixelColor(0, strip.Color(0, 64 * tuningVoltage / ANALOG_HIGH, 64 * inputVoltage / ANALOG_HIGH));
    strip.show();

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
    switch (osc1Waveform)
    {
    case 0: // Sawtooth
        sawtoothValue = phase * DAC_HIGH;
        analogWrite(DAC_PIN, sawtoothValue);
        break;
    case 1: // Square
        squareValue = round(phase) * DAC_HIGH;
        analogWrite(DAC_PIN, squareValue);
        break;
    case 2: // Triangle
        triangleValue = abs((2 * (phase - 0.5)) * DAC_HIGH);
        analogWrite(DAC_PIN, triangleValue);
        break;
    case 3: // Sine
        sineValue = (approxSine(2 * PI * phase) + 1) * DAC_HIGH / 2;
        analogWrite(DAC_PIN, sineValue);
        break;
    default:
        break;
    }

    // Update phase accumulator
    phase += phaseIncrement;
    if (phase >= 1.0)
    {
        phase -= 1.0;
    }
}