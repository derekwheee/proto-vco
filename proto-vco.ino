// TODO: OSC_1 detune pot isn't working, ground is disconnected
// TODO: Triangle oscillator has a high pitch ring and clips
// TODO: See if shortening the loop interval improves glide

#include "SAMDTimerInterrupt.h"
#include <Adafruit_DotStar.h>
#include "config.h"

SAMDTimer ITimer(SELECTED_TIMER);
Adafruit_DotStar pixel(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Constants
const int ANALOG_HIGH = 1023;
const int DAC_HIGH = 4095;

// Define parameters
const float baseFrequency = 32.704; // C1
const double voltageRange[2] = {0, ANALOG_HIGH};
const double tuningRange[2] = {0.5, 2};
const double detuneRange[2] = {0.5, 2};
const double glideRange[2] = {1, 0.01};
float sineTable[SAMPLE_RATE];

// Input voltage smoothing
float smoothingFactor = 0.2;
float smoothedGlideFactor = 0.1;
float smoothedControlFrequency = 0;
float smoothedTunedFrequency = 0;
float smoothedOsc1DetuneFactor = 1;
float smoothedOsc2DetuneFactor = 1;

enum Waveform
{
    SAWTOOTH,
    SQUARE,
    TRIANGLE,
    __COUNT // This allows us to get the length of the enum
};

struct Oscillator
{
    int outputPin;
    int detunePin;
    int waveformPins[2];
    int octavePins[2];
    float detuneFactor;
    Waveform waveform;
    int waveformLastValue;
    int octave;
    int octaveLastValue;
    float phase;
    float phaseIncrement;
};

Oscillator oscillators[2] = {
    // Oscillator 1
    {
        outputPin : DAC_1_PIN,
        detunePin : OSC_1_DETUNE_PIN,
        waveformPins : {OSC_1_WAVEFORM_UP_PIN, OSC_1_WAVEFORM_DOWN_PIN},
        octavePins : {OSC_1_OCTAVE_UP_PIN, OSC_1_OCTAVE_DOWN_PIN},
        detuneFactor : 1.0,
        waveform : SAWTOOTH,
        waveformLastValue : 0,
        octave : 0,
        octaveLastValue : 0,
        phase : 0.0,
        phaseIncrement : 0.0
    },
    // Oscillator 2
    {
        outputPin : DAC_2_PIN,
        detunePin : OSC_2_DETUNE_PIN,
        waveformPins : {OSC_2_WAVEFORM_UP_PIN, OSC_2_WAVEFORM_DOWN_PIN},
        octavePins : {OSC_2_OCTAVE_UP_PIN, OSC_2_OCTAVE_DOWN_PIN},
        detuneFactor : 1.0,
        waveform : SAWTOOTH,
        waveformLastValue : 0,
        octave : 0,
        octaveLastValue : 0,
        phase : 0.0,
        phaseIncrement : 0.0
    }};

int oscillatorCount = sizeof(oscillators) / sizeof(oscillators[0]);

unsigned long loopLastUpdateTime = 0;
const unsigned long loopUpdateInterval = 50;

void setup()
{
    for (int i = 0; i < oscillatorCount; i++)
    {
        pinMode(oscillators[i].waveformPins[0], INPUT_PULLDOWN);
        pinMode(oscillators[i].waveformPins[1], INPUT_PULLDOWN);
        pinMode(oscillators[i].octavePins[0], INPUT_PULLDOWN);
        pinMode(oscillators[i].octavePins[1], INPUT_PULLDOWN);
    }

    ITimer.attachInterruptInterval(TIMER_INTERVAL_US, updateVCO);

    pixel.begin();
    pixel.show();

    Serial.begin(115200);

    for (int i = 0; i < SAMPLE_RATE; i++)
    {
        sineTable[i] = sin(i / 6000);
    }
}

void loop()
{
    unsigned long now = millis();

    if (now - loopLastUpdateTime >= loopUpdateInterval)
    {
        // Record the last update time
        loopLastUpdateTime = now;

        uint32_t inputVoltage = analogRead(CV_PIN);
        uint32_t glideVoltage = analogRead(GLIDE_PIN);
        uint32_t tuningVoltage = analogRead(TUNING_PIN);
        uint32_t osc1Detune = analogRead(oscillators[0].detunePin);
        uint32_t osc2Detune = analogRead(oscillators[1].detunePin);

        handleWaveformChange();
        handleOctaveChange();

        // Get smoothed glide factor
        float glideValue = scale(glideVoltage, voltageRange, glideRange);
        smoothedGlideFactor = (smoothingFactor * glideValue) + ((1 - smoothingFactor) * smoothedGlideFactor);

        // Get smoothed tuning frequency
        float tunedFrequency = baseFrequency * scale(tuningVoltage, voltageRange, tuningRange);
        smoothedTunedFrequency = (smoothingFactor * tunedFrequency) + ((1 - smoothingFactor) * smoothedTunedFrequency);

        // Get smoothed control voltage
        float controlFrequency = smoothedTunedFrequency * pow(2, inputVoltage * 5 / ANALOG_HIGH);
        smoothedControlFrequency = (smoothedGlideFactor * controlFrequency) + ((1 - smoothedGlideFactor) * smoothedControlFrequency);

        // Get smoothed oscillator detune factor
        oscillators[0].detuneFactor = scale(osc1Detune, voltageRange, detuneRange);
        smoothedOsc1DetuneFactor = (smoothingFactor * oscillators[0].detuneFactor) + ((1 - smoothingFactor) * smoothedOsc1DetuneFactor);

        oscillators[1].detuneFactor = scale(osc2Detune, voltageRange, detuneRange);
        smoothedOsc2DetuneFactor = (smoothingFactor * oscillators[1].detuneFactor) + ((1 - smoothingFactor) * smoothedOsc2DetuneFactor);

        oscillators[0].phaseIncrement = (smoothedControlFrequency * smoothedOsc1DetuneFactor * pow(2, oscillators[0].octave)) / SAMPLE_RATE;
        oscillators[1].phaseIncrement = (smoothedControlFrequency * smoothedOsc2DetuneFactor * pow(2, oscillators[1].octave)) / SAMPLE_RATE;

        if (Serial)
        {
            String blank = "";
            String tab = "\t";
            Serial.println(blank + "FREQ" + tab + "DET" + tab + "WAVE" + tab + "OCT");
            Serial.println(blank + smoothedTunedFrequency + tab + oscillators[0].detuneFactor + tab + oscillators[0].waveform + tab + oscillators[0].octave);
            Serial.println(blank + smoothedGlideFactor + tab + oscillators[1].detuneFactor + tab + oscillators[1].waveform + tab + oscillators[1].octave);
            Serial.println("--------------------");
            Serial.println(oscillators[0].waveform == SAWTOOTH);
        }

        pixel.setPixelColor(0, pixel.Color(0, 64 * tuningVoltage / ANALOG_HIGH, 64 * inputVoltage / ANALOG_HIGH));
        pixel.show();
    }
}

float scale(float value, const double r1[2], const double r2[2])
{
    return (value - r1[0]) * (r2[1] - r2[0]) / (r1[1] - r1[0]) + r2[0];
}

void updateVCO()
{
    for (int i = 0; i < oscillatorCount; i++)
    {
        Oscillator &osc = oscillators[i];
        uint32_t output;

        switch (osc.waveform)
        {
        case SAWTOOTH:
            output = osc.phase * DAC_HIGH;
            break;
        case SQUARE:
            output = round(osc.phase) * DAC_HIGH;
            break;
        case TRIANGLE:
            output = abs((2 * (osc.phase - 0.5)) * DAC_HIGH);
            break;
            break;
        default:
            break;
        }

        analogWrite(osc.outputPin, output);

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
    for (int i = 0; i < oscillatorCount; i++)
    {
        Oscillator &osc = oscillators[i];
        int maxWaveformIndex = static_cast<Waveform>(__COUNT) - 1;
        int waveformIndex = static_cast<Waveform>(osc.waveform);

        if (digitalRead(osc.waveformPins[0]) == LOW && osc.waveformLastValue == 1)
        {
            osc.waveform = osc.waveform == static_cast<Waveform>(maxWaveformIndex) ? static_cast<Waveform>(maxWaveformIndex) : static_cast<Waveform>(waveformIndex + 1);
            osc.waveformLastValue = 0;
        }
        else if (digitalRead(osc.waveformPins[1]) == LOW && osc.waveformLastValue == -1)
        {
            osc.waveform = osc.waveform == static_cast<Waveform>(0) ? static_cast<Waveform>(0) : static_cast<Waveform>(waveformIndex - 1);
            osc.waveformLastValue = 0;
        }
        else if (digitalRead(osc.waveformPins[0]) == HIGH)
        {
            osc.waveformLastValue = 1;
        }
        else if (digitalRead(osc.waveformPins[1]) == HIGH)
        {
            osc.waveformLastValue = -1;
        }
    }
}
void handleOctaveChange()
{
    for (int i = 0; i < oscillatorCount; i++)
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