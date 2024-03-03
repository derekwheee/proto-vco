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

Adafruit_DotStar strip(NUM_LEDS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

// Constants
#define ANALOG_HIGH 1023.0
#define DAC_HIGH 4095.0

// Define parameters
const int cvPin = A4;
const int tuningPin = A5;
const int dacPin = A0;
const int sawtoothPin = 7;
const int squarePin = 9;
const int trianglePin = 10;
const int sinePin = 11;
const float baseFrequency = 16.352;
const double voltageRange[2] = {0, ANALOG_HIGH};
const double tuningRange[2] = {0.5, 2};

// Variables
float sineTable[SAMPLE_RATE];
float phase = 0.0;
float phaseIncrement = 0.0;
float sawtoothValue = 0;
float squareValue = 0;
float sineValue = 0;
float triangleValue = 0;
int outputType = 0;

void setup()
{
    pinMode(sawtoothPin, INPUT_PULLDOWN);
    pinMode(squarePin, INPUT_PULLDOWN);
    pinMode(trianglePin, INPUT_PULLDOWN);
    pinMode(sinePin, INPUT_PULLDOWN);

    initSineTable();

    ITimer.attachInterruptInterval(TIMER_INTERVAL_US, updateVCO);

    strip.begin();
    strip.show();

    Serial.begin(9600);
}

void loop()
{
    uint32_t inputVoltage = analogRead(cvPin);
    uint32_t tuningVoltage = analogRead(tuningPin);

    float tunedFrequency = baseFrequency * scale(tuningVoltage, voltageRange, tuningRange);

    // Adjust frequency based on control voltage
    float controlVoltage = tunedFrequency * pow(2, inputVoltage * 6 / ANALOG_HIGH);
    phaseIncrement = controlVoltage / SAMPLE_RATE;

    if (digitalRead(sawtoothPin) == HIGH)
    {
        outputType = 0;
    }
    else if (digitalRead(squarePin) == HIGH)
    {
        outputType = 1;
    }
    else if (digitalRead(trianglePin) == HIGH)
    {
        outputType = 2;
    }
    else if (digitalRead(sinePin) == HIGH)
    {
        outputType = 3;
    }

    Serial.println(inputVoltage);
    Serial.println(tunedFrequency);
    Serial.println(controlVoltage);
    Serial.println(outputType);
    Serial.println("--------------------");

    strip.setPixelColor(0, strip.Color(0, 64 * tuningVoltage / ANALOG_HIGH, 64 * inputVoltage / ANALOG_HIGH));
    strip.show();

    delay(100);
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
    switch (outputType)
    {
    case 0: // Sawtooth
        sawtoothValue = phase * DAC_HIGH;
        analogWrite(dacPin, sawtoothValue);
        break;
    case 1: // Square
        squareValue = round(phase) * DAC_HIGH;
        analogWrite(dacPin, squareValue);
        break;
    case 2: // Triangle
        triangleValue = abs((2 * (phase - 0.5)) * DAC_HIGH);
        analogWrite(dacPin, triangleValue);
        break;
    case 3: // Sine
        sineValue = (approxSine(2 * PI * phase) + 1) * DAC_HIGH / 2;
        analogWrite(dacPin, sineValue);
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