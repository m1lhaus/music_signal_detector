#include "Arduino.h"

#define DEBUG_PLOT 1
#define DEBUG_PRINT 1

// Constants
constexpr int SILENCE_VAR_THRESHOLD = 5;   // ADC units of variation for signal to be considered silence
constexpr float SILENCE_ADAPT_RATE = 0.05; // How quickly to adapt (0-1) the silence level
constexpr int SILENCE_BUFFER_SIZE = 100;   // Number of samples to take for silence detection

constexpr int SAMPLE_EVERY_MS = 10; // How often to sample / run the loop (ms)

constexpr float DIFF_ADAPT_RATE = 0.1;
constexpr float DIFF_THRESHOLD = SILENCE_VAR_THRESHOLD;

constexpr uint32_t BOOT_DELAY_MS = (uint32_t)1U * 1000U;      // delay before we start the music detection
constexpr uint32_t TURN_ON_DELAY_MS = (uint32_t)1U * 1000U;   // min length of active signal to be considered as music (in ms)
constexpr uint32_t TURN_OFF_DELAY_MS = (uint32_t)30U * 1000U; // delay before we turn off the power

constexpr int MUSIC_ON_LED_PIN = 2;  // Pin for status LED
constexpr int MUSIC_OFF_LED_PIN = 3; // Pin for status LED

class RingBuffer
{
private:
    int m_buffer[SILENCE_BUFFER_SIZE];

    uint32_t m_head;
    uint32_t m_count;

public:
    RingBuffer() : m_head(0), m_count(0)
    {
        memset(m_buffer, 0U, sizeof(m_buffer));
    }

    void push(uint16_t value)
    {
        m_buffer[m_head] = value;
        m_head = (m_head + 1U) % SILENCE_BUFFER_SIZE;
        if (m_count < SILENCE_BUFFER_SIZE)
        {
            m_count++;
        }
    }

    const int *getPtr() const
    {
        return m_buffer;
    }

    uint32_t size() const
    {
        return m_count;
    }

    bool isFull() const
    {
        return m_count == SILENCE_BUFFER_SIZE;
    }
} g_RingBuffer;

// Variables
float g_silenceLevel = 512; // Starting point (~2.5V equivalent)
float g_signalCumSum = 0.0f;

// Global variables for timing
unsigned long g_bootTime = 0;
unsigned long g_musicStartTime = 0;
unsigned long g_silenceStartTime = 0;
bool g_musicActive = true;

float toVoltage(const int value)
{
    return (value / 1024.0) * 5; // Convert ADC value to voltage
}

/// Detects the initial silence level by analyzing analog signal samples and calculating the average.
void initialSilenceLevelDetection()
{
    // Read signal and calculate short-term variation
    uint32_t signalSum = 0;
    int minVal = 1023;
    int maxVal = 0;

    while (!g_RingBuffer.isFull())
    {
        int sample = analogRead(A0);
        g_RingBuffer.push(sample);

        signalSum += sample;
        if (sample < minVal)
            minVal = sample;
        if (sample > maxVal)
            maxVal = sample;
    }

    // update default value only if the variation is below the threshold since a music might be already playing
    const int currentVariation = maxVal - minVal;
    if (currentVariation < SILENCE_VAR_THRESHOLD)
    {
        float signalAvg = signalSum / g_RingBuffer.size();
        g_silenceLevel = signalAvg;
    }
#ifdef DEBUG_PRINT
    Serial.print("Initial silence level: ");
    Serial.print(g_silenceLevel);
    Serial.print(" (");
    Serial.print(toVoltage(g_silenceLevel));
    Serial.print(" V)\n");
#endif
}

/// Adjusts silence level based on short-term signal variation using a weighted average if variation is below threshold.
void adjustSilenceLevel()
{
    uint32_t signalSum = 0;
    int minVal = 1023;
    int maxVal = 0;

    const int *bufferData = g_RingBuffer.getPtr();
    for (size_t i = 0; i < g_RingBuffer.size(); i++)
    {
        int sample = bufferData[i];
        signalSum += sample;
        if (sample < minVal)
            minVal = sample;
        if (sample > maxVal)
            maxVal = sample;
    }

    // If we detect silence, slowly adjust our silence level
    const int currentVariation = maxVal - minVal;
    if (currentVariation < SILENCE_VAR_THRESHOLD)
    {
        float signalAvg = signalSum / g_RingBuffer.size();
        g_silenceLevel = g_silenceLevel * (1.0f - SILENCE_ADAPT_RATE) + signalAvg * SILENCE_ADAPT_RATE;
    }
}

void detectMusic()
{
    unsigned long currentTime = millis();

    // is some music signal detected?
    if (g_signalCumSum > DIFF_THRESHOLD)
    {
        g_silenceStartTime = 0U; // turn off silence timer

        if (!g_musicActive)
        {
            if (g_musicStartTime == 0)
            {
                g_musicStartTime = currentTime; // start measuring music time
            }
            else if (currentTime - g_musicStartTime > TURN_ON_DELAY_MS)
            {
                g_musicActive = true; // Music detected for long enough
#ifdef DEBUG_PRINT
                Serial.println("Music detected");
#endif
            }
        }
    }
    // no, just silence detected
    else
    {
        g_musicStartTime = 0; // turn off music timer

        if (g_musicActive)
        {
            if (g_silenceStartTime == 0U)
            {
                g_silenceStartTime = currentTime; // start measuring silence time
            }
            else if (currentTime - g_silenceStartTime > TURN_OFF_DELAY_MS) // silence long enough ?
            {
                g_musicActive = false;
#ifdef DEBUG_PRINT
                Serial.println("Music stopped");
#endif
            }
        }
    }
}

void reactOnMusicStatus()
{
    digitalWrite(MUSIC_ON_LED_PIN, g_musicActive ? HIGH : LOW);
    digitalWrite(MUSIC_OFF_LED_PIN, g_musicActive ? LOW : HIGH);

    // TODO: handle relay control here
}

// ---------------------------------------------------

void setup()
{
    pinMode(MUSIC_ON_LED_PIN, OUTPUT);
    pinMode(MUSIC_OFF_LED_PIN, OUTPUT);
    Serial.begin(9600);

    digitalWrite(MUSIC_ON_LED_PIN, LOW);
    digitalWrite(MUSIC_OFF_LED_PIN, LOW);

    delay(BOOT_DELAY_MS); // wait for the system to stabilize

    initialSilenceLevelDetection();
}

void loop()
{
    int analogIn = analogRead(A0);
    g_RingBuffer.push(analogIn);

    adjustSilenceLevel();

    // Adjust the signal cumulative sum based on the silence level
    float signal = analogIn - g_silenceLevel;
    float signalAbs = abs(signal);
    g_signalCumSum = (g_signalCumSum * (1.0f - DIFF_ADAPT_RATE)) + (signalAbs * DIFF_ADAPT_RATE);

    detectMusic();
    reactOnMusicStatus();

#ifdef DEBUG_PLOT
    Serial.print(">a0:");           // add spacing between variables
    Serial.print(analogIn);         // output sin(t) variable
    Serial.print(",silence:");      // add spacing between variables
    Serial.print(g_silenceLevel);   // output filtered value
    Serial.print(",signal:");       // add spacing between variables
    Serial.print(signal);           // output sin(t) variable
    Serial.print(",signalAbs:");    // add spacing between variables
    Serial.print(signalAbs);        // output sin(t) variable
    Serial.print(",signalCumSum:"); // add spacing between variables
    Serial.print(g_signalCumSum);   // output sin(t) variable

    // Serial.print(",a0V:");                   // add spacing between variables
    // Serial.print(toVoltage(analogIn));       // output sin(t) variable
    // Serial.print(",silenceV:");              // add spacing between variables
    // Serial.print(toVoltage(g_silenceLevel)); // output filtered value
    // Serial.print(",signalV:");               // add spacing between variables
    // Serial.print(toVoltage(signal));         // output sin(t) variable
    // Serial.print(",signalAbsV:");            // add spacing between variables
    // Serial.print(toVoltage(signalAbs));      // output sin(t) variable

    Serial.println();
#endif

    delay(SAMPLE_EVERY_MS);
}

/////////////////////////////////////////////////////////