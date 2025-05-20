#include "Arduino.h"

// Constants
constexpr int SAMPLE_EVERY_MS = 10; // How often to sample / run the loop (ms)

// Buffer sizes
constexpr int SAMPLE_BUFFER_SIZE = 25; // Raw samples buffer size
constexpr int VAR_BUFFER_SIZE = 25;    // Variance values buffer size

// Thresholds for detection
constexpr float VAR_OF_VAR_THRESHOLD = 5.0f; // Threshold for variance of variance to detect music

// Timing constants
constexpr uint32_t BOOT_DELAY_MS = 500U;                  // delay before we start the music detection
constexpr uint32_t TURN_ON_DELAY_MS = 1U * 1000U;         // min length of active signal to be considered as music (on debug 1 second is used)
constexpr uint32_t TURN_OFF_DELAY_MS = 10U * 60U * 1000U; // delay before we turn off the power (on debug 10 seconds is used)

constexpr int RELAY_PIN = 2;         // Pin for relay control
constexpr int MUSIC_OFF_LED_PIN = 3; // Pin for status LED
constexpr int MUSIC_ON_LED_PIN = 4;  // Pin for status LED
constexpr int DEBUG_SWITCH_PIN = 5;  // Pin to turn on/off debug output

// Generic circular buffer template with statistical functions
template <typename T, size_t SIZE>
class CircularBuffer
{
private:
    T m_buffer[SIZE];
    size_t m_head;
    size_t m_count;

    T get_(size_t index) const
    {
        return m_buffer[(m_head - 1 - index + SIZE) % SIZE];
    }

public:
    CircularBuffer() : m_head(0), m_count(0)
    {
        memset(m_buffer, 0, sizeof(m_buffer));
    }

    void push(T value)
    {
        m_buffer[m_head] = value;
        m_head = (m_head + 1) % SIZE;
        if (m_count < SIZE)
        {
            m_count++;
        }
    }

    T get(size_t index) const
    {
        if (index >= m_count)
            return T();
        return get_(index);
    }

    size_t size() const
    {
        return m_count;
    }

    bool isFull() const
    {
        return m_count == SIZE;
    }

    // Calculate mean of buffer values
    float mean() const
    {
        if (m_count == 0)
            return 0.0f;

        float sum = 0.0f;
        for (size_t i = 0; i < m_count; i++)
        {
            sum += get_(i);
        }
        return sum / m_count;
    }

    // Calculate variance of buffer values
    float variance() const
    {
        if (m_count <= 1)
            return 0.0f;

        float avg = mean();
        float sumSquareDiff = 0.0f;

        for (size_t i = 0; i < m_count; i++)
        {
            float diff = get_(i) - avg;
            sumSquareDiff += diff * diff;
        }

        return sumSquareDiff / (m_count - 1); // Sample variance
    }
};

// Buffer instances
CircularBuffer<int, SAMPLE_BUFFER_SIZE> g_SampleBuffer;
CircularBuffer<float, VAR_BUFFER_SIZE> g_VarBuffer;

// Global variables
float g_currentVariance = 0.0f;
float g_varOfVar = 0.0f;
bool g_isDebugOn = false;

// Global variables for timing
unsigned long g_musicStartTime = 0;
unsigned long g_silenceStartTime = 0;

// Global state variables
bool g_statsInitialized = false;
bool g_musicActive = false;

// Calculate variance and update all buffers with new sample
void updateVarianceMetrics(int newSample)
{
    // Add sample to the main buffer
    g_SampleBuffer.push(newSample);

    // Only proceed with variance calculation when buffer has enough samples
    if (g_SampleBuffer.isFull())
    {
        // Calculate and store current variance
        g_currentVariance = g_SampleBuffer.variance();
        g_VarBuffer.push(g_currentVariance);

        // Calculate variance of variance when enough variance samples are collected
        if (g_VarBuffer.isFull())
        {
            g_varOfVar = g_VarBuffer.variance();

            g_statsInitialized = true;
        }
    }
}

// Detect music based on variance metrics
void detectMusic()
{
    unsigned long currentTime = millis();
    bool musicSignalDetected = false;

    if (g_statsInitialized)
    {
        // Music is detected when variance or variance-of-variance exceeds thresholds
        musicSignalDetected = (g_varOfVar > VAR_OF_VAR_THRESHOLD);
    }

    // Handle music state transitions with timing
    if (musicSignalDetected)
    {
        g_silenceStartTime = 0; // Reset silence timer

        if (!g_musicActive)
        {
            const auto turnOnDelay = g_isDebugOn ? (1U * 1000U) : TURN_ON_DELAY_MS; // Use shorter delay for debug mode
            if (g_musicStartTime == 0)
            {
                g_musicStartTime = currentTime; // Start music timer
            }
            else if (currentTime - g_musicStartTime > turnOnDelay)
            {
                g_musicActive = true; // Activate music state after delay
                // Serial.println("Music detected");
            }
        }
    }
    else // Silence detected
    {
        g_musicStartTime = 0; // Reset music timer

        if (g_musicActive)
        {
            const auto turnOffDelay = g_isDebugOn ? (10U * 1000U) : TURN_OFF_DELAY_MS; // Use shorter delay for debug mode
            if (g_silenceStartTime == 0)
            {
                g_silenceStartTime = currentTime; // Start silence timer
            }
            else if (currentTime - g_silenceStartTime > turnOffDelay)
            {
                g_musicActive = false; // End music state after silence delay
                // Serial.println("Silence detected");
            }
        }
    }
}

// Handle LED indicators and relay control
void reactOnMusicStatus()
{
    digitalWrite(MUSIC_ON_LED_PIN, g_musicActive ? HIGH : LOW);
    digitalWrite(MUSIC_OFF_LED_PIN, g_musicActive ? LOW : HIGH);
    digitalWrite(RELAY_PIN, g_musicActive ? LOW : HIGH);
}

// ---------------------------------------------------

void setup()
{
    pinMode(DEBUG_SWITCH_PIN, INPUT);
    pinMode(MUSIC_ON_LED_PIN, OUTPUT);
    pinMode(MUSIC_OFF_LED_PIN, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);

    Serial.begin(9600);

    digitalWrite(MUSIC_ON_LED_PIN, LOW);
    digitalWrite(MUSIC_OFF_LED_PIN, LOW);
    digitalWrite(RELAY_PIN, HIGH); // Turn off relay initially

    delay(BOOT_DELAY_MS); // wait for the system to stabilize

    g_isDebugOn = (digitalRead(DEBUG_SWITCH_PIN) == HIGH);
    if (!g_isDebugOn)
    {
        Serial.print("Debugging output is disabled! Flip the jumper to see the Serial plot data.");
        Serial.println();
    } 
}

void loop()
{
    g_isDebugOn = (digitalRead(DEBUG_SWITCH_PIN) == HIGH);

    int analogIn = analogRead(A0);

    // Update all statistical metrics
    updateVarianceMetrics(analogIn);

    // Detect music based on statistics
    detectMusic();

    // Update outputs
    reactOnMusicStatus();

    if (g_isDebugOn)
    {
        Serial.print(">a0:"); // raw signal
        Serial.print(analogIn);
        Serial.print(",variance:"); // signal variance
        Serial.print(g_currentVariance);
        Serial.print(",var_of_var:"); // variance of variance
        Serial.print(g_varOfVar);
        Serial.println();
    }

    delay(SAMPLE_EVERY_MS);
}
