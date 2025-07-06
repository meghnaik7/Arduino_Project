#include <Arduino.h>
#include <driver/i2s.h>
#include <ArduinoFFT.h> // Ensure the "ArduinoFFT" library is installed

#define I2S_WS 15  // L/R Select (Word Select)
#define I2S_SCK 14 // Serial Clock (Bit Clock)
#define I2S_SD 32  // Serial Data (DOUT)

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000 // 16kHz sample rate
#define SAMPLE_SIZE 1024  // Number of samples for FFT

double realPart[SAMPLE_SIZE];
double imagPart[SAMPLE_SIZE];

ArduinoFFT<double> FFT(realPart, imagPart, SAMPLE_SIZE, (double)SAMPLE_RATE);

// I2S Configuration
void setupI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false};

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE, // Corrected placement
        .data_in_num = I2S_SD};

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

void setup()
{
    Serial.begin(115200);
    setupI2S();
    Serial.println("ESP32 INMP441 Frequency Detector");
}

void loop()
{
    int16_t i2sBuffer[SAMPLE_SIZE];
    size_t bytesRead;

    i2s_read(I2S_PORT, i2sBuffer, SAMPLE_SIZE * sizeof(int16_t), &bytesRead, portMAX_DELAY);

    // Convert samples to FFT input format
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        realPart[i] = (double)i2sBuffer[i]; // Convert audio data to double
        imagPart[i] = 0;                    // No imaginary part for real signals
    }

    // Perform FFT
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // Find the dominant frequency
    double peakFrequency = 0;
    double peakValue = 0;

    for (int i = 1; i < SAMPLE_SIZE / 2; i++)
    {                                                       // Ignore DC component at index 0
        double frequency = (i * SAMPLE_RATE) / SAMPLE_SIZE; // Frequency bin
        if (realPart[i] > peakValue)
        {
            peakValue = realPart[i];
            peakFrequency = frequency;
        }
    }

    Serial.print("Dominant Frequency: ");
    Serial.print(peakFrequency);
    Serial.println(" Hz");

    delay(500); // Delay for stability
}
