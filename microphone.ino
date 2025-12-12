#include <driver/i2s.h>
#include <ESP32Servo.h>

// --- Configuration ---
#define SAMPLE_RATE 16000
#define TARGET_FREQ 3200        
#define NOISE_FREQ  1000        
#define BLOCK_SIZE 512          

// --- TUNING ---
#define ABSOLUTE_THRESHOLD 50000   
#define PURITY_RATIO 2.0           
#define SWEEP_RATIO 5.0            
#define KEEP_ALIVE_MS 500  // <--- NEW: Keep servo active for 500ms after sound stops

// --- Pins ---
const i2s_port_t I2S_PORT = I2S_NUM_0;
const int I2S_SCK = 26; 
const int I2S_WS = 25;  
const int I2S_SD = 22;  
const int servoPin = 13;

Servo myServo;
int16_t rawBuffer[BLOCK_SIZE * 2]; 
float monoBuffer[BLOCK_SIZE];      

// <--- NEW: Timer Variable
unsigned long lastDetectionTime = 0;

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin, 500, 2400);
  myServo.write(90); 

  delay(1000);
  Serial.println("Initializing Selective Frequency Detector...");

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
  
  Serial.println("Ready. Anti-Jitter logic enabled.");
}

void loop() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, &rawBuffer, sizeof(rawBuffer), &bytesRead, portMAX_DELAY);

  if (bytesRead > 0) {
    int samplesRead = bytesRead / sizeof(int16_t);
    int frames = samplesRead / 2;

    long dcOffset = 0;
    for (int i = 0; i < frames; i++) {
        monoBuffer[i] = (float)(rawBuffer[i * 2] + rawBuffer[i * 2 + 1]); 
        dcOffset += monoBuffer[i];
    }
    float average = dcOffset / (float)frames;
    for (int i = 0; i < frames; i++) monoBuffer[i] -= average;

    float targetMag = goertzelMagnitude(monoBuffer, frames, TARGET_FREQ, SAMPLE_RATE);
    float noiseMag = goertzelMagnitude(monoBuffer, frames, NOISE_FREQ, SAMPLE_RATE);

    if (noiseMag < 1.0) noiseMag = 1.0; 
    float ratio = targetMag / noiseMag;

    // Logic: Check if we HEAR it right now
    bool isDetected = false;
    bool isSweeping = false;

    if (targetMag > ABSOLUTE_THRESHOLD) {
       if (ratio > SWEEP_RATIO) {
          isDetected = true;
          isSweeping = true;
       } else if (ratio > PURITY_RATIO) {
          isDetected = true;
       }
    }

    // --- SMOOTHING LOGIC ---
    
    if (isDetected) {
       // 1. We hear the sound -> Reset the "Cooldown Timer"
       lastDetectionTime = millis();
       
       Serial.print("Target:"); Serial.print(targetMag);
       Serial.print(" Ratio:"); Serial.print(ratio);
       
       if (isSweeping) {
          Serial.println("  <<< SWEEPING >>>");
          // Perform the sweep
          myServo.write(0);
          delay(150);
          myServo.write(180);
          delay(150);
          // Update timer again after the delay so we don't timeout immediately
          lastDetectionTime = millis(); 
       } else {
          Serial.println("  <<< DETECTED >>>");
          myServo.write(0);
       }
       
    } else {
       // 2. We do NOT hear the sound right now.
       //    BUT... has it been less than 500ms since we last heard it?
       
       if (millis() - lastDetectionTime < KEEP_ALIVE_MS) {
          // Keep doing whatever we were doing (Hold Position)
          // We intentionally do NOT write(90) here. 
          // We let it stay at 0 or 180 to bridge the gap.
       } else {
          // It has been silent for a long time. Reset to Neutral.
          myServo.write(90);
          Serial.println("."); // Print dot for silence
       }
    }
  }
}

float goertzelMagnitude(float* samples, int numSamples, int targetFreq, int sampleRate) {
  int k = (int)(0.5 + ((float)numSamples * targetFreq) / sampleRate);
  float omega = (2.0 * PI * k) / numSamples;
  float sine = sin(omega);
  float cosine = cos(omega);
  float coeff = 2.0 * cosine;
  float q0 = 0, q1 = 0, q2 = 0;

  for (int i = 0; i < numSamples; i++) {
    q0 = coeff * q1 - q2 + samples[i];
    q2 = q1;
    q1 = q0;
  }
  float magnitudeSquared = (q1 * q1) + (q2 * q2) - (q1 * q2 * coeff);
  return sqrt(magnitudeSquared);
}
