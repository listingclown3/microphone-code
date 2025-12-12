const int LASER_PIN = 23;    // Laser Transmitter
const int SENSOR_PIN = 35;   // Photoresistor (must be an Analog pin like 32, 33, 34, 35)

// CHANGE THIS NUMBER based on your calibration readings!
int THRESHOLD = 2000; 

void setup() {
    Serial.begin(115200);
    
    pinMode(LASER_PIN, OUTPUT);
    // GPIO 34 is input-only by hardware design, but defining it is good practice
    pinMode(SENSOR_PIN, INPUT); 

    Serial.println("System Initialized.");
    Serial.println("Turn on Laser and check Serial Monitor values.");
}

void loop() {
    // 1. Keep Laser ON
    digitalWrite(LASER_PIN, HIGH);

    // 2. Read the Light Level (0 to 4095)
    int lightValue = analogRead(SENSOR_PIN);
    
    // 3. Print the raw value so you can calibrate
    Serial.print("Raw Value: ");
    Serial.print(lightValue);

    // 4. Check if Laser is hitting the sensor
    if (lightValue > THRESHOLD) {
        Serial.println(" | Status: HIT (Laser Detected)");
    } else {
        Serial.println(" | Status: MISS (Beam Broken)");
    }

    delay(200); // Small delay to make the text readable
}
