/*
 * Ultrasonic Sensor Test for ESP32
 * 
 * Connections:
 * - TRIG: GPIO26
 * - ECHO: GPIO27
 * - VCC: 5V
 * - GND: GND
 * 
 * This code continuously measures distance and displays it on Serial Monitor
 */

#define TRIG_PIN 26
#define ECHO_PIN 27

// Variables for timing
unsigned long lastMeasurement = 0;
const unsigned long MEASUREMENT_INTERVAL = 100; // Measure every 100ms

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial.println("====================================");
  Serial.println("Ultrasonic Sensor Test");
  Serial.println("TRIG: GPIO26, ECHO: GPIO27");
  Serial.println("====================================");
  
  // Configure pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Make sure trigger is low
  digitalWrite(TRIG_PIN, LOW);
  delay(100);
  
  Serial.println("Setup complete. Starting measurements...\n");
}

float measureDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10us trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration
  // Timeout after 30ms (about 5m range)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance
  // Speed of sound = 343 m/s = 0.0343 cm/us
  // Distance = (duration * 0.0343) / 2
  // Simplified: distance = duration * 0.01715
  
  if (duration == 0) {
    return -1; // No echo received (out of range or error)
  }
  
  float distance = duration * 0.0343 / 2.0;
  return distance;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Take measurement at regular intervals
  if (currentTime - lastMeasurement >= MEASUREMENT_INTERVAL) {
    lastMeasurement = currentTime;
    
    float distance = measureDistance();
    
    // Display results
    Serial.print("Time: ");
    Serial.print(currentTime / 1000.0, 1);
    Serial.print("s | ");
    
    if (distance < 0) {
      Serial.println("Distance: OUT OF RANGE or NO ECHO");
    } else {
      Serial.print("Distance: ");
      Serial.print(distance, 1);
      Serial.print(" cm");
      
      // Visual distance indicator
      Serial.print(" | ");
      int bars = distance / 5; // One bar per 5cm
      if (bars > 20) bars = 20; // Max 20 bars
      
      for (int i = 0; i < bars; i++) {
        Serial.print("█");
      }
      
      // Warning indicators
      if (distance < 10) {
        Serial.print(" ⚠️ VERY CLOSE!");
      } else if (distance < 20) {
        Serial.print(" ⚠️ WARNING!");
      } else if (distance < 30) {
        Serial.print(" ⚡ CAUTION");
      }
      
      Serial.println();
    }
    
    // Extra warning for critical distance
    if (distance > 0 && distance < 20) {
      Serial.println(">>> EMERGENCY STOP WOULD TRIGGER <<<");
    }
  }
  
  // Check for user commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    if (cmd == 'c' || cmd == 'C') {
      // Clear screen (works in most terminals)
      Serial.write(27);       // ESC command
      Serial.print("[2J");    // Clear screen
      Serial.write(27);       // ESC command  
      Serial.print("[H");     // Cursor to home
      Serial.println("Screen cleared. Continuing measurements...\n");
    } else if (cmd == 'h' || cmd == 'H') {
      Serial.println("\n--- Commands ---");
      Serial.println("C - Clear screen");
      Serial.println("H - Show this help");
      Serial.println("R - Reset statistics");
      Serial.println("----------------\n");
    } else if (cmd == 'r' || cmd == 'R') {
      Serial.println("\nStatistics reset.\n");
    }
  }
}

/* 
 * Troubleshooting Tips:
 * 
 * 1. If always reading 0 or very small values:
 *    - Check wiring connections
 *    - Make sure sensor is powered (5V)
 *    - Try adding pull-up resistor on ECHO pin
 * 
 * 2. If readings are unstable:
 *    - Add capacitor (100nF) between VCC and GND near sensor
 *    - Increase measurement interval
 *    - Average multiple readings
 * 
 * 3. If no readings at all:
 *    - Check GPIO pin numbers match your wiring
 *    - Test with different pins
 *    - Check sensor with multimeter
 * 
 * 4. Maximum range is typically 400cm
 * 5. Minimum range is typically 2cm
 * 6. Best results with flat, hard surfaces
 */