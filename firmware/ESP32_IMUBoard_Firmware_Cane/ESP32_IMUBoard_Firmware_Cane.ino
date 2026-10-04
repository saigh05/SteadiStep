// ESP 32 IMU Board Rev1 Test 
#include "Arduino_BMI270_BMM150.h"
#include <SPI.h>
#include <WiFi.h>
#define AIO_NO_SSL
#include "AdafruitIO_WiFi.h"

// These are the GPIO pins for LED and SD Card
#define USR_LED_PIN 10
#define SD_CARD_CS_PIN 7
#define SD_CARD_Detect_PIN 3

// Setup I2C bus
#define I2C_SDA 20
#define I2C_SCL 21

// WiFi Credentials
const char* ssid = "MSetup";
const char* password = ""; // change to hotspot password if needed

// Adafruit IO
#define IO_USERNAME  "meghanas7"
#define IO_KEY       "aio_ILpm68hQlJhf2Xz1NYyJIteZk9Bc"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, ssid, password);
AdafruitIO_Feed *analogFeed = io.feed("cane-test");

// State machine definitions
enum CaneState {
  STATE_NORMAL,
  STATE_FREEFALL,
  STATE_USER_FALLING,
  STATE_IMPACT,
  STATE_FALL_CONFIRMED
};

CaneState currentState = STATE_NORMAL;
unsigned long fallStartTime = 0;

// Dynamic Thresholds
const float FREEFALL_G_LIMIT = 0.60;
const float IMPACT_G_LIMIT = 1.25;
const float ROTATION_LIMIT = 150.0;

// Function Declarations
void ProcessFallDetection(float total_g, float total_gyro, float ax, float ay, float az, int rawTouch);

void setup() 
{
  Serial.begin(115200);
  delay(2000); // Allow Serial Monitor to connect

  Serial.println("\n--- STARTING CONNECTION PIPELINE ---");

  pinMode(USR_LED_PIN, OUTPUT);
  digitalWrite(USR_LED_PIN, HIGH);

  analogReadResolution(12);

  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL); 
  IMU.begin();

  Serial.println("1. IMU Initialized.");

  // Force clean Wi-Fi state for hotspot compatibility
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

  Serial.print("2. Connecting via Adafruit IO to SSID: ");
  Serial.println(ssid);

  io.connect();

  while (io.status() < AIO_CONNECTED) 
  {
    io.run();
    Serial.print("   Current AIO Status: ");
    Serial.println(io.statusText());
    delay(1000);
  }

  Serial.println("3. FULLY CONNECTED TO WI-FI & ADAFRUIT IO!");
  Serial.print("   IP Address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(USR_LED_PIN, LOW);
}

// Diagnostic variables
float min_g_recorded = 99.0;
float max_g_recorded = 0.0;
float max_gyro_recorded = 0.0;

void loop() 
{
  io.run();

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  // Read sensor values
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
  {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float total_g = sqrt(ax * ax + ay * ay + az * az);
    float total_gyro = sqrt(gx * gx + gy * gy + gz * gz);
    int rawTouch = analogRead(SD_CARD_Detect_PIN);

    // Run core fall-detection state machine
    ProcessFallDetection(total_g, total_gyro, ax, ay, az, rawTouch);

    // Track extremes
    if (total_g < min_g_recorded) min_g_recorded = total_g;
    if (total_g > max_g_recorded) max_g_recorded = total_g;
    if (total_gyro > max_gyro_recorded) max_gyro_recorded = total_gyro;

    // Trigger instant alert print ONLY during strong impacts or drops
    if (total_g < FREEFALL_G_LIMIT || total_g > 1.80) 
    {
      Serial.print(">>> EVENT DETECTED! Total_g: ");
      Serial.print(total_g, 2);
      Serial.print("g | Gyro: ");
      Serial.print(total_gyro, 1);
      Serial.print(" deg/s | Touch Raw: ");
      Serial.println(rawTouch);
    }
    delay(20);
  }

  // Summary output every 3 seconds
  static unsigned long lastSummary = 0;
  if (millis() - lastSummary > 3000) 
  {
    lastSummary = millis();
    Serial.print("[SUMMARY] Min_g: ");
    Serial.print(min_g_recorded, 2);
    Serial.print("g | Max_g: ");
    Serial.print(max_g_recorded, 2);
    Serial.print("g | Max_Gyro: ");
    Serial.print(max_gyro_recorded, 1);
    Serial.println(" deg/s");

    // Reset trackers
    min_g_recorded = 99.0;
    max_g_recorded = 0.0;
    max_gyro_recorded = 0.0;
  }
}

void ProcessFallDetection(float total_g, float total_gyro, float ax, float ay, float az, int rawTouch)
{
  // 1. SIMPLE TOUCH LOGIC
  // Touched = under 200 | Released = over 1000
  static bool handOn = false;

  if (rawTouch < 200) {
    handOn = true;
  } else if (rawTouch > 1000) {
    handOn = false;
  }

  // Print state change cleanly on transition
  static bool lastHandState = false;
  if (handOn != lastHandState) {
    lastHandState = handOn;
    Serial.print("[TOUCH] Hand ");
    Serial.println(handOn ? "ON" : "OFF");
  }

  // 2. TILT ANGLE CALCULATION (0° = Upright, 90° = Lying flat)
  float safe_g = (total_g < 0.1) ? 0.1 : total_g;
  float tiltAngle = acos(fabs(az) / safe_g) * (180.0 / 3.14159);

  // 3. STATE MACHINE
  switch (currentState) {
    case STATE_NORMAL:
      // PATH A: Freefall / Dropped Cane
      if (total_g < FREEFALL_G_LIMIT) {
        fallStartTime = millis();
        currentState = STATE_FREEFALL;
        Serial.println("\n[STATE] -> CANE DROPPED DETECTED (Freefall)!");
      }
      // PATH B: User Fall Candidate (Requires Hand ON + High Rotation)
      else if (handOn && total_gyro > ROTATION_LIMIT) {
        fallStartTime = millis();
        currentState = STATE_USER_FALLING;
        Serial.println("\n[STATE] -> USER FALL IN PROGRESS!");
      }
      break;

    case STATE_FREEFALL:
      if (total_g > IMPACT_G_LIMIT) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> Cane hit floor (Dropped Cane Reset - No Emergency Alert)");
      } else if (millis() - fallStartTime > 1500) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> FREEFALL TIMEOUT (Reset to NORMAL)");
      }
      break;

    case STATE_USER_FALLING:
      if (total_g > IMPACT_G_LIMIT) {
        currentState = STATE_IMPACT;
        fallStartTime = millis(); // Reset timer to evaluate resting orientation
        Serial.println("[STATE] -> USER IMPACT DETECTED! Checking post-impact tilt angle...");
      } else if (millis() - fallStartTime > 2000) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> MOTION TIMEOUT (Wild swing/No impact - Resetting)");
      }
      break;

    case STATE_IMPACT:
      // Wait 1 second after impact to verify resting orientation on the floor
      if (millis() - fallStartTime > 1000) {
        if (tiltAngle > 60.0 && total_gyro < 40.0) {
          currentState = STATE_FALL_CONFIRMED;
          Serial.println("==========================================");
          Serial.println("!!! EMERGENCY: USER FALL CONFIRMED !!!");
          Serial.print("    Resting Tilt Angle: ");
          Serial.print(tiltAngle, 1);
          Serial.println(" deg (Flat on floor)");
          Serial.println("==========================================");
        } else {
          currentState = STATE_NORMAL;
          Serial.print("[STATE] -> REJECTED: Cane upright or moving (Tilt: ");
          Serial.print(tiltAngle, 1);
          Serial.println(" deg). Resetting to NORMAL.");
        }
      }
      break;

    case STATE_FALL_CONFIRMED:
      // Reset if user picks up handle after 5 seconds
      if (handOn && (millis() - fallStartTime > 5000)) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> Handle grasped / System Resetting to NORMAL.\n");
      }
      break;
  }
}