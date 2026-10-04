// ESP 32 IMU Board Rev1 Test 
#include <WiFi.h>
#include <ESP_Mail_Client.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "Arduino_BMI270_BMM150.h"
#include <SPI.h>
#include "secrets.h"

#define AIO_NO_SSL
#include "AdafruitIO_WiFi.h"

// Pin Definitions
#define USR_LED_PIN 10
#define SD_CARD_CS_PIN 7
#define SD_CARD_Detect_PIN 3

// I2C Bus Pins
#define I2C_SDA 20
#define I2C_SCL 21

// WiFi Credentials
const char* ssid = "MSetup";
const char* password = "";

SMTPSession smtp;
bool fallSent = false;

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, ssid, password);
AdafruitIO_Feed *analogFeed = io.feed("cane-test");

// State Machine Definitions
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
const float FREEFALL_G_LIMIT = 0.35;
const float IMPACT_G_LIMIT = 1.25;

// Function Prototype
void ProcessFallDetection(float total_g, float total_gyro, float ax, float ay, float az, int rawTouch);

void setup() 
{
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(2000);

  Serial.println("\n--- STARTING CONNECTION PIPELINE ---");

  pinMode(USR_LED_PIN, OUTPUT);
  digitalWrite(USR_LED_PIN, HIGH);

  analogReadResolution(12);

  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL); 
  delay(300);
  while (!IMU.begin()) { delay(500); }

  Serial.println("1. IMU Initialized.");

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

// Diagnostic Variables
float min_g_recorded = 99.0;
float max_g_recorded = 0.0;
float max_gyro_recorded = 0.0;

void loop() 
{
  digitalWrite(USR_LED_PIN, (millis() / 500) % 2);
  io.run();

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
  {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float total_g = sqrt(ax * ax + ay * ay + az * az);
    float total_gyro = sqrt(gx * gx + gy * gy + gz * gz);
    int rawTouch = analogRead(SD_CARD_Detect_PIN);

    // Process touch and fall logic
    ProcessFallDetection(total_g, total_gyro, ax, ay, az, rawTouch);

    // Track extremes
    if (total_g < min_g_recorded) min_g_recorded = total_g;
    if (total_g > max_g_recorded) max_g_recorded = total_g;
    if (total_gyro > max_gyro_recorded) max_gyro_recorded = total_gyro;

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

    min_g_recorded = 99.0;
    max_g_recorded = 0.0;
    max_gyro_recorded = 0.0;
  }
}

void sendEmergencyAlert() {
  Session_Config config;
  config.server.host_name = "smtp.gmail.com";
  config.server.port = 465;
  config.login.email = SENDER_EMAIL;
  config.login.password = SENDER_PASSWORD;

  SMTP_Message message;
  message.sender.name = "SteadiStep";
  message.sender.email = SENDER_EMAIL;
  message.subject = "URGENT: Fall Detected on Smart Cane!"; // Blank subject prevents MIME header clutter on some gateways
  message.addRecipient("Emergency Contact", RECIPIENT_EMAIL);
  
  String textMsg = "CRITICAL ALERT: Fall detected on SteadiStep cane. Immediate assistance required.";
  message.text.content = textMsg.c_str();
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = "7bit";

  if (!smtp.connect(&config)) {
    Serial.println("SMTP Connection Failed!");
    return;
  }

  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.println("Error sending email: " + smtp.errorReason());
  } else {
    Serial.println("Emergency alert sent successfully!");
  }
  
  smtp.closeSession();
}

void ProcessFallDetection(float total_g, float total_gyro, float ax, float ay, float az, int rawTouch)
{
  // 1. TOUCH SENSING LOGIC
  static bool handOn = false;
  static bool lastHandState = false;
  static unsigned long lastTouchTime = 0;

  if (rawTouch < 200) {
    handOn = true;
    lastTouchTime = millis();
  } else if (rawTouch > 1000) {
    handOn = false;
  }

  bool recentTouch = handOn || (millis() - lastTouchTime < 500);

  // Print Hand state on change
  if (handOn != lastHandState) {
    lastHandState = handOn;
    Serial.print("[TOUCH] Hand ");
    Serial.println(handOn ? "ON" : "OFF");
  }

  // 2. TILT ANGLE CALCULATION (0° = Upright, 90° = Flat)
  float safe_g = (total_g < 0.1) ? 0.1 : total_g;
  float tiltAngle = acos(fabs(az) / safe_g) * (180.0 / 3.14159);

  // 3. FALL DETECTION PIPELINE
  switch (currentState) {

    case STATE_NORMAL:
      {
        static bool wasUpright = true;
        if (tiltAngle < 30.0) {
          wasUpright = true;
        }
        
        // PATH A: Freefall (Dropped Cane)
        if (total_g < FREEFALL_G_LIMIT) {
          fallStartTime = millis();
          currentState = STATE_FREEFALL;
          wasUpright = false;
          Serial.println("\n[STATE] -> CANE DROPPED (Freefall)");
          analogFeed->save("CANE_DROPPED"); // Send state to AIO dashboard
        }
        // PATH B: User Fall Candidate (Requires Hand ON + High Gyro + Tilt > 35°)
        else if (wasUpright && recentTouch && total_gyro > 150.0 && tiltAngle > 35.0) {
          fallStartTime = millis();
          currentState = STATE_USER_FALLING;
          Serial.println("\n[STATE] -> USER FALL INITIATED (Rotation + Tilt)");
          analogFeed->save("FALL_INITIATED"); // Send state to AIO dashboard
        }
      }
      break;

    case STATE_FREEFALL:
      if (total_g > IMPACT_G_LIMIT) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> Dropped cane hit ground. Resetting.");
        analogFeed->save("NORMAL");
      } else if (millis() - fallStartTime > 1500) {
        currentState = STATE_NORMAL;
        analogFeed->save("NORMAL");
      }
      break;

    case STATE_USER_FALLING:
      // Look for floor impact
      if (total_g > 2.0) {
        fallStartTime = millis();
        currentState = STATE_IMPACT;
        Serial.println("[STATE] -> USER FALL IMPACT DETECTED! Checking post-impact tilt...");
        analogFeed->save("IMPACT_DETECTED"); // Send state to AIO dashboard
      } 
      else if (millis() - fallStartTime > 2000) {
        currentState = STATE_NORMAL;
        Serial.println("[STATE] -> Motion timeout (Swung/Moved cane, no impact). Resetting.");
        analogFeed->save("NORMAL");
      }
      break;

    case STATE_IMPACT:
      // Check rest orientation 1.5s post-impact
      if (millis() - fallStartTime > 2000) {
        if (tiltAngle > 65.0 && total_gyro < 35.0) {
          currentState = STATE_FALL_CONFIRMED;
          Serial.println("\n==========================================");
          Serial.println("!!! CONFIRMED USER FALL DETECTED !!!");
          Serial.println("==========================================\n");

          // Send final alert to Adafruit IO Feed
          analogFeed->save("FALL_CONFIRMED");
          Serial.println("[AIO] Final Fall alert published to Adafruit IO feed!");

          if (!fallSent) {
            Serial.println("Sending direct emergency notification...");
            sendEmergencyAlert();
            fallSent = true; 
          }

        } else {
          currentState = STATE_NORMAL;
          Serial.println("[STATE] -> Cane upright/moving. False alarm reset.");
          analogFeed->save("NORMAL");
        }
      }
      break;

    case STATE_FALL_CONFIRMED:
      if (handOn && (millis() - fallStartTime > 4000)) {
        currentState = STATE_NORMAL;
        fallSent = false; // Reset lock for next time
        Serial.println("[STATE] -> Handle grasped / Reset to NORMAL.\n");
        analogFeed->save("NORMAL");
      }
      break;
  }
}