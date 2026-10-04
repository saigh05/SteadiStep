// ESP 32 IMU Board Rev1 Test 
// #include "SparkFun_SHTC3.h"
#include "Arduino_BMI270_BMM150.h"
// #include <Arduino_APDS9960.h>
#include <SPI.h>
// #include <SD.h>
// #include "FS.h"
#include <WiFi.h>
#define AIO_NO_SSL
#include "AdafruitIO_WiFi.h"

/* Usage Notes
Hello Meghana,

I will try my best to describe this board. I can give you a schematic if you are interested later.

Step One: How to flash firmware.
So I don’t have a reset button on this board. I didn’t have space for one. Plus, I wanted to make this board very small. 

To flash this, you need to put this Microcontroller in download mode. Remember the other ESP32 board? But since we don’t have a reset button,
this is the procedure. 
1. Unplug the board. 
2. Press the boot button while plugging in the cable. This will put it in download mode.
3. After flashing firmware, unplug and replug to reboot. 

Now that I am thinking about it, a reset button would have been more convenient ...............
Well, you are a student, you need to learn things the hard way ;)

Pinouts:

This board is a custom design by me, and it is specifically designed for you. So, the pinouts are not standard. 

LED Pin                 - Pin 10 (This is the USR LED light that you see in the middle of the board. I picked a blue led for this. But if you want a different color, 
                                  we can desolder that and put a new one in. I have yellow, green, blue, red and orange LEDs lying around)
SD Card Chip Select Pin - Pin 7
SD Card Detect Pin      - Pin 3. This pin has an external pull up to 3.3V. When you plug an SD Card, this pin will go High.
                                This is how you will detect if an SD card is plugged in. You are free to ignore this.
                                But if you do use it, it’ll help you with logic like
                                ---> If SD Card not found, don’t bother recording. It’s a very mechanical way to do it
I2C bus SDA             - Pin 20 (Data Line)
I2C bus SCL             - Pin 21 (Clock Line)

UART Tx                 - Pin 21
UART Rx                 - Pin 20

UART and I2C are on the same lines. To use the UART header, move BOTH the jumpers to UART on the board.
To use I2C, move BOTH the jumpers to I2C. I did it this way because the ESP32 has very few pins. I ended up using all of them.
Note that you don’t need UART separately because you are getting UART over USB anyway. I left this functionality for you to
learn how to use it.
If you are wondering what is UART -> Universal Asynchronous Receive Transmit, it is the protocol you use when you 
type Serial.print("Something"). UART was around as an RS232 protocol back in the day. It was eventually replaced by
a new protocol called the Universal Serial Bus or USB

Part numbers for the sensors:
6Axis IMU - Bosch BMI270 (Yes, a fantastic Inertial Measurement Unit from Bosch. This is a very professional sensor that is used in the 
                          industry by professionals. https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi270/)
Temperate and Humidity Sensor - Sensirion SHCT3 (Another very accurate sensor. I had a lot of these lying around)
Proximity/Gesture/Light and Color sensor - Avago Technologies APDS9960
                This sensor can read the intensity of the Red, Green and Blue color in the atmosphere. Of course, if you combine it,
                you can read the intensity of white light. You can use to see how much light is in the room. 
                (Hint -> You can use the light sensor to see how dark it is at home, use the wifi portion to talk to some 
                smart home device and turn on the light. I use something like this to turn on my Philips Hue lights at home when it is.
                dark inside)
                This sensor can also do proximity. How close you are to the sensor. You can use it as a sensor to do collision avoidance.
                So, it can also detect gestures. See the examples below. You can swipe your hand left and right and up and down
                and it can detect the direction of motion
                (Hint - You can use this to wave your hand and trigger some automation using WiFi and some IOT server. I suggest
                using the free ADAFRUIT.IO. It is very easy to set up. I use it to water my plants remotely when I am on vacation
                and to turn off and on my pc (which has a motor attached to it) when I am not at home)
                
This example sketch uses the following Libraries for the sensors. You are free to write your own from scratch but the arduino people wrote
most of these. You can also use any library you like. Make sure for the I2C, plug the jumpers the right way and in the set up add this line
      -> Wire.begin(I2C_SDA, I2C_SCL); This way it will initialize the I2C at my custom pins and not the ESP32 firmware defaults

1. SD Card -> Arduino's own SD Card library
2. Bosch BMI270 -> Arduino BMI270
3. Temperature SHTC3 -> Sparkfun's library for it
4. APDS9960 -> Arduino again

Please go through the examples in each library after you install it.
To install the library, click tools-> manage library and then search for it. The examples can be found in File-> Examples-> Library name (keep scrolling till you find it).
For the Bosch IMU, do go through all the examples. You can configure the hardware filter, sample rate etc etc. 
For this, talk to your mom because she works closely with sensors like these. A variant of this sensor is actually installed in Autoliv/Veoneer and Bosch restraints module or
the airbag module her and I work on. Yes, it is a very professional sensor, and you can use this to build your drones etc. 

This sketch has every sensor initialized and captured as a starting point.

You can uncomment the functions in the main loop to test it. Use this as an example to build anything you want with this board.

I hope you enjoy using this Sensor Board and I hope it can help you get started in your robotics career. The IMU will be the *MOST* crucial sensor for you to 
do any robotics, aviation, avionics etc. Mastering that sensor will teach you the following at a minimum
  - Motion, Newtons Laws of motion
  - Understanding linear accelerations
  - Angular Velocities
  - Understand how to get speed and distance from accelerations (Think position of control surfaces, actuators, robotic end-effectors or arms)
  - Filter data (because you need to remove certain frequencies you don’t want in your data)
  - Here is the *BIG* one -> Discrete Time Mathematics. You are no longer working in continuous time because the digital world samples everything. More on this later
  - Integrals in Discrete Time
  - Fourier Transforms in Discrete Time (FFT or Fast Fourier Transform Algorithm to transform the Time domain data to Discrete Time)

So, the real world is analog so everything is continuous time. But all the computers around you can’t understand that. So, all the data that is read in to a computer
is sampled at a certain frequency. For example, the IMU is not reading continuous accelerations, but accelerations at maybe 100 times a second or 1000 times a second or 5 times 
a second. All this broken time data is called discrete time. The math for this is very different. As you go through your 3rd year, you will be studying all this but
you will be ahead of your class already because you have this system that can teach you. 
Here is an example. You know of Velocity as V = dX/dT right. So if you do a continuous integral, Integral [0 to T] dV.dT, you will get X -> Position. Now this only
works when the time is continuous. When you are in discrete time, the time samples are at some broken range. If you sampled the data at 1000times a second (1000 Hz), each data 
sample comes in every 0.0001 second. A better way to integrate that is to treat the integral as a trapeziod (area under the curve) and the width of the trapezoid is 
your 0.0001 second time delta. This might sound confusing but once you see a diagram of it, it’ll make all the sense in the world. 

Filers are very important to learn too but you will study about this soon

So in short, Filters, Discrete Time Math concepts, Control Theory, IMUs etc are very important concepts in robots and I made this board specifically to cover these topics

You can stream all the data using Serial.println to some USB terminal and save it to your PC using python, C, matlab or Arduino terminal.
Or you can stream it over WiFi or Bluetooth.
Or use and save the data to the SD card and see it on a PC later.

The two very big things you will learn are interfaces. SPI (Serial Peripheral Interface), I2C (Inter IC Connect), UART, WiFi and Bluetooth
When you get into any industry, interfaces become important. SPI, I2C, CAN, Ethernet, PSI5 (Peripheral Sensor Interface) etc
 are very very very well known and widely used interfaces. Knowledge of these will be the difference between earning an average salary and earning a very high salary.

That should cover everything. I hope you enjoy this board. Consider this board as a birthday gift :)

- Aston D'souza

*/

// These are the GPIO pins for LED and SD Card
#define USR_LED_PIN 10

#define SD_CARD_CS_PIN 7
#define SD_CARD_Detect_PIN 3

// Setup I2C bus. Note that this board has a different pinout for I2C
#define I2C_SDA 20
#define I2C_SCL 21

// Temperature Sensor
// SHTC3 tempSensor;              // Declare an instance of the SHTC3 class

bool ledValue = 0;

// Gesture/Color/Light/Proximity Sensor Init Variables (Sensor Part Number APDS9960)
// int proximity = 0;
// int r = 0, g = 0, b = 0;

// SD Card
// File myFile;

// WiFi
const char* ssid = "MSetup";
const char* password = ""; //change to hotspot and password

// Adafruit IO
#define IO_USERNAME  "meghanas7"
#define IO_KEY       "aio_ILpm68hQlJhf2Xz1NYyJIteZk9Bc"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, ssid, password);
AdafruitIO_Feed *analogFeed = io.feed("cane-test");

// Set the interval to send data to Adafruit IO
unsigned long lastSend = 0;
const unsigned long sendInterval = 10000; 

// State machine definitions
enum CaneState {
  STATE_NORMAL,
  STATE_FREEFALL,
  STATE_IMPACT,
  STATE_FALL_CONFIRMED
}

CaneState currentState = STATE_NORMAL;
unsigned long = freefallStartTime = 0;

// Emperical thresholds
const int TOUCH_THRESHOLD = 2400;
const float FREEFALL_G_LIMIT = 0.60;
const float IMPACT_G_LIMIT = 1.50;
const float ROTATION_LIMIT = 150.0;


//Function Declarations: It's a C thing
// void ReadTemperatureSensor();
void ReadAcceleration();
void ReadGyroscope();
void ReadTouchSensor();

bool isHandOnHandle();
// void ReadColorPrxSensor();
// void SDCardReadWrite();
// void PrintCurrentSPIPins();

// void SDCARD_Init();
// void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
// void createDir(fs::FS &fs, const char * path);
// void removeDir(fs::FS &fs, const char * path);
// void readFile(fs::FS &fs, const char * path);
// void writeFile(fs::FS &fs, const char * path, const char * message);
// void appendFile(fs::FS &fs, const char * path, const char * message);
// void renameFile(fs::FS &fs, const char * path1, const char * path2);
// void deleteFile(fs::FS &fs, const char * path);
// void testFileIO(fs::FS &fs, const char * path);

// void errorDecoder(SHTC3_Status_TypeDef message);

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

  // Force clean Wi-Fi state for iPhone hotspot compatibility
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);

  Serial.print("2. Connecting via Adafruit IO to SSID: ");
  Serial.println(ssid);

  // Adafruit IO will automatically manage Wi-Fi + MQTT connections
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

// Diagnostic variables for Phase 3 drop capture
float min_g_recorded = 99.0;
float max_g_recorded = 0.0;
float max_gyro_recorded = 0.0;
bool motion_detected = false;

void loop() 
{
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  // Read raw values as fast as possible
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) 
  {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float total_g = sqrt(ax * ax + ay * ay + az * az);
    float total_gyro = sqrt(gx * gx + gy * gy + gz * gz);
    int rawTouch = analogRead(SD_CARD_Detect_PIN);

    // Track dynamic extremes
    if (total_g < min_g_recorded) min_g_recorded = total_g;
    if (total_g > max_g_recorded) max_g_recorded = total_g;
    if (total_gyro > max_gyro_recorded) max_gyro_recorded = total_gyro;

    // Trigger instant print ONLY when dynamic motion happens (<0.6g freefall OR >1.8g impact)
    if (total_g < 0.60 || total_g > 1.80 || total_gyro > 50.0) 
    {
      Serial.print(">>> EVENT DETECTED! Total_g: ");
      Serial.print(total_g, 2);
      Serial.print("g | Gyro: ");
      Serial.print(total_gyro, 1);
      Serial.print(" deg/s | Touch: ");
      Serial.println(rawTouch);
      
      motion_detected = true;
    }
  }

  // Every 3 seconds, print the absolute min/max baseline summary
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

    // Reset trackers for the next drop test
    min_g_recorded = 99.0;
    max_g_recorded = 0.0;
    max_gyro_recorded = 0.0;
  }
}

// void ReadTemperatureSensor()
// {
//   // This is the example code from the sparkfun library. Note that you can use any library 
//   // or write the I2C interface yourself using the datasheet. Just make sure to use the correct SDA and SCL pins from the note on top
//   // of this page

//   SHTC3_Status_TypeDef result = tempSensor.update();
//   if(tempSensor.lastStatus == SHTC3_Status_Nominal)              // You can also assess the status of the last command by checking the ".lastStatus" member of the object
//   {
//     Serial.print("RH = "); 
//     Serial.print(tempSensor.toPercent());                        // "toPercent" returns the percent humidity as a floating point number
//     Serial.print("%, T = "); 
//     Serial.print(tempSensor.toDegF());                           // "toDegF" and "toDegC" return the temperature as a flaoting point number in deg F and deg C respectively 
//     Serial.println(" deg F"); 
//   }
//   else
//   {
//     Serial.print("Update failed, error: "); 
//     errorDecoder(tempSensor.lastStatus);
//     Serial.println();
//   }
// }

// Example Code from Arduino Library
void ReadAcceleration()
{
  float ax = 0, ay = 0, az = 0;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(ax, ay, az);
    float total_g = sqrt(ax * ax + ay * ay + az * az);

    // Formatted key:value format for Arduino Serial Plotter
    Serial.print("Ax:"); Serial.print(ax, 2);
    Serial.print("\tAy:"); Serial.print(ay, 2);
    Serial.print("\tAz:"); Serial.print(az, 2);
    Serial.print("\tTotal_g:"); Serial.print(total_g, 2);
  }
}
void ReadGyroscope()
{
  float gx = 0, gy = 0, gz = 0;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    float total_gyro = sqrt(gx * gx + gy * gy + gz * gz);

    //Serial.print("Gyro: ");
    Serial.print("\tGx:"); Serial.print(gx, 2);
    Serial.print("\tGy:"); Serial.print(gy, 2);
    Serial.print("\tGz:"); Serial.print(gz, 2);
    Serial.print("\tTotal_deg:"); Serial.println(total_gyro, 2);
  }
}
void ReadTouchSensor()
{
  int rawTouch = analogRead(SD_CARD_Detect_PIN);

  Serial.print("Touch Raw: ");
  Serial.println(rawTouch);
}

bool isHandOnHandle()
{
  int rawTouch = analogRead(SD_CARD_Detect_PIN);
  return (rawTouch < 2400);
}

// void ReadColorPrxSensor()
// {
//   // Check if a proximity reading is available.
//   if (APDS.proximityAvailable()) {
//     proximity = APDS.readProximity();
//   }

//   // Check if a gesture reading is available
//   if (APDS.gestureAvailable()) {
//     int gesture = APDS.readGesture();
//     switch (gesture) {
//       case GESTURE_UP:
//         Serial.println("Detected UP gesture");
//         break;

//       case GESTURE_DOWN:
//         Serial.println("Detected DOWN gesture");
//         break;

//       case GESTURE_LEFT:
//         Serial.println("Detected LEFT gesture");
//         break;

//       case GESTURE_RIGHT:
//         Serial.println("Detected RIGHT gesture");
//         break;

//       default:
//         // Ignore
//         break;
//     }
//   }

//   // Check if a color reading is available
//   if (APDS.colorAvailable()) {
//     APDS.readColor(r, g, b);
//   }

//   Serial.print("PR=");
//   Serial.print(proximity);
//   Serial.print(" RGB=");
//   Serial.print(r);
//   Serial.print(",");
//   Serial.print(g);
//   Serial.print(",");
//   Serial.println(b);
// }

// void PrintCurrentSPIPins()
// {
//   // Use this function to see what the current assigned SPI Pins are.
//   // You can change them but it work for you on this board because, I have used all the pins available to 
//   // attact all the sensors. So these are the only pins available for SPI
//   Serial.print("MOSI: ");
//   Serial.println(MOSI);
//   Serial.print("MISO: ");
//   Serial.println(MISO);
//   Serial.print("SCK: ");
//   Serial.println(SCK);
//   Serial.print("SS: ");
//   Serial.println(SS); 
// }

// // Sparkfun makes this function to show some errors on serial
// void errorDecoder(SHTC3_Status_TypeDef message)                             // The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
// {
//   switch(message)
//   {
//     case SHTC3_Status_Nominal : Serial.print("Nominal"); break;
//     case SHTC3_Status_Error : Serial.print("Error"); break;
//     case SHTC3_Status_CRC_Fail : Serial.print("CRC Fail"); break;
//     default : Serial.print("Unknown return code"); break;
//   }
// }

// /************* This section does all the SD Card Stuf!! It needs a filesystem *********/
// // This code is from the ESP32 Eaxmple. It can do everything from making Directories to file etc. You wont need all of this for your projects
// // You might just makes files and store data. Use as needed. 

// void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
//     Serial.printf("Listing directory: %s\n", dirname);

//     File root = fs.open(dirname);
//     if(!root){
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if(!root.isDirectory()){
//         Serial.println("Not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while(file){
//         if(file.isDirectory()){
//             Serial.print("  DIR : ");
//             Serial.println(file.name());
//             if(levels){
//                 listDir(fs, file.path(), levels -1);
//             }
//         } else {
//             Serial.print("  FILE: ");
//             Serial.print(file.name());
//             Serial.print("  SIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void createDir(fs::FS &fs, const char * path){
//     Serial.printf("Creating Dir: %s\n", path);
//     if(fs.mkdir(path)){
//         Serial.println("Dir created");
//     } else {
//         Serial.println("mkdir failed");
//     }
// }

// void removeDir(fs::FS &fs, const char * path){
//     Serial.printf("Removing Dir: %s\n", path);
//     if(fs.rmdir(path)){
//         Serial.println("Dir removed");
//     } else {
//         Serial.println("rmdir failed");
//     }
// }

// void readFile(fs::FS &fs, const char * path){
//     Serial.printf("Reading file: %s\n", path);

//     File file = fs.open(path);
//     if(!file){
//         Serial.println("Failed to open file for reading");
//         return;
//     }

//     Serial.print("Read from file: ");
//     while(file.available()){
//         Serial.write(file.read());
//     }
//     file.close();
// }

// void writeFile(fs::FS &fs, const char * path, const char * message){
//     Serial.printf("Writing file: %s\n", path);

//     File file = fs.open(path, FILE_WRITE);
//     if(!file){
//         Serial.println("Failed to open file for writing");
//         return;
//     }
//     if(file.print(message)){
//         Serial.println("File written");
//     } else {
//         Serial.println("Write failed");
//     }
//     file.close();
// }

// void appendFile(fs::FS &fs, const char * path, const char * message){
//     Serial.printf("Appending to file: %s\n", path);

//     File file = fs.open(path, FILE_APPEND);
//     if(!file){
//         Serial.println("Failed to open file for appending");
//         return;
//     }
//     if(file.print(message)){
//         Serial.println("Message appended");
//     } else {
//         Serial.println("Append failed");
//     }
//     file.close();
// }

// void renameFile(fs::FS &fs, const char * path1, const char * path2){
//     Serial.printf("Renaming file %s to %s\n", path1, path2);
//     if (fs.rename(path1, path2)) {
//         Serial.println("File renamed");
//     } else {
//         Serial.println("Rename failed");
//     }
// }

// void deleteFile(fs::FS &fs, const char * path){
//     Serial.printf("Deleting file: %s\n", path);
//     if(fs.remove(path)){
//         Serial.println("File deleted");
//     } else {
//         Serial.println("Delete failed");
//     }
// }

// void testFileIO(fs::FS &fs, const char * path){
//     File file = fs.open(path);
//     static uint8_t buf[512];
//     size_t len = 0;
//     uint32_t start = millis();
//     uint32_t end = start;
//     if(file){
//         len = file.size();
//         size_t flen = len;
//         start = millis();
//         while(len){
//             size_t toRead = len;
//             if(toRead > 512){
//                 toRead = 512;
//             }
//             file.read(buf, toRead);
//             len -= toRead;
//         }
//         end = millis() - start;
//         Serial.printf("%u bytes read for %u ms\n", flen, end);
//         file.close();
//     } else {
//         Serial.println("Failed to open file for reading");
//     }


//     file = fs.open(path, FILE_WRITE);
//     if(!file){
//         Serial.println("Failed to open file for writing");
//         return;
//     }

//     size_t i;
//     start = millis();
//     for(i=0; i<2048; i++){
//         file.write(buf, 512);
//     }
//     end = millis() - start;
//     Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
//     file.close();
// }

// void SDCardReadWrite()
// {
//   if(!SD.begin()){
//         Serial.println("Card Mount Failed");
//         return;
//     }

//    uint8_t cardType = SD.cardType();

//     if(cardType == CARD_NONE){
//         Serial.println("No SD card attached");
//         return;
//     }

//     Serial.print("SD Card Type: ");
//     if(cardType == CARD_MMC){
//         Serial.println("MMC");
//     } else if(cardType == CARD_SD){
//         Serial.println("SDSC");
//     } else if(cardType == CARD_SDHC){
//         Serial.println("SDHC");
//     } else {
//         Serial.println("UNKNOWN");
//     }

//     uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//     Serial.printf("SD Card Size: %lluMB\n", cardSize);

//     listDir(SD, "/", 0);
//     createDir(SD, "/mydir");
//     listDir(SD, "/", 0);
//     removeDir(SD, "/mydir");
//     listDir(SD, "/", 2);
//     writeFile(SD, "/hello.txt", "Hello ");
//     appendFile(SD, "/hello.txt", "World!\n");
//     readFile(SD, "/hello.txt");
//     deleteFile(SD, "/foo.txt");
//     renameFile(SD, "/hello.txt", "/foo.txt");
//     readFile(SD, "/foo.txt");
//     testFileIO(SD, "/test.txt");
//     Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
//     Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
// }