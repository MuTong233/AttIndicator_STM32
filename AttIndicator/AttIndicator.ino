/*

  MuTong233's [*] code
  Using Arduino IDE because I need to do "cross-platform" coding.

  The code is distributed under the GNU GPL-3.0-or-later open-source license.
  You may or already used the code, but when used, you must publish modified code as-is.

  Team MyGensou, 2024, All rights reserved.

*/

// Begin of the code.
// Software libraries
// Multithread Configuration
#include <coop_threads.h>
#define CONFIG_IDLE_CB_ALT
#define THREAD_STACK_SIZE 0x200U // Use 0x100U as the stack size for STM32F1
#define CONFIG_MAX_THREADS 6     // Increase some number for better event handling
// May reduce RAM usage...? But we have plenty of RAM anyway
// #define CONFIG_NOEXIT_STATIC_THREADS 1
#if !CONFIG_OPT_IDLE
#error CONFIG_OPT_IDLE need to be configured
#endif
// Serial based on software(built-in)
#include <SoftwareSerial.h>

// Hardware libraries
// LCD Configuration with SPI support
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>             // General SPI Library
// MPU6050 Multi-functional Sensor Configuration
#include <Adafruit_MPU6050.h> // Hardware-specific library for MPU6050
#include <Adafruit_Sensor.h>  // Core sensors library
#include <Wire.h>             // General Wire Library
// HC-SR04 Ultrasonic Distance Measure Device
#include <Ultrasonic.h>

// Ports Macro definition
// Not const char anymore yay
// Available Ports can be found in the src dir.
// Hardware I2C Port can be found on PB6: CL PB7: DA
#define KEY1 PA4
#define KEY2 PA5
#define KEY3 PA6
#define KEY4 PA7
#define TFT_SCLK PB13
#define TFT_CS PB12
#define TFT_RST PB3
#define TFT_DC PB4
#define TFT_MOSI PB15
#define TFT_BLK PB5
#define US_TRIG PA0
#define US_ECHO PA1
#define LED0 PC13
#define ESP_TX PB10
#define ESP_RX PB11
#define SYS_TX PA9
#define SYS_RX PA10

// The GensouRTOS Runtime version, also represent as the Application version.
// Used for OTA Update and other various ways.
// Structure should follow Major.Minor.Debug style like 1.00.00
#define OSVER "1.00.12"
#define SYSBAUD 38400
#define OSBAUD 115200

// Device Structure build
// ESP8266 Network Software Serial Connection
SoftwareSerial osNetSerial(ESP_RX, ESP_TX);
// For ST7735-based displays, we will use this call
// Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
SPIClass SPI_2(PB15, PB14, PB13);
Adafruit_ST7735 tft = Adafruit_ST7735(&SPI_2, TFT_CS, TFT_DC, TFT_RST);
// MPU6050 sensor uses I2C to communicate with the board
// I2C use device number based detection, make sure I2C hardware port is reserved!
Adafruit_MPU6050 mpu;
// HC-SR04 Ultrasonic uses a time-gap method to tell user the detailed distance information.
Ultrasonic ultrasonic(US_TRIG, US_ECHO);

// Global Variants
unsigned int osAppX = 0;                                                    // Buffer for application id
unsigned int osAppN = 3;                                                    // Total application number
unsigned int osState = 0;                                                   // System status check
unsigned int osIdleTime = 10;                                               // Wait X before systemui update
unsigned int osIdleRel = 0;                                                 // Buffer for Idle count
unsigned int keyInput = 0;                                                  // Buffer for key input, support multi-key input.
unsigned int distance;                                                      // Buffer for Ultrasonic
float mpuacclx, mpuaccly, mpuacclz, mpugyrox, mpugyroy, mpugyroz, mputempc; // Buffer for MPU6050
String inString = "";                                                       // UART Buffer

// Sensitive Settings
// As we use standalone SR04 package we no longer have a ultrasonic sensitivity.
void doSetSensitive(int mpuband, int mpurang, int mpurana)
{
  // TODO: Make this function has a interger output for debug.
  // TODO: And make this function a MPU6050 related one.
  switch (mpuband)
  {
  case 0:
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    break;
  case 1:
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
    break;
  case 2:
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
    break;
  case 3:
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    break;
  case 4:
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    break;
  case 5:
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
    break;
  case 6:
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    break;
  }
  switch (mpurang)
  {
  case 0:
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    break;
  case 1:
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    break;
  case 2:
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    break;
  case 3:
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    break;
  }
  switch (mpurana)
  {
  case 0:
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    break;
  case 1:
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    break;
  case 2:
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    break;
  case 3:
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    break;
  }
}

// Factory Default Settings
void doSystemReset()
{
  // TODO: Finish the configuration data structure.
  osAppX = 0;
  doSetSensitive(4, 1, 2);
}

// Sensor Value Update Function
void doSensorUpdate(int sta)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  switch (sta)
  {
  // In case if we want to refresh specific data rapidly like the ultrasonic.
  // We need a switch to let the system refresh specific data.
  // But we shouldn't have any latency huh?
  // We can let the data just refresh rapidly.
  case 0:
    mpuacclx = a.acceleration.x;
    mpuaccly = a.acceleration.y;
    mpuacclz = a.acceleration.z;
    mpugyrox = g.gyro.x;
    mpugyroy = g.gyro.y;
    mpugyroz = g.gyro.z;
    mputempc = temp.temperature;
    distance = ultrasonic.read();
    break;
  case 1:
    mpuacclx = a.acceleration.x;
    mpuaccly = a.acceleration.y;
    mpuacclz = a.acceleration.z;
    break;
  case 2:
    mpugyrox = g.gyro.x;
    mpugyroy = g.gyro.y;
    mpugyroz = g.gyro.z;
    break;
  case 3:
    mputempc = temp.temperature;
    break;
  case 4:
    distance = ultrasonic.read();
    break;
  default:
    Serial.println("Invalid data passed into the sensor updater.");
  }
}

int doSensorCheck()
{
  // TODO: We can check if the system has the right value against the MPU6050 register.

  return 0;
}

void doSensorDataAnalyze()
{
  // TODO: Based on MPU6050 Data, calculate possible attitude and climb rate etc.
  // TODO: Algorithm design :)
}

// System Info Serial Output
void doSystemInfoS()
{
  // TODO: Finish the configuration data structure.
  Serial.println("Current Configuration:");
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.print("Running Application ID: ");
  Serial.println(osAppX);
}

// Version Info Serial Output
void doSystemVersionS()
{
  Serial.print("STM32duino GensouRTOS ");
  Serial.println(OSVER);
}

/*
  Application Main Threads Programming
  WARNING: DO NOT USE ANY SOFTWARE DELAY FUNCTION!!!
  WARNING: IF YOU WANT TO PAUSE THE THREAD FOR A BIT
  WARNING: USE coop_idle INSTEAD OF delay FUNCTION
*/

// Thread 1: Main Worker for all computing related things
extern "C" void proc_worker(void *arg)
{
  doSensorUpdate(0);
}

// Thread 2: System UI handler for all display output things
extern "C" void proc_systemui(void *arg)
{
  if (osAppX == 0)
  {
    doAppBasic();
  }
  else if (osAppX == 1)
  {
    doAppBasicUI();
  }
  else if (osAppX == 2)
  {
    doAppVersion();
  }
  else if (osIdleRel != osIdleTime)
  {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(30, 60);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(1);
    tft.println("Please Wait");
  }
  else
  {
    osState = 2;
  }
}

// Thread 3: Input event handler
extern "C" void proc_input(void *arg)
{
  (digitalRead(KEY1) == 0) ? keyInput = keyInput | 0x01 : keyInput = keyInput & 0x0e;
  (digitalRead(KEY2) == 0) ? keyInput = keyInput | 0x02 : keyInput = keyInput & 0x0d;
  (digitalRead(KEY3) == 0) ? keyInput = keyInput | 0x04 : keyInput = keyInput & 0x0b;
  (digitalRead(KEY4) == 0) ? keyInput = keyInput | 0x08 : keyInput = keyInput & 0x07;
}

// Thread 4: Serial Communication handler
extern "C" void proc_serial(void *arg)
{
  while (Serial.available() > 0)
  {
    // Create a place to hold the incoming message
    int inChar = Serial.read();

    if (inChar == '\n')
    {
      // If full message received...
      // Check the message and do things.
      if (inString == "ENUM")
      {
        doSystemInfoS();
      }
      else if (inString == "PANIC")
      {
        Serial.println("OK");
        osState = 255;
      }
      else if (inString == "PROG")
      {
        if (osAppX < osAppN)
        {
          Serial.println("OK");
          osAppX++;
        }
        else
        {
          osAppX = 0;
          Serial.println("OK");
        }
      }
      else if (inString == "RESET")
      {
        Serial.println("Resetting Default Settings");
        doSystemReset();
      }
      else if (inString == "HELP")
      {
        Serial.println("You may use Available Commands:");
        Serial.println("ENUM - Show System Information");
        Serial.println("PANIC - Make a system panic");
        Serial.println("PROG - Change Active Application");
        Serial.println("RESET - Reset the system");
        doSystemVersionS();
      }
      else if (inString == "READ")
      {
        doSensorValueS();
      }
      else
      {
        Serial.print("[Serial] Unknown Command: ");
        Serial.print(inString);
        Serial.println(".");
      }
      inString = ""; // Clear buffer
    }
    else
    {
      // Not receiving full message, continue counting.
      inString += (char)inChar;
    }
  }
}

// TODO: ESP8266 External Data Transfer Thread
// TODO: Finish ESP8266 part and establish stable connection with STM32.
// TODO: All end-user interface should be finished in ESP8266 flash.
extern "C" void proc_network(void *arg)
{
  // TODO: Finish ESP8266 part
  osNetSerial.print("AT");
  osNetSerial.print("AT+CWMODE=3");
  osNetSerial.print("AT+CIPMUX=1");
  coop_idle(1000);
}

// Sensor Value Serial Output
void doSensorValueS()
{
  Serial.print("Acceleration X: ");
  Serial.print(mpuacclx);
  Serial.print(", Y: ");
  Serial.print(mpuaccly);
  Serial.print(", Z: ");
  Serial.print(mpuacclz);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(mpugyrox);
  Serial.print(", Y: ");
  Serial.print(mpugyroy);
  Serial.print(", Z: ");
  Serial.print(mpugyroz);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(mputempc);
  Serial.println(" degC");

  Serial.print("Obstacle Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("Key Register: ");
  Serial.println(keyInput);
}

int doSystemTest()
{
  // TODO: Finish the system test to make sure it only includes necessary tests.
  // TODO: And make sure the system integrity test can be finished within 1 minute.

  doSensorUpdate(0);
  doSensorValueS();
  delay(500);

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;
  Serial.print("draw function take time: ");
  Serial.println(time, DEC);

  // optimized lines
  testfastlines(ST77XX_RED, ST77XX_BLUE);
  delay(1000);

  tft.fillScreen(ST77XX_BLACK);
  Serial.println("System test done.");

  return 0;
}

// Device Initialization
void setup()
{
  // Initialize Serial Communication
  // This should be fixed port for Tx1/Rx1(PA9/PA10).
  // ESP8266 Software serial will be initialized after system test and have a check.
  Serial.setTx(SYS_TX);
  Serial.setRx(SYS_RX);
  Serial.begin(SYSBAUD);
  Serial.println("Early console at PA9 and PA10 with 9600 baud rate.");
  Serial.println("[ DEVICE INITIALIZE START ]");
  Serial.println("Setting basic Ports...");
  // Digital Pins Initialization, only key input and LED should be initialized.
  pinMode(LED0, OUTPUT);
  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);
  pinMode(KEY3, INPUT);
  pinMode(KEY4, INPUT);
  digitalWrite(LED0, LOW); // Indicate the system is working now
  // Early LCD Initialization
  Serial.println("Initializing LCD Device...");
  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  tft.setSPISpeed(240000000);
  tft.initR(INITR_144GREENTAB); // Initialize 1.44 inch TFT screen with ST7735
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(25, 40);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("WELCOME");
  tft.setCursor(8, 80);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("System is coming up");
  // MPU6050 Sensor communication establish
  Serial.println("Searching MPU6050...");
  delay(10); // Wait for I2C stabilize
  if (!mpu.begin())
  {
    // The MPU6050 connection should be established with an address of 0x68
    // If not established, tell the user system is not available to use.
    Serial.println("[ ERROR ] No MPU6050 Device found!");
    while (1)
    {
      // TODO: Since the TFT Display is initialized, we can display some kind of error code here.
      delay(100);
    }
  }
  // If everything is ok, load the basic system parameter and roll out the setup phase.
  Serial.println("MPU6050 Communication OK.");
  Serial.println("Loading System Parameters...");
  doSystemReset();
  Serial.println("[ DEVICE INITIALIZE FINISH ]");
  Serial.println("Code by MuTong233 at https://mygensou.net/ All rights reserved.");
  Serial.println("[ EARLY SYSTEM BOOTLOADER FINISH ]");
  Serial.println("[ SYSTEM TEST IN PROGRESS ]");
  doSystemVersionS();
  osState = doSystemTest();
  Serial.println(osState);
  Serial.println("[ SYSTEM TEST COMPLETED ]");
}

// Main code here, to run repeatedly:
void loop()
{
  if (osState != 0)
  {
    // Check if the system failed, otherwise start code execution.
    Serial.print("System failed with an error code ");
    Serial.println(osState);
    tft.fillScreen(ST77XX_BLUE);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.println("!!! SYSTEM PANIC !!!");
    tft.setTextSize(3);
    tft.println(":(");
    tft.setTextSize(1);
    tft.setTextWrap(true);
    tft.print("Your board has ran into a problem and need restart. The system failed with an error code of ");
    tft.println(osState);
    Serial.println("[ SYSTEM HALTED ]");
    while (1)
    {
      delay(10);
    }
  }
  else
  {
    // Serial.println("Initializing Network Connection...");
    // osNetSerial.begin(OSBAUD);
    // Serial.println("Initialized.");
    // TODO: Network connection check.
    // Schedule to run threads in async mode
    // Actually we need to manage priority by ourselves as the scheduler
    // only provides basic thread management like a RTOS but not real RTOS.
    coop_sched_thread(proc_worker, "thrd_1", THREAD_STACK_SIZE, (void *)1);
    coop_sched_thread(proc_input, "thrd_4", THREAD_STACK_SIZE, (void *)1);
    coop_sched_thread(proc_serial, "thrd_2", THREAD_STACK_SIZE, (void *)1);
    if (osIdleRel == osIdleTime)
    {
      coop_sched_thread(proc_systemui, "thrd_3", THREAD_STACK_SIZE, (void *)1);
    }
    else
    {
      osIdleRel++;
    }
    // Start the service
    coop_sched_service();
  }
}

// Following functions are used for TFT drawing.
// Screen check
void testfastlines(uint16_t color1, uint16_t color2)
{
  tft.fillScreen(ST77XX_BLACK);
  for (int16_t y = 0; y < tft.height(); y += 5)
  {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x = 0; x < tft.width(); x += 5)
  {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

// Data Indicator (Serial Replacement)
void doAppBasic()
{
  int i;
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.print("BASIC");
  tft.setCursor(0, 20);
  tft.setTextSize(1);
  for (i = 20; i != 50; i++)
  {
    tft.drawFastHLine(0, i, 128, ST7735_BLACK);
  }
  tft.print("Accel X: ");
  tft.print(mpuacclx);
  tft.println(" m/s^2");
  tft.print("Accel Y: ");
  tft.print(mpuaccly);
  tft.println(" m/s^2");
  tft.print("Accel Z: ");
  tft.print(mpuacclz);
  tft.println(" m/s^2");
  for (i = 50; i != 80; i++)
  {
    tft.drawFastHLine(0, i, 128, ST7735_BLACK);
  }
  tft.print("Gyro X: ");
  tft.print(mpugyrox);
  tft.println(" rad/s");
  tft.print("Gyro Y: ");
  tft.print(mpugyroy);
  tft.println(" rad/s");
  tft.print("Gyro Z: ");
  tft.print(mpugyroz);
  tft.println(" rad/s");
  for (i = 80; i != 110; i++)
  {
    tft.drawFastHLine(0, i, 128, ST7735_BLACK);
  }
  tft.print("Temp: ");
  tft.print(mputempc);
  tft.println(" degC");

  tft.print("Obstacle: ");
  tft.print(distance);
  tft.println(" cm");

  tft.print("Key Register: ");
  tft.println(keyInput);
  tft.print("Uptime: ");
  int uptime = millis() / 1000;
  tft.print(uptime);
  tft.println("s");
  tft.setCursor(0, 110);
  tft.println("Press < or > continue");
  if (osIdleRel == osIdleTime)
  {
    doAppSelect();
  }
}

// Main Attitude Indicator
// TODO: Algorithm for attitude indicator :) HARDEST PART
void doAppBasicUI()
{
  int i;
  for (i = 0; i != 64; i++)
  {
    tft.drawFastHLine(0, i, 128, ST7735_BLUE);
  }
  tft.drawFastHLine(0, 64, 128, ST7735_WHITE);
  for (i = 65; i != 129; i++)
  {
    tft.drawFastHLine(0, i, 128, 0xb2c0);
  }

  if (osIdleRel == osIdleTime)
  {
    doAppSelect();
  }
}

// About system
void doAppVersion()
{
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.print("SYSTEM");
  tft.setCursor(0, 20);
  tft.setTextSize(1);
  tft.println("Hardware: STM32F1");
  tft.println("System: GensouRTOS");
  tft.print("Version: ");
  tft.println(OSVER);
  tft.println("2024, Team MyGensou");
  tft.println("All rights reserved.");
  tft.setCursor(0, 110);
  tft.println("Press < or > continue");

  if (osIdleRel == osIdleTime)
  {
    doAppSelect();
  }
}

// TODO: System Settings

// Application Selector (General)
void doAppSelect()
{
  unsigned int origApp = osAppX;
  (keyInput == 1) ? osAppX-- : osAppX;
  (keyInput == 2) ? osAppX++ : osAppX;
  (osAppX == -1) ? osAppX = osAppN - 1 : osAppX;
  (osAppX == osAppN) ? osAppX = 0 : osAppX;
  if (osAppX != origApp)
  {
    tft.fillScreen(ST7735_BLACK);
    osIdleRel = 0;
  }
}