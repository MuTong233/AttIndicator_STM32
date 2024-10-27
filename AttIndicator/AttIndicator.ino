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
#define OSVER "1.00.07"
#define SYSBAUD 9600
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
unsigned int osAppX = 0;
unsigned int osAppN = 2;
unsigned int osState = 0;
unsigned int osIdleTime = 10000;
unsigned int keyInput = 0;
bool osAbleToRun = false;
bool osPrevHasErr = false;
unsigned int distance;
float mpuacclx, mpuaccly, mpuacclz, mpugyrox, mpugyroy, mpugyroz, mputempc;

// UART Buffer
String inString = "";

// For test purpose only
float p = 3.1415926;

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
  osAbleToRun = true;
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
  // TODO: Need real hardware testing, waiting for board arrival.
}

// System Info Serial Output
void doSystemInfoS()
{
  // TODO: Finish the configuration data structure.
  Serial.println("Current Configuration:");
  Serial.print("System Working: ");
  Serial.println(osAbleToRun);
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
}

// Version Info Serial Output
void doSystemVersionS()
{
  Serial.print("STM32duino GensouRTOS ");
  Serial.println(OSVER);
}

/* Application Main Threads Programming
   WARNING: DO NOT USE ANY SOFTWARE DELAY FUNCTION!!!
   WARNING: IF YOU WANT TO PAUSE THE THREAD FOR A BIT
   WARNING: USE coop_idle INSTEAD OF delay FUNCTION */

// Thread 1: Main Worker for all computing related things
extern "C" void proc_worker(void *arg)
{
  doSensorUpdate(0);
}

// Thread 2: System UI handler for all display output things
extern "C" void proc_systemui(void *arg)
{
  // TODO: Make a user-friendly interface for end-user to use.
  // TODO: This section should only handle TFT Output, or status detection.
  // TODO: Input handler should be only handled in its belonged thread.
  // TODO: Like Serial thread or Input thread.
  testdrawtext("System UI under construction, Please use Serial to control this device. Serial started with TX on PA9, RX on PA10, Baud Rate 9600", ST77XX_WHITE);
}

// Thread 3: Input event handler
extern "C" void proc_input(void *arg)
{
  (digitalRead(KEY1) == 1) ? keyInput = keyInput | 0x01 : keyInput = keyInput & 0x0e;
  (digitalRead(KEY2) == 1) ? keyInput = keyInput | 0x02 : keyInput = keyInput & 0x0d;
  (digitalRead(KEY3) == 1) ? keyInput = keyInput | 0x04 : keyInput = keyInput & 0x0b;
  (digitalRead(KEY4) == 1) ? keyInput = keyInput | 0x08 : keyInput = keyInput & 0x07;
}

// Thread 4: Serial Communication handler
extern "C" void proc_serial(void *arg)
{
  // Ciallo
  // TODO: This thread should handle all of the serial communication.
  // TODO: Any serial data receiving or sending should be finished in this thread.
  // TODO: It should use less hardware interrupt as it may disrupt the system working.
  // Below Section is for USB Serial Communication.
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
        osAbleToRun = false;
        osState = 255;
        osPrevHasErr = true;
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
  delay(500);

  // large block of text
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("This is a block of text test why the fuck original version takes a freaking long string that will make the flash full ok never mind it doesn't matter still getting f-ed by space", ST77XX_WHITE);
  delay(1000);

  // tft print function!
  tftPrintTest();
  delay(4000);

  // optimized lines
  testfastlines(ST77XX_RED, ST77XX_BLUE);
  delay(500);

  tft.fillScreen(ST77XX_BLACK);
  Serial.println("System test done.");
  delay(1000);

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
  tft.setSPISpeed(40000000);
  tft.initR(INITR_144GREENTAB); // Initialize 1.44 inch TFT screen with ST7735
  // TODO: Make a simple bootloader splash screen here, instead of boring serial outputs.
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
  // Now we are in the GensouRTOS environment, we need to do some preparation to start system.
  // Print out the build info and test the system to ensure it meets the standard condition.
  // When in production, reduce non-necessary test phase to save time and space.
  if (osState != 0)
  {
    // Check if the system failed, otherwise start code execution.
    // TODO: Print out the error code on the screen.
    Serial.print("System Test failed with an error code ");
    Serial.println(osState);
    // In case of an hardware-attack, set the execution bit to false.
    osAbleToRun = false;
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
    osAbleToRun = true;
    if (osAbleToRun)
    {
      // Schedule to run threads in async mode
      // Actually we need to manage priority by ourselves as the scheduler
      // only provides basic thread management like a RTOS but not real RTOS.
      coop_sched_thread(proc_worker, "thrd_1", THREAD_STACK_SIZE, (void *)1);
      coop_sched_thread(proc_serial, "thrd_2", THREAD_STACK_SIZE, (void *)1);
      coop_sched_thread(proc_systemui, "thrd_3", THREAD_STACK_SIZE, (void *)1);
      coop_sched_thread(proc_input, "thrd_4", THREAD_STACK_SIZE, (void *)1);
      // Start the service
      coop_sched_service();
    }
  }
}

/*
  Following functions are used for TFT drawing test.
  CAUTION: Not all function are useful for the system design.
  They only provide a basic example of the system capability.
  And most of them only runs at the system test not the final OS
  So it's important to delete them after development complete.
*/

// TODO: This function should be transferred into a general one.
// TODO: Only for error handler or demostration purpose.
void testdrawtext(char *text, uint16_t color)
{
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

// TODO: Rest all function when tested in the real hardware
// TODO: Make them all related to the UI design.
// TODO: Following function are directly from example code.
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

void tftPrintTest()
{
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  float p = 3.1415926;
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}
