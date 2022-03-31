/* Code for watchx to send mpu data from bluetooth */
#include <Arduino.h>
//#include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_SSD1306.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif


#define INTERRUPT_PIN 2
#define LED_PIN 13

#define OLED_DC     A3
#define OLED_CS     A5
#define OLED_RESET  A4
#define CS_PIN  A5
#define RST_PIN A4
#define DC_PIN  A3

#define VOLTAGEDIV 0.5
#define BATTERYENERGY 4
#define BATTERYINPUT A11

#define BUTTON1 8
#define BUTTON2 10
#define BUTTON3 11
// ================================================================
// ===                  DISPLAY RELATED PART                    ===
// ================================================================
SSD1306AsciiSpi oled;

// ================================================================
// ===                  BATTERY RELATED PART                    ===
// ================================================================
float batteryLevel = 0;
void show_battery() {
  digitalWrite(BATTERYENERGY, HIGH);
  delay(50);
  float voltage = analogRead(BATTERYINPUT);
  voltage = (voltage / 1024) * 3.35;
  voltage = voltage / VOLTAGEDIV;
  delay(50);
  digitalWrite(BATTERYENERGY, LOW);
  batteryLevel = (voltage - 3.38) / 0.0084;
  oled.print("Bat:");
  oled.print((int) batteryLevel);
  oled.println("%");
}

// ================================================================
// ===                    MPU RELATED PART                      ===
// ================================================================
MPU6050 mpu(MPU6050_ADDRESS_AD0_HIGH);
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                 BLUETOOTH RELATED PART                   ===
// ================================================================
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */

int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

// ================================================================
// ===                       SLEEP SETUP                        ===
// ================================================================
bool sleep = false;
bool awake = false;
bool wake = false;
bool show = false;
bool show_state = false;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
bool serial_open = true;
void setup(void)
{

  Wire.begin();
  Wire.setClock(400000);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  //display.begin(SSD1306_SWITCHCAPVCC);
  oled.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  oled.setFont(Adafruit5x7);

  Serial.begin(115200);
  int go = 0;
  while (!Serial && go < 20) {
    go++;
    delay(500);// required for Flora & Micro
  }
  if (go>=20) serial_open = false;
  delay(500);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.CalibrateAccel(20);
    mpu.CalibrateGyro(20);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  ble.begin(VERBOSE_MODE);
  ble.factoryReset();
  ble.echo(false);
  ble.info();
  ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit TEZ"));
  ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=14, VALUE=00-40"), &hrmMeasureCharId);

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  ble.reset();
}

// ================================================================
// ===                      LOOP INFINITY                       ===
// ================================================================
void loop(void)
{
  sleep = digitalRead(BUTTON1);
  wake = digitalRead(BUTTON2);
  show = digitalRead(BUTTON3);
  
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }
  char ax_norm = map(aaReal.x, -32800, 32800, 0, 255);
  char ay_norm = map(aaReal.y, -32800, 32800, 0, 255);
  char az_norm = map(aaReal.z, -32800, 32800, 0, 255);
  //char gx_norm = map(q.x, -1, 1, 0, 255);
  //char gy_norm = map(q.y, -1, 1, 0, 255);
  //char gz_norm = map(q.z, -1, 1, 0, 255);
  //char gw_norm = map(q.w, -1, 1, 0, 255);
  char gx_norm = (q.x + 1.0) * 255;
  char gy_norm = (q.y + 1.0) * 255;
  char gz_norm = (q.z + 1.0) * 255;
  char gw_norm = (q.w + 1.0) * 255;
  if(serial_open){  
    Serial.print(F("Updating HRM value to "));
    Serial.print("\t");
    Serial.print((uint8_t)ax_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)ay_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)az_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)gx_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)gy_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)gz_norm,HEX);
    Serial.print("\t");
    Serial.print((uint8_t)gw_norm,HEX);
    Serial.print("\t");
    Serial.println(F(" BPM"));
  
  }
  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",") );
  ble.print(ax_norm);
  ble.print(ay_norm);
  ble.print(az_norm);
  ble.print(gx_norm);
  ble.print(gy_norm);
  ble.print(gz_norm);
  ble.println(gw_norm);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("F"));   
  }
  if(!wake){
    oled.clear();
    oled.println("I am going to sleeping");
    delay(2000);
    oled.clear();
    awake = false;
   }
   
  if(!sleep){
    oled.clear();
    oled.println("I am going to awake");
    delay(2000);
    oled.clear();
    awake = true;
  }
  if(!show){
    oled.clear();
    if(show_state){
      oled.println("I will stop showing");
    }
    else oled.println("I will start to show");
    delay(2000);
    oled.clear();
    show_state = !show_state;
  }

  /* Delay before next measurement update */
  if (awake) {
    delay(100);
  }
  else {
    delay(5000);
  }
  if (show_state){
    oled.clear();
    show_battery();
    if(awake) oled.println("awake");
    else oled.println("sleep");
    delay(2000);
    oled.clear();
  }
}
