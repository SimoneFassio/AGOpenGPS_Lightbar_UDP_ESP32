/*
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file!

To use with https://github.com/farmerbriantee/AgOpenGPS V5

AGOpenGPS Lightbar for WLAN/UDP connection with ESP32 and led strip ws2812b

inspired by https://github.com/mnltake/AOGLightbar_M5stickC/blob/master/AOGLightbar_M5stickC_BT_v5_1.ino as well as parts of the code are copied and modified into this sketch

do not forget to include Libraries:
  https://github.com/hagre/VerySimpleTimer_Library.git
  https://github.com/hagre/SyncWiFIConnectionESP32_Library.git
  fastled/FastLED@^3.4.0

ESP32 firmware
Basic selection via #define in the top of this code

by hagre
05 2021
*/

// version
#define VERSION 0
#define SUB_VERSION 3

// features
#define DEBUG_UART_ENABLED // enable or disable with // in front of #define     //self explaining (more or less just for me to use)
// #define LED_BRIGHTNESS_CONTROL_ENABLED //enable or disable with // in front of #define   you need to connect a potentiometer to the pin

// network IP
#define IP_DIGIT1_OF_THIS_NODE 192
#define IP_DIGIT2_OF_THIS_NODE 168
#define IP_DIGIT3_OF_THIS_NODE 137
#define IP_DIGIT4_OF_THIS_NODE 128

// WIFI settings
#define YOUR_WIFI_SSID "GPStrattore"
#define YOUR_WIFI_PASSWORD "AGOPENGPS"
#define YOUR_WIFI_HOSTNAME "AOG_Lightbar"
#define PORT_TO_LISTEN 8888  // local port to listen for UDP packets
#define PORT_FROM_PANDA 9999 // 1526
#define UDP_PACKET_SIZE 1024
#define MAX_WAIT_TIME_UDP 2000 // ms timeout for waiting on valide UDP msg

// LED Settings
#define NUMPIXELS 71         // Odd number dont use =0
#define Neopixel_Pin 13      // GPIO16 Set this to the pin number you are using for the Neopixel strip controll line
#define cmPerLightbarPixel 2 // Must be a multiple of cmPerDistInt
#define cmPerDistInt 2       // The number of centimeters represented by a change in 1 of the AOG cross track error byte
#define LED_UPDATE_TIME 200  // ms
#define LED_BRIGHTNESS 100   // 0-255
#ifdef LED_BRIGHTNESS_CONTROL_ENABLED
#define ANALOG_PIN 34          // GPIO 34 (Analog ADC1_CH6)
#define MIN_LED_BRIGHTNESS 20  // 0-255
#define MAX_LED_BRIGHTNESS 150 // 0-255
#endif
#include <FastLED.h>
#define LED_TYPE WS2812B
#define LED_COLOR_SETTING GRB
#define LED_CORRECTION TypicalLEDStrip

// Serial Port config - do not change
#define DEBUG_UART_HARDWARE_PORT 0 // config as required // USB == 0
#define DEBUG_UART_BOUD 115200     // config as required
#define UART0RX -1                 // ESP32 GPIOs
#define UART0TX -1

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_ADDRESS 0x3C // Address 0x3D for 128x64
//++++++++++++++++++++++++++++++++++++++++++++++++ END of CONFIG +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <SyncWifiConnectionESP32.h>
#include "NMEAParser.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define BUTTON_PIN 18 // GPIO pin connected to button

// Variables will change:
int8_t buttonLastState = HIGH; // the previous state from the input pin
int8_t buttonCurrentState;     // the current reading from the input pin
float zero_altitude = 0;

SyncWifiConnectionESP32 SyncWifiConnection;
int8_t LANStatus = -5; // Connected to Network //WIFI or ETHERNET //-5 = init, -2 = just disconnected, -1 = wait to reconnect, 0 = disconnected, 1 = connecting, 2 = just connected,  3 = still connected
int8_t LANStatusOld = -6;
const char *ssid = YOUR_WIFI_SSID;
const char *password = YOUR_WIFI_PASSWORD;
const IPAddress Node_IP(IP_DIGIT1_OF_THIS_NODE, IP_DIGIT2_OF_THIS_NODE, IP_DIGIT3_OF_THIS_NODE, IP_DIGIT4_OF_THIS_NODE);

// A UDP instance to let us send and receive packets over UDP
AsyncUDP udp;
AsyncUDP udpPANDA;
unsigned int localPort = PORT_TO_LISTEN; // local port to listen for UDP packets
unsigned int PANDAPort = PORT_FROM_PANDA;
// byte packetBuffer[UDP_PACKET_SIZE];      // buffer to hold incoming packets
// byte packetBufferPANDA[UDP_PACKET_SIZE]; // buffer to hold incoming packets
unsigned long lastvalideXtrackreceived = 0;
byte *packetBuffer;
byte *packetBufferPANDA;

NMEAParser<1> parser;
char altitude_c[12];
float altitude;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

CRGBArray<NUMPIXELS> leds;
const uint8_t centerpixel = (NUMPIXELS - 1) / 2;
uint8_t Neopixel_Brightness = LED_BRIGHTNESS; // default brightness value between 0 and 255
unsigned long lastLEDUpdate = 0;

const uint8_t cmPerLBPixel = cmPerLightbarPixel / cmPerDistInt;
int16_t cross_track_error = 0;
int16_t cross_track_errorOld = 0;

int8_t statusOfProgram = -3; // -3 init / -2 No WLAN connection / -1 no UDP msg arrive or timeout / 0 not in AutoMode (255) / 1 receiving valid distance
int8_t statusOfProgramOld = -3;

uint32_t serialDebugPortBoud = DEBUG_UART_BOUD;
HardwareSerial SerialDebug(DEBUG_UART_HARDWARE_PORT);

TaskHandle_t taskHandle_LEDupdate;

void getDataFromAOGWiFi();
void getDataFromPANDA();
void FastLEDupdate(void *pvParameters);
void errorHandler();
void PANDA_Handler();
void OLED_Update();
void buttonHandler();

void setup()
{
// put your setup code here, to run once:
#ifdef DEBUG_UART_ENABLED
  SerialDebug.begin(serialDebugPortBoud, SERIAL_8N1, UART0RX, UART0TX); // Debug output, usually USB Port
  SerialDebug.println("Setup");
  // Serial.begin(115200);
#endif

  statusOfProgram = -2;

#ifdef DEBUG_UART_ENABLED
  SerialDebug.println("Starting WIFI");
  SyncWifiConnection.setWifiDebugSerial(&SerialDebug);
#endif
  SyncWifiConnection.init(WIFI_STA, Node_IP, YOUR_WIFI_HOSTNAME, YOUR_WIFI_SSID, YOUR_WIFI_PASSWORD);

  FastLED.addLeds<LED_TYPE, Neopixel_Pin, LED_COLOR_SETTING>(leds, NUMPIXELS).setCorrection(LED_CORRECTION);
  lastLEDUpdate = millis();

  xTaskCreate(FastLEDupdate, "FastLEDupdate", 3072, NULL, 2, &taskHandle_LEDupdate);

  parser.setErrorHandler(errorHandler);
  parser.addHandler("PANDA", PANDA_Handler);

  if (!display.begin(OLED_ADDRESS, true))
    SerialDebug.println(F("SH1106 allocation failed"));

  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 20);
  // Display static text
  display.setRotation(2);
  display.println("PRONTO");
  display.display();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(10000);


  getDataFromAOGWiFi();
  getDataFromPANDA();
}

void loop()
{
  // put your main code here, to run repeatedly:
  LANStatus = SyncWifiConnection.loop(millis());
#ifdef DEBUG_UART_ENABLED
  // SerialDebug.print ("Status: ");
  // SerialDebug.println (statusOfProgramOld);
#endif
  if (LANStatus != LANStatusOld)
  {
    if (LANStatusOld < 3 && LANStatus == 3)
    {
      statusOfProgram = -1;
    }
    if (LANStatusOld == 3 && LANStatus <= 3)
    {
      statusOfProgram = -2;
    }
#ifdef DEBUG_UART_ENABLED
    SerialDebug.print(LANStatusOld);
    SerialDebug.println(LANStatus);
#endif
    LANStatusOld = LANStatus;
  }

  buttonHandler();
}

void getDataFromAOGWiFi()
{
  if (udp.listen(PORT_TO_LISTEN))
  {
#ifdef DEBUG_UART_ENABLED
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
#endif
    udp.onPacket([](AsyncUDPPacket packet)
                 {
          packetBuffer = packet.data();
      if (packetBuffer[0] == 0x80 && packetBuffer[1] == 0x81 && packetBuffer[2] == 0x7f && packetBuffer[3] == 0xfe)
      {
        lastvalideXtrackreceived = millis();
#ifdef DEBUG_UART_ENABLED
        SerialDebug.print(packetBuffer[10]);
        SerialDebug.print(" Correct Header found. Distance = ");
#endif
        if (packetBuffer[10] == 255)
        {
          cross_track_error = 0;
          statusOfProgram = 0;
#ifdef DEBUG_UART_ENABLED
          SerialDebug.println(" AUTO not active");
#endif
        }
        else
        {
          cross_track_error = packetBuffer[10] - 127;
          statusOfProgram = 1;
#ifdef DEBUG_UART_ENABLED
          SerialDebug.println(cross_track_error);
#endif
        }
      }
        if (millis() - lastvalideXtrackreceived > MAX_WAIT_TIME_UDP)
          statusOfProgram = -1; });
  }
}

void getDataFromPANDA()
{
  if (udpPANDA.listen(PORT_FROM_PANDA))
  {
#ifdef DEBUG_UART_ENABLED
    Serial.print("UDP PANDA Listening on IP: ");
    Serial.println(WiFi.localIP());
#endif
    udpPANDA.onPacket([](AsyncUDPPacket packet)
                      {
          packetBufferPANDA = packet.data();
      for (uint16_t i = 0; i < packet.length(); i++)
          parser << packetBufferPANDA[i]; });
  }
}

void FastLEDupdate(void *pvParameters)
{
  for (;;)
  {
    if (cross_track_error != cross_track_errorOld || statusOfProgram != statusOfProgramOld || millis() - lastLEDUpdate > LED_UPDATE_TIME)
    {

      lastLEDUpdate = millis();

#ifndef LED_BRIGHTNESS_CONTROL_ENABLED
      FastLED.setBrightness(Neopixel_Brightness);
#else
      uint16_t analog_Neopixel_Brightness = map(analogRead(ANALOG_PIN), 0, 4095, MIN_LED_BRIGHTNESS, MAX_LED_BRIGHTNESS);
      FastLED.setBrightness(analog_Neopixel_Brightness);
#endif

      if (statusOfProgram == -2)
      {
        for (int i = 0; i < NUMPIXELS; i++)
        {
          if (i == 0 || i == NUMPIXELS - 1)
          { // Left end right end of Leds
            leds[i] = CRGB::Red;
          }
          else
          {
            leds[i] = CRGB::Black; // Clear
          }
        }
      }
      else if (statusOfProgram == -1)
      { // CONNECTED
        for (int i = 0; i < NUMPIXELS; i++)
        {
          if (i == 0 || i == NUMPIXELS - 1)
          { // Left end right end of Leds
            leds[i] = CRGB::Blue;
          }
          else
          {
            leds[i] = CRGB::Black; // Clear
          }
        }
      }
      else if (statusOfProgram == 0)
      { // READY
        for (int i = 0; i < NUMPIXELS; i++)
        {
          if (i == 0 || i == NUMPIXELS - 1)
          { // Left end right end of Leds
            leds[i] = CRGB::Green;
          }
          else
          {
            leds[i] = CRGB::Black; // Clear
          }
        }
      }
      else if (statusOfProgram == 1)
      {
        int8_t level = constrain((int8_t)(cross_track_error / cmPerLBPixel), -centerpixel, centerpixel);
        int8_t n = level + centerpixel;
        for (int i = 0; i < NUMPIXELS; i++)
        {
          if (i == centerpixel && i == n)
          { // Center
            leds[i] = CRGB::Yellow;
          }
          else if (level < 0 && i >= n && i < centerpixel)
          { // Right Bar
            leds[i] = CRGB::Green;
          }
          else if (level > 0 && i <= n && i > centerpixel)
          { // Left Bar
            leds[i] = CRGB::Red;
          }
          else
          {
            leds[i] = CRGB::Black; // Clear
          }
        }
      }

      FastLED.show();
      statusOfProgramOld = statusOfProgram;
      cross_track_errorOld = cross_track_error;
    }
    vTaskDelay(20);
  }
}

// if odd characters showed up.
void errorHandler()
{
  // nothing at the moment
}

void PANDA_Handler() // Rec'd GGA
{
  // altitude
  if (parser.getArg(8, altitude_c))
  {
    altitude = atof(altitude_c);
  }

#ifdef DEBUG_UART_ENABLED
  SerialDebug.println("PANDA parsed");
  SerialDebug.print("altitude: ");
  SerialDebug.println(altitude);
#endif
  OLED_Update();
}

void OLED_Update()
{
  char output[12];
  sprintf(output, "%3.2f", (altitude - zero_altitude));

  display.clearDisplay();

  display.setTextSize(3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 20);
  // Display static text
  display.println(output);
  display.display();

#ifdef DEBUG_UART_ENABLED
  SerialDebug.print("output: ");
  SerialDebug.println(output);
#endif
}

void buttonHandler()
{
  buttonCurrentState = digitalRead(BUTTON_PIN);

  if (buttonLastState == HIGH && buttonCurrentState == LOW)
  {
    if (zero_altitude == 0)
      zero_altitude = altitude;
    else
      zero_altitude = 0;
  }

  // save the last state
  buttonLastState = buttonCurrentState;
}