#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sps30.h"
#include "ThingSpeak.h"
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include "RemoteDebug.h"

#include "secrets.h"
/*
 * // Contents of secrets.h
 * #define SECRET_OTA_NAME     <ota_name>
 * #define SECRET_OTA_PASS     <ota_password>
 * #define SECRET_CH_ID        <thingspeak_channel>
 * #define SECRET_WRITE_APIKEY <thingspeak_apikey>
 */


#define HOST_NAME "pmonitor"

// Wifi manager
WiFiManager wm;

// Over the air update username and password
const char *OTAName     = SECRET_OTA_NAME;
const char *OTAPassword = SECRET_OTA_PASS;

// ThingSpeak channel and write key
unsigned long myChannelNumber = SECRET_CH_ID;
const char * myWriteAPIKey    = SECRET_WRITE_APIKEY;
WiFiClient  client;

// OLED display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// SPS30 Particulate senseor
#define SP30_COMMS Wire
SPS30 sps30;

// Remote debug
RemoteDebug Debug;

// Sensor readings
float mass_pm1p0 = -1;
float mass_pm2p5 = -1;
float mass_pm10  = -1;
float part_size  = -1;
bool new_reading = false;

unsigned long prev_millis = 0;

const long update_interval = 5*60*1000; // every 5 minutes

void setup() {
  // Start wifi in station mode
  WiFi.mode(WIFI_STA);

  // Start serial port
  Serial.begin(115200);

  // Setup multicast DNS name
  String hostNameWifi = HOST_NAME;
  hostNameWifi.concat(".local");
  WiFi.hostname(hostNameWifi);
  if (MDNS.begin(HOST_NAME)) {
      Serial.print("* MDNS responder started. Hostname -> ");
      Serial.println(HOST_NAME);
  }

  // Setup remote debug
  Debug.begin(HOST_NAME);         // Initialize the WiFi server
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true);       // Enable time profiling
  Debug.showColors(true);         // Enable olors
  MDNS.addService("telnet", "tcp", 23);

  // Setup wifimanager in non-blocking mode
  wm.setConfigPortalBlocking(false);
  wm.autoConnect("SetupPMonitor");

  // Setup OTA service
  startOTA();

  // Start OLED display
  startOLED();

  // Start Particulate Sensor
  startSPS30();

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

/*
 * Proceedure:
 *  - sleep sps30 for 5 minutes
 *  - start sps30 measure mode for 30s to refresh
 *  - take 30 measurements, one each second
 *  - put sps30 back to sleep
 *  - average measurements
 *  - upload to thingspeak
 *  - update display
 *  - repeat
 *
 * Improvements
 *  - better error handling (show on display)
 *    * no wifi, keep taking measurements and updating display
 *    * no sps30, make note on display, reattempt every 5 minutes
 *  - countdown to next measurement (maybe a circle that gets filled in)
 *  - web page with current measurement status, timer countdown, auto update button
 *  - debug countdown to next reading info
 */

void loop() {
  unsigned long cur_millis;

  // Handle library processing functions
  Debug.handle();       // remote debug
  wm.process();         // wifi manager
  ArduinoOTA.handle();  // OTA update
  yield();              // ESP processing time

  cur_millis = millis();
  if (cur_millis - prev_millis >= update_interval){
    prev_millis = cur_millis;

    // Read from sps30
    read_sps30();

    // Update the display
    update_display();

    // Update ThingSpeak
    if (new_reading) {
      update_thingspeak();
    }
  }
}

void startOLED() {
  // Set address for LCD display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  update_display();
}

void startSPS30() {
  // Start communication with SPS30
  SP30_COMMS.begin();
  if (sps30.begin(&SP30_COMMS) == false) {
    Errorloop((char *) "Could not set I2C communication channel.", 0);
  }

  // check for SPS30 connection
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
  else  Serial.println(F("Detected SPS30."));

  // reset SPS30 connection
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0);

  // start measurement
  if (sps30.start()) Serial.println(F("Measurement started"));
  else Errorloop((char *) "Could NOT start measurement", 0);
}

bool read_sps30(){
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {
    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == SPS30_ERR_DATALENGTH){
        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          return(false);
        }
        debugD("Data not ready, wait 1s\n");
        delay(1000);
    }

    // if other error
    else if(ret != SPS30_ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != SPS30_ERR_OK);

  debugV("Update readings:  P2.5 = %fμg/m3,  Part Size = %fμm\n", val.MassPM2, val.PartSize);

  mass_pm1p0 = val.MassPM1;
  mass_pm2p5 = val.MassPM2;
  mass_pm10 = val.MassPM10;
  part_size = val.PartSize;

  new_reading = true;

  if (Debug.isActive(Debug.INFO)) {
    sps30_device_info();
  }

  return true;

}

/**
 * @brief : read and display device info
 */
void sps30_device_info() {
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == SPS30_ERR_OK) {
    debugI("Serial number : %s\n", buf);
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == SPS30_ERR_OK)  {
    debugI("Product name  : %s\n", buf);

  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != SPS30_ERR_OK) {
    debugI("Can not read version info\n");
    return;
  }

  debugI("Firmware level: %u.%u\n", v.major, v.minor);

  debugI("Hardware level: %u\n", v.HW_version);

  debugI("SHDLC protocol: %u.%u\n", v.SHDLC_major, v.SHDLC_minor);

  debugI("Library level : %u.%u\n", v.DRV_major, v.DRV_minor);
}

void update_display() {
  display.clearDisplay();
  display.setCursor(0, 0);

  if (WiFi.status() == WL_CONNECTED) {
    display.print(F("IP: "));
    display.println(WiFi.localIP());
  } else {
    display.println(F("Not Connected"));
  }
  display.println("");
  display.println("");

  display.print(F("P2.5 = "));
  if (mass_pm2p5 > 0) {
    display.print(mass_pm2p5);
    display.println(F("ug/m3"));
  } else {
    display.println("N/A");
  }

  display.println("");

  display.print(F("Part Size = "));
  if (part_size > 0) {
    display.print(part_size);
    display.println(F("um"));
  } else {
    display.println("N/A");
  }
  display.display();
}

void update_thingspeak() {
  ThingSpeak.setField(1, mass_pm2p5);
  ThingSpeak.setField(2, part_size);
  ThingSpeak.setField(3, mass_pm1p0);
  ThingSpeak.setField(4, mass_pm10);
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    debugV("Channel update successful.\n");
  }
  else{
    debugE("Problem updating channel. HTTP error code %s\n", String(x));
  }
}

/**
 *  @brief : display error message
 *  @param mess : message to display
 *  @param r : error code
 *
 */
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

/**
 *  @brief : continued loop after fatal error
 *  @param mess : message to display
 *  @param r : error code
 *
 *  if r is zero, it will only display the message
 */
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}


// Start the OTA service
void startOTA() {
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready\r\n");
}
