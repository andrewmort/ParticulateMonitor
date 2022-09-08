#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "sps30.h"
#include "secrets.h"
#include "ThingSpeak.h"

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

  // Start serial port at 9600
  Serial.begin(115200);

  // Setup wifimanager in non-blocking mode
  wm.setConfigPortalBlocking(false);
  wm.autoConnect("SetupParticulateMonitor");

  // Setup OTA service
  startOTA();

  // Start OLED display
  startOLED();

  // Start Particulate Sensor
  startSPS30();

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  unsigned long cur_millis;

  // Process wifi manager
  wm.process();

  // Process OTA update
  ArduinoOTA.handle();

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
        Serial.println("Data not ready, wait 1s");
        delay(1000);
    }

    // if other error
    else if(ret != SPS30_ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != SPS30_ERR_OK);

  Serial.print(F("Update readings:  P2.5 = "));
  Serial.print(val.MassPM2);
  Serial.print(F("μg/m3,  Part Size = "));
  Serial.print(val.PartSize);
  Serial.println(F("μm"));

  mass_pm1p0 = val.MassPM1;
  mass_pm2p5 = val.MassPM2;
  mass_pm10 = val.MassPM10;
  part_size = val.PartSize;

  new_reading = true;
  return true;
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
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
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
