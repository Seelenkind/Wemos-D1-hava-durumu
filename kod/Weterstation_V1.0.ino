/*
  3.7 V Lipo at Vin NodeMCU / UV Sensor / BME 280
  GND NodemCU / UV Sensor / BME 280
  A0 to A Out UV Sensor
  D0 to RST at Arduino for wakinp up from deepsleep
  D1 to SCL BME280
  D2 to SDA BME280
  change in library Adafruit_BME280.h default adress >>> #define BME280_ADDRESS (0x77) in >>> #define BME280_ADDRESS (0x76)
*/

#include <BlynkSimpleEsp8266.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_BME280.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

Adafruit_BME280 bme;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ptbtime1.ptb.de", 3600);

const int FW_VERSION = 10017;

const char *ssid = "FRITZ!Box 6490 Cable";
const char *password = "42011226516608002284";
const char auth[] = "UhKin08xBD4FRefWg4xqz3G7ZPst7gny";

const float inMin = 1.1, inMax = 2.9, outMin = 0.0, outMax = 15.0, humyf = 4.43, tempf = 1.0, pressf = 1.0; // UV sensor voltage output range and correction factor for sensors
const byte numberOfReadings = 8; // UV sensor reading count
float temperatur, pressure, humidity, outputVoltage, uvIntensity;
int UVsensorIn = A0, uvLevel, tiMer1 = 0; //Analog Input A0

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int averageAnalogRead(int pinToRead)
{
  unsigned int runningValue = 0;
  for (int x = 0; x < numberOfReadings; x++)
  {
    runningValue += analogRead(pinToRead);
  }
  return (runningValue / numberOfReadings);
}

void checkForUpdates() {
  String  fwVersionURL = "http://www.hilmi-soenmez.com/firmware/firmware.version";
  Serial.println( "Checking for firmware updates." );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if ( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();
    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );
    int newVersion = newFWVersion.toInt();
    if ( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );
      String fwImageURL = "http://www.hilmi-soenmez.com/firmware/Weterstation_V1.0.ino.d1.bin";
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );
      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("HTTP_UPDATE_SUCCESFULL");
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}

void getValues()
{
  temperatur = bme.readTemperature() - tempf;
  pressure = int(bme.readPressure() / 100.0F) - pressf;
  humidity = bme.readHumidity() + humyf;
  uvLevel = averageAnalogRead(UVsensorIn);
  outputVoltage = 3.3 * uvLevel / 1024;//
  uvIntensity = constrain(mapfloat(outputVoltage, inMin, inMax, outMin, outMax), 0, 15);
  timeClient.update();
  Blynk.virtualWrite(V4, timeClient.getFormattedTime() + " Firmware: " + String(FW_VERSION) );
  Blynk.virtualWrite(V0, temperatur);
  Blynk.virtualWrite(V1, pressure);
  Blynk.virtualWrite(V2, humidity);
  Blynk.virtualWrite(V3, uvIntensity);
}

void startDeepSleep() {
  ESP.deepSleep(60 * 60e6);
  yield();
}


void setup()
{
  Serial.begin(9600);
  Blynk.begin(auth, ssid, password);
  bme.begin();
  Blynk.run();
  getValues();
  checkForUpdates();
  startDeepSleep();
}

void loop()
{

}
