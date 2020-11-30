// ESP32 - Lolin D32
// SHT31
// BMP280
// low power technique - https://blog.voneicken.com/2018/lp-wifi-esp32-mqtt/

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <Wire.h>

#include <Adafruit_BMP280.h>
#include <SHT3x.h>
#include <PubSubClient.h>




#define UNIQUE_ID "004"
#define DEVICE_NAME "LolinD32-" UNIQUE_ID

#define WLAN_SSID "Sajhalud2.4"
#define WLAN_PASSWD "xxx"

#define MQTT_SERVER "192.168.1.yyy"
#define MQTT_PORT 1883
#define MQTT_USERNAME "sensor"
#define MQTT_PASSWORD "xxx"

#define MQTT_SENS_TEMP_TOPIC "hm49/sens/temp/sht31-" UNIQUE_ID
#define MQTT_SENS_HMDT_TOPIC "hm49/sens/hmdt/sht31-" UNIQUE_ID
#define MQTT_SENS_PRSS_TOPIC "hm49/sens/prss/bmp280-" UNIQUE_ID
#define MQTT_HWRD_BATT_TOPIC "hm49/hwrd/" DEVICE_NAME "/batt"

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define DEEP_SLEEP_TIME  15*60     /* Time ESP32 will go to sleep (in seconds) */

#define POWER_PIN 32

#define ATMOSPHERIC_PRESSURE_OFFSET_PA   2356 // check METAR near your location (for Brno: https://en.allmetsat.com/metar-taf/poland.php?icao=LKTB)
                                              // get METAR pressure, compare with BMP280 pressure and calculate offset in Pa




//#define MQTTIDENT "mqtt PSK identity"
//#define MQTTPSK "mqtt hex PSK string"
//const char mqtt_ident[] = MQTTIDENT;
//const char mqtt_psk[] = MQTTPSK;




static RTC_DATA_ATTR struct {
    byte mac [ 6 ];
    byte mode;
    byte chl;
    uint32_t ip;
    uint32_t gw;
    uint32_t msk;
    uint32_t dns;
    uint32_t chk;
} cfgbuf;




WiFiClient client;
PubSubClient mqtt(client); //lib required for mqtt
char msg[10];

SHT3x sht31( 0x44,         //Set the address
             SHT3x::Zero,  //Functions will return zeros in case of error
             255,          //If you DID NOT connected RESET pin
             SHT3x::SHT31, //Sensor type
             SHT3x::Single_HighRep_ClockStretch //Low repetability mode
            );
Adafruit_BMP280 bmp280; // I2C

float temp_sht31, relh_sht31;
float pres_bmp280;
float batt_vltg;




float readBatteryVoltage() {               // Lolin D32 has a 1/2 voltage divider -> if vbat = 4.2V then GPIO35 = vbat/2 = 2.1V
   return analogRead(35)/4096.0 * 7.445;   // adc reading scaled to voltage with 7.445 factor
}

bool checkCfg() {
  uint32_t x = 0;
  uint32_t *p = (uint32_t *)cfgbuf.mac;
  for (uint32_t i = 0; i < sizeof(cfgbuf)/4; i++) x += p[i];
  //printf("RTC read: chk=%x x=%x ip=%08x mode=%d %s\n", cfgbuf.chk, x, cfgbuf.ip, cfgbuf.mode,
  //        x==0?"OK":"FAIL");
  if (x == 0 && cfgbuf.ip != 0) return true;
  Serial.println("NVRAM cfg init");
  // bad checksum, init data
  for (uint32_t i = 0; i < 6; i++) cfgbuf.mac[i] = 0xff;
  cfgbuf.mode = 0; // chk err, reconfig
  cfgbuf.chl = 0;
  cfgbuf.ip = IPAddress(0, 0, 0, 0);
  cfgbuf.gw = IPAddress(0, 0, 0, 0);
  cfgbuf.msk = IPAddress(255, 255, 255, 0);
  cfgbuf.dns = IPAddress(0, 0, 0, 0);
  return false;
}

void writecfg(void) {
  // save new info
  uint8_t *bssid = WiFi.BSSID();
  for (int i=0; i<sizeof(cfgbuf.mac); i++) cfgbuf.mac[i] = bssid[i];
  cfgbuf.chl = WiFi.channel();
  cfgbuf.ip = WiFi.localIP();
  cfgbuf.gw = WiFi.gatewayIP();
  cfgbuf.msk = WiFi.subnetMask();
  cfgbuf.dns = WiFi.dnsIP();
  // recalculate checksum
  uint32_t x = 0;
  uint32_t *p = (uint32_t *)cfgbuf.mac;
  for (uint32_t i = 0; i < sizeof(cfgbuf)/4-1; i++) x += p[i];
  cfgbuf.chk = -x;
  //printf("RTC write: chk=%x x=%x ip=%08x mode=%d\n", cfgbuf.chk, x, cfgbuf.ip, cfgbuf.mode);
}

void deepSleep() {
  delay(1);
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME * uS_TO_S_FACTOR); // Deep-Sleep time in microseconds
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_deep_sleep_start();
}

void callback(char* topic, byte* payload, unsigned int length) {   //callback includes topic and payload ( from which (topic) the payload is comming)
  Serial.println("Message arrived");
}


void setup() {
  setCpuFrequencyMhz(80); // 63mA -> 30mA

  Serial.begin(115200);
  Serial.println("LolinD32-THP");

  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  delay(1);

  sht31.Begin();
  sht31.UpdateData();
  temp_sht31 = sht31.GetTemperature();
  if ((temp_sht31 == 0.)||(temp_sht31 < -40)||(temp_sht31 > 70)) {
    temp_sht31 = -100; // signalize error
  }
  relh_sht31 = sht31.GetRelHumidity();
  if ((relh_sht31 == 0.)||(relh_sht31 < 10)||(relh_sht31 > 90)) {
    relh_sht31 = -100; // signalize error
  }

  if (bmp280.begin(BMP280_ADDRESS_ALT)) {
    bmp280.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */  // handheld device low-power (e.g. Android) Ultra high resolution
                       Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                       Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                       Adafruit_BMP280::FILTER_X4 ,      /* Filtering. */
                       Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    pres_bmp280 = bmp280.readPressure() + ATMOSPHERIC_PRESSURE_OFFSET_PA;
    if ((pres_bmp280 < 90000)||(pres_bmp280 > 110000)) pres_bmp280 = -100; // signalize error
  } else {
    pres_bmp280 = -100; // signalize error
  }

  digitalWrite(POWER_PIN, LOW);
  pinMode(POWER_PIN, INPUT);   // reduces power consumption

  batt_vltg = readBatteryVoltage();
  if (batt_vltg < 3.0) {
    Serial.println("Low battery voltage");
    deepSleep();
  }

  Serial.print("Temperature: ");
  Serial.print(temp_sht31);
  Serial.write("\xC2\xB0"); //The Degree symbol
  Serial.println("C");
  Serial.print("Humidity: ");
  Serial.print(relh_sht31);
  Serial.println("%");
  Serial.print("Pressure: ");
  Serial.print(pres_bmp280);
  Serial.println("Pa");
  Serial.print("Battery: ");
  Serial.print(batt_vltg);
  Serial.println("V");

  // Wifi connection
  checkCfg();

  // Make sure Wifi settings in flash are off so it doesn't start automagically at next boot
  if (WiFi.getMode() != WIFI_OFF) {
    Serial.println("Wifi wasn't off");
    WiFi.persistent(true);
    WiFi.mode(WIFI_OFF);
  }

  // Init Wifi in STA mode and connect
  WiFi.persistent(false);  // Disable the WiFi persistence.  The ESP32 will not load and save WiFi settings in the flash memory.
  WiFi.mode(WIFI_STA);

  bool ok;
  if (cfgbuf.mode) { // repeated connection
    ok = WiFi.config(cfgbuf.ip, cfgbuf.gw, cfgbuf.msk, cfgbuf.dns);
    if (ok) ok = WiFi.begin(WLAN_SSID, WLAN_PASSWD, cfgbuf.chl, cfgbuf.mac);
  } else { // first connection
    ok = WiFi.begin(WLAN_SSID, WLAN_PASSWD);
  }

  if (!ok) {
    Serial.println("Wifi.begin failed");
    cfgbuf.mode = 0;
    deepSleep();
  }

  uint16_t retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    retries++;
    if (retries >= (5000/2)) { // wait max 5s = 2500 iterations with 2ms waiting time
      Serial.println("Cannot connect to WIFI");
      cfgbuf.mode = 0;
      deepSleep();
    }
    delay(2);
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // MQTT connection
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);//connecting to mqtt server
  mqtt.setCallback(callback);
  // PSK cypher - Pre-shared key
  //client.setPreSharedKey(mqtt_ident, mqtt_psk);
  int i = 0;
  while (!mqtt.connected()) {
     i++;
    if (mqtt.connect(DEVICE_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqtt.state());
      if (i < 10) {
        delay(200);
      } else {
        deepSleep();
      }
    }
  }

  //mqtt.loop();
  snprintf (msg, 10, "%04.2f", temp_sht31);
  mqtt.publish(MQTT_SENS_TEMP_TOPIC, msg, false);
  snprintf (msg, 10, "%04.2f", relh_sht31);
  mqtt.publish(MQTT_SENS_HMDT_TOPIC, msg, false);
  snprintf (msg, 10, "%04.2f", pres_bmp280);
  mqtt.publish(MQTT_SENS_PRSS_TOPIC, msg, false);
  snprintf (msg, 10, "%04.2f", batt_vltg);
  mqtt.publish(MQTT_HWRD_BATT_TOPIC, msg, false);

  writecfg();
  Serial.println("done");
  deepSleep();
}

void loop() {
}
