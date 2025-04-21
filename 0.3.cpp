#define TINY_GSM_MODEM_BG96 // EC25

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <time.h>
#include <Update.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SSLClient.h>
#include "cert.h"
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include <WebServer.h>

#include <ModbusMaster.h>
#include "REG_CONFIG_soil.h"
#include <esp_sleep.h>

// #include <HTTPUpdate.h>
// #include <http_ota.h>

struct Meter
{
  // for Soil Sensor
  String Moisture;
  String Temp_Soil;
  String EC;
  String Ph;
  String Nitrogen;
  String Phosphorus;
  String Potassium;
  String bat;
  String rssi;
};

Meter meter;
uint16_t modbusdata[14];

#define FirmwareVersion "0.3"
String Project = "greenio_AIS_Soil_Station_EC25_4G_Magellan";
String current_version = "0.2";

const char *passAP = "greenio7650";
#define pingCount 5 // Error time count 5 to reset
unsigned int CountPing = 0;
#define TINY_GSM_RX_BUFFER 1030
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_TCP true
#define SerialMon Serial
#define GSM_PIN ""
#define CURRENT_VERSION_ADDR 2
#define UART_BAUD 115200

#define MODEM_TX 27
#define MODEM_RX 14
#define GSM_RESET 25

#define dir485 4
#define battPIN 34
#define donePIN 25

#define StepupPin 26
#define POWER_PIN 4 // Edit this

// Your GPRS credentials
const char apn[] = "internet";
const char user[] = "";
const char pass[] = "";

ModbusMaster node;

void sendAttribute();
void processAtt(char jsonAtt[]);
void heartBeat();

int periodSendTelemetry = 60; // the value is a number of seconds

#define WDTPin 33 // Watch Dog pin for Trig


String deviceToken = "";
String user_mqtt = "GreenIO";
String key = "";
String secret = "";
String token = "";
const char *magellanServer = "device-entmagellan.ais.co.th";
const int PORT = 1883;

const char serverOTA[] = "raw.githubusercontent.com";
const int port = 443;

String new_version;
String version_url = "/Jadsadaa/" + Project + "/main/bin_version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path
// const char version_url[] = "/Jadsadaa/" + String(Project) + "/main/bin_version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path

String version_url_WiFiOTA = "https://raw.githubusercontent.com/Jadsadaa/" + Project + "/main/version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path
// const char *version_url_WiFiOTA = "https://raw.githubusercontent.com/Jadsadaa/" + String(Project) + "/main/version.txt"; // "/IndustrialArduino/OTA-on-ESP/release/version.txt";  https://raw.githubusercontent.com/:owner/:repo/master/:path
String firmware_url = "";
// String firmware_url = "/Jadsadaa/greenio_AIS_Soil_Station_EC25_4G_Magellan/main/firmware.bin";

int c_sent = 0;
int c_time = 0;

// WiFi&OTA 参数
String HOSTNAME = "Soil_station-";
#define PASSWORD "green7650" // the password for OTA upgrade, can set it in any char you want

HardwareSerial SerialAT(1);
HardwareSerial modbus(2);
WiFiManager wifiManager;

// GSM Object
TinyGsm modem(SerialAT);

// HTTPS Transport MQTT
TinyGsmClient gsm_mqtt_client(modem, 0);
PubSubClient GSMmqtt(gsm_mqtt_client);

// HTTPS Transport OTA
TinyGsmClient base_client(modem, 1);
SSLClient secure_layer(&base_client);
HttpClient GSMclient = HttpClient(secure_layer, serverOTA, port);

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80);

int pro_ref;

boolean wifi_sta = false;
String host = "";
String mqttStatus = "";
boolean GSMnetwork = false;
boolean GSMgprs = false;

bool connectWifi = false;
String json = "";
String imsi = "";
String NCCID = "";

int status = WL_IDLE_STATUS;

uint32_t lastReconnectAttempt = 0;

unsigned long previous_t1 = 0;
unsigned long previous_t2 = 0;

boolean config = false;
uint8_t EEPROM0;
uint8_t EEPROM1;
uint8_t EEPROM2;

template <typename T>
void printlnSerial(T message)
{
  Serial.println(message);
  if (wifi_sta == true)
  {

    WebSerial.println(message);
  }
}

template <typename T>
void printSerial(T message)
{
  Serial.print(message);
  if (wifi_sta == true)
  {

    WebSerial.print(message);
  }
}

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println(len);
  if (len == 36)
  {
    WebSerial.print("Configtoken : ");
    {
      String d = "";
      for (int i = 0; i < len; i++)
      {
        d += char(data[i]);
        EEPROM.write(i, data[i]);
      }
      EEPROM.commit();
      EEPROM0 = data[0];
      WebSerial.println(d);
    }
  }

  if (len == 19)
  {
    WebSerial.print("ConfigKey : ");
    {
      String d = "";
      for (int i = 0; i < len; i++)
      {
        d += char(data[i]);
        EEPROM.write(i + 37, data[i]);
      }
      EEPROM.commit();
      EEPROM1 = data[0];
      WebSerial.println(d);
    }
  }
  if (len == 15)
  {
    WebSerial.print("ConfigSecret : ");
    {
      String d = "";
      for (int i = 0; i < len; i++)
      {
        d += char(data[i]);
        EEPROM.write(i + 56, data[i]);
      }
      EEPROM.commit();
      EEPROM2 = data[0];
      WebSerial.println(d);
    }
  }
}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  // if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void processAtt(char jsonAtt[])
{
  char *aString = jsonAtt;
  Serial.println("OK");
  Serial.print(F("+:topic v1/devices/me/attributes , "));
  Serial.println(aString);
  client.publish("v1/devices/me/attributes", aString);
}

boolean reconnectWiFiMqtt()
{

  printSerial("Connecting to ");
  printSerial(String(magellanServer));

  boolean status = client.connect(user_mqtt.c_str(), key.c_str(), secret.c_str());
  // boolean status = GSMmqtt.connect("GreenIO", "1665032200000000000", "166522000000000");

  if (status == false)
  {
    printlnSerial(" fail");
    mqttStatus = "Failed to Connect Server with WiFi!";
    return false;
  }
  printlnSerial(" success");
  mqttStatus = "Succeed to Connect Server with WiFi!";
  Serial.println(F("Connect MQTT Success."));
  // client.subscribe(("api/v2/thing/" + token + "/delta/resp").c_str());
  return client.connected();
}

boolean reconnectGSMMqtt()
{
  printSerial("Connecting to ");
  printSerial(String(magellanServer));

  boolean status = GSMmqtt.connect(user_mqtt.c_str(), key.c_str(), secret.c_str());
  // boolean status = GSMmqtt.connect("GreenIO", "1665032200000000000", "166522000000000");

  if (status == false)
  {
    printlnSerial(" fail");
    return false;
  }
  printlnSerial(" success");
  Serial.println(F("Connect MQTT Success."));
  String AAA = "thing";
  // GSMmqtt.subscribe(("api/v2/thing/" + token + "/delta/resp").c_str());
  return GSMmqtt.connected();
}

float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  int Vbat = 0;
  float R1 = 10000.0;
  float R2 = 5000.0;
  int bitRes = 4096;
  int16_t adc = 0;
  // Serial.println("Read_Batt()");
  //  for (int a = 0; a < 10; a++)
  //  {
  //    adc  += analogRead(battPIN);
  //    // Serial.print(adc);
  //    delay(1);
  //  }
  adc = analogRead(battPIN);
  vRAW = adc;
  // vRAW = adc / 10;
  Vout = (vRAW * 3.3) / bitRes;
  Vin = (Vout / (R2 / (R1 + R2)))+0.2;
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }

  // คำนวณเปอร์เซ็นต์แบตเตอรี่
  if (Vin >= 3.7 && Vin <= 4.0)
  {
    Vbat = (Vin - 3.7) / (4.0 - 3.7) * 100;
  }
  else if (Vin < 3.7)
  {
    Vbat = 0;
  }
  else
  {
    Vbat = 100;
  }

  printlnSerial("BAT:" + String(Vin) + " BAT % :" + String(Vbat) + "%");
  // WebSerial.print("BAT:");
  // Serial.println("end.Read_Batt()");
  return Vbat;
}

void preTransmission()
{
  digitalWrite(dir485, HIGH);
}

void postTransmission()
{
  digitalWrite(dir485, LOW);
}

void readModbus(char addr, uint16_t REG, uint8_t Read)
{
  static uint32_t i;
  uint16_t j, result;
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(addr, modbus);
  // node.preTransmission(preTransmission);
  // node.postTransmission(postTransmission);
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, Read);
  // Serial.print("result:"); Serial.println(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    printSerial(REG);
    printSerial(">>>");
    for (j = 0; j < Read; j++)
    {
      modbusdata[j] = node.getResponseBuffer(j);
      delay(500);
      printSerial(j);
      printSerial(":");
      printSerial(modbusdata[j]);
      printSerial("\t");
    }
    printSerial("\n");

    switch (addr)
    {
    case ID_SoilSensor:

      meter.Moisture = String(float(modbusdata[0]) / 10);
      meter.Temp_Soil = String(float(modbusdata[1]) / 10);
      meter.EC = modbusdata[2];
      meter.Ph = String(float(modbusdata[3]) / 10);
      meter.Nitrogen = modbusdata[4];
      meter.Phosphorus = modbusdata[5];
      meter.Potassium = modbusdata[6];
      break;
    default:
      break;
    }
  }
  else
  {
    printSerial("Connec modbus fail. REG >>> ");
    printlnSerial(String(REG, HEX)); // Debug
  }
  delay(500);
}


void composeJson()
{
  json = "";
  printlnSerial("----- JSON Compose -----");
  json.concat("{\"v\":\"");
  json.concat(current_version);

  json.concat("\",\"bat\":");
  json.concat(meter.bat);
  // From modbus
  json.concat(",\"Moisture\":");
  json.concat(meter.Moisture);
  json.concat(",\"Temp\":");
  json.concat(meter.Temp_Soil);
  json.concat(",\"Conductivity\":");
  json.concat(meter.EC);
  json.concat(",\"Ph\":");
  json.concat(meter.Ph);

  if (connectWifi)
  {
    json.concat(",\"Mode\":\"WIFI\"");
    json.concat(",\"wifi_sig\":");
    json.concat(WiFi.RSSI());
  }
  else
  {
    json.concat(",\"Mode\":\"4G\"");
    json.concat(",\"lte_sig\":");
    json.concat(modem.getSignalQuality());
  }
  json.concat("}");
  printlnSerial(json);

  // Length (with one extra character for the null terminator)
  int str_len = json.length() + 1;
  // Prepare the character array (the buffer)
  char char_array[str_len];
  // Copy it over
  json.toCharArray(char_array, str_len);
  String topic = "api/v2/thing/" + token + "/report/persist";
  if (connectWifi)
  {
    client.publish(topic.c_str(), char_array);
  }
  else
  {

    GSMmqtt.publish(topic.c_str(), char_array);
  }
  printlnSerial(" ");
}

void heartBeat()
{
  //   Sink current to drain charge from watchdog circuit
  pinMode(WDTPin, OUTPUT);
  digitalWrite(WDTPin, LOW);

  delay(100);

  // Return to high-Z
  pinMode(WDTPin, INPUT);

  printlnSerial(" ");
  printlnSerial("Trig - Heartbeat (!!)");
  printlnSerial(" ");
}

String a0(int n)
{
  return (n < 10) ? "0" + String(n) : String(n);
}

void getMac()
{
  byte mac[6];
  WiFi.macAddress(mac);
  printlnSerial("OK");
  printSerial("+deviceToken: ");
  printlnSerial(WiFi.macAddress());
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] < 0x10)
    {
      deviceToken += "0"; // Add leading zero if needed
    }
    deviceToken += String(mac[i], HEX); // Convert byte to hex
  }
  deviceToken.toUpperCase();
}

bool checkForUpdate(String &firmware_url)
{
  heartBeat();

  printlnSerial("Making GSM GET request securely...");
  GSMclient.get(version_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  String response_body = GSMclient.responseBody();
  delay(1000);

  printSerial("Status code: ");
  printlnSerial(String(status_code));
  printSerial("Response: ");
  printlnSerial(String(response_body));

  response_body.trim();
  response_body.replace("\r", ""); // Remove carriage returns
  response_body.replace("\n", ""); // Remove newlines

  // Extract the version number from the response
  new_version = response_body;

  printlnSerial("Current version: " + current_version);
  printlnSerial("Available version: " + new_version);
  GSMclient.stop();

  if (new_version != current_version)
  {
    printlnSerial("New version available. Updating...");
    firmware_url = "/Jadsadaa/" + Project + "/main/" + new_version + ".bin";
    printlnSerial("Firmware URL: " + firmware_url);
    return true;
  }
  else
  {
    printlnSerial("Already on the latest version");
  }

  return false;
}

// Update the latest firmware which has uploaded to Github
void performOTA(const char *firmware_url)
{
  heartBeat();

  // Initialize HTTP
  printlnSerial("Making GSM GET firmware OTA request securely...");
  GSMclient.get(firmware_url);
  int status_code = GSMclient.responseStatusCode();
  delay(1000);
  long contentlength = GSMclient.contentLength();
  delay(1000);

  Serial.print("Contentlength: ");
  Serial.println(contentlength);

  if (status_code == 200)
  {

    if (contentlength <= 0)
    {
      printlnSerial("Failed to get content length");
      GSMclient.stop();
      return;
    }

    // Begin OTA update
    bool canBegin = Update.begin(contentlength);
    size_t written;
    long totalBytesWritten = 0;
    uint8_t buffer[1024];
    int bytesRead;
    long contentlength_real = contentlength;

    if (canBegin)
    {
      heartBeat();

      while (contentlength > 0)
      {
        heartBeat();

        bytesRead = GSMclient.readBytes(buffer, sizeof(buffer));
        if (bytesRead > 0)
        {
          written = Update.write(buffer, bytesRead);
          if (written != bytesRead)
          {
            printlnSerial("Error: written bytes do not match read bytes");
            Update.abort();
            return;
          }
          totalBytesWritten += written; // Track total bytes written

          // Serial.printf("Write %.02f%% (%ld/%ld)\n", (float)totalBytesWritten / (float)contentlength_real * 100.0, totalBytesWritten, contentlength_real);

          printlnSerial("Write " + String((float)totalBytesWritten / (float)contentlength_real * 100.0, 2) + "%"); // พิมพ์เปอร์เซ็นต์

          // Serial.printf("Write %.02f%% (%ld/%ld)\n", (float)totalBytesWritten / (float)contentlength_real * 100.0, totalBytesWritten, contentlength_real);

          String OtaStat = "OTA Updating: " + String((float)totalBytesWritten / (float)contentlength_real * 100.0) + " % ";
          contentlength -= bytesRead; // Reduce remaining content length
        }
        else
        {
          printlnSerial("Error: Timeout or no data received");
          break;
        }
      }

      if (totalBytesWritten == contentlength_real)
      {
        printlnSerial("Written : " + String(totalBytesWritten) + " successfully");
      }
      else
      {
        printlnSerial("Written only : " + String(written) + "/" + String(contentlength_real) + ". Retry?");
        // otaStat.deleteSprite();
      }

      if (Update.end())
      {
        printlnSerial("OTA done!");
        if (Update.isFinished())
        {
          printlnSerial("Update successfully completed. Rebooting.");
          delay(300);
          ESP.restart();
        }
        else
        {
          printlnSerial("Update not finished? Something went wrong!");
        }
      }
      else
      {
        printlnSerial("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      printlnSerial("Not enough space to begin OTA");
    }
  }
  else
  {
    printlnSerial("Cannot download firmware. HTTP code: " + String(status_code));
  }

  GSMclient.stop();
}

void GSM_OTA()
{
  printlnSerial("---- GSM OTA Check version before update ----");

  if (checkForUpdate(firmware_url))
  {
    performOTA(firmware_url.c_str());
  }
}
/*
bool WiFicheckForUpdate(String &firmware_url)
{
  heartBeat();
  Serial.println("Making WiFi GET request securely...");

  HTTPClient http;
  http.begin(version_url_WiFiOTA);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    new_version = http.getString();
    new_version.trim();

    Serial.print("Response: ");
    Serial.println(new_version);
    Serial.println("Current version: " + current_version);
    Serial.println("Available version: " + new_version);

    if (new_version != current_version)
    {
      Serial.println("New version available. Updating...");
      firmware_url = String("https://raw.githubusercontent.com/prakit340/GreenIO-OTA/main/ota/product/qualcomm/airmass25/firmware_v") + new_version + ".bin";
      Serial.println("Firmware URL: " + firmware_url);
      return true;
    }
    else
    {
      Serial.println("Already on the latest version");
    }
  }
  else
  {
    Serial.println("Failed to check for update, HTTP code: " + String(httpCode));
  }

  http.end();
  return false;
}

void WiFiperformOTA(const char *firmware_url)
{
  heartBeat();

  // Initialize HTTP
  Serial.println("Making WiFi GET fiemware OTA request securely...");

  HTTPClient http;

  http.begin(firmware_url);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK)
  {
    int contentLength = http.getSize();
    bool canBegin = Update.begin(contentLength);
    long contentlength_real = contentLength;

    Serial.print("Contentlength: ");
    Serial.println(contentLength);

    if (canBegin)
    {
      heartBeat();

      Serial.println("WiFi OTA Updating..");

      size_t written = Update.writeStream(http.getStream());

      if (written == contentLength)
      {
        Serial.println("Written : " + String(written) + " successfully");
      }
      else
      {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
      }

      if (Update.end())
      {
        Serial.println("OTA done!");
        if (Update.isFinished())
        {
          Serial.println("Update successfully completed. Rebooting.");
          delay(300);
          ESP.restart();
        }
        else
        {
          Serial.println("Update not finished? Something went wrong!");
        }
      }
      else
      {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    }
    else
    {
      Serial.println("Not enough space to begin OTA");
    }
  }
  else
  {
    Serial.println("Cannot download firmware. HTTP code: " + String(httpCode));
  }

  http.end();
}

void WiFi_OTA()
{
  Serial.println("---- WiFi OTA Check version before update ----");

  if (WiFicheckForUpdate(firmware_url))
  {
    WiFiperformOTA(firmware_url.c_str());
  }
}
*/

void setupOTA()
{
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  // No authentication by default
  ArduinoOTA.setPassword(PASSWORD);
  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]()
                     {

    Serial.println("Start Updating....");
    WebSerial.println("Start Updating....");

    WebSerial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem"); });
  ArduinoOTA.onEnd([]()
                   { ESP.restart(); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        {

                          int pro = (progress / (total / 100)) ;

                          //    int progressbar = (progress / (total / 100));
                          // int progressbar = (progress / 5) % 100;
                          // int pro = progress / (total / 100);
                          // WebSerial.printf("Progress: %u%%\n", (progress / (total / 100)));
                          if(pro != pro_ref){
                          pro_ref = pro;
                          Serial.printf("Progress: %d %\n", pro);
                          WebSerial.printf("Progress: %d %\n", pro);
                          }

                          // Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
                          // WebSerial.printf("Progress: %u%%\n", (progress / (total / 100)));
                          if (pro ==100){
                          Serial.printf("Progress: %d %\n", pro);
                          WebSerial.printf("Progress: %d\n", pro);
                          WebSerial.println("Update Complete!");
                          Serial.println("Update Complete!");
                          } });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    WebSerial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;
      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;
      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;
      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;
      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }
    Serial.println(info);
    ESP.restart(); });
  ArduinoOTA.begin();
}

void setup()
{
  pinMode(StepupPin, OUTPUT);
  digitalWrite(StepupPin, HIGH);

  pinMode(dir485, OUTPUT);
  digitalWrite(dir485, LOW);

  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);

  Serial.begin(115200);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  esp_sleep_enable_timer_wakeup(1000000 * 60 * 10);

  // wifiManager.setAPCallback(configModeCallback);
  // wifiManager.setConfigPortalTimeout(60); // auto close configportal after n seconds
  // wifiManager.setAPClientCheck(true);     // avoid timeout if client connected to softap
  // wifiManager.setBreakAfterConfig(true);  // always exit configportal even if wifi save fails
  // if (!wifiManager.autoConnect())
  // {
  //   printlnSerial("failed to connect and hit timeout");
  //   delay(1000);
  // }
  // else
  // {

  //   if (WiFi.status() == WL_CONNECTED)
  //   {
  //     wifi_sta = true;
  //     WebSerial.begin(&server);
  //     WebSerial.msgCallback(recvMsg);
  //     server.begin();
  //   }
  //   printlnSerial("connected...yeey :)");
  // }

  EEPROM.begin(512);

  // EEPROM.write(80, 1);
  // EEPROM.commit();
  // for (int i = 0; i < 36; i++)
  // {
  //   EEPROM.write(i, 255);
  // }
  // EEPROM.commit();

  EEPROM0 = EEPROM.read(0);
  while (EEPROM0 == 255)
  {
    printlnSerial("press config Thing Key& Thing Secret & Thing Token~~~");
    delay(1000);
    if (config == false)
    {
      wifiManager.setAPCallback(configModeCallback);
      wifiManager.setConfigPortalTimeout(60); // auto close configportal after n seconds
      wifiManager.setAPClientCheck(true);     // avoid timeout if client connected to softap
      wifiManager.setBreakAfterConfig(true);  // always exit configportal even if wifi save fails
      if (!wifiManager.autoConnect())
      {
        printlnSerial("failed to connect and hit timeout");
        delay(1000);
      }
      else
      {
        if (WiFi.status() == WL_CONNECTED)
        {
          wifi_sta = true;
          WebSerial.begin(&server);
          WebSerial.msgCallback(recvMsg);
          server.begin();
        }
        printlnSerial("connected...yeey :)");
      }
      config = true;
    }
  }

  char readString[37]; // 36 characters + 1 null terminator
  for (int i = 0; i < 36; i++)
  {
    readString[i] = EEPROM.read(i);
  }
  token = String(readString);

  char readString2[20]; // 36 characters + 1 null terminator
  for (int i = 0; i < 19; i++)
  {
    readString2[i] = EEPROM.read(i + 37);
  }
  key = String(readString2);

  char readString3[16]; // 36 characters + 1 null terminator
  for (int i = 0; i < 15; i++)
  {
    readString3[i] = EEPROM.read(i + 56);
  }
  secret = String(readString3);

  printlnSerial("key : " + key);
  printlnSerial("secret : " + secret);
  printlnSerial("token : " + token);

  // c_time = EEPROM.read(100);
  // printlnSerial("c_time :" + String(c_time));
  // if (c_time < 1)
  // {
  //   c_time++;
  //   EEPROM.write(100, c_time);
  //   EEPROM.commit();
  //   printlnSerial("deep_sleep_start");
  //   delay(2000);
  //   esp_deep_sleep_start(); // เริ่มเข้าสู่โหมด Deep Sleep
  // }

  modbus.begin(4800, SERIAL_8N1, 16, 17);
  node.begin(1, modbus);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  heartBeat();
  // getMac();
  heartBeat();

  Serial.println(F("---- Starting... ----"));
  boolean GSMgprs = false;

  delay(2000);
  // pinMode(GSM_RESET, OUTPUT);
  // digitalWrite(GSM_RESET, HIGH); // RS-485
  delay(10);
  printlnSerial("Wait...");

  secure_layer.setCACert(root_ca);
  heartBeat();

  printlnSerial("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  printSerial("Modem: ");
  printlnSerial(modemInfo);

  heartBeat();

  printSerial("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    GSMnetwork = false;
    printlnSerial(" fail");
    delay(10000);
  }
  else
  {
    GSMnetwork = true;
    printlnSerial(" OK");
  }
  delay(2000);
  heartBeat();

  if (GSMnetwork)
  {
    String showText = "Connecting to ";
    showText += apn;
    showText += " ...";
    printlnSerial("Connecting to ");
    printlnSerial(String(apn));
    if (!modem.gprsConnect(apn, user, pass))
    {
      GSMgprs = false;
      printlnSerial(" fail");
      delay(10000);
    }
    else
    {
      GSMgprs = true;
      printlnSerial(" OK");
    }
    delay(3000);
  }

  heartBeat();
  printlnSerial(" ");
  printlnSerial("===================");
  printSerial("GSM Network: ");
  printSerial(GSMnetwork);
  printSerial("\tGSM GPRS: ");
  printlnSerial(String(GSMgprs));
  printlnSerial("===================");
  printlnSerial(" ");

  if ((GSMnetwork == true) && (GSMgprs == true))
  {
    connectWifi = false;
  }

  if ((GSMnetwork == false) || (GSMgprs == false))
  {
    connectWifi = true;
  }
  heartBeat();

  if (connectWifi)
  {
    printlnSerial("Online by WiFi");
    client.setServer(magellanServer, PORT);
    reconnectWiFiMqtt();

    printlnSerial("Token: " + token);
    printlnSerial("Key: " + key);
    printlnSerial("Secret: " + secret);
    setupOTA();
    // WiFi_OTA();
  }
  else
  {
    printlnSerial("Online by GSM");
    GSM_OTA();
    GSMmqtt.setServer(magellanServer, PORT);
    reconnectGSMMqtt();
  }
  delay(3000);
  heartBeat();
  delay(200);
  previous_t1 = 10 * 60;
}

void loop()
{
  unsigned long currentMillis = millis() / 1000;

  if (connectWifi)
  {
    if (!client.connected())
    {
      printlnSerial("=== WiFi MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis() / 1000;
      if (t - lastReconnectAttempt >= 30)
      {
        lastReconnectAttempt = t;
        if (CountPing >= pingCount)
        {
          CountPing = 0;
          ESP.restart();
        }
        CountPing++;

        if (reconnectWiFiMqtt())
        {
          CountPing = 0;
          lastReconnectAttempt = 0;
        }
      }
      delay(100);
      return;
    }
    ArduinoOTA.handle();
    client.loop();
  }
  else
  {
    if (!modem.isNetworkConnected())
    {
      SerialMon.println("Network disconnected");
      if (!modem.waitForNetwork(180000L, true))
      {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isNetworkConnected())
      {
        SerialMon.println("Network re-connected");
      }
      // and make sure GPRS/EPS is still connected
      if (!modem.isGprsConnected())
      {
        SerialMon.println("GPRS disconnected!");
        SerialMon.print(F("Connecting to "));
        SerialMon.print(apn);
        if (!modem.gprsConnect(apn, user, pass))
        {
          SerialMon.println(" fail");
          delay(10000);
          return;
        }
        if (modem.isGprsConnected())
        {
          SerialMon.println("GPRS reconnected");
        }
      }
    }

    if (!GSMmqtt.connected())
    {
      SerialMon.println("=== GSM MQTT NOT CONNECTED ===");
      // Reconnect every 10 seconds
      uint32_t t = millis() / 1000;
      if (t - lastReconnectAttempt >= 30)
      {
        lastReconnectAttempt = t;

        if (CountPing >= pingCount)
        {
          CountPing = 0;
          ESP.restart();
        }
        CountPing++;

        if (reconnectGSMMqtt())
        {
          CountPing = 0;
          lastReconnectAttempt = 0;
        }
      }
      delay(100);
      return;
    }

    GSMmqtt.loop();
  }

  if ((currentMillis - previous_t1) >= 60)
  {
    c_sent++;
    previous_t1 = millis() / 1000;
    meter.bat = Read_Batt();
    readModbus(ID_SoilSensor, Address[0], 7);
    composeJson();
    if (c_sent == 2)
    {
      // c_time = 0;
      // EEPROM.write(100, c_time);
      // EEPROM.commit();
      // printlnSerial("c_time:" + String(c_time));
      delay(1000);
      printlnSerial("deep_sleep_start");
      delay(2000);
      esp_deep_sleep_start(); // เริ่มเข้าสู่โหมด Deep Sleep
    }
  }

  //   if ((currentMillis - previous_t2) >= 300)
  // {
  //   previous_t5 = millis() / 1000;

  //   if (connectWifi)
  //   {
  //     //WiFi_OTA();
  //   }
  //   else
  //   {
  //     GSM_OTA();
  //   }
  // }
}