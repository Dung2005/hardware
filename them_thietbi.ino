#include <WiFi.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include "MQ135.h"
#include <HTTPClient.h>

#define EEPROM_SIZE 512
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RESET_BUTTON_PIN 27
#define WARN_LED_DO 33
#define WARN_LED_VANG 25
#define WARN_LED_XANH 26

#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#define LED_CONTROLL 2
#define PIN_MQ135 22
MQ135 mq135_sensor = MQ135(PIN_MQ135);
#define LIGH_SENSOR 34
#define SOIL_SENSOR 33
#define RAIN_SENSOR 35
#define RELAY1 4
#define RELAY2 16
#define RELAY3 17

String statusDevice = "";

bool resetButtonPressed = false;
unsigned long resetButtonPressTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

char wifiSSID[32];
char wifiPass[64];
char mqttUser[64];
char mqttServer[64];
char *mqttPass = "";
char *mqttTopic = "v1/devices/me/telemetry";
int mqttPort = 1883;

char provisionDeviceKey[64] = "provision_key_123";
char provisionDeviceSecret[64] = "provision_secret_abc";
char provisionAccessToken[64] = "";
bool isProvisioned = false;

bool wifiConnected = false;
bool mqttConnected = false;
bool isChangedData = false;
//
bool isBLEActive = false;
bool isClaimKeySent = false;

BLECharacteristic *pCharacteristic;
BLEServer *pServer;
String rxValue = "";

bool provisionDevice();
void setWarnLed(const int type);
void setupBLE();
bool connectToWiFi(const char *ssid, const char *password);
bool connectToMQTT();
void sendDataMqtt();
void sendDHT11Data();
void onBLEReceive(String jsonData);
void saveCredentialsToEEPROM();
void loadCredentialsFromEEPROM();
void handleResetButton();
void resetDevice();
void callback(char *topic, byte *payload, unsigned int length);
void requestClaimKey();

class MyBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxData = pCharacteristic->getValue().c_str();
    if (rxData.length() > 0) {
      Serial.println("Received credentials over BLE");
      rxValue += rxData;
      Serial.println("Received Data: " + rxValue);
      if (rxValue.indexOf(';') != -1) {
        onBLEReceive(rxValue);
        Serial.println("BLECALLBACK.LOG -> " + rxValue);
        rxValue = "";
      }
    } else {
      Serial.println("Received empty data");
    }
  }
};
void setup() {
  Serial.begin(115200);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_CONTROLL, OUTPUT);
  pinMode(WARN_LED_DO, OUTPUT);
  pinMode(WARN_LED_VANG, OUTPUT);
  pinMode(WARN_LED_XANH, OUTPUT);
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  client.setCallback(callback);
  dht.begin();
  EEPROM.begin(EEPROM_SIZE);
  loadCredentialsFromEEPROM();
  setWarnLed(0);

  if (statusDevice == "no_data") {
    setWarnLed(2);
    setupBLE();
    while (statusDevice != "connected") {
      if (isChangedData) {
        saveCredentialsToEEPROM();
        statusDevice = "data_found";
        isChangedData = false;
        break;
      }
      delay(200);
    }

    connectToWiFi(wifiSSID, wifiPass);
    if (WiFi.status() == WL_CONNECTED) {
      if (strlen(provisionAccessToken) == 0) {
        Serial.println("No access token, start provisioning...");
        if (provisionDevice()) {
          Serial.println("Provisioning success");
          strlcpy(mqttUser, provisionAccessToken, sizeof(mqttUser));
          EEPROM.writeString(192, provisionAccessToken);
          EEPROM.commit();
          statusDevice = "data_found";
        } else {
          Serial.println("Provisioning failed.");
          while (1) delay(1000);
        }
      }

      strcpy(mqttUser, provisionAccessToken);
      connectToMQTT();

      // Gửi Claim Key
      requestClaimKey();
      delay(1500);  // Chờ server phản hồi

      // Chờ đến khi gửi xong Claim Key (do hàm callback xử lý notify)
      unsigned long waitStart = millis();
      while (!isClaimKeySent && millis() - waitStart < 5000) {
        client.loop();  // Lắng nghe phản hồi từ server
        delay(100);
      }

      // Sau khi gửi xong BLE ClaimKey → tắt BLE
      if (isBLEActive) {
        BLEDevice::deinit();
        esp_bt_controller_disable();
        isBLEActive = false;
      }

      setWarnLed(1);
      //ESP.restart();
    }
  }

  // Nếu đã có dữ liệu sẵn
  if (statusDevice != "no_data") {
    strcpy(mqttUser, provisionAccessToken);
    if (connectToWiFi(wifiSSID, wifiPass)) {
      connectToMQTT();
      setWarnLed(1);
    } else {
      setWarnLed(3);  // WiFi lỗi
    }
  }
}

int timeSend = 5 * 1000;
unsigned long lastTime = 0;

void loop() {
  unsigned long currentTime = millis();

  handleResetButton();

  if (WiFi.status() != WL_CONNECTED) {
    setWarnLed(3);
    WiFi.disconnect();
    WiFi.reconnect();
    delay(1000);
    return;
  }

  if (!mqttConnected || !client.connected()) {
    setWarnLed(3);
    connectToMQTT();
    if (!mqttConnected) {
      delay(1000);
      return;
    }
  }

  setWarnLed(1);

  if (currentTime - lastTime >= timeSend) {
    sendDataMqtt();
    lastTime = currentTime;
  }

  client.loop();
  delay(100);
}

void sendDataMqtt() {
  int lightPercentage = map(analogRead(LIGH_SENSOR), 0, 1023, 0, 100);
  int rainPercentage = map(analogRead(RAIN_SENSOR), 0, 1023, 0, 100);

  String payload = "{";
  payload += "\"light\":" + String(lightPercentage) + ",";
  payload += "\"rain\":" + String(rainPercentage);
  payload += "}";
  client.publish(mqttTopic, payload.c_str());
  Serial.println("Data sent to MQTT: " + payload);
}

void setWarnLed(const int type) {
  bool red = false, green = false, blue = false;
  if (type == 1) red = true;
  if (type == 2) green = true;
  if (type == 3) blue = true;
  digitalWrite(WARN_LED_DO, red);
  digitalWrite(WARN_LED_VANG, green);
  digitalWrite(WARN_LED_XANH, blue);
}

bool connectToWiFi(const char *ssid, const char *password) {
  Serial.println("Connecting to WiFi...");
  Serial.println("CONNECT WITH " + String(ssid) + " | " + String(password));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startTime > 30000) {
      Serial.println("\nFailed to connect to WiFi");
      wifiConnected = false;
      statusDevice = "no_connected";
      return false;
    }
  }
  Serial.println("\nWiFi connected");
  statusDevice = "wifi_connected";
  wifiConnected = true;
  return true;
}

bool connectToMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  Serial.print("Connecting to MQTT...");
  if (client.connect("ESP32Client", mqttUser, mqttPass)) {
    Serial.println("MQTT connected");
    mqttConnected = true;
    // Đăng ký nhận thông tin Shared Attributes khi có thay đổi
    client.subscribe("v1/devices/me/attributes");
    // Gửi yêu cầu lấy giá trị Shared Attributes
    String request = "{\"sharedKeys\":\"lightState,pin_1,pin_2,pin_3\"}";
    client.publish("v1/devices/me/attributes/request/1", request.c_str());
    if (statusDevice == "wifi_connected") {
      statusDevice = "connected";
      setWarnLed(1);
    }
    mqttConnected = true;
    return true;
  } else {
    Serial.print("Failed to connect to MQTT, rc=");
    Serial.println(client.state());
    mqttConnected = false;
    return false;
  }
}
void setupBLE() {
  BLEDevice::init("ESP32_BLE");
  isBLEActive = true;
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->setCallbacks(new MyBLECallbacks());
  pService->start();
  pServer->getAdvertising()->start();
}

void onBLEReceive(String jsonData) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) return;
  if (doc.containsKey("reset") && doc["reset"] == true) {
    resetDevice();
    return;
  }
  strcpy(wifiSSID, doc["ssid"]);
  strcpy(wifiPass, doc["password"]);
  strcpy(mqttUser, doc["mqttUser"]);
  strcpy(mqttServer, doc["mqttServer"]);
  mqttPort = doc["mqttPort"];
  isChangedData = true;
}

bool provisionDevice() {
  HTTPClient http;
  String url = "http://192.168.0.136:8080/api/v1/provision";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  DynamicJsonDocument doc(256);
  doc["provisionDeviceKey"] = provisionDeviceKey;
  doc["provisionDeviceSecret"] = provisionDeviceSecret;
  doc["deviceName"] = "ESP32_" + String(ESP.getEfuseMac(), HEX);

  String payload;
  serializeJson(doc, payload);
  Serial.println("Đang gửi provisioning payload:");
  Serial.println(payload);

  int httpCode = http.POST(payload);
  if (httpCode == 200) {
    String response = http.getString();
    Serial.println("Server Response:");
    Serial.println(response);
    DynamicJsonDocument respDoc(256);
    if (deserializeJson(respDoc, response) == DeserializationError::Ok && respDoc.containsKey("credentialsValue")) {
      strlcpy(provisionAccessToken, respDoc["credentialsValue"], sizeof(provisionAccessToken));
      isProvisioned = true;
      http.end();
      return true;
    }
  }
  http.end();
  return false;
}

void requestClaimKey() {
  if (client.connected()) {
    String claimPayload = "{\"durationMs\":60000}";
    client.publish("v1/devices/me/claim", claimPayload.c_str());
    Serial.println("Sending claim request..." +  claimPayload);
  }
}

void saveCredentialsToEEPROM() {
  EEPROM.writeString(0, wifiSSID);
  EEPROM.writeString(32, wifiPass);
  EEPROM.writeString(96, mqttUser);
  EEPROM.writeString(128, mqttServer);
  EEPROM.write(160, (uint8_t)(mqttPort >> 8));
  EEPROM.write(161, (uint8_t)(mqttPort & 0xFF));
  EEPROM.writeString(192, provisionAccessToken);

  EEPROM.commit();

  Serial.println(" Dữ liệu đã được lưu vào EEPROM:");
  Serial.printf("  Save_EEPROM.Log -> SSID: %s\n", wifiSSID);
  Serial.printf("  Save.EEPROM.log -> Password: %s\n", wifiPass);
  Serial.printf("  Save.EEPROM.Log -> MQTT User (Token): %s\n", mqttUser);
  Serial.printf("  Save.EEPROM.Log -> MQTT Server: %s\n", mqttServer);
  Serial.printf("  Save.EEPROM.Log -> MQTT Port: %d\n", mqttPort);
  Serial.printf("  Save.EEPROM.Log -> Provision Token: %s\n", provisionAccessToken);
}


void loadCredentialsFromEEPROM() {
  EEPROM.readString(0, wifiSSID, 32);
  EEPROM.readString(32, wifiPass, 64);
  EEPROM.readString(96, mqttUser, 64);
  EEPROM.readString(128, mqttServer, 64);
  mqttPort = (EEPROM.read(160) << 8) | EEPROM.read(161);
  EEPROM.readString(192, provisionAccessToken, 64);

  Serial.println(" Đang đọc dữ liệu từ EEPROM:");
  Serial.printf("  Save_EEPROM.Log -> SSID: %s\n", wifiSSID);
  Serial.printf("  Save.EEPROM.log -> Password: %s\n", wifiPass);
  Serial.printf("  Save.EEPROM.Log -> MQTT User (Token): %s\n", mqttUser);
  Serial.printf("  Save.EEPROM.Log -> MQTT Server: %s\n", mqttServer);
  Serial.printf("  Save.EEPROM.Log -> MQTT Port: %d\n", mqttPort);
  Serial.printf("  Save.EEPROM.Log -> Provision Token: %s\n", provisionAccessToken);

  if (strlen(wifiSSID) == 0 || strlen(wifiPass) == 0 || strlen(mqttUser) == 0 || strlen(mqttServer) == 0) {
    statusDevice = "no_data";
  } else {
    statusDevice = "data_found";
  }
}

#define DEBOUNCE_DELAY 50

void handleResetButton() {
  static unsigned long lastDebounceTime = 0;
  int buttonState = digitalRead(RESET_BUTTON_PIN);

  if (buttonState == HIGH) {
    if (!resetButtonPressed && millis() - lastDebounceTime > DEBOUNCE_DELAY) {
      resetButtonPressTime = millis();
      resetButtonPressed = true;
      lastDebounceTime = millis();
    } else if (resetButtonPressed && millis() - resetButtonPressTime > 3000) {
      resetDevice();
      resetButtonPressed = false;
    }
  } else {
    resetButtonPressed = false;
  }
}

void resetDevice() {
  memset(wifiSSID, 0, sizeof(wifiSSID));
  memset(wifiPass, 0, sizeof(wifiPass));
  memset(mqttUser, 0, sizeof(mqttUser));
  memset(mqttServer, 0, sizeof(mqttServer));
  mqttPort = 0;
  memset(provisionAccessToken, 0, sizeof(provisionAccessToken));
  saveCredentialsToEEPROM();
  ESP.restart();
}

void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (String(topic).endsWith("/claim/response")) {
    if (isBLEActive && pCharacteristic) {
      pCharacteristic->setValue(message.c_str());
      pCharacteristic->notify();
      isClaimKeySent = true;
      Serial.println("Claim key sent via BLE: " + message);
    }
  }
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  if (!error) {
    bool lightState = doc["lightState"];
    if (doc["shared"]["lightState"])
      lightState = doc["shared"]["lightState"];
    if (doc["shared"]["timeSend"])
      timeSend = doc["shared"]["timeSend"];
    if (doc["timeSend"])
      timeSend = doc["timeSend"];
    digitalWrite(LED_CONTROLL, lightState ? HIGH : LOW);

    if (doc["shared"].containsKey("pin_1"))
      digitalWrite(RELAY1, doc["shared"]["pin_1"]);
    if (doc["shared"].containsKey("pin_2"))
      digitalWrite(RELAY2, doc["shared"]["pin_2"]);
    if (doc["shared"].containsKey("pin_3"))
      digitalWrite(RELAY3, doc["shared"]["pin_3"]);

    if (doc.containsKey("pin_1"))
      digitalWrite(RELAY1, doc["pin_1"]);
    if (doc.containsKey("pin_2"))
      digitalWrite(RELAY2, doc["pin_2"]);
    if (doc.containsKey("pin_3"))
      digitalWrite(RELAY3, doc["pin_3"]);
  }
}
