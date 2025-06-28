//tự động bật tắt đèn dùng chân led_pin17 và bật tắt led15 qua api_rpc
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define WIFI_AP_NAME "Wifi_tầng 2_Dũng"
#define WIFI_PASSWORD "2334445555"
#define TOKEN "93SlARNKjRRDlo6Kot6g"
#define THINGSBOARD_SERVER "192.168.0.101"
#define SERIAL_DEBUG_BAUD 115200

#define LED_PIN_15 15  // Chân GPIO 15 điều khiển đèn LED 1
#define LED_PIN_17 17  // Chân GPIO 17 điều khiển đèn LED 2
#define LIGHT_SENSOR_PIN 34

bool mqttConnected = false;
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  setup_wifi();
  client.setServer(THINGSBOARD_SERVER, 1883);
  client.setCallback(callback);

  pinMode(LED_PIN_15, OUTPUT);
  pinMode(LED_PIN_17, OUTPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);
  digitalWrite(LED_PIN_15, LOW);  // Khởi tạo đèn LED 15 tắt
  digitalWrite(LED_PIN_17, LOW);  // Khởi tạo đèn LED 17 tắt
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_AP_NAME);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", TOKEN, "")) {
      Serial.println("connected");
      mqttConnected = true;
      client.subscribe("v1/devices/me/attributes");
      client.subscribe("v1/devices/me/telemetry");
      client.subscribe("v1/devices/me/rpc/request/+");  // Subscribe topic RPC
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  // Xử lý RPC request
  if (String(topic).startsWith("v1/devices/me/rpc/request/")) {
    String requestId = String(topic).substring(26);  // Lấy request ID
    String method = doc["method"];                   // Lấy phương thức RPC

    if (method == "setLedState") {

      bool state = doc["params"]["state"];  // Lấy trạng thái (true/false)
      changeLedState(15, state);


      String responseTopic = "v1/devices/me/rpc/response/" + requestId;
      String responsePayload = "{\"success\": true}";
      client.publish(responseTopic.c_str(), responsePayload.c_str());
    }
  }
}

void changeLedState(int ledPin, bool state) {
  if (ledPin == 15) {
    digitalWrite(LED_PIN_15, state ? HIGH : LOW);
  } else if (ledPin == 17) {
    digitalWrite(LED_PIN_17, state ? HIGH : LOW);
  }
  String payload = "{\"led\":" + String(ledPin) + ", \"state\":" + String(state) + "}";
  Serial.print("Publishing state: ");
  Serial.println(payload);
}

void sendLightSensorData() {
  Serial.println("Reading light sensor...");
  float lightValue = analogRead(34);
  Serial.print("Light value = ");
  Serial.println(lightValue);

  int rainIntensity = map((int)lightValue, 0, 4095, 0, 100);  // chuyển sang %

  Serial.print("rainIntensity: ");
  Serial.print(rainIntensity);
  Serial.println("%");


  String payload = "{\"light_intensity\":" + String(lightValue) + "}";
  client.publish("v1/devices/me/telemetry", payload.c_str());
  if (lightValue < 1000) {
    Serial.println("Tối -> Bật đèn");
    changeLedState(17, true);  // bật LED 17 nếu ánh sáng yếu
  } else {
    Serial.println("Sáng -> Tắt đèn");
    changeLedState(17, false);  // tắt LED 17 nếu ánh sáng mạnh
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static unsigned long lastSendTelemetry = millis();
  const unsigned long timeout = 5000;

  // CHỈ GỬI NẾU ĐÃ CÓ KẾT NỐI MQTT
  if (mqttConnected && millis() - lastSendTelemetry > timeout) {
    sendLightSensorData();
    Serial.println("Send Telemetry");
    lastSendTelemetry = millis();
  }
}
