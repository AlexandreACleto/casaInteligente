#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

#define LED_PIN 2

// ===== WIFI =====
#define WIFI_SSID "."
#define WIFI_PASS "."

// ===== MQTT =====
const char* MQTT_HOST = "172.20.10.4";   // IP do broker (seu PC)
const uint16_t MQTT_PORT = 1883;
// Opcional: defina um nome fixo para o dispositivo; se vazio, usa MAC
#define DEVICE_NAME "Esp-1"

// ===== HTTP =====
WebServer server(80);

// ===== MQTT clients =====
WiFiClient net;
PubSubClient mqtt(net);

// Estado
bool ledState = false;

// IDs/Tópicos dinâmicos
String deviceId;                // ex.: "A1B2C3"
char topicCmd[64];              // esp32/<ID>/led/set
char topicCmdAll[] = "esp32/all/led/set";
char topicState[64];            // esp32/<ID>/led/state
char topicStatus[64];           // esp32/<ID>/status
char mqttClientId[48];          // esp32-led-<ID>

// ---------- Helpers ----------
String htmlPage() {
  String stateText = ledState ? "LIGADO" : "DESLIGADO";
  String color = ledState ? "#16a34a" : "#b91c1c";
  String buttons = R"(
<div style="display:flex; gap:12px; margin-top:16px;">
  <a href="/on"><button style="padding:10px 18px; font-size:16px;">Ligar</button></a>
  <a href="/off"><button style="padding:10px 18px; font-size:16px;">Desligar</button></a>
  <a href="/toggle"><button style="padding:10px 18px; font-size:16px;">Alternar</button></a>
</div>
)";
  String html = R"(
<!doctype html><html lang="pt-br"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>ESP32 LED</title></head>
<body style="font-family:Arial,Helvetica,sans-serif; padding:24px;">
<h2>ESP32 — Controle do LED interno</h2>
<p>Dispositivo: <code>)" + deviceId + R"(</code></p>
<p>Estado atual do LED: <b style="color:)" + color + R"(;">)" + stateText + R"(</b></p>)" + buttons + R"(
<p style="margin-top:24px; color:#555;">
Topics individuais: <code>esp32/)" + deviceId + R"(/led/set</code> | <code>esp32/)" + deviceId + R"(/led/state</code><br>
Broadcast: <code>esp32/all/led/set</code>
</p></body></html>)";
  return html;
}

void publishState() {
  mqtt.publish(topicState, ledState ? "ON" : "OFF", true);
}

void setLed(bool on) {
  ledState = on;
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  if (mqtt.connected()) publishState();
}

bool parseOnOff(const char* payload) {
  String p(payload); p.toUpperCase();
  if (p == "ON" || p == "1")  return true;
  if (p == "OFF" || p == "0") return false;
  return ledState;
}

// ---------- HTTP ----------
void handleRoot()    { server.send(200, "text/html", htmlPage()); }
void handleOn()      { setLed(true);  server.send(200, "text/html", htmlPage()); }
void handleOff()     { setLed(false); server.send(200, "text/html", htmlPage()); }
void handleToggle()  { setLed(!ledState); server.send(200, "text/html", htmlPage()); }
void handleNotFound(){ server.send(404, "text/plain", "Use /, /on, /off ou /toggle"); }

// ---------- Wi-Fi ----------
void connectWiFi() {
  Serial.printf("Conectando a %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (millis() - t0 > 20000) { Serial.println("\nWiFi FAIL. Reiniciando..."); delay(2000); ESP.restart(); }
  }
  Serial.printf("\nWiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());
}

// ---------- ID/Topics ----------
String getDeviceId() {
  if (String(DEVICE_NAME).length() > 0) return String(DEVICE_NAME);
  String mac = WiFi.macAddress(); // "AA:BB:CC:DD:EE:FF"
  mac.replace(":", "");
  String id = mac.substring(6);   // últimos 6 hex -> "DDEEFF"
  id.toUpperCase();
  return id;
}

void buildTopics() {
  snprintf(topicCmd,   sizeof(topicCmd),   "esp32/%s/led/set",   deviceId.c_str());
  snprintf(topicState, sizeof(topicState), "esp32/%s/led/state", deviceId.c_str());
  snprintf(topicStatus,sizeof(topicStatus),"esp32/%s/status",    deviceId.c_str());
  snprintf(mqttClientId,sizeof(mqttClientId),"esp32-led-%s",     deviceId.c_str());
  Serial.printf("Topics:\n  CMD:   %s\n  STATE: %s\n  LWT:   %s\n", topicCmd, topicState, topicStatus);
}

// ---------- MQTT ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  static char buf[64];
  length = min(length, (unsigned int)(sizeof(buf) - 1));
  memcpy(buf, payload, length); buf[length] = '\0';

  // Aceita comando se veio do seu tópico individual OU do broadcast
  if (strcmp(topic, topicCmd) == 0 || strcmp(topic, topicCmdAll) == 0) {
    String p(buf); p.toUpperCase();
    if (p == "TOGGLE") setLed(!ledState);
    else               setLed(parseOnOff(buf));
  }
}

void connectMQTT() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  while (!mqtt.connected()) {
    Serial.print("MQTT conectando...");
    if (mqtt.connect(mqttClientId, topicStatus, 0, true, "offline")) {
      Serial.println("OK");
      mqtt.publish(topicStatus, "online", true);
      mqtt.subscribe(topicCmd);
      mqtt.subscribe(topicCmdAll);
      publishState();
    } else {
      Serial.printf("fail rc=%d\n", mqtt.state());
      delay(1000);
    }
  }
}

// ---------- Setup/Loop ----------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(150);

  connectWiFi();
  deviceId = getDeviceId();
  buildTopics();
  connectMQTT();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/on", HTTP_GET, handleOn);
  server.on("/off", HTTP_GET, handleOff);
  server.on("/toggle", HTTP_GET, handleToggle);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP pronto.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();
  server.handleClient();
}
