/*
 * Exemplo de firmware para ESP32 que controla uma luz (LED) e uma cortina (servo).
 * Além das funcionalidades de sensor de presença e LDR, envia atualizações de
 * estado via MQTT e responde a pings para sinalizar que está vivo.
 *
 * Este código utiliza a biblioteca PubSubClient para comunicação MQTT.
 * Ajuste as credenciais de Wi‑Fi e do broker MQTT conforme seu ambiente.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// =================== MODO DE USO ===================
// Tinkercad/sim: deixar 1. Hardware real: 0 (usa PIR real em GPIO26).
#define MODO_TINKERCAD 1

// -------------------- PINAGEM (ESP32) --------------------
const int LDR_PIN   = 34;  // ADC1_CH6 (entrada somente)
const int PIR_PIN   = 26;  // PIR (digital IN)
const int LED_PIN   = 23;  // LED (sala)
const int SERVO_PIN = 18;  // Servo (PWM LEDC)
const int BOTAO_PIN = 19;  // Botão MANUAL (abre/fecha) - INPUT_PULLUP

// -------------------- SERVO --------------------
const int SERVO_FECHADO  = 5;    // ajuste ao seu hardware
const int SERVO_ABERTO   = 100;  // ajuste ao seu hardware
const int SERVO_PASSO    = 1;
const int SERVO_DELAY_MS = 10;

const int SERVO_MIN_US = 500;    // ajuste fino conforme servo
const int SERVO_MAX_US = 2400;   // ~500–2400 µs

// -------------------- LDR --------------------
const int N_SAMPLES = 10;
int LIMIAR_CLARO  = 800;  // só muda pra CLARO acima disso
int LIMIAR_ESCURO = 300;  // só muda pra ESCURO abaixo disso

// -------------------- PRESENÇA --------------------
#if MODO_TINKERCAD
const unsigned long PIR_WARMUP_MS = 0UL;       // sem aquecimento no simulado
#else
const unsigned long PIR_WARMUP_MS = 20000UL;   // ~20 s no PIR real
#endif
const unsigned long PIR_HOLD_MS   = 15000UL;   // segura presença por 15 s

// -------------------- VARIÁVEIS DE ESTADO --------------------
Servo janela;
bool estadoJanelaAberta = false;
bool estadoLedAceso     = false;

unsigned long inicioMs = 0;
unsigned long ultimoMovimentoMs = 0;  // última detecção de presença efetiva

// LDR média móvel
int  amostras[N_SAMPLES];
int  idx = 0;
long soma = 0;

// Debounce BOTÃO manual
bool ultimoEstadoBotao = HIGH; // INPUT_PULLUP: solto=HIGH, pressionado=LOW
unsigned long ultimoTempoBotao = 0;
const unsigned long debounceMsManual = 50;

// Debounce BOTÃO que simula o PIR (borda)
int  pirUltimoEstado = HIGH;           // INPUT_PULLUP (no sim)
unsigned long pirUltimaMudanca = 0;
const unsigned long debounceMsPIR = 40;

// Controle manual x automático
bool controleManual = false;
unsigned long tempoUltimoBotaoManual = 0;
const unsigned long TEMPO_BLOQUEIO_AUTOMATICO = 7000; // 7 s após botão manual

// -------------------- Wi‑Fi e MQTT --------------------
const char* ssid       = "YOUR_WIFI_SSID";
const char* password   = "YOUR_WIFI_PASSWORD";
const char* mqttServer = "YOUR_MQTT_BROKER"; // exemplo: "broker.hivemq.com"
const int   mqttPort   = 1883;                // porta padrão MQTT sem TLS

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long ultimoStatusMs = 0;
const unsigned long INTERVALO_STATUS_MS = 20000; // 20 s

// -------------------- AUXILIARES --------------------
int mediaLdr() { return (int)(soma / N_SAMPLES); }

void adicionarAmostra(int valor) {
  soma -= amostras[idx];
  amostras[idx] = valor;
  soma += valor;
  idx = (idx + 1) % N_SAMPLES;
}

void moverServoSuave(int fromAngle, int toAngle) {
  if (fromAngle == toAngle) return;
  int passo = (toAngle > fromAngle) ? SERVO_PASSO : -SERVO_PASSO;
  for (int a = fromAngle; (passo > 0) ? (a <= toAngle) : (a >= toAngle); a += passo) {
    janela.write(a);
    delay(SERVO_DELAY_MS);
  }
}

void abrirJanela() {
  if (!estadoJanelaAberta) {
    moverServoSuave(SERVO_FECHADO, SERVO_ABERTO);
    estadoJanelaAberta = true;
    Serial.println(F("[ACAO] Janela ABERTA"));
  }
}

void fecharJanela() {
  if (estadoJanelaAberta) {
    moverServoSuave(SERVO_ABERTO, SERVO_FECHADO);
    estadoJanelaAberta = false;
    Serial.println(F("[ACAO] Janela FECHADA"));
  }
}

void ligarLed() {
  if (!estadoLedAceso) {
    digitalWrite(LED_PIN, HIGH);
    estadoLedAceso = true;
    Serial.println(F("[ACAO] LED LIGADO (escuro)"));
  }
}

void desligarLed() {
  if (estadoLedAceso) {
    digitalWrite(LED_PIN, LOW);
    estadoLedAceso = false;
    Serial.println(F("[ACAO] LED DESLIGADO (claro)"));
  }
}

// -------------------- MQTT callback --------------------
// Trata mensagens recebidas. Neste exemplo, respondemos a pings publicados pelo dashboard.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topico = String(topic);
  String mensagem;
  for (unsigned int i = 0; i < length; i++) {
    mensagem += (char)payload[i];
  }
  Serial.print("[MQTT] Mensagem recebida em ");
  Serial.print(topico);
  Serial.print(": ");
  Serial.println(mensagem);
  // Se recebermos um ping, respondemos com um status para sinalizar que estamos conectados
  if (topico == "home/ping") {
    client.publish("home/status/esp1", "alive");
  }
  // Poderia adicionar mais comandos (ex.: forçar abrir/fechar cortina via MQTT)
}

// Conecta/reconecta ao broker MQTT
void reconnectMQTT() {
  // Fica em loop até conectar
  while (!client.connected()) {
    Serial.print("Conectando ao broker MQTT...");
    String clientId = "esp32_light_curtain_";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("conectado");
      // Inscreve-se no tópico de ping para responder à página web
      client.subscribe("home/ping");
    } else {
      Serial.print("falha, rc=");
      Serial.print(client.state());
      Serial.println(" tentando novamente em 5s");
      delay(5000);
    }
  }
}

// Conecta ao Wi‑Fi
void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando em ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

#if MODO_TINKERCAD
  pinMode(PIR_PIN, INPUT_PULLUP);  // botão simulando PIR
#else
  pinMode(PIR_PIN, INPUT);         // saída digital do PIR real
#endif

  pinMode(BOTAO_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // ---- SERVO (ESP32) ----
  janela.setPeriodHertz(50); // 50 Hz
  janela.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  janela.write(SERVO_FECHADO);

  // ---- ADC (ESP32) ----
  analogReadResolution(12); // 0..4095
  analogSetPinAttenuation(LDR_PIN, ADC_11db);

  // Inicializa buffer da média móvel
  int leitura0 = analogRead(LDR_PIN);
  for (int i = 0; i < N_SAMPLES; i++) {
    amostras[i] = leitura0;
    soma += leitura0;
  }

  inicioMs = millis();
  Serial.println(F("Sistema iniciado (ESP32)."));
#if !MODO_TINKERCAD
  Serial.println(F("Aguardando aquecimento do PIR real..."));
#endif
  // Conecta Wi‑Fi e MQTT
  setupWifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
}

// -------------------- LOOP --------------------
void loop() {
  unsigned long agora = millis();

  // Assegura que estamos conectados ao MQTT
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // ========== 1) BOTÃO MANUAL (abre/fecha) ==========
  int leituraBotao = digitalRead(BOTAO_PIN);
  if (leituraBotao != ultimoEstadoBotao && (agora - ultimoTempoBotao) > debounceMsManual) {
    ultimoTempoBotao = agora;
    ultimoEstadoBotao = leituraBotao;
    if (leituraBotao == LOW) { // pressionado
      if (estadoJanelaAberta) fecharJanela();
      else abrirJanela();
      controleManual = true;
      tempoUltimoBotaoManual = agora;
    }
  }

  if (controleManual && (agora - tempoUltimoBotaoManual > TEMPO_BLOQUEIO_AUTOMATICO)) {
    controleManual = false;
  }

  // ========== 2) LDR com média móvel + histerese ==========
  int leitura = analogRead(LDR_PIN);
  adicionarAmostra(leitura);
  int luzMedia = mediaLdr();
  static bool estaClaro = false; // estado com histerese
  if (!estaClaro && luzMedia >= LIMIAR_CLARO) {
    estaClaro = true;
  } else if (estaClaro && luzMedia <= LIMIAR_ESCURO) {
    estaClaro = false;
  }

  if (estaClaro) desligarLed();
  else ligarLed();

  // ========== 3) PRESENÇA (PIR real OU botão simulando) ==========
#if MODO_TINKERCAD
  int pirLeitura = digitalRead(PIR_PIN);
  bool pirPronto = true; // sem warmup
  if ((pirLeitura != pirUltimoEstado) && (agora - pirUltimaMudanca > debounceMsPIR)) {
    pirUltimaMudanca = agora;
    pirUltimoEstado = pirLeitura;
    if (pirLeitura == LOW) {
      ultimoMovimentoMs = agora;
      Serial.println(F("[EVENTO] Presença simulada (botão PIR)"));
    }
  }
#else
  bool pirPronto = (agora - inicioMs) > PIR_WARMUP_MS;
  bool pirLeituraReal = (digitalRead(PIR_PIN) == HIGH);
  if (pirLeituraReal && pirPronto) {
    ultimoMovimentoMs = agora;
  }
#endif
  bool haPresenca = pirPronto && (agora - ultimoMovimentoMs <= PIR_HOLD_MS);

  // ========== 4) CONTROLE AUTOMÁTICO DA JANELA ==========
  if (!controleManual) {
    if (estaClaro && haPresenca) abrirJanela();
    else                         fecharJanela();
  }

  // ========== 5) LOG e envio MQTT ==========
  static unsigned long ultimoLog = 0;
  if (agora - ultimoLog > 1000) {
    ultimoLog = agora;
    Serial.print(F("LDR(avg)=")); Serial.print(luzMedia);
    Serial.print(F(" | Claro? ")); Serial.print(estaClaro ? "SIM" : "NAO");
#if MODO_TINKERCAD
    Serial.print(F(" | PIR(sim)=")); Serial.print((digitalRead(PIR_PIN) == LOW) ? "BTN" : "IDLE");
#else
    Serial.print(F(" | PIR(real)=")); Serial.print((digitalRead(PIR_PIN) == HIGH) ? "HIGH" : "LOW");
#endif
    Serial.print(F(" | Presenca? ")); Serial.print(haPresenca ? "SIM" : "NAO");
    Serial.print(F(" | LED=")); Serial.print(estadoLedAceso ? "ON" : "OFF");
    Serial.print(F(" | Janela=")); Serial.println(estadoJanelaAberta ? "ABERTA" : "FECHADA");
  }

  // Envia status periodicamente
  if (agora - ultimoStatusMs > INTERVALO_STATUS_MS) {
    ultimoStatusMs = agora;
    // publica que está vivo
    client.publish("home/status/esp1", "alive");
    // publica o estado da luz e da cortina
    client.publish("home/light", estadoLedAceso ? "on" : "off");
    client.publish("home/curtain", estadoJanelaAberta ? "open" : "closed");
  }

  // Pequeno atraso para não travar a CPU
  delay(5);
}