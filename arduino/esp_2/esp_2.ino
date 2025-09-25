#include <ESP32Servo.h>  // Biblioteca para controlar servos no ESP32
#include <WiFi.h>
#include <PubSubClient.h>

// -------------------- PINAGEM --------------------
// --- Ambiente ---
const int LDR_PIN     = 32;  // Pino analógico do LDR (usado com divisor resistivo)
const int PIR_PIN     = 12;  // Pino digital do sensor de presença PIR
const int LED_SALA    = 27;  // LED que simula iluminação da sala
const int SERVO_PIN   = 33;  // Pino PWM que controla o servo da janela
// --- Alarme de gás ---
#define MQ2_DO    18   // Saída digital do sensor MQ-2
#define LED_GAS   17   // LED indicador de detecção de gás
#define RELAY_PIN 19   // Relé (pode acionar exaustor ou outro dispositivo)
#define BUZZER_PIN 21  // Buzzer para alarme sonoro
// -------------------- SERVO --------------------
// Configurações de posição e movimento do servo
const int SERVO_FECHADO  = 10;   // Ângulo da posição fechada
const int SERVO_ABERTO   = 170;  // Ângulo da posição aberta
const int SERVO_PASSO    = 1;    // Incremento de movimento (passo por vez)
const int SERVO_DELAY_MS = 10;   // Delay entre passos (movimento suave)
// -------------------- LDR --------------------
// Controle de luminosidade com média móvel e histerese
const int N_SAMPLES = 10;        // Número de amostras para média móvel
int LIMIAR_CLARO  = 1200;        // Considera claro se valor ≥ 1200
int LIMIAR_ESCURO = 1000;        // Considera escuro se valor ≤ 1000
// -------------------- PRESENÇA --------------------
// Configuração do PIR (tempo de aquecimento e "hold")
const unsigned long PIR_WARMUP_MS = 20000UL;  // ~20 segundos para estabilizar
const unsigned long PIR_HOLD_MS   = 5000UL;   // Mantém presença ativa por 5s
// -------------------- VARIÁVEIS DE ESTADO --------------------
// Guardam estados atuais do sistema
Servo janela;                       // Objeto que representa o servo da janela
bool estadoJanelaAberta = false;    // Estado atual da janela (aberta/fechada)
bool estadoLedSalaAceso = false;    // Estado atual da luz da sala (ligada/desligada)
// Controle de tempo para o PIR
unsigned long inicioMs = 0;         // Marca quando o sistema iniciou
unsigned long ultimoMovimentoMs = 0;// Última detecção de presença
// -------------------- Wi‑Fi e MQTT --------------------
const char* ssid       = "iPhone de Alexandre A G";
const char* password   = "alexagc41";
const char* mqttServer = "172.20.10.4"; // exemplo: "broker.hivemq.com"
const int   mqttPort   = 1883;                // porta padrão MQTT sem TLS

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long ultimoStatusMs = 0;
const unsigned long INTERVALO_STATUS_MS = 20000; // 20 s
// Buffer circular para média móvel do LDR
int amostras[N_SAMPLES];  
int idx = 0;  
long soma = 0;  
// -------------------- FUNÇÕES AUXILIARES --------------------
// Retorna média das últimas leituras do LDR
int mediaLdr() {
 return (int)(soma / N_SAMPLES);
}
// Atualiza buffer circular do LDR com nova leitura
void adicionarAmostra(int valor) {
 soma -= amostras[idx];           // Remove valor mais antigo da soma
 amostras[idx] = valor;           // Substitui pelo novo valor
 soma += valor;                   // Adiciona novo valor à soma
 idx = (idx + 1) % N_SAMPLES;     // Avança posição no buffer circular
}
// Movimenta servo suavemente de um ângulo a outro
void moverServoSuave(int fromAngle, int toAngle) {
 if (fromAngle == toAngle) return;   // Se ângulos iguais, não faz nada
 int passo = (toAngle > fromAngle) ? SERVO_PASSO : -SERVO_PASSO;
 for (int a = fromAngle; (passo > 0) ? (a <= toAngle) : (a >= toAngle); a += passo) {
   janela.write(a);                 // Escreve ângulo no servo
   delay(SERVO_DELAY_MS);           // Pausa para suavizar movimento
 }
}
// Abre a janela caso esteja fechada
void abrirJanela() {
 if (!estadoJanelaAberta) {
   moverServoSuave(SERVO_FECHADO, SERVO_ABERTO);
   estadoJanelaAberta = true;
   Serial.println(F("[ACAO] Janela ABERTA"));
 }
}
// Fecha a janela caso esteja aberta
void fecharJanela() {
 if (estadoJanelaAberta) {
   moverServoSuave(SERVO_ABERTO, SERVO_FECHADO);
   estadoJanelaAberta = false;
   Serial.println(F("[ACAO] Janela FECHADA"));
 }
}
// Liga o LED da sala
void ligarLedSala() {
 if (!estadoLedSalaAceso) {
   digitalWrite(LED_SALA, HIGH);
   estadoLedSalaAceso = true;
   Serial.println(F("[ACAO] LED SALA LIGADO (escuro + presença)"));
 }
}
// Desliga o LED da sala
void desligarLedSala() {
 if (estadoLedSalaAceso) {
   digitalWrite(LED_SALA, LOW);
   estadoLedSalaAceso = false;
   Serial.println(F("[ACAO] LED SALA DESLIGADO"));
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
 Serial.begin(115200);  // Inicializa comunicação serial
 // --- PIR / LDR / LED sala ---
 pinMode(PIR_PIN, INPUT);        // PIR como entrada
 pinMode(LED_SALA, OUTPUT);      // LED como saída
 digitalWrite(LED_SALA, LOW);    // Garante LED desligado
 // Configuração inicial do servo
 janela.attach(SERVO_PIN);
 janela.write(SERVO_FECHADO);    // Janela inicia fechada
 // Inicializa média móvel do LDR com a primeira leitura
 int leitura0 = analogRead(LDR_PIN);
 for (int i = 0; i < N_SAMPLES; i++) {
   amostras[i] = leitura0;
   soma += leitura0;
 }
 inicioMs = millis();  // Marca o tempo de início do sistema
 // --- MQ-2 + Alarme ---
 pinMode(MQ2_DO, INPUT);
 pinMode(LED_GAS, OUTPUT);
 pinMode(RELAY_PIN, OUTPUT);
 pinMode(BUZZER_PIN, OUTPUT);
 // Garante que todos os atuadores de alarme começam desligados
 digitalWrite(LED_GAS, LOW);
 digitalWrite(RELAY_PIN, LOW);
 digitalWrite(BUZZER_PIN, LOW);
 Serial.println(F("Sistema iniciado."));
 Serial.println(F("Aguardando aquecimento do PIR real..."));
 setupWifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
}
// -------------------- LOOP PRINCIPAL --------------------
void loop() {
 unsigned long agora = millis();  // Tempo atual

 // Assegura que estamos conectados ao MQTT
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
 // ===== 1) LDR: média móvel + histerese =====
 int leitura = analogRead(LDR_PIN);  // Nova leitura do LDR
 adicionarAmostra(leitura);          // Atualiza buffer circular
 int luzMedia = mediaLdr();          // Calcula média das leituras
 static bool estaClaro = false;      // Estado lógico (CLARO/ESCURO)
 if (!estaClaro && luzMedia >= LIMIAR_CLARO) {
   estaClaro = true;
 } else if (estaClaro && luzMedia <= LIMIAR_ESCURO) {
   estaClaro = false;
 }
 // ===== 2) PRESENÇA (PIR real) =====
 bool pirPronto = (agora - inicioMs) > PIR_WARMUP_MS;     // PIR só conta após aquecimento
 bool pirLeituraReal = (digitalRead(PIR_PIN) == HIGH);    // Leitura atual do PIR
 if (pirLeituraReal && pirPronto) {
   ultimoMovimentoMs = agora;  // Atualiza última detecção
 }
 bool haPresenca = pirPronto && (agora - ultimoMovimentoMs <= PIR_HOLD_MS);
 // ===== 3) Controle do LED da sala =====
 if (!estaClaro && haPresenca) ligarLedSala();
 else desligarLedSala();
 // ===== 4) Controle automático da janela =====
 if (estaClaro && haPresenca) abrirJanela();
 else fecharJanela();
 // ===== 5) MQ-2: Detecção de gás =====
 int estadoMQ2 = digitalRead(MQ2_DO);
 if (estadoMQ2 == LOW) {  // LOW normalmente = gás detectado
   digitalWrite(LED_GAS, HIGH);
   digitalWrite(RELAY_PIN, HIGH);
   digitalWrite(BUZZER_PIN, HIGH);
   Serial.println("⚠️ GÁS DETECTADO! LED_GAS, RELÉ e BUZZER ativados.");
 } else {
   digitalWrite(LED_GAS, LOW);
   digitalWrite(RELAY_PIN, LOW);
   digitalWrite(BUZZER_PIN, LOW);
 }
 // ===== 6) LOG a cada 1 segundo =====
 static unsigned long ultimoLog = 0;
 if (agora - ultimoLog > 1000) {
   ultimoLog = agora;
   Serial.print(F("LDR(avg)=")); Serial.print(luzMedia);
   Serial.print(F(" | Claro? ")); Serial.print(estaClaro ? "SIM" : "NAO");
   Serial.print(F(" | PIR=")); Serial.print((digitalRead(PIR_PIN) == HIGH) ? "HIGH" : "LOW");
   Serial.print(F(" | Presenca? ")); Serial.print(haPresenca ? "SIM" : "NAO");
   Serial.print(F(" | LED_SALA=")); Serial.print(estadoLedSalaAceso ? "ON" : "OFF");
   Serial.print(F(" | Janela=")); Serial.print(estadoJanelaAberta ? "ABERTA" : "FECHADA");
   Serial.print(F(" | MQ2=")); Serial.println(estadoMQ2 == LOW ? "GAS!" : "OK");
   // publica que está vivo
    client.publish("home/status/esp1", "alive");
    // publica o estado da luz e da cortina
    client.publish("home/light", estadoLedSalaAceso ? "on" : "off");
    client.publish("home/curtain", estadoJanelaAberta ? "open" : "closed");
    client.publish("home/smoke", estadoMQ2 ? "GAS!" : "OK");
    client.publish("home/pres", haPresenca ? "SIM" : "NAO");
  
 }

    
 delay(5);
}