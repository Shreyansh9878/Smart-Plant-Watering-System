#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <string.h> 
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <DHT.h>

const char *ssid = "ShreyanshWiFi";          
const char *password = "0123456789";   

// Your MQTT broker ID
const char *mqtt_server = "192.168.185.93";
const int mqttPort = 1883;

// MQTT topics
const char *publishTopicPlant1Moisture = "plant1-moisture";
const char *publishTopicPlant2Moisture = "plant2-moisture";

const char *publishTopicPlant1PIR = "plant1-pir";
const char *publishTopicPlant2PIR = "plant2-pir";

const char *publishTopicPlant1Motor = "plant1-motor";
const char *publishTopicPlant2Motor = "plant2-motor";
const char *subscribeTopicPlant1Motor = "plant1-motor";
const char *subscribeTopicPlant2Motor = "plant2-motor";

const char *publishTopicNotification = "plant-notification";

const char *publishTopicNPK = "npk";
const char *publishTopicTemp = "temp";
const char *publishTopicHum = "humidity";

#define MOISTURE_LOW_THRESHOLD 30 
#define MOISTURE_HIGH_THRESHOLD 70 

#define bjt_SEL1 D0
#define bjt_SEL2 D1
#define DHT_PIN D5
#define MOISTURE_PIN A0
#define PIR1_PIN D2
#define PIR2_PIN D3
#define MOTOR1_PWM_PIN D6
#define MOTOR2_PWM_PIN D8
#define RX_PIN D4
#define TX_PIN D7 
#define DHTTYPE DHT11 

DHT dht(DHT_PIN, DHTTYPE);
SoftwareSerial modbusSerial(RX_PIN, TX_PIN);
ModbusMaster node;

// Global variables
int moisture1 = 0;
int moisture2 = 0;
int pir1 = 0;
int pir2 = 0;
int npk_n = 0;
int npk_p = 0;
int npk_k = 0;
int temp = 0;
int humidity = 0;
int motor1_running = 0;
int motor2_running = 0;

int lastMoisture1 = 0;
int lastMoisture2 = 0;
int lastPir1 = 0;
int lastPir2 = 0;
int lastNpk_n = 0;
int lastNpk_p = 0;
int lastNpk_k = 0;
int lastTemp = 0;
int lastHumidity = 0;
int lastmotor1_running = 0;
int lastmotor2_running = 0;

unsigned long lastWifiCheck = 0;
const int wifiCheckInterval = 60000;
unsigned long lastSensorCheck = 0;
const int sensorCheckInterval = 5000;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void publishNotification(const char* message) {
  snprintf(msg, MSG_BUFFER_SIZE, "%s", message);
  Serial.print("Notification: ");
  Serial.println(msg);
  client.publish(publishTopicNotification, msg);
}

void checkAndControlMotor(int motorNum, int moisture) {
  if (motorNum == 1) {
    if (moisture < MOISTURE_LOW_THRESHOLD) {
      if (!motor1_running) {
        analogWrite(MOTOR1_PWM_PIN, 255);
        motor1_running = 1;
        publishNotification("Plant 1 moisture low, automatically starting watering");
      }
    } 
    else if (moisture > MOISTURE_HIGH_THRESHOLD) {
      if (motor1_running) {
        analogWrite(MOTOR1_PWM_PIN, 0);
        motor1_running = 0;
        publishNotification("Plant 1 adequately watered, stopping water pump");
      }
    }
  } 
  else if (motorNum == 2) {
    if (moisture < MOISTURE_LOW_THRESHOLD) {
      if (!motor2_running) {
        analogWrite(MOTOR2_PWM_PIN, 255);
        motor2_running = 1;
        publishNotification("Plant 2 moisture low, automatically starting watering");
      }
    } 
    else if (moisture > MOISTURE_HIGH_THRESHOLD) {
      if (motor2_running) {
        analogWrite(MOTOR2_PWM_PIN, 0);
        motor2_running = 0;
        publishNotification("Plant 2 adequately watered, stopping water pump");
      }
    }
  }
}

void handleMotorRequest(int motorNum, int requested) {
  String notificationMsg;
  
  if (motorNum == 1) {
    if (requested == 1) {
      if (moisture1 > MOISTURE_HIGH_THRESHOLD) {
        notificationMsg = "Cannot water Plant 1: Soil moisture already sufficient (";
        notificationMsg += moisture1;
        notificationMsg += "%)";
        publishNotification(notificationMsg.c_str());
        analogWrite(MOTOR1_PWM_PIN, 0);
        motor1_running = 0;
      } else {
        analogWrite(MOTOR1_PWM_PIN, 255);
        motor1_running = 1;
        notificationMsg = "Manual watering of Plant 1 started";
        publishNotification(notificationMsg.c_str());
      }
    } else {
      if (moisture1 < MOISTURE_LOW_THRESHOLD) {
        notificationMsg = "Warning: Plant 1 soil is dry (";
        notificationMsg += moisture1;
        notificationMsg += "%), motor is running";
        publishNotification(notificationMsg.c_str());
      }else{
        analogWrite(MOTOR1_PWM_PIN, 0);
        motor1_running = 0;
        notificationMsg = "Manual watering of Plant 1 stopped";
        publishNotification(notificationMsg.c_str());
      }
    }
  } 

  else if (motorNum == 2) {
    if (requested > 0) {
      if (moisture2 > MOISTURE_HIGH_THRESHOLD) {
        notificationMsg = "Cannot water Plant 2: Soil moisture already sufficient (";
        notificationMsg += moisture2;
        notificationMsg += "%)";
        publishNotification(notificationMsg.c_str());
        analogWrite(MOTOR2_PWM_PIN, 0);
        motor2_running = 0;
      } else {
        analogWrite(MOTOR2_PWM_PIN, requested);
        motor2_running = 1;
        notificationMsg = "Manual watering of Plant 2 started";
        publishNotification(notificationMsg.c_str());
      }
    } else {
      if (moisture2 < MOISTURE_LOW_THRESHOLD) {
        notificationMsg = "Warning: Plant 2 soil is dry (";
        notificationMsg += moisture2;
        notificationMsg += "%), motor is running";
        publishNotification(notificationMsg.c_str());
      }else{
        analogWrite(MOTOR2_PWM_PIN, 0);
        motor2_running = 0;
        notificationMsg = "Manual watering of Plant 2 stopped";
        publishNotification(notificationMsg.c_str());
      }
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
    Serial.print((char)payload[i]);
  }
  Serial.println();

  int requested = message.toInt();

  if (strcmp(topic, subscribeTopicPlant1Motor) == 0) {
    Serial.println("Command received for motor 1");
    handleMotorRequest(1, requested);
  }
  
  if (strcmp(topic, subscribeTopicPlant2Motor) == 0) {
    Serial.println("Command received for motor 2");
    handleMotorRequest(2, requested);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      client.subscribe(subscribeTopicPlant1Motor);
      client.subscribe(subscribeTopicPlant2Motor);
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publish_Moisture1() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", moisture1);
    Serial.print("Publish moisture1: ");
    Serial.println(msg);
    client.publish(publishTopicPlant1Moisture, msg);
    lastMoisture1 = moisture1;
}

void publish_Moisture2() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", moisture2);
    Serial.print("Publish moisture2: ");
    Serial.println(msg);
    client.publish(publishTopicPlant2Moisture, msg);
    lastMoisture2 = moisture2;
}

void publish_PIR1() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", pir1);
    Serial.print("Publish PIR1: ");
    Serial.println(msg);
    client.publish(publishTopicPlant1PIR, msg);
    lastPir1 = pir1;
}

void publish_PIR2() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", pir2);
    Serial.print("Publish PIR2: ");
    Serial.println(msg);
    client.publish(publishTopicPlant2PIR, msg);
    lastPir2 = pir2;
}

void publish_Motor1() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", motor1_running);
    Serial.print("Publish Motor1: ");
    Serial.println(msg);
    client.publish(publishTopicPlant1Motor, msg);
    lastmotor1_running = motor1_running;
}

void publish_Motor2() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", motor2_running);
    Serial.print("Publish Motor2: ");
    Serial.println(msg);
    client.publish(publishTopicPlant2Motor, msg);
    lastmotor2_running = motor2_running;
}

void publish_NPK() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d,%d,%d", npk_n, npk_p, npk_k);
    Serial.print("Publish NPK: ");
    Serial.println(msg);
    client.publish(publishTopicNPK, msg);
    lastNpk_k = npk_k;
    lastNpk_n = npk_n;
    lastNpk_p = npk_p;
}

void publish_temp() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", temp);
    Serial.print("Publish temp: ");
    Serial.println(msg);
    client.publish(publishTopicTemp, msg);
    lastTemp = temp;
}

void publish_hum() {
    snprintf(msg, MSG_BUFFER_SIZE, "%d", humidity);
    Serial.print("Publish humidity: ");
    Serial.println(msg);
    client.publish(publishTopicHum, msg);
    lastHumidity = humidity;
}

void readSensors() {
  // Read moisture sensor 1
  digitalWrite(bjt_SEL1, HIGH);
  digitalWrite(bjt_SEL2, LOW);
  delay(100);
  moisture1 = map(analogRead(MOISTURE_PIN), 0, 1023, 100, 0); // Convert to 0-100% (0=dry, 100=wet)

  // Read moisture sensor 2
  digitalWrite(bjt_SEL1, LOW);
  digitalWrite(bjt_SEL2, HIGH);
  delay(100);
  moisture2 = map(analogRead(MOISTURE_PIN), 0, 1023, 100, 0); // Convert to 0-100% (0=dry, 100=wet)
  
  // Read DHT (temperature and humidity)
  // DHT sensor is directly connected to its pin - no selector needed
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  // Check if readings are valid
  if (!isnan(h) && !isnan(t)) {
    humidity = (int)h;
    temp = (int)t;
  } else {
    Serial.println("Failed to read from DHT sensor!");
  }

  // Reset selector pins for moisture sensors
  digitalWrite(bjt_SEL1, LOW);
  digitalWrite(bjt_SEL2, LOW);
  
  // Read PIR sensors
  pir1 = digitalRead(PIR1_PIN);
  pir2 = digitalRead(PIR2_PIN);
  
  // Read NPK sensor via Modbus
  uint8_t result = node.readInputRegisters(0x0000, 6);
  if (result == node.ku8MBSuccess) {
    npk_n = node.getResponseBuffer(0);
    npk_p = node.getResponseBuffer(1);
    npk_k = node.getResponseBuffer(2);
  } else {
    Serial.print("Modbus read failed with error code: ");
    Serial.println(result);
  }
  
  // Check soil moisture and control motors if needed
  checkAndControlMotor(1, moisture1);
  checkAndControlMotor(2, moisture2);
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.disconnect();
    setup_wifi();
    if (WiFi.status() == WL_CONNECTED) {
      publishNotification("WiFi reconnected");
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(bjt_SEL1, OUTPUT);
  pinMode(bjt_SEL2, OUTPUT);
  pinMode(PIR1_PIN, INPUT);
  pinMode(PIR2_PIN, INPUT);
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  
  digitalWrite(bjt_SEL1, LOW);
  digitalWrite(bjt_SEL2, LOW);
  analogWrite(MOTOR1_PWM_PIN, 0);
  analogWrite(MOTOR2_PWM_PIN, 0);

  modbusSerial.begin(9600);
  node.begin(1, modbusSerial);

  setup_wifi();
  
  // Initialize DHT sensor
  dht.begin();
  delay(2000); // Give DHT sensor time to stabilize

  client.setServer(mqtt_server, mqttPort);
  client.setCallback(callback);
  
  delay(1000); 
  publishNotification("Plant monitoring system initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastWifiCheck > wifiCheckInterval) {
    lastWifiCheck = currentMillis;
    checkWiFiConnection();
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (currentMillis - lastSensorCheck > sensorCheckInterval) {
    lastSensorCheck = currentMillis;
    readSensors();
  }
  
  if (currentMillis - lastMsg > 10000) {
    lastMsg = currentMillis;

    if (moisture1 != lastMoisture1) publish_Moisture1();
    if (moisture2 != lastMoisture2) publish_Moisture2();
    if (pir1 != lastPir1) publish_PIR1();
    if (pir2 != lastPir2) publish_PIR2();
    if ((npk_k != lastNpk_k) || (npk_n != lastNpk_n) || (npk_p != lastNpk_p)) publish_NPK();
    if (temp != lastTemp) publish_temp();
    if (humidity != lastHumidity) publish_hum();
    if (motor1_running != lastmotor1_running) publish_Motor1();
    if (motor2_running != lastmotor2_running) publish_Motor2();
  }

  delay(10);
}