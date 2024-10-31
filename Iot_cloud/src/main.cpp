#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "DHTesp.h"
// #include <ESP32Servo.h>
//#include <Stepper.h>
const char *WIFI_SSID = "Fakeme C53";
const char *WIFI_PWD = "helloooo";

// Thông tin kết nối HiveMQ Public Broker (không TLS)
const char *MQTT_SERVER = "15dde8f446184978829a951a7530f634.s1.eu.hivemq.cloud"; 
const uint16_t MQTT_PORT = 8883; 
const char *mqtt_user = "iot2021"; 
const char *mqtt_pass = "12345678"; 
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

const int DHT_PIN = 13; //Do am `nhiet do
const int LDR_PIN = 32; // Do anh sang
const int LED_PIN = 14; // Chân LED
const int FAN_PIN = 26; // Chân Fan
const int servoPin = 27;  
const int stepsPerRevolution = 500; 
int inputPin = 2;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;                    // variable for reading the pin status
const int curtain_pin = 26;
const int sensor_pin = 25;
const int sensor_pin2 = 23;


#define VCC 3.3
#define R1 10000
#define RA 18000
#define EA 10
#define LAMDA 0.9

// Debounce time to prevent false triggers (adjust as needed)
const unsigned long debounceDelay = 500; 
const long interval = 5000;
unsigned long current_time = millis();
unsigned long last_trigger = 0;
boolean timer_on = false;

// Forward declaration
void IRAM_ATTR movement_detection(); 
//Stepper myStepper(stepsPerRevolution, 15, 2, 0, 4);// initialize the stepper library on pins 8 through 11:
//Servo servo;
DHTesp dhtSensor;

// LDR Characteristics
const float GAMMA = 0.7;
const float RL10 = 50;
int pos = 0;


//String status_led;
void CallbackMqtt(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0'; 

  Serial.print("Message: ");
  Serial.println(message);

  if (strcmp(topic, "HCMUS/p501/iot/sensors/led") == 0) {
    if (strcmp(message, "ON") == 0) {
      digitalWrite(LED_PIN, HIGH); 
      //status_led = "ON";
    } else if (strcmp(message, "OFF") == 0) {
      digitalWrite(LED_PIN, LOW);  
      //status_led = "OFF";
    }
  }
  if (strcmp(topic, "HCMUS/p501/iot/sensors/fan") == 0) {
    if (strcmp(message, "ON") == 0) {
      digitalWrite(FAN_PIN, HIGH); 
      //status_led = "ON";
    } else if (strcmp(message, "OFF") == 0) {
      digitalWrite(FAN_PIN, LOW);  
      //status_led = "OFF";
    }
  }
  if (strcmp(topic, "HCMUS/p501/iot/sensors/door") == 0) {
    if (strcmp(message, "OPENED") == 0) {
      //servo.write(180); // Mở cửa (hoặc góc tối đa bạn muốn)
      digitalWrite(servoPin, HIGH);

    } else if (strcmp(message, "CLOSED") == 0) {
      //servo.write(0);   // Đóng cửa (hoặc góc tối thiểu bạn muốn)
      digitalWrite(servoPin, LOW);
    }
  }
    if (strcmp(topic, "HCMUS/p501/iot/sensors/curtain") == 0) {
    if (strcmp(message, "OPENED") == 0) {
      //servo.write(180); // Mở cửa (hoặc góc tối đa bạn muốn)
      digitalWrite(curtain_pin, HIGH);

    } else if (strcmp(message, "CLOSED") == 0) {
      //servo.write(0);   // Đóng cửa (hoặc góc tối thiểu bạn muốn)
      digitalWrite(curtain_pin, LOW);
    }
  }
  // if (strcmp(topic, "HCMUS/p501/iot/sensors/curtain") == 0) {
  //   if (strcmp(message, "OPEND") == 0) {
  //     myStepper.step(stepsPerRevolution);
  //   } else if (strcmp(message, "CLOSED") == 0) {
  //      myStepper.step(-stepsPerRevolution);
  //   }
  // }
}

void SetupMqtt() {
  // Tạm thời vô hiệu hóa xác minh chứng chỉ
  wifiClient.setInsecure(); 

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(CallbackMqtt);
}

void ConnectToMqtt() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
    char clientId[100] = "\0";
    byte mac[6];
    WiFi.macAddress(mac);
    sprintf(clientId, "ESP32Client-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("MQTT Client ID: ");
    Serial.println(clientId);

    if (mqttClient.connect(clientId, mqtt_user, mqtt_pass)) {
      Serial.println("Connected to MQTT broker.");
      // mqttClient.subscribe("HCMUS/p501/iot/sensors/commands");
      mqttClient.subscribe("HCMUS/p501/iot/sensors/led"); 
      mqttClient.subscribe("HCMUS/p501/iot/sensors/door"); 
      mqttClient.subscribe("HCMUS/p501/iot/sensors/curtain");
      mqttClient.subscribe("HCMUS/p501/iot/sensors/fan");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void ConnectToWiFi() {
  Serial.print("Connecting to WiFi ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PWD, 6);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nConnected to ");
  Serial.println(WIFI_SSID);
}


void setup() {
  Serial.begin(9600);
  Serial.println("Setup begin");
  pinMode(LED_PIN, OUTPUT); 
  delay(100);
  pinMode(FAN_PIN, OUTPUT); 
  delay(100);
  pinMode(servoPin, OUTPUT); 
  delay(100);
  pinMode(curtain_pin, OUTPUT); 
  delay(100);
  ConnectToWiFi();
  delay(100);
  SetupMqtt();
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  
    // set the speed at 60 rpm:
 // myStepper.setSpeed(60);
  delay(100);
  pinMode(sensor_pin, INPUT_PULLUP);
  pinMode(sensor_pin2, INPUT_PULLUP);
  delay(100);
  attachInterrupt(digitalPinToInterrupt(sensor_pin), movement_detection, RISING);
  delay(100);
  Serial.println("Setup end");
}
volatile int soluong = 0; // Make soluong volatile for interrupt access

void IRAM_ATTR movement_detection() {
  static unsigned long lastDebounceTime = 0;  
  unsigned long currentTime = millis();

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    soluong++;
    Serial.println("Motion was detected");
    Serial.print("So luong: ");
    Serial.println(soluong);
    lastDebounceTime = currentTime;
  }
}
void loop() {
  if (!mqttClient.connected()) {
    ConnectToMqtt();
  }
  mqttClient.loop(); 


  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  int analogValue = analogRead(LDR_PIN);
  float voltage = (analogValue / 4095.0) * 3.3;

  float r2 = (-voltage*R1)/(voltage - VCC);
  float eb = pow(RA / r2, 1/LAMDA) * 10;
  //float IE = voltage*100.0;
  //float resistance = (4095.0 * 10000.0)/voltage - 10000.0;
  //float lux = IE*(4.0/3.0);
  val = digitalRead(inputPin);  // read input value
  static uint64_t last_time;
  uint64_t now = millis();
  if (now - last_time > 1 * 3000) {
    // mqttClient.publish("HCMUS/p501/iot/sensors/hello", "hello world");
    mqttClient.publish("HCMUS/p501/iot/sensors/nhietdo", String(data.temperature).c_str());
    mqttClient.publish("HCMUS/p501/iot/sensors/doam", String(data.humidity).c_str());
    mqttClient.publish("HCMUS/p501/iot/sensors/anhsang", String(eb).c_str());
    mqttClient.publish("HCMUS/p501/iot/sensors/soluong", String(soluong).c_str());
    Serial.println("Published 'Thông tin:'");
    Serial.println("Temp: " + String(data.temperature, 2) + "°C");
    Serial.println("Humidity: " + String(data.humidity, 1) + "%");
    Serial.println("Lux: " + String(eb));
    Serial.println("So luong: " + String(soluong));
    // Serial.println("Status led: " + status_led);
    last_time = now;
  }
}