#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <LiquidCrystal_I2C.h>

// Configuraciones de la pantalla LCD
#define I2C_ADDR 0x27
#define LCD_COLUMNS 20
#define LCD_LINES 4

// Pines y configuraciones
const int DHT_PIN = 15;
const int MQ135_PIN = 34;
DHTesp dhtSensor;
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

// Credenciales WiFi y MQTT
const char* ssid = "LUBIGARF";
const char* password = "CABO.16@DAKOTA";
const char* mqtt_server = "homeassistant.local";
const char* mqtt_user = "monitoreoMQTT";
const char* mqtt_password = "OLIVER.22";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);

  // Configuración WiFi y MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);

  // Inicializar sensor DHT22 y LCD
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  lcd.init();
  lcd.backlight();
}

void setup_wifi() {
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado!");
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando en 5 segundos");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer datos del DHT22
  TempAndHumidity data = dhtSensor.getTempAndHumidity();

  // Leer datos del sensor MQ-135
  int mq135_value = analogRead(MQ135_PIN);
  float calidad_aire = map(mq135_value, 0, 4095, 0, 100); // Escalar de 0 a 100%

  // Mostrar temperatura, humedad y calidad del aire en LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: " + String(data.temperature, 1) + "\xDF" + "C");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(data.humidity, 1) + "%");
  lcd.setCursor(0, 2);
  lcd.print("Air Quality: " + String(calidad_aire, 1) + "%");

  // Mostrar estado de temperatura
  lcd.setCursor(0, 3);
  if (data.temperature < 18) {
    lcd.print("Low temperature   ");
  } else if (data.temperature <= 28) {
    lcd.print("Stable temperature");
  } else {
    lcd.print("High temperature  ");
  }

  // Publicar datos en MQTT
  client.publish("home/esp32one/temperature", String(data.temperature).c_str());
  client.publish("home/esp32one/humidity", String(data.humidity).c_str());
  client.publish("home/esp32one/airquality", String(calidad_aire).c_str());

  delay(2000); // Esperar antes de la siguiente lectura
}



