//interno
#include <WiFi.h>  // Library to handle WiFi connections on ESP32
#include <PubSubClient.h>  // Library for MQTT protocol on ESP32
#include "DHTesp.h"  // Library for working with the DHT sensor
#include <LiquidCrystal_I2C.h>  // Library to control the I2C LCD display
 
// Constant definitions
const int I2C_ADDR = 0x27;  // I2C address for the LCD display
const int LCD_COLUMNS = 20;  // Number of columns on the LCD display
const int LCD_LINES = 4;  // Number of rows on the LCD display
const int DHT_PIN = 15;  // Data pin for the DHT sensor
const int RELE_LUZ = 26;  // Pin to control the light relay
const int VENTILADOR_PIN = 23;  // Pin for the fan connected to a relay
 
DHTesp dhtSensor;  // Create an object to handle the DHT sensor
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);  // Create an object to handle the LCD display
 
const char* ssid = "Heracles";  // WiFi network name
const char* password = "f1ct1c10S_";  // WiFi password
const char* mqtt_server = "homeassistant.local";  // MQTT server address
const char* mqtt_user = "monitoreoMQTT";  // MQTT username
const char* mqtt_password = "OLIVER.22";  // MQTT password
 
WiFiClient espClient;  // Create WiFi client
PubSubClient client(espClient);  // Create MQTT client
 
bool luz_estado = false;  // Variable to store light state (on or off)
bool ventilador_estado = false;  // Variable to store fan state
String modo = "automatic";  // Variable to store operating mode ("manual" or "automatic")
 
void setup() {
  Serial.begin(115200);  // Begin serial communication at 115200 bps
  setup_wifi();  // Connect to WiFi network
  client.setServer(mqtt_server, 1883);  // Configure MQTT server
  client.setCallback(callback);  // Set callback function to handle MQTT messages
 
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);  // Set up DHT sensor
  lcd.init();  // Initialize LCD display
  lcd.backlight();  // Turn on LCD backlight
 
  pinMode(RELE_LUZ, OUTPUT);  // Set light relay pin as output
  pinMode(VENTILADOR_PIN, OUTPUT);  // Set fan pin as output
  digitalWrite(RELE_LUZ, LOW);  // Turn off light on startup
  digitalWrite(VENTILADOR_PIN, LOW);  // Turn off fan on startup
}
 
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");  // Display a message in the serial monitor
  Serial.println(ssid);  // Print WiFi network name
  WiFi.begin(ssid, password);  // Start WiFi connection
  while (WiFi.status() != WL_CONNECTED) {  // Wait until connection is complete
    delay(500);
    Serial.print(".");  // Print a dot every 500 ms until connected
  }
  Serial.println("\nWiFi connected");  // Message when connected to WiFi
}
 
void reconnect() {
  while (!client.connected()) {  // Try to reconnect if not connected to MQTT server
    Serial.print("Connecting to MQTT server...");
    if (client.connect("ESP32Client_internal", mqtt_user, mqtt_password)) {  // Try to connect to MQTT server
      Serial.println("Connected!");  // Success message
      client.subscribe("home/salon/luz/set");  // Subscribe to light control topic
      client.subscribe("home/salon/ventilador/set");  // Subscribe to fan control topic
      client.subscribe("home/mode/set");  // Subscribe to mode change topic
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());  // Print error code if connection fails
      delay(5000);  // Wait 5 seconds before retrying
    }
  }
}
 
void callback(char* topic, byte* payload, unsigned int length) {
  String mensaje;
  for (int i = 0; i < length; i++) {  // Build message from received data
    mensaje += (char)payload[i];
  }
 
  if (String(topic) == "home/salon/luz/set") {  // If message is for the light
    if (mensaje == "ON") {
      luz_estado = true;
      digitalWrite(RELE_LUZ, HIGH);  // Turn on the light
    } else if (mensaje == "OFF") {
      luz_estado = false;
      digitalWrite(RELE_LUZ, LOW);  // Turn off the light
    }
  } else if (String(topic) == "home/salon/ventilador/set") {  // If message is for the fan
    if (mensaje == "ON") {
      ventilador_estado = true;
      digitalWrite(VENTILADOR_PIN, HIGH);  // Turn on the fan
    } else if (mensaje == "OFF") {
      ventilador_estado = false;
      digitalWrite(VENTILADOR_PIN, LOW);  // Turn off the fan
    }
  } else if (String(topic) == "home/mode/set") {  // If message is for the mode
    modo = mensaje;  // Change operating mode
    Serial.println("Mode changed to: " + modo);  // Print mode to serial monitor
  }
}
 
void loop() {
  if (!client.connected()) {  // If not connected, try to reconnect
    reconnect();
  }
  client.loop();  // Keep MQTT connection active
  TempAndHumidity data = dhtSensor.getTempAndHumidity();  // Read temperature and humidity from sensor
  float temperature = data.temperature;
  // Display temperature and humidity on LCD
  lcd.setCursor(0, 0);  // Set cursor to first line
  lcd.print("  Temp: " + String(temperature, 1) + "\xDF" + "C  ");  // Display temperature
  lcd.setCursor(0, 1);  // Set cursor to second line
  lcd.print(" Humidity: " + String(data.humidity, 1) + "% ");  // Display humidity
 
  // Publish temperature and humidity data to MQTT topics
  client.publish("home/esp32two/temperatureint", String(temperature).c_str());  // Publish temperature
  client.publish("home/esp32two/humidityint", String(data.humidity).c_str());  // Publish humidity
  if (modo == "manual") {  // If in manual mode
    lcd.setCursor(0, 2);  // Set cursor to third line
    lcd.print("Mode: Manual       ");  // Display "Mode: Manual" on screen
  } else if (modo == "automatic") {  // If in automatic mode
    lcd.setCursor(0, 2);  // Set cursor to third line
    if (temperature < 20.0) {
      lcd.print("Status: Low       ");  // Display "Low" if temperature is low
      digitalWrite(VENTILADOR_PIN, LOW);  // Turn off the fan
      digitalWrite(RELE_LUZ, HIGH); // Turn On the light
    } else if (temperature >= 20.0 && temperature <= 25.0) {
      lcd.print("Status: Stable    ");  // Display "Stable" if temperature is stable
      digitalWrite(VENTILADOR_PIN, LOW);  // Turn off the fan
      digitalWrite(RELE_LUZ, LOW); // Turn Off the light
    } else {
      lcd.print("Status: High      ");  // Display "High" if temperature is high
      digitalWrite(VENTILADOR_PIN, HIGH);  // Turn on the fan
      digitalWrite(RELE_LUZ, LOW); // Turn Off the light
    }
  }
 
  delay(2000);  // Wait 5 seconds before the next sensor reading
}
 