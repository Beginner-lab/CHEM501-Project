#include <Arduino_BHY2Host.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <Arduino.h>

// Sensor setup
Sensor temp(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
SensorBSEC bsec(SENSOR_ID_BSEC);
Sensor gas(SENSOR_ID_GAS);

// Wi-Fi credentials
char ssid[] = "VodafoneMobileWiFi-72AF05";  // your network SSID (name)
char pass[] = "7353626621";  // your network password

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "pf-eveoxy0ua6xhtbdyohag.cedalo.cloud";
int port = 1883;
const char topic1[] = "alextemp";
const char topic2[] = "alexhumidity";
const char topic3[] = "alextime";  // Topic for time counter data
const char topic4[] = "alexgas";

// Set interval for sending messages (milliseconds)
const long interval = 5000; // 1 second
unsigned long previousMillis = 0;

int count = 0;

void setup() {
  Serial.begin(115200);
  BHY2Host.begin();
  temp.begin();
  humidity.begin();
  gas.begin();

  // Connect to Wi-Fi
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");

  // Connect to MQTT broker
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
}

void loop() {
  // Call poll() regularly to keep MQTT connection alive
  mqttClient.poll();
  BHY2Host.update();
  unsigned long currentMillis = millis(); // Get the current time (in milliseconds)

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Save the last time a message was sent

    // Measure sensor values
    float dataTemp = temp.value();
    float dataH = humidity.value();
    float dataG = gas.value();

    // Simulate time counter (seconds since start)
    count = currentMillis / 1000;  // Convert milliseconds to seconds

    // Print and send temperature data
    Serial.print("Sending message to topic1: ");
    Serial.println(topic1);
    Serial.println(dataTemp);
    mqttClient.beginMessage(topic1);
    mqttClient.print(dataTemp);
    mqttClient.endMessage();

    // Print and send humidity data
    Serial.print("Sending message to topic2: ");
    Serial.println(topic2);
    Serial.println(dataH);
    mqttClient.beginMessage(topic2);
    mqttClient.print(dataH);
    mqttClient.endMessage();

    // Print and send time counter data (seconds)
    Serial.print("Sending message to topic3: ");
    Serial.println(topic3);
    Serial.println(count);  // Send the time counter (in seconds)
    mqttClient.beginMessage(topic3);
    mqttClient.print(count);  // Send time as a simple count of seconds
    mqttClient.endMessage();

    Serial.print("Sending message to topic4: ");
    Serial.println(topic4);
    Serial.println(dataG);  
    mqttClient.beginMessage(topic4);
    mqttClient.print(dataG);  
    mqttClient.endMessage();
  }
}
