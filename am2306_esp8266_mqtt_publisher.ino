//Example.
//Not for real use.
//No security mechanisms are implemented here, so it is recommended to use this code only for private needs in protected private networks.

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "am2306.h"

//Wireless network settings:
#define WIFI_SSID "mySSID"
#define WIFI_PASSWORD "mypassword"

//MQTT publishing client settings:
#define MQTT_SERVER_ADDRESS "192.168.123.1"
#define MQTT_SERVER_PORT 1883
#define MQTT_TOPIC "my/topic"
#define MQTT_CLIENT_ID "myid"
#define MQTT_PUBLISHING_PERIOD 60000 //Every 1 minute, the period for sending the sensor readings to the MQTT broker(server).

//ESP8266 pin(GPIO) where the sensor is connected:
#define ESP_PIN 3 //In this case 3 - we will use RX pin to connect to the sensor, so the code will call a function pinMode(io_pin, FUNCTION_3) that will put the pin in normal GPIO mode.

WiFiClient ip_client;
PubSubClient mqtt_client(ip_client);

unsigned long milliseconds_difference = 0;
unsigned long previous_milliseconds = 0;
unsigned long current_milliseconds = 0;

void setup() {

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  mqtt_client.setServer(MQTT_SERVER_ADDRESS, MQTT_SERVER_PORT);

  //Set RX pin (3) as a regular GPIO if it is selected for sensor connection.
  #ifdef ESP_PIN
  #if (ESP_PIN == 3)
  pinMode(ESP_PIN, FUNCTION_3);
  #endif
  #endif

}

void loop() {

  if (!mqtt_client.connected()) { while (!mqtt_client.connected()) { if (mqtt_client.connect(MQTT_CLIENT_ID)) { previous_milliseconds = current_milliseconds; } else { delay(5000); } } }
  mqtt_client.loop();

  current_milliseconds = millis();

  //Sometimes timer overflows...
  if (current_milliseconds > previous_milliseconds) { milliseconds_difference = current_milliseconds - previous_milliseconds; } else { milliseconds_difference = ULONG_MAX - previous_milliseconds + current_milliseconds; }

  if (milliseconds_difference > MQTT_PUBLISHING_PERIOD) {

    previous_milliseconds = current_milliseconds;

    //There's a nuance... The sensor always gives its previous measurement, not the current one. Therefore, to get the actual value, we can poll it twice with a minimum delay of 2 seconds.
    //But i think that we don't need this in case of continuous polling.

    int humidity, temperature;
    if (!get_relative_humidity_and_celsius_temperature_from_am2306_sensor(ESP_PIN, &humidity, &temperature)) {

      char mqtt_message[9]; //3 position for humidity, 4 for temperature, 1 for comma, 1 for end of string sign.
      snprintf(mqtt_message, 9, "%d,%d", humidity, temperature);
      mqtt_client.publish(MQTT_TOPIC, mqtt_message);

    }

  }

}
