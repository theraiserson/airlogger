// A lot of the BSEC code is taken over from https://github.com/BoschSensortec/BSEC-Arduino-library/blob/996a03594faf3b889efd69a0ca92e55a9ce242b2/examples/basic/basic.ino

#include <Arduino.h>
#include "bsec.h"
#include <PubSubClient.h>

// Copy over "_constants.h.example" to "_constants.h" and update it with the according
//	values for your network
#include "_constants.h"

#if defined(ESP32)
    #include <SparkFun_SCD30_Arduino_Library.h>
    #include <WiFi.h>
#elif defined(ESP8266)
    #include <paulvha_SCD30.h>
    #include <ESP8266WiFi.h>
#endif

// For some reason "LED_BUILTIN" is not defined for ESP32
#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif

#define SCD30WIRE Wire

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void setupWifi(void);
void reconnectMqtt(void);
void publishMqtt(String topic, String payload);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER_IP;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
SCD30 scd30;
Bsec iaqSensor;
String output;

void setup() {
    delay(1000);
    SCD30WIRE.begin();
    Serial.begin(9600);
    Serial.println("Los geht's!");

    //Check SCD30
    if (scd30.begin(Wire)) {
        Serial.println("SCD30 detected.");
    } else {
        Serial.println("SCD30 not detected.");
    }

    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();

    setupWifi();
}

void loop() {
    if (!mqttClient.connected()) {
        reconnectMqtt();
    }
    mqttClient.loop();
    
    unsigned long time_trigger = millis();
    if (iaqSensor.run()) { // If new data is available
        output = "New data for timestamp " + String(time_trigger) + ":";
        output += "\n raw temperature [°C]: " + String(iaqSensor.rawTemperature);
        output += "\n pressure [hPa]: " + String(iaqSensor.pressure / 100);
        output += "\n raw relative humidity [%]: " + String(iaqSensor.rawHumidity);
        output += "\n gas [Ohm]: " + String(iaqSensor.gasResistance);
        output += "\n IAQ: " + String(iaqSensor.iaq);
        output += "\n IAQ accuracy: " + String(iaqSensor.iaqAccuracy);
        output += "\n temperature [°C]: " + String(iaqSensor.temperature);
        output += "\n relative humidity [%]: " + String(iaqSensor.humidity);
        output += "\n Static IAQ: " + String(iaqSensor.staticIaq);
        output += "\n CO2 equivalent: " + String(iaqSensor.co2Equivalent);
        output += "\n breath VOC equivalent: " + String(iaqSensor.breathVocEquivalent);
        Serial.println(output);

        publishMqtt("rawTemperature", String(iaqSensor.rawTemperature));
        publishMqtt("pressure", String(iaqSensor.pressure / 100));
        publishMqtt("rawHumidity", String(iaqSensor.rawHumidity));
        publishMqtt("gasResistance", String(iaqSensor.gasResistance));
        publishMqtt("iaq", String(iaqSensor.iaq));
        publishMqtt("iaqAccuracy", String(iaqSensor.iaqAccuracy));
        publishMqtt("temperature", String(iaqSensor.temperature));
        publishMqtt("humidity", String(iaqSensor.humidity));
        publishMqtt("staticIaq", String(iaqSensor.staticIaq));
        publishMqtt("co2Equivalent", String(iaqSensor.co2Equivalent));
        publishMqtt("breathVocEquivalent", String(iaqSensor.breathVocEquivalent));
        
        if (scd30.dataAvailable()) {
            String scd30Co2 = String(scd30.getCO2());
            Serial.println(" co2(ppm): " + scd30Co2);
            Serial.println(" temp(C): " + String(scd30.getTemperature()));
            Serial.println(" humidity(%): " + String(scd30.getHumidity()));
            Serial.println();
            publishMqtt("co2", scd30Co2);
        } else {
            Serial.println("SCD30: No data.");
        }
    } else {
        checkIaqSensorStatus();
    }
}

void checkIaqSensorStatus(void) {
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            output = "BSEC error code : " + String(iaqSensor.status);
            Serial.println(output);
            // I encoutered status code -2 error once and this locked up the board as expected
            // Howver I'm interested in what happens if we just let it continue,
            //  therefore just blinking for a short bit now
            //for (;;)
            //    errLeds(); /* Halt in case of failure */
            publishMqtt("iaqError", String(iaqSensor.status).c_str());
            delay(1000);
            errLeds();
            delay(1000);
            errLeds();
            delay(1000);
        } else {
            output = "BSEC warning code : " + String(iaqSensor.status);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            output = "BME680 error code : " + String(iaqSensor.bme680Status);
            Serial.println(output);
            //for (;;)
            //    errLeds(); /* Halt in case of failure */
            publishMqtt("iaqError", String(iaqSensor.status));
            delay(1000);
            errLeds();
            delay(1000);
            errLeds();
            delay(1000);
        } else {
            output = "BME680 warning code : " + String(iaqSensor.bme680Status);
            Serial.println(output);
        }
    }
}

void publishMqtt(String topic, String payload) {
    mqttClient.publish(String(MQTT_TOPIC_PREFIX + topic).c_str(), payload.c_str());
}

void setupWifi() {
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    WiFi.setAutoReconnect(true);

    Serial.println("WiFi connected");
    mqttClient.setServer(mqtt_server, 1883); //connecting to mqtt server
}

void reconnectMqtt() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("airlogger")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void errLeds(void) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}
