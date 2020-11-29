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

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void setupWifi(void);
void reconnectMqtt(void);
void publishMqtt(String topic, String payload);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER_IP;

const int pressureUpdateInterval = 60000 * 5; // every 5 minutes
const int scd30MeasurementInterval = 5000; // every 5 seconds

WiFiClient espClient;
PubSubClient mqttClient(espClient);
SCD30 scd30;
Bsec iaqSensor;

void setup() {
    Serial.begin(9600);
    Serial.println("Los geht's!");
    Wire.begin();

    //Check SCD30
    if (scd30.begin(Wire)) {
        Serial.println("SCD30 detected.");
    } else {
        Serial.println("SCD30 not detected.");
    }
    if (!scd30.setMeasurementInterval(scd30MeasurementInterval / 1000)) {
        Serial.println("Failed to set SCD30 measurement interval");
    }

    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    Serial.println("\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));
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

unsigned long lastPressureUpdate = 0;
unsigned long lastScd30Check = 0;

void loop() {
    if (!mqttClient.connected()) {
        reconnectMqtt();
    }
    mqttClient.loop();
    
    unsigned long now = millis();
    if (iaqSensor.run()) { // If new data is available
        const float pressureHpa = iaqSensor.pressure / 100;
        String output;
        output = "BSEC: New data for timestamp " + String(now);
        output += "\n raw temperature [°C]: " + String(iaqSensor.rawTemperature);
        output += "\n pressure [hPa]: " + String(pressureHpa);
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
        publishMqtt("pressure", String(pressureHpa));
        publishMqtt("rawHumidity", String(iaqSensor.rawHumidity));
        publishMqtt("gasResistance", String(iaqSensor.gasResistance));
        publishMqtt("iaq", String(iaqSensor.iaq));
        publishMqtt("iaqAccuracy", String(iaqSensor.iaqAccuracy));
        publishMqtt("temperature", String(iaqSensor.temperature));
        publishMqtt("humidity", String(iaqSensor.humidity));
        publishMqtt("staticIaq", String(iaqSensor.staticIaq));
        publishMqtt("co2Equivalent", String(iaqSensor.co2Equivalent));
        publishMqtt("breathVocEquivalent", String(iaqSensor.breathVocEquivalent));
        
        if (now - lastPressureUpdate > pressureUpdateInterval) {
            // Update ambient pressure
            Serial.println("Updating SCD30 with new ambient pressure value [mbar]: " + String(pressureHpa));

            if (!scd30.setAmbientPressure((uint16_t) pressureHpa)) {
                Serial.println("Failed to update SCD30 with current ambient pressure");
            }
            lastPressureUpdate = now;
        } ;
    }

    if (now - lastScd30Check > scd30MeasurementInterval) {
        if (scd30.dataAvailable()) {
            String scd30Co2 = String(scd30.getCO2());
            Serial.println("SCD30: New data");
            Serial.println(" co2(ppm): " + scd30Co2);
            Serial.println(" temp(C): " + String(scd30.getTemperature()));
            Serial.println(" humidity(%): " + String(scd30.getHumidity()));
            Serial.println();
            publishMqtt("co2", scd30Co2);
        } else {
            Serial.println("SCD30: No data.");
        }
        lastScd30Check = now;
    }
}

void checkIaqSensorStatus(void) {
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            Serial.println("BSEC error code : " + String(iaqSensor.status));
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
            Serial.println("BSEC warning code : " + String(iaqSensor.status));
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
            //for (;;)
            //    errLeds(); /* Halt in case of failure */
            publishMqtt("iaqError", String(iaqSensor.status));
            delay(1000);
            errLeds();
            delay(1000);
            errLeds();
            delay(1000);
        } else {
            Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
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
