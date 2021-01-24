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
void reconnectTS();
void readSensors();
void publish();
void publishMqtt(String topic, String payload);
void publishSerial();
void publishMqtt();
void publishTS();


const bool use_mqtt = USE_MQTT;
const bool use_ts = USE_TS;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER_IP;
const char* ts_username = TS_USERNAME;
const char* ts_mqtt_api = TS_MQTT_API;
const char* ts_channel_write_api = TS_CHANNEL_WRITE_API;
const char* ts_server = TS_SERVER;

const int pressureUpdateInterval = 60000 * 5; // every 5 minutes
const int scdMeasurementInterval = SCD_MEASUREMENT_INTERVAL;

const long ts_channel_id = TS_CHANNEL_ID;
const unsigned long ts_publish_interval = TS_PUBLISH_INTERVAL*1000;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
PubSubClient tsClient(espClient);
SCD30 scd30;
Bsec iaqSensor;

void setup() {
    delay(100); //SCD30 requires >50ms to turn on
    Serial.begin(9600);
    Serial.println("Los geht's!");
    setupWifi();

    Wire.begin();

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

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP); //Low Power Mode, every 3 seconds
    checkIaqSensorStatus();

    // Start SCD30 after BSEC to allow it to
    // configure clock streching correctly on ESP8266
    if (scd30.begin(Wire, false)) {
        Serial.println("SCD30 detected");
    } else {
        Serial.println("SCD30 not detected on first attempt");
        //Give it 2s to boot and then try again
        delay(2000);
        if (scd30.begin(Wire, false)) {
          Serial.println("SCD30 detected on second attempt");
        }
    }

    Serial.print("Setting SCD30 measuerement interval to ");
    Serial.print(scdMeasurementInterval);
    Serial.println(" seconds");
    if (!scd30.setMeasurementInterval(scdMeasurementInterval)) {
        Serial.println("Failed to set SCD30 measurement interval");
    }

    Serial.println("Current configuration on SCD30:");
    Serial.print("  Auto calibration set to ");
    if (scd30.getAutoSelfCalibration()) {
        Serial.println("TRUE");
    } else {
        Serial.println("FALSE");
    }

    unsigned int altitude = scd30.getAltitudeCompensation();
    Serial.print("  Altitude compentsation set to ");
    Serial.print(altitude);
    Serial.println("m");

    float offset = scd30.getTemperatureOffset();
    Serial.print("  Temperature offset set to ");
    Serial.print(offset, 2);
    Serial.println("C");
}

unsigned long lastPressureUpdate = 0;
unsigned long lastScd30Check = 0;
unsigned long now = 0;
unsigned long lastTsPublish = 0;
bool scd_new = false;
bool iaq_new = false;


void loop() {
    if (use_mqtt) {
        if (!mqttClient.connected()) {
            reconnectMqtt();
        }
        mqttClient.loop();
    }
    
    if (use_ts) {
        if (!tsClient.connected())
        {
            reconnectTS();
        }
        tsClient.loop();        
    }
    
    now = millis();
    
    readSensors();

    publish();
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

void readSensors(){
    if (iaqSensor.run() ) {
        iaq_new = true;

        //Update SCD30 with new ambient pressure value (mbar)
        if (now - lastPressureUpdate > pressureUpdateInterval) {
            // Update ambient pressure
            Serial.println("Updating SCD30 with new ambient pressure value [mbar].");
            if (!scd30.setAmbientPressure((uint16_t) (iaqSensor.pressure / 100))) {
                Serial.println("Failed to update SCD30 with current ambient pressure");
            }
            lastPressureUpdate = now;
        }
    }
    if (scd30.dataAvailable()) {
        scd_new = true;
    }
}

void publish() {
    if (iaq_new || scd_new) {    
        if (PUBLISH_METHOD == 0 && iaq_new && scd_new)
        {
            publishSerial();
            if (use_mqtt) publishMqtt();
            if (use_ts) publishTS();                 
        } else if (PUBLISH_METHOD == 1)
        {
            publishSerial();
            if (use_mqtt) publishMqtt();
            if (use_ts) publishTS();
        } else if (PUBLISH_METHOD == 2)
        {
            publishSerial();
            if (use_mqtt) publishMqtt();
            if (use_ts) publishTS();
        } 
        iaq_new = false;
        scd_new = false;       
    }
    
}

void publishMqtt(String topic, String payload) {
    mqttClient.publish(String(MQTT_TOPIC_PREFIX + topic).c_str(), payload.c_str());
}

void publishMqtt() {
    if (PUBLISH_METHOD !=2 || iaq_new)
    {   
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
    }
    if (PUBLISH_METHOD !=2 || scd_new) {
        publishMqtt("co2", String(scd30.getCO2()));
    }
}

void publishSerial() {
    String output = "\nTimestamp: " + String(now) + "\n";
    if (PUBLISH_METHOD != 2) {
        if (iaq_new) output += "New IAQ-Data!";
        else output += "Old IAQ-Data.";
        if (scd_new) output += " New SCD-Data!";
        else output += " Old SCD-Data.";
    } else if (iaq_new) output += "New IAQ-Data!";
    else output += "New SCD-Data!";
    if (PUBLISH_METHOD !=2 || iaq_new) {
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
    }
    if (PUBLISH_METHOD !=2 || scd_new) {
        output += "\n CO2(ppm) " + String(scd30.getCO2()) ;
        output += "\n temperature [°C] " + String(scd30.getTemperature());
        output += "\n relative humidity [%] " + String(scd30.getHumidity());
    }
    Serial.println(output);  
}

void publishTS() {
    if (now - lastTsPublish > ts_publish_interval) {
        String output;
        output = "field1=" + String(iaqSensor.temperature);
        output += "&field2=" + String(iaqSensor.pressure/100);
        output += "&field3=" + String(iaqSensor.humidity);
        output += "&field4=" + String(iaqSensor.staticIaq);
        output += "&field5=" + String(iaqSensor.staticIaqAccuracy);
        output += "&field6=" + String(iaqSensor.co2Equivalent);
        output += "&field7=" + String(iaqSensor.co2Accuracy);
        output += "&field8=" + String(scd30.getCO2());

        String topic = "channels/" + String(ts_channel_id) + "/publish/" + String(ts_channel_write_api);
        tsClient.publish(topic.c_str(), output.c_str());

        lastTsPublish = now;
    }

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
    tsClient.setServer(ts_server, 1883);
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

void reconnectTS() {
    while (!tsClient.connected()) {
    Serial.print("Attempting TS-MQTT connection...");
    // Attempt to connect
    if (tsClient.connect("airlogger",ts_username,ts_mqtt_api)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(tsClient.state());
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
