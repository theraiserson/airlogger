#include <Arduino.h>
#include <paulvha_SCD30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define SCD30WIRE Wire

#define SEALEVELPRESSURE_HPA (1013.25)

SCD30 scd30;
Adafruit_BME680 bme680;

void setup() {
    SCD30WIRE.begin();

    Serial.begin(9600);
    delay(5000);
    Serial.println("Los geht's!");

    //Check SCD30
    if (scd30.begin(Wire)) {
        Serial.println("SCD30 detected.");
    } 
    else {
        Serial.println("SCD30 not detected.");
    }

    //Check BME680
    if (bme680.begin()) {
        Serial.println("BME680 detected.");
    }
    else {
        Serial.println("BME680 not detected.");
    }
    
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
    if (scd30.dataAvailable())
    {
        Serial.print("co2(ppm):");
        Serial.print(scd30.getCO2());
        Serial.print(" temp(C):");
        Serial.print(scd30.getTemperature(),1);
        Serial.print(" humidity(%)");
        Serial.print(scd30.getHumidity(),1);

        Serial.println();
    }
    else
    {
        Serial.println("SCD30: No data.");
    }
    if (bme680.performReading())
    {
        Serial.print("Temperature (C): ");
        Serial.println(bme680.temperature);

        Serial.print("Pressure (hPa): ");
        Serial.println(bme680.pressure);
        
        Serial.print("Humidity (%): ");
        Serial.println(bme680.humidity);

        Serial.print("VOC (KOhms)= ");
        Serial.println(bme680.gas_resistance / 1000.0);

        Serial.print("Approx. Altitude (m)= ");
        Serial.println(bme680.readAltitude(SEALEVELPRESSURE_HPA));        
    }
    else
    {
        Serial.println("BME680: No data.");
    }
    
    delay(2000);
    
}