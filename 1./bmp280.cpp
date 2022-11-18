#include <SPI.h>                         // Needed for legacy versions of Arduino.
#include <BME280Spi.h>

#define SERIAL_BAUD 115200

#define DEVICE_PIN 10

BME280Spi::Settings settings(DEVICE_PIN); // Default : forced mode, standby time = 1000 ms
                                          //           Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

BME280Spi bme(settings);


//////////////////////////////////////////////////////////////////
void setup()
{
   Serial.begin(SERIAL_BAUD);

   while(!Serial) {} // Wait

   SPI.begin();

   while(!bme.begin())
   {
      Serial.println("Could not find BME280 sensor!");
      delay(1000);
   }

   switch(bme.chipModel())
   {
      case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        Serial.println("Found UNKNOWN sensor! Error!");
   }
}

//////////////////////////////////////////////////////////////////
void loop()
{
   printBME280Data(&Serial);
   delay(500);
}

//////////////////////////////////////////////////////////////////
void printBME280Data
(
   Stream* client
)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("Pa");

   delay(1000);
}
