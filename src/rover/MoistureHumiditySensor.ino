/*************************************************
OSEPP Moisture & Humidity Sensor breakout Example

Read the current ambient temperature & humidity 
air levels along with the moisture water content.

   Copyright (C) 2016  Daniel Jay

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
*************************************************/


#include "DHT.h"
#define DHTPIN 2 
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);


int moisturePin = A0;    // select the input pin for the potentiometer
int moistureValue = 0;  // variable to store the value coming from the sensor
 
void setup() {
  // declare the ledPin as an OUTPUT:
   Serial.begin(9600);  
   dht.begin();
   
}
 
void loop() {

   delay(1000);
   
  // read the value from the sensor:
  moistureValue = analogRead(moisturePin);              
  Serial.print("moisture value  = " );                       
  Serial.println(moistureValue);       


               // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
}
