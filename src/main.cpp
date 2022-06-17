#include <Arduino.h>

#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

//init DHT-Sensor
#define dhtType DHT11
const u_int8_t dhtPin = 22;     //pin connected to the dhtSensor
DHT dht(dhtPin, dhtType);

//init SoilHumiditySensor
const u_int8_t soilPin = 32;

const u_int8_t boardLed = 16;
u_int8_t bootFlag = 1;

void ledOff(){ 
  digitalWrite(boardLed, HIGH); //LED is inverted...
}

void ledOn(){
  digitalWrite(boardLed, LOW); //LED is inverted...
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // run on serialPort 9600 for debugOutput
  dht.begin();  // init dhtSensor

  pinMode(boardLed,OUTPUT); //Setup PinMode for LED
  ledOff(); //LED is inverted...
  //digitalWrite(PIN, STATE); STATE = LOW/HIGH
}

void ledflash(u_int16_t timer) {
  ledOn();
  delay(timer);            
  ledOff();
  delay(timer);             
}

void debugOutput(float humidity, float temperature,float heatIndex, float soilHumidity){
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    ledOn();
    return;
  }
  else if (isnan(soilHumidity))
  {
    Serial.println(F("Failed to read from Soil-Humidity Sensor!"));
    ledOn();
    return;
  }
  
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.print(F("°C  Heat index: "));
  Serial.print(heatIndex);
  Serial.print(F("°C "));
  Serial.print(F("Soil-Humidity: "));
  Serial.println(analogRead(soilPin));
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Wait a few seconds between measurements.
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  float humidity = dht.readHumidity();

  // Read temperature as Celsius (the default)
  float temperature = dht.readTemperature();

   // Check if any reads failed and exit early (to try again).


  // Compute heat index in Celsius (Temperature,Humidity,isFahreheit)
  float heatIndex = dht.computeHeatIndex(temperature, humidity, false);

  // get Analog Reading of SoilHumidity Sensor
  uint16_t soilHumidity = analogRead(soilPin);
  debugOutput(humidity,temperature,heatIndex,soilHumidity);
}

