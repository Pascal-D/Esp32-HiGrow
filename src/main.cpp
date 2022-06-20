#include <Arduino.h>

#include <Adafruit_Sensor.h>

#include <DHT.h>
#include <DHT_U.h>

//init dht sensor
#define dhtType DHT11
const u_int8_t dhtPin = 22;
DHT dht(dhtPin, dhtType);

//Board-LED is inverted...
const u_int8_t boardLed = 16;

const u_int8_t soilPin = 32;

hw_timer_t * timer = NULL;

void ledOff(u_int8_t led,bool inverted){
  if (inverted){
    digitalWrite(led, HIGH); 
  } else{
    digitalWrite(led, LOW); 
  }
}

void ledOn(u_int8_t led,bool inverted){
  if (inverted){
    digitalWrite(led, LOW); 
  } else{
    digitalWrite(led, HIGH); 
  }
}

void ledflash(u_int8_t led,bool inverted,u_int16_t timer) {
  ledOn(led,inverted);
  delay(timer);            
  ledOff(led,inverted);
  delay(timer);             
}

void IRAM_ATTR onErrorTimer(){
    if(digitalRead(boardLed)){
        ledOn(boardLed,true);
    } else{
      ledOff(boardLed,true);
    }
}

void errorTimer(){
  timer = timerBegin(0,80,true);
  timerAttachInterrupt(timer,&onErrorTimer,true);
  timerAlarmWrite(timer,1000000,true);
  timerAlarmEnable(timer);
}

void setup() {
   // run on serialPort 9600 for debugOutput
  Serial.begin(9600);

  //start dht sensor
  dht.begin();

  //Setup Board-LED
  pinMode(boardLed,OUTPUT); 
  ledOff(boardLed,true);
}

void debugOutput(float humidity, float temperature,float heatIndex, float soilHumidity){
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    if (timer == NULL){
      errorTimer();
    }
    return;
  } else if (isnan(soilHumidity)){
    Serial.println(F("Failed to read from Soil-Humidity Sensor!"));
    if (timer == NULL){
      errorTimer();
    }
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

  // Compute heat index in Celsius (Temperature,Humidity,isFahreheit)
  float heatIndex = dht.computeHeatIndex(temperature, humidity, false);

  // get Analog Reading of SoilHumidity Sensor
  uint16_t soilHumidity = analogRead(soilPin);
  debugOutput(humidity,temperature,heatIndex,soilHumidity);
}

