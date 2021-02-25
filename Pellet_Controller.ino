/*********************

Example code for the Adafruit RGB Character LCD Shield and Library

This code displays text on the shield, and also reads the buttons on the keypad.
When a button is pressed, the backlight changes color.

**********************/

// include the library code:
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Adafruit_BME280.h>


// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

/* Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
*/
/* 
  Connect 5V on Arduino to VCC on Relay Module
  Connect GND on Arduino to GND on Relay Module 
  Connect GND on Arduino to the Common Terminal (middle terminal) on Relay Module. */
 
 #define CH1 8   // Connect Digital Pin 8 on Arduino to CH1 on Relay Module
 #define CH3 7   // Connect Digital Pin 7 on Arduino to CH3 on Relay Module
 #define CH2 6   // Connect Digital Pin 6 on Arduino to CH1 on Relay Module
 #define CH4 5   // Connect Digital Pin 5 on Arduino to CH3 on Relay Module
 #define TS1 A0

int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures
uint8_t i=0; 
uint8_t selection=0;
uint8_t timer=0;
uint8_t buttons=0;
int reading=0;
float voltage=0.0;                        
uint8_t db_time=0;
uint8_t pot=0;

void setup() {
  // Debugging output
  Serial.begin(115200);
  Serial.println(F("BME280 Sensor event test"));

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }
  
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print("Starting ");
  delay(2000);

  // Print a message to the LCD. We track how long it takes since
  // this library has been optimized a bit and we're proud of it :)
  db_time = millis() - db_time;
  Serial.print("Took "); Serial.print(db_time); Serial.println(" ms");
  lcd.setBacklight(WHITE);

  //Setup all the Arduino Pins
   pinMode(CH1, OUTPUT);
   pinMode(CH3, OUTPUT);
   pinMode(TS1, INPUT);
   
  //Turn OFF any power to the Relay channels
   digitalWrite(CH1,HIGH);
   digitalWrite(CH3,HIGH);
   delay(2000); //Wait 2 seconds before starting sequence
}

void loop() {
/*
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Humidity = "));
  Serial.print(humidity_event.relative_humidity);
  Serial.println(" %");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();  */
  
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
 lcd.setCursor(0, 0);
 lcd.clear();
  // print the number of seconds since reset:
 lcd.print("Timer: ");
 timer=millis()/1000;
 lcd.print(timer);
    
 //getting the voltage reading from the temperature sensor
 reading = analogRead(sensorPin);  

// converting that reading to voltage, for 3.3v arduino use 3.3
 voltage = (reading * 3.3);
 voltage /= 1024.0;
 
// Debug
 Serial.print(voltage); Serial.println(" volts");
 Serial.print(reading); Serial.println(" sensor");
 
 // print out the voltage
 lcd.setCursor(0, 1);
// lcd.print(reading); 
// lcd.println(" sensor     ");

 if(reading > 160)
 {
   digitalWrite(CH1,LOW);

 }
 else
 {
   digitalWrite(CH1,HIGH);

 }
 if(reading > 176)
 {
   digitalWrite(CH3,LOW);

 }
 else
 {
   digitalWrite(CH3,HIGH);

 }
 
 // now print out the temperature
 float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree with 500 mV offset
 
//to degrees ((voltage - 500mV) times 100)
 lcd.setCursor(0,1);
 lcd.print(temperatureC); 
 lcd.print(" C ");
 
 // now convert to Fahrenheit
float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
lcd.print(temperatureF); 
lcd.println(" F  ");
 
 delay(1000);                  
  buttons = lcd.readButtons();

  if (buttons) {
    lcd.clear();
    lcd.setCursor(0,0);
    if (buttons & BUTTON_UP) {
      lcd.print("Auger OFF ");
      selection=1;
      lcd.print(selection);
      lcd.setBacklight(RED);
      digitalWrite(CH1,HIGH);
    }
    if (buttons & BUTTON_DOWN) {
      lcd.print("Auger ON ");
      selection=2;
      lcd.print(selection);
      lcd.setBacklight(YELLOW);
      digitalWrite(CH1,LOW);
     }
    if (buttons & BUTTON_LEFT) {
      lcd.print("Fan OFF ");
      selection=3;
      lcd.print(selection);
      lcd.setBacklight(GREEN);
      digitalWrite(CH3,HIGH);
    }
    if (buttons & BUTTON_RIGHT) {
      lcd.print("FAN ON ");
      selection=4;
      lcd.print(selection);
      lcd.setBacklight(TEAL);
      digitalWrite(CH3,LOW);
    }
    if (buttons & BUTTON_SELECT) {
      lcd.print("SELECT ");
      selection=0;
      lcd.print(selection);
      lcd.setBacklight(VIOLET);
      digitalWrite(CH3,HIGH);
      digitalWrite(CH1,HIGH);
    }
  }
 }
  
