/*************************************************** 
  This is an example sketch for our optical Fingerprint sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> http://www.adafruit.com/products/751

  These displays use TTL Serial to communicate, 2 pins are required to 
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Keypad.h"
#include "Adafruit_Fingerprint.h"
#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#define analogPin A1 //the thermistor attach to 
#define beta 3950 //the beta of the thermistor
#define resistance 10 //the value of the pull-up resistor
LiquidCrystal_I2C lcd(0x27,20,4);
int getFingerprintIDez();

// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
SoftwareSerial mySerial(10, 11);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// On Leonardo/Micro or others with hardware serial, use those! #0 is green wire, #1 is white
//Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial1);
int pushButton = 2;
const int FSR_PIN = A0; // Pin connected to FSR/resistor divider

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k resistor

void setup()  
{
  Serial.begin(9600);
   finger.begin(57600);
  //lcd.begin(16, 2);
  Serial.print("PIN Lock ");
  pinMode(pushButton, INPUT);
  //lcd.clear();
  Serial.print("  Enter PIN...");
                    // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
 
pinMode(FSR_PIN, INPUT);
  //lcd.setCursor(3,0);
  //lcd.println("Adafruit finger detect test");
   //lcd.clear();
  // set the data rate for the sensor serial port
  
  pinMode(pushButton, INPUT);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1);
  }
  //analogWrite(6,0);
}





void loop()                     // run over and over again
{
    long a =1023 - analogRead(analogPin);
  analogWrite(6,0);
  lcd.clear(); 
  lcd.setCursor(4,0);
   lcd.print("Welcome!");
  lcd.setCursor(0,1);
  
  lcd.print("System is Armed!!");
     float tempC = beta /(log((1025.0 * 10 / a - 10) / 10) + beta / 298.0) - 273.0;
     float tempF = 1.8*tempC + 32.0;
     lcd.setCursor(0, 2);// set the cursor to column 0, line 0
  lcd.print("Temp: "); // Print a message of "Temp: "to the LCD.
  lcd.print(tempC); // Print a centigrade temperature to the LCD.
  // Print the unit of the centigrade temperature to the LCD.
  lcd.write(char(223));
  lcd.print("C");//print the unit" ℃ "
  lcd.setCursor(0, 3);// set the cursor to column 0, line 1
  lcd.print("Fahr: ");
  lcd.print(tempF);// Print a Fahrenheit temperature to the LCD.
  lcd.write(char(223)); // Print the unit of the Fahrenheit temperature to the LCD.
  lcd.print("F");//print the unit"°F"
  int i=0;
  int fsrADC = analogRead(FSR_PIN);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    Serial.println("Resistance: " + String(fsrR) + " ohms");
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;
    Serial.println("Force: " + String(force) + " g");
    Serial.println();
     delay(100);
  }
  else
  {
    // No pressure detected
  }
 
  int buttonState = digitalRead(pushButton);
  // print out the state of the button:
  Serial.println(buttonState);
  if(buttonState==0) {tone(9,1074);  }
  if(force>30)tone(9,1074);
  getFingerprintIDez();
  //Serial.print(getFingerprintIDez());
  lcd.clear();
  //delay(100);

}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }
  
  // OK converted!
  p = finger.fingerFastSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }   
  
  // found a match!
  
  Serial.print("Found ID 76567#"); Serial.print(finger.fingerID); 
  Serial.print(" with confidence of "); Serial.println(finger.confidence); 
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  { return -1; }
   
  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  {  return -1; }
  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK) { return -1; }

  // found a match!
   
   
   //tone(9,1074);
   //delay(100);
  noTone(9);
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Access Granted!!"); lcd.setCursor(10,0);   
 digitalWrite(10,LOW); digitalWrite(11,LOW); 
 analogWrite(6,255);
 for(int timeRemaining = 7;timeRemaining > 0; timeRemaining--)  {lcd.setCursor(0,1);
  lcd.print("time remaining");  lcd.setCursor(16,1);
  lcd.print(timeRemaining); delay(1000);
  }
  //delay(5000);
  lcd.clear();
  Serial.print("Found ID3212314 #"); Serial.print(finger.fingerID); 
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return 1;
}
