/*
  LiquidCrystal Library - Hello World

  Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
  library works with all LCD displays that are compatible with the
  Hitachi HD44780 driver. There are many of them out there, and you
  can usually tell them by the 16-pin interface.

  This sketch prints "Hello World!" to the LCD
  and shows the time.

  The circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)

  Library originally added 18 Apr 2008
  by David A. Mellis
  library modified 5 Jul 2009
  by Limor Fried (http://www.ladyada.net)
  example added 9 Jul 2009
  by Tom Igoe
  modified 22 Nov 2010
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/LiquidCrystal
*/


// include the library code:
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <DHT.h>
#include <DHT_U.h>
#include <math.h>

#define DHTPIN            13         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11 

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float temperature;
float ext_temperature;

bool f = true;
//Relays.
bool R1 = false;
bool R2 = false;
bool R3 = false;
bool R4 = false;
bool R5 = false;
bool R6 = false;
bool R7 = false;
bool R8 = false;
//Light groups
bool L1 = false;
bool L2 = false;
bool L3 = false;
bool L4 = false;

int S1 = 4;
int S2 = 3;
int S3 = 2;
int S4 = 5;
#define SREVERSE 6 // Reverse trigger pin
int Sreverse_state = HIGH;
int S1_state = HIGH;
int S2_state = HIGH;
int S3_state = HIGH;
int S4_state = HIGH;
int S5_state = HIGH;
int S1_count = 0;
int S2_count = 0;
int S3_count = 0;
int S4_count = 0;
int ticks = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  pinMode(S1, INPUT_PULLUP);
  pinMode(S2, INPUT_PULLUP);
  pinMode(S3, INPUT_PULLUP);
  pinMode(S4, INPUT_PULLUP);
  pinMode(SREVERSE, INPUT_PULLUP);
  Serial.begin(300);
  //dht.begin();
  // set up the LCD's number of columns and rows:
  lcd.begin(20, 4);
  blank();
  /* Initialise the sensor */
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    lcd.print("BMP085 Not detected.");
    lcd.print("Check your wiring or I2C address.");
    while (1);
  }
  redraw();
}

void loop() {
  //Reverse trigger handler. Use relay to gnd/low on "REV" lead +12 at headunit
  
  if (digitalRead(SREVERSE) == LOW && Sreverse_state == HIGH) {
      R7 = true;
      R8 = true;
      sendSwitchChange();
      drawSwitches();
  }else if (digitalRead(SREVERSE) == HIGH && Sreverse_state == LOW) {
      R7 = false;
      R8 = false;
      sendSwitchChange();
      drawSwitches();
  }
  Sreverse_state = digitalRead(SREVERSE);

  
  //Switches
  if (digitalRead(S1) == LOW && S1_state == HIGH) {
    if (R1 == false) {
      R1 = true;
      R2 = true;
    } else {
      R1 = false;
      R2 = false;
    }
    sendSwitchChange();
    drawSwitches();
  } else if (digitalRead(S1) == HIGH && S1_state == HIGH) {
    // Count to detect long presses for additional functions.
    S1_count++;
  }
  S1_state = digitalRead(S1); // Only on transition once
  if (digitalRead(S2) == LOW && S2_state == HIGH) {
    if (R3 == false) {
      R3 = true;
      R4 = true;
    } else {
      R3 = false;
      R4 = false;
    }
    sendSwitchChange();
    drawSwitches();
  } else if (digitalRead(S2) == HIGH && S2_state == HIGH) {
    // Count to detect long presses for additional functions.
    S2_count++;
  }
  S2_state = digitalRead(S2); // Only on transition once
  if (digitalRead(S3) == LOW && S3_state == HIGH) {
    if (R5 == false) {
      R5 = true;
      R6 = true;
    } else {
      R5 = false;
      R6 = false;
    }
    sendSwitchChange();
    drawSwitches();
  } else if (digitalRead(S3) == HIGH && S3_state == HIGH) {
    // Count to detect long presses for additional functions.
    S3_count++;
  }
  S3_state = digitalRead(S3); // Only on transition once
  

  if (digitalRead(S4) == LOW && S4_state == HIGH) {
    if (R7 == false) {
      R7 = true;
      R8 = true;
    } else {
      R7 = false;
      R8 = false;
    }
    sendSwitchChange();
    drawSwitches();
  } else if (digitalRead(S4) == HIGH && S4_state == HIGH) {
    // Count to detect long presses for additional functions.
    S4_count++;
  }
  S4_state = digitalRead(S4); // Only on transition once

  // Environment Display
  bmp.getEvent(&event);
  bmp.getTemperature(&temperature);

  //Only update the temp/pressure display once every 100 loop()s

  if (ticks >= 100)
  {

    ext_temperature = 0; //Get remote here?
    updateDisplay(event.pressure, temperature, ext_temperature);
    ticks = 0;
  }
  else
  {
    ticks++;
  }
  delay(10);


}

void echo(char e) {
  lcd.setCursor(0, 3);
  lcd.print(e);
}

void redraw() {
  blank();
  //lcd.setCursor(0, 1);
  //lcd.print("Pressure:");
  //lcd.setCursor(0, 2);
  //lcd.print("Temp 1  :");
}

void updateDisplay(float prsr_hpa, float temp_c, float ext_temp_c) {
  lcd.setCursor(10, 1);
  lcd.print(round(prsr_hpa));
  lcd.print("Mbar ");

  if (f) {
    lcd.setCursor(11, 2);
    float temp_f = c_to_f(temp_c);
    if (temp_f < 100.0) {
      lcd.print(" "); // blank the 100s column
    }
    lcd.print(round(temp_f));
    lcd.print((char)223);
    lcd.print("F  ");
  } else {
    lcd.setCursor(11, 2);
    lcd.print(" "); // it will never be 100C. But if we switched from F to C in runtime...
    lcd.print(temp_c);
    lcd.print("C  ");
  }
  drawSwitches();
}

int c_to_f(float temp) {
  return temp * 1.8 + 32.0;
}

float hpa_to_inhg(float hpa) {
  return hpa * 0.29529983071445;
}

void drawSwitches() {
  lcd.setCursor(4, 0);
  if (R1) {
    lcd.print("On ");
  } else {
    lcd.print("Off");
  }
  lcd.setCursor(4, 1);
  if (R3) {
    lcd.print("On ");
  } else {
    lcd.print("Off");
  }
  lcd.setCursor(4, 2);
  if (R5) {
    lcd.print("On ");
  } else {
    lcd.print("Off");
  }
  lcd.setCursor(4, 3);
  if (R7) {
    lcd.print("On ");
  } else {
    lcd.print("Off");
  }
}


void blank() {
  lcd.setCursor(0, 0);
  lcd.print("L1:                 ");
  lcd.print("L3:                 ");
  //LCD line wrap is weird, 1,3,2,4 is intentional.
  lcd.print("L2:                 ");
  lcd.print("L4:                 ");
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void sendSwitchChange() {

  char l_state[] = "00000000";

  if (R1 == true) {
    l_state[0] = '1';
  } else {
    l_state[0] = '0';
  }

  if (R2 == true) {
    l_state[1] = '1';
  } else {
    l_state[1] = '0';
  }

  if (R3 == true) {
    l_state[2] = '1';
  } else {
    l_state[2] = '0';
  }

  if (R4 == true) {
    l_state[3] = '1';
  } else {
    l_state[3] = '0';
  }

  if (R5 == true) {
    l_state[4] = '1';
  } else {
    l_state[4] = '0';
  }

  if (R6 == true) {
    l_state[5] = '1';
  } else {
    l_state[5] = '0';
  }

  if (R7 == true) {
    l_state[6] = '1';
  } else {
    l_state[6] = '0';
  }

  if (R8 == true) {
    l_state[7] = '1';
  } else {
    l_state[7] = '0';
  }

  Serial.write("001:1:"); //Light controller 001, bank 1,
  Serial.write(l_state);
  Serial.write(";");
}

