/*************************************************************************************************
 * name:       Analog Input
 * function:  Drive a Unipolar 5 lead coil stepper motor with variable speed in both directions
 *            and display the speed, direction, and position on a "20 cols, 4 rows" LCD Display  
 *                      
 *************************************************************************************************
 *
 The circuit:
 * I2C LCD2004 SDA attached to pin A4
 * I2C LCD2004 SCL attached to pin A5
 * Momentary Push Button attached to pin D12 (Yellow button for Zero) 
 * Momentary Push Button attached to pin D11 (Blue button for Unit Change)
 * DPDT switch attached to pins D7 and D8 for rotation direction control where neutral is zero speed
 * Stepper motor pins D6 thru D3 attached to ULN2003 driver and 28BYJ-48 stepper motor
 * Potentiometer attached to pin A0
 * LED attached from pin D13 to ground
 * Relay +in attached to pin D2
 * pushbutton attached from pins D12/D11 to +5V
 * 10K resistor attached from pin D12 to ground

/********************************/
// References:
// https://42bots.com/tutorials/28byj-48-stepper-motor-with-uln2003-driver-and-arduino-uno/
// http://www.airspayce.com/mikem/arduino/AccelStepper/
// http://forum.arduino.cc/index.php?topic=28388.0
// http://www.circuitbasics.com/setting-up-a-5v-relay-on-the-arduino/
// https://42bots.com/tutorials/28byj-48-stepper-motor-with-uln2003-driver-and-arduino-uno/

// Library code:
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

// Definitions:
#define HALFSTEP 8
// Motor pin definitions
#define motorPin1  3     // IN1 on the ULN2003 driver 1
#define motorPin2  4     // IN2 on the ULN2003 driver 1
#define motorPin3  5     // IN3 on the ULN2003 driver 1
#define motorPin4  6     // IN4 on the ULN2003 driver 1
#define M_PI 3.14159265358979323846 /* pi */
// Initialize LCD Display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

/*********************************************************/
// Constants:
// set pin numbers:
const int buttonPin12 = 12;    // the number of the pushbutton pin zero (yellow)
const int buttonPin11 = 11;    // the number of the pushbutton pin units (blue)
const int ledPin = 13;      // the number of the LED pin
const int RelayPin = 2;      // the number of the Relay pin to power the stepper motor
int potPin = A0;    // select the input pin for the potentiometer
int potValue = 0;  // variable to store the value coming from the sensor
int LCD_update = 0; // LCD update trigger, if 0 last_update gets new time

// Variables which change:
int ledState12 = HIGH;         // the current state of the output pin zero (yellow)
int buttonState12;             // the current reading from the input pin zero (yellow)
int lastButtonState12 = LOW;   // the previous reading from the input pin zero (yellow)
int ledState11 = HIGH;         // the current state of the output pin units (blue)
int buttonState11;             // the current reading from the input pin units (blue)
int lastButtonState11 = LOW;   // the previous reading from the input pin units (blue)
int motorSpeed = 0; // Motor Speed integer
int units = 1; // 1 for mm, 0 for in

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime12 = 0;  // the last time the output pin was toggled
long lastDebounceTime11 = 0;  // the last time the output pin was toggled
long debounceDelay = 250;    // the debounce time; increase if the output flickers
long last_update = 0; // the last time the LCD screen was refreshed for DPDT
long last_update_state = 0; // the last time the LCD screen was refreshed for state of vel and pos
long LCD_delay = 750; // the LCD refresh rate

// distance, speed
double ind_pos = 0.0; // the distance as measured by counting stepper steps
double ind_speed = 0.0; // the speed as measured by counting steps over time
double mm_to_in = 0.0393701; // scale factor to go from mm to in, same for mm/s to in/s
double mm_wheel_radius = 17; // drive wheel radius in mm

// the following variables are strings for display purposes
char string_units_pos[] = " mm "; // units swap string from mm to in
char string_print[20]; // generic string variable
char ind_dir[] = "FWD"; // indicated rotation direction
char outstr[15]; // buffer output string for double to string

/*********************************************************/
void setup()
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  lcd.init();  //initialize the lcd
  lcd.backlight();  //open the backlight 
  
  lcd.setCursor ( 0, 0 );            // go to the top left corner
  strcat(string_print,"CableFeeder U:");
  strcat(string_print,string_units_pos);
  lcd.print(string_print); // write this string on the top row
  memset(string_print, 0, sizeof(string_print)); //reset string_print
  
  lcd.setCursor ( 0, 1 );            // go to the 2nd row
  strcat(string_print,"Spd:   ");
  dtostrf(ind_speed,5,2,outstr);
  strcat(string_print,outstr);
  lcd.print(string_print); // pad string with spaces for centering
  memset(string_print, 0, sizeof(string_print)); //reset string_print
  
  lcd.setCursor ( 0, 2 );            // go to the third row
  strcat(string_print,"Dir:     ");
  strcat(string_print,ind_dir);
  lcd.print(string_print); // pad with spaces for centering
  memset(string_print, 0, sizeof(string_print)); //reset string_print
  
  lcd.setCursor ( 0, 3 );            // go to the fourth row
  strcat(string_print,"Pos:   ");
  dtostrf(ind_pos,5,2,outstr);
  strcat(string_print,outstr);
  lcd.print(string_print);
  memset(string_print, 0, sizeof(string_print)); //reset string_print

  pinMode(buttonPin12, INPUT); // Zero button
  pinMode(buttonPin11, INPUT); // Unit switch button
  pinMode(7, INPUT); // BCK dir
  pinMode(8, INPUT); // FWD dir
  pinMode(ledPin, OUTPUT);
  pinMode(RelayPin, OUTPUT);
  
  // set initial state
  digitalWrite(ledPin, ledState12);
  digitalWrite(RelayPin, HIGH);
  
  // set stepper motor initial speed in steps/sec
  stepper1.setMaxSpeed(1000); 
}
/*********************************************************/
void loop() 
{
  // read the state of the switch into a local variable:
  int reading12 = digitalRead(buttonPin12);
  int reading11 = digitalRead(buttonPin11);
  int sensorValueTw = digitalRead(7);
  int sensorValueTh = digitalRead(8);
  

  
  // read the pot analog value
  potValue = analogRead(potPin);
  // map it to the range of the analog out:
  motorSpeed = map(potValue, 0, 1023, 0, 1000); 
  ind_speed = ((double)motorSpeed)/4076; // speed in rev per second, as it takes 4076 steps to achieve one revolution
  ind_speed = (double)ind_speed*2*M_PI*mm_wheel_radius ; // speed in mm/s for drive wheel 
  ind_pos = (double)stepper1.currentPosition()/4076; // step position in rev
  ind_pos = (double)ind_pos*2*M_PI*mm_wheel_radius; // step position in mm for drive wheel
  
  // LCD update init
  if (LCD_update==0)
  {
  last_update = millis();
  last_update_state = millis();
  LCD_update=1;
  }
  
  // unit definition, starts with mm
  if(units == 1){
    ind_pos = ind_pos;
    ind_speed = ind_speed;
  }
  else{
    ind_pos = ind_pos*mm_to_in;
    ind_speed = ind_speed*mm_to_in;
  }
  
  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  
  // If the switch changed, due to noise or pressing:
  if (reading12 != lastButtonState12) {
    // reset the debouncing timer
    lastDebounceTime12 = millis();
  } 
  if (reading11 != lastButtonState11) {
    // reset the debouncing timer
    lastDebounceTime11 = millis();
  } 

  // Zero Button
  if ((millis() - lastDebounceTime12) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading12 != buttonState12) {
      buttonState12 = reading12;

      // only toggle the LED if the new button state is HIGH
      if (buttonState12 == HIGH) {
        ledState12 = !ledState12;
        stepper1.setCurrentPosition(0);
        lcd.setCursor ( 0, 3 );            // go to the fourth row
        strcat(string_print,"Pos:    ");
        dtostrf(ind_pos,5,2,outstr);
        strcat(string_print,outstr);
        lcd.print(string_print);
        memset(string_print, 0, sizeof(string_print)); //reset string_print
      }
    }
  }
  
  // Unit Button
  if ((millis() - lastDebounceTime11) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading11 != buttonState11) {
      buttonState11 = reading11;

      // only toggle the LED if the new button state is HIGH
      if (buttonState11 == HIGH) {
        ledState11 = !ledState11;
      }
        if( ledState11 == HIGH){
        units = 1;
        memset(string_units_pos, 0, sizeof(string_units_pos)); //reset string_print
        strcat(string_units_pos," mm ");
        
        lcd.setCursor ( 0, 0 );            // go to the top left corner
        strcat(string_print,"CableFeeder U:");
        strcat(string_print,string_units_pos);
        lcd.print(string_print); // write this string on the top row
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        
        lcd.setCursor ( 7, 1 );            // go to the 2nd row
        dtostrf(ind_speed,5,2,outstr);
        strcat(string_print,outstr);
        lcd.print(string_print); // pad string with spaces for centering
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        lcd.setCursor ( 13, 1 );            // go to the 2nd row
        lcd.print("   "); // pad string with spaces for centering
        
        lcd.setCursor ( 8, 3 );            // go to the fourth row
        dtostrf(ind_pos,5,2,outstr);
        strcat(string_print,outstr);
        lcd.print(string_print);
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        lcd.setCursor (14,3);
        lcd.print("   ");
        }
      
      if (ledState11 == LOW) {
        units = 0;
        memset(string_units_pos, 0, sizeof(string_units_pos)); //reset string_print
        strcat(string_units_pos," in ");
    
        lcd.setCursor ( 0, 0 );            // go to the top left corner
        strcat(string_print,"CableFeeder U:");
        strcat(string_print,string_units_pos);
        lcd.print(string_print); // write this string on the top row
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        
        lcd.setCursor ( 7, 1 );            // go to the 2nd row
        dtostrf(ind_speed,5,2,outstr);
        strcat(string_print,outstr);
        lcd.print(string_print); // pad string with spaces for centering
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        lcd.setCursor ( 13, 1 );            // go to the 2nd row
        lcd.print("   "); // pad string with spaces for centering
        
        lcd.setCursor ( 8, 3 );            // go to the fourth row
        dtostrf(ind_pos,5,2,outstr);
        strcat(string_print,outstr);
        lcd.print(string_print);
        memset(string_print, 0, sizeof(string_print)); //reset string_print
        lcd.setCursor (14,3);
        lcd.print("   ");
      }
    }
  }
  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState12 = reading12;
  lastButtonState11 = reading11;
  // set the LED:
  digitalWrite(ledPin, ledState12);

  // set the direction of the stepper motor
 if (sensorValueTw == 1) stepper1.setSpeed(-motorSpeed);
 if (sensorValueTh == 1) stepper1.setSpeed(motorSpeed);
 if (sensorValueTh == 0 && sensorValueTw == 0) stepper1.setSpeed(0);

stepper1.runSpeed();
 // Display Speed, Position and Direction
 
  if ((millis() - last_update_state) > LCD_delay)
  {
    last_update_state = millis();
    lcd.setCursor ( 7, 1 );            // go to the 2nd row
    dtostrf(ind_speed,5,2,outstr);
    strcat(string_print,outstr);
    lcd.print(string_print); // pad string with spaces for centering
    memset(string_print, 0, sizeof(string_print)); //reset string_print

    memset(ind_dir, 0, sizeof(ind_dir)); //reset string_print
    if (sensorValueTh == 1) strcat(ind_dir,"FWD");
    if (sensorValueTw == 1) strcat(ind_dir,"BCK");
    if (sensorValueTh == 0 && sensorValueTw == 0) strcat(ind_dir,"STP");
    lcd.setCursor ( 9, 2 );            // go to the third row
    strcat(string_print,ind_dir);
    lcd.print(string_print); // pad with spaces for centering
    memset(string_print, 0, sizeof(string_print)); //reset string_print
  
    lcd.setCursor ( 8, 3 );            // go to the fourth row
    dtostrf(ind_pos,5,2,outstr);
    strcat(string_print,outstr);
    lcd.print(string_print);
    memset(string_print, 0, sizeof(string_print)); //reset string_print
  } 

  
}
/************************************************************/
