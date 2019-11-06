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

// Library code:
#include <digitalWriteFast.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

// Definitions:
#define  I2C_Address 0x27
#define  ADA_Address 0x20


#define MOTOR_STEPS 200
#define MICROSTEPS 2
#define DIR 4
#define STEP 5


/*
#define HALFSTEP 8
// Motor pin definitions
#define motorPin1  3     // IN1 on the ULN2003 driver 1
#define motorPin2  4     // IN2 on the ULN2003 driver 1
#define motorPin3  5     // IN3 on the ULN2003 driver 1
#define motorPin4  6     // IN4 on the ULN2003 driver 1
*/
#define M_PI 3.14159265358979323846 /* pi */
// Initialize LCD Display


LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
AccelStepper stepper1(1, DIR, STEP);

//A4988 stepper1(MOTOR_STEPS, DIR, STEP);
//Adafruit_LiquidCrystal lcd(0x20);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
//AccelStepper stepper1(HALFSTEP, motorPin1, motorPin2, motorPin3, motorPin4);

/*
PD0 = 0
PD1 = 1
...
PD6 = 6
PD7 = 7

PB0 = 8
..
PB5 = 13

PC0 = A0
PC5 = PC5 


PD3 - PD6 = motorpins
A  - Trimpot


*/


/*********************************************************/
// Constants:
// set pin numbers: 
const int zeroButton = 12;    // the number of the pushbutton pin zero (yellow)
const int unitsButton = 11;    // the number of the pushbutton pin units (blue)

const int forwardButton = 7;
const int backwardButton = 8;

const int ledPin = 13;      // the number of the LED pin
const int RelayPin = 2;      // the number of the Relay pin to power the stepper motor

int potPin = A0;    // select the input pin for the potentiometer
//-------------------------------------------------------------------

// static variables

const uint8_t LCD_delay = 100; // the LCD refresh rate
const double mm_to_in = 0.0393701; // scale factor to go from mm to in, same for mm/s to in/s
const double mm_wheel_radius = 17; // drive wheel radius in mm

const double speedToMM = 2*M_PI*mm_wheel_radius;

const int NEMA17_STEPS = 200;
const int NEMA17_FULLREVOLUTION = NEMA17_STEPS*MICROSTEPS;

// Variables which change:
int motorSpeed = 0; // Motor Speed integer

float ind_pos = 0.0; // the distance as measured by counting stepper steps
float ind_speed = 0.0; // the speed as measured by counting steps over time

uint8_t LCD_update = 0;

int potValue = 0;  // variable to store the value coming from the sensor
bool units = false; // true for mm, false for in

bool readZero=false;      // = digitalReadFast(zeroButton);
bool readUnits=false;      //= digitalReadFast(unitsButton);
bool readMotorForward=false;  // = digitalReadFast(7);
bool readMotorBackward=false;  //= digitalReadFast(8);

uint8_t debounce_loop=0;
uint8_t debounce=0;

bool zeroLast     =false;
bool unitsLast    =false;
bool forwardLast  =false;
bool backwardLast =false; 

uint8_t currentMotorSetting=0; //0 = stopped, 1 = forward, 2 = back
uint8_t previousMotorSetting=0;

bool updateLCDDirection=false;
bool updateLCDunits=false;

/*********************************************************/
void setup()
{  
 
  // initialize serial communications at 9600 bps:
  //Serial.begin(9600);
  
  //lcd.begin(20,4);  //initialize the lcd
  lcd.begin();
  //lcd.backlight();  //open the backlight 
  
  lcd.setCursor ( 0, 0 );            // go to the top left corner  
  lcd.print(F("CableFeeder U: inch"));
  
  lcd.setCursor ( 0, 1 );            // go to the 2nd row
  lcd.print(F("Spd:"));

  lcd.setCursor ( 0, 2 );  
  lcd.print(F("Dir:        STP")); 

  lcd.setCursor ( 0, 3 );  
  lcd.print(F("Pos:")); 
   
  pinModeFast(zeroButton, INPUT);
  pinModeFast(unitsButton, INPUT);
  pinModeFast(7, INPUT);
  pinModeFast(8, INPUT);

  stepper1.setMaxSpeed(1000); 
 
}
/*********************************************************/
void loop() 
{

  // SEQ1: Read Inputs
  
  //1-1-------------------------------------------------------------------

  //for loops are used to detecting debouncing problems
  
 
  for(debounce_loop=0,debounce=0; debounce_loop<10;debounce_loop++)
  {
    if(digitalReadFast(zeroButton))
    debounce++;
  }
  
  if(debounce>7)
  readZero=true;
  else
  readZero=false;  
  
  for(debounce_loop=0,debounce=0; debounce_loop<10;debounce_loop++)
  {
    if(digitalReadFast(unitsButton))
    debounce++;
  }
  
  if(debounce>7)
  readUnits=true;
  else
  readUnits=false;  

  for(debounce_loop=0,debounce=0; debounce_loop<10;debounce_loop++)
  {
    if(digitalReadFast(forwardButton))
    debounce++;
  }
  
  if(debounce>7)
  readMotorForward=true;
  else
  readMotorForward=false;

  for(debounce_loop=0,debounce=0; debounce_loop<10;debounce_loop++)
  {
    if(digitalReadFast(backwardButton))
    debounce++;
  }
  
  if(debounce>7)
  readMotorBackward=true;
  else
  readMotorBackward=false;
  
  //1-2-------------------------------------------------------------------
  
  // read the pot analog value

  potValue = analogRead(potPin);
  motorSpeed = map(potValue, 0, 1023, 0, 1000);  
  
  //--------------------------------------------------------------------

  //SEQ2 - update settings

  if(readUnits!=unitsLast)
  {
    unitsLast=readUnits;
    if(!readUnits)
    {
      units=!units; 
      updateLCDunits=true;
    } 
  }
  
 if(readZero!=zeroLast)
 {
    zeroLast=readZero;
    if(readZero)    
      stepper1.setCurrentPosition(0);  
 }
 
  if(readMotorForward)
    currentMotorSetting=1;
  else if(readMotorBackward)
    currentMotorSetting=2;
  else  
    currentMotorSetting=0;
    
  if(currentMotorSetting!=previousMotorSetting)
  {
     previousMotorSetting=currentMotorSetting;
     updateLCDDirection=true; 
  }  

//--------------------------------------------------------------------

  //SEQ3 update speed and pos settings

  ind_speed = (((float)motorSpeed/NEMA17_FULLREVOLUTION ) * speedToMM) ; // speed in mm/s for drive wheel 
  ind_pos   = (((float)stepper1.currentPosition()/NEMA17_FULLREVOLUTION ) * speedToMM); // step position in mm for drive wheel
  
  if(!units)
  {
    ind_pos = ind_pos*mm_to_in; 
    ind_speed = ind_speed*mm_to_in; 
  }
  
  // set the direction of the stepper motor
  if (readMotorBackward)
  stepper1.setSpeed(-motorSpeed);
  else if (readMotorForward) 
  stepper1.setSpeed(motorSpeed);
  else
  stepper1.setSpeed(0);
  
  stepper1.runSpeed();
  //--------------------------------------------------------------------

  // SEQ4 Update LCD
  
  // Display Speed, Position and Direction after 100ms    
  if ((millis() - LCD_update ) > LCD_delay)
  {
    LCD_update = millis();
        
    lcd.setCursor ( 12, 1 );            // go to the 2nd row
    lcd.print(ind_speed,3);

    lcd.setCursor ( 12, 3 );            // go to the fourth row 
    lcd.print(ind_pos,5);
      
    if(updateLCDDirection)
    {      
      
      lcd.setCursor ( 12, 2 );                   
      if(currentMotorSetting==2)
        lcd.print(F("BCK"));        
      else if(currentMotorSetting==1)
        lcd.print(F("FWD"));        
      else 
        lcd.print(F("STP"));

      updateLCDDirection=false;
    }

    if(updateLCDunits)
    {
       lcd.setCursor(15,0);
       
       if(units)
       lcd.print(F("mm  "));
       else
       lcd.print(F("inch"));

       updateLCDunits=false;      
    }
     
  } 
  
}
/************************************************************/
