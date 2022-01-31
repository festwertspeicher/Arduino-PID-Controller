//-------------------------------------------------------------------
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
// Norman Heck - for the lulz
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

/* TODO IMPROVEMENTS
 * get higher resolution temperature readings
 * add socket for different sensor probes (e.g. PT1000) and sensor selection menu or auto deterction
 * add sensor calibration mode
 * add memory function to store and load the settings for different environment this controller is used in (P,I,D, direction, sensor, sensor calibration)
 * add Calibration Mode for temperature sensor
 * add Error message if autotune fails
 * normalize noisy temperature readings
 * make digits of setpoint and sensor reading beneath one another with trailing zeros
 * define special characters for the lcd as array
 * match speed of controll loop to frequency of main power, use optocoupler to detect zero crossing
 */

// ************************************************
// Libraries
// ************************************************

#include <Arduino.h>

// Libraries for PID Controll
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the LCD
#include <LiquidCrystal_I2C.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>  // #include <Wire.h>
#include <DallasTemperature.h>

// Library for the Encoder http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
#include <RotaryEncoder.h>

// For saving and retrieving settings on the internal memory
#include <EEPROM.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay Pin
#define RelayPin 10

// Encoder Pin
#define PIN_ENCODER1 2
#define PIN_ENCODER2 3
#define PIN_BUTTON 4 // pushbutton pin

// One-Wire Temperature Sensor
// (Use GPIO pins for power/ground to simplify the wiring)
#define ONE_WIRE_BUS 8

// ************************************************
// Objects, Variables
// ************************************************

//Rotary Encoder
RotaryEncoder *encoder = nullptr; // A pointer to the dynamic created rotary encoder instance. This will be done in setup()

int encoderRepeatsMultiplier = 0;
bool encoderDirection[] = {false, false}; // array that holds the last two turns of the encoder, to check if there was a direction change. anti-clockwise => false; clockwise => true
bool encoderClick[] = {true, true}; // array that holds the last two klick States of the Encoder Button, to check if there was a change. not clicked => true

// Periodically Logs to Serial
const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2); //LCD Object, HEX-Adresse 0x27 (see Hardware Jumer), 16 Chars, 2 Lines

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// custom characters
#if defined(ARDUINO) && ARDUINO >= 100
  #define printByte(args)  write(args);
#else
  #define printByte(args)  print(args,BYTE);
#endif
uint8_t degree[8]  = {0x6,0x9,0x9,0x6,0x0,0x0,0x0,0x0};
uint8_t heart[8] = {0x0,0xA,0x1F,0x1F,0xE,0x4,0x0,0x0};
uint8_t arrow[8] = {0x10,0x18,0x1C,0x1E,0x1C,0x18,0x10,0x00};

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;  // Target Temperature
double Input; // Measured Temperature
double Output;

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// States for state machine & Navigation
// ************************************************

static int navigationEncoderValue = 0;

enum operatingState { SPLASH, RUN, NAV, SETP, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = SPLASH;


// ************************************************
// Setup
// ************************************************

void setup() {
  Serial.begin(57600);
  initLcd();
  initEncoder();
  initSensors();
  initPID();
}

// ************************************************
// Interrupts
// ************************************************

// This interrupt routine will be called on any change of one of the input signals coming from the encoder
void checkPosition() {
  encoder->tick(); // just call tick() to check the state.
}

SIGNAL(TIMER2_OVF_vect) 
{
  if (opState != RUN){
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else {
    DriveOutput();
  }
}

// ************************************************
// Loop
// ************************************************

void loop() {
  if(Input != DEVICE_DISCONNECTED_C) {   
    switch(opState) {
      case SPLASH:
        splashScreen();
        break;
      case RUN:
        runControlLoop();
        break;
      case NAV:
        navigation();
        break;
      case SETP:
        setpoint();
        break;
      case TUNE_P:
        tuneP();
        break;
      case TUNE_I:
        tuneI();
        break;
      case TUNE_D:
        tuneD();
        break;
      case AUTO:
        autotunePID();
        break;
    }
  }
  else {
    PrintSensorError();
  }
}


// ************************************************
// Controll Loop (Called by ISR every 15ms to drive the output)
// ************************************************

void DriveOutput() {  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize) { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime))) {
     digitalWrite(RelayPin,HIGH);
  }
  else {
     digitalWrite(RelayPin,LOW);
  }
}


// ************************************************
// Execute the control loop
// ************************************************

void DoControl()
{
  // Read the input:
  if(sensors.isConversionComplete())
  {
    Input = sensors.getTempC(tempSensor);
    sensors.requestTemperatures(); // start an asynchronous temperature reading for the next loop
  }
  
  if(tuning) { // run the auto-tuner
     if(aTune.Runtime()) { // returns 'true' when done
        FinishAutoTune();
     }
  }
  else { // Execute control algorithm
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// States
// ************************************************

void changeStateIfClicked(operatingState state) {
  if(getEncoderClick() == true) {
    lcd.clear();
    opState = state;
  }
}

void splashScreen() {
  lcd.setCursor(0, 0); // First Char, First Row
  lcd.print("Klabautermann");
  lcd.setCursor(0, 1); // First Char, Second Row
  lcd.print("PID-Controller ");
  lcd.printByte(1);
  delay(2000); // Splash Screen
  lcd.clear();
  opState = NAV;
}


void runControlLoop() {
  // print Setpoint on the first line of the lcd
  lcd.print(F("Target: "));
  lcd.print(Setpoint);
  lcd.printByte(0);
  lcd.print("C");
  
  //SaveParameters();
  //myPID.SetTunings(Kp,Ki,Kd);
  
  while(true) {    
    DoControl();
  
    // print Sensor Value on the first line of the lcd
    lcd.setCursor(0,1);
    lcd.print(F("Actual: "));
    lcd.print(Input);
    lcd.printByte(0);
    lcd.print("C");

    lcd.setCursor(0,0);
    
    logToSerial();

    if(getEncoderClick() == true) {
      lcd.clear();
      opState = NAV;
      return;
    }

    delay(100);
  }
}

// navigation (initial state)
void navigation() {
  digitalWrite(RelayPin, LOW); // make sure Relay is off
  myPID.SetMode(MANUAL); // ... and not updating automatically
  
  int newPos = getEncoderRotation();
  if(newPos != 0) { // if rotation is detected, update navigationEncoderValue and clear screen
    navigationEncoderValue += newPos;
    lcd.clear();
  }

  switch (navigationEncoderValue) {
    // RUN, NAV, SETP, TUNE_P, TUNE_I, TUNE_D, AUTO
    case 0:
      lcd.clear();
      navigationEncoderValue = 1;
      break;
      
    case 1:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.printByte(2); // arrow
      lcd.print("Run ControlLoop");
      lcd.setCursor(0, 1); // First Char, Second Row
      lcd.print(" Setpoint");

      
      //transition to RUN state
      if(getEncoderClick() == true) {
         sensors.requestTemperatures(); // Start an asynchronous temperature reading

         //turn the PID on
         myPID.SetMode(AUTOMATIC);
         windowStartTime = millis();
         opState = RUN; // start control
        
        lcd.clear();
        opState = RUN;
      }
      
      break;
      
    case 2:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.print(" Run ControlLoop");
      lcd.setCursor(0, 1);
      lcd.printByte(2); // arrow
      lcd.print("Setpoint");

      changeStateIfClicked(SETP);
      break;
      
    case 3:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.printByte(2); // arrow
      lcd.print("Autotune PID");
      lcd.setCursor(0, 1);
      lcd.print(" Tune P");

      changeStateIfClicked(AUTO);
      break;
      
    case 4:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.print(" Autotune PID");
      lcd.setCursor(0, 1);
      lcd.printByte(2); // arrow
      lcd.print("Tune P");
      
      changeStateIfClicked(TUNE_P);
      break;
      
    case 5:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.printByte(2); // arrow
      lcd.print("Tune I");
      lcd.setCursor(0, 1);
      lcd.print(" Tune D");
      
      changeStateIfClicked(TUNE_I);
      break;
      
    case 6:
      lcd.setCursor(0, 0); // First Char, First Row
      lcd.print(" Tune I");
      lcd.setCursor(0, 1);
      lcd.printByte(2); // arrow
      lcd.print("Tune D");

      changeStateIfClicked(TUNE_D);
      break;
      
    case 7:
      navigationEncoderValue = 6;
      break;
  }
}

void setpoint() {
  Setpoint += (int)getEncoderRotationAccelerated(2, 50, 20, 4) / 10; // tenth of a degree
  
  lcd.setCursor(0, 0); // First Char, First Row
  lcd.print("Setpoint:");

  lcd.setCursor(0, 1); // First Char, Second Row
  lcd.print(Setpoint);
  lcd.printByte(0);
  lcd.print("C");

  changeStateIfClicked(NAV);
}

void tuneP() {
  Kp += (int)getEncoderRotationAccelerated(2, 50, 20, 4);
  
  lcd.setCursor(0, 0); // First Char, First Row
  lcd.print(F("Tune Proportion:"));
  lcd.setCursor(0, 1); // First Char, First Row
  lcd.print(Kp);

  //transition to RUN state
  if(getEncoderClick() == true) {
    SaveParameters();
    myPID.SetTunings(Kp,Ki,Kd);
    
    lcd.clear();
    opState = NAV;
  }
}

void tuneI() {
  Ki += getEncoderRotationAccelerated(2, 50, 20, 4) / 100; // hundredth
  
  lcd.setCursor(0, 0); // First Char, First Row
  lcd.print(F("Tune Integral:"));
  lcd.setCursor(0, 1); // First Char, First Row
  lcd.print(Ki);

  //transition to RUN state
  if(getEncoderClick() == true) {
    SaveParameters();
    myPID.SetTunings(Kp,Ki,Kd);
    
    lcd.clear();
    opState = NAV;
  }
}

void tuneD() {
  Kd += getEncoderRotationAccelerated(2, 50, 20, 4) / 100; // hundredth
  
  lcd.setCursor(0, 0); // First Char, First Row
  lcd.print(F("Tune Derivative:"));
  lcd.setCursor(0, 1); // First Char, First Row
  lcd.print(Kd);

  //transition to RUN state
  if(getEncoderClick() == true) {
    SaveParameters();
    myPID.SetTunings(Kp,Ki,Kd);
    
    lcd.clear();
    opState = NAV;
  }  
}

void autotunePID() {
  // print Setpoint on the first line of the lcd
  lcd.print(F("Autotuning"));
  lcd.setCursor(0,1);
  lcd.print(F("Please wait..."));

  StartAutoTune();
  
  while(true) {
    DoControl();
      
    logToSerial();

    while(tuning == false) {
      //Autotune Finished
      digitalWrite(RelayPin, LOW);  // make sure Relay is off
      myPID.SetMode(MANUAL); //No Automatic updates.
      
      lcd.clear();
      lcd.print(F("Done Autotuning."));
      lcd.print(F("Press Button"));
      
      changeStateIfClicked(NAV);
    }

    delay(100);
  }  
}

// ************************************************
// Inits
// ************************************************

void initLcd() {
  lcd.init();
  lcd.backlight(); //lcd.noBacklight();

  // create symbols from the binary
  lcd.createChar(0, degree);
  lcd.createChar(1, heart);
  lcd.createChar(2, arrow);
  
  lcd.home();
}

void initEncoder() {
  // encoder Object:
  // use FOUR3 mode when PIN_ENCODER1, PIN_ENCODER2 signals are always HIGH in latch position.
  // use FOUR0 mode when PIN_ENCODER1, PIN_ENCODER2 signals are always LOW in latch position.
  // use TWO03 mode when PIN_ENCODER1, PIN_ENCODER2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_ENCODER1, PIN_ENCODER2, RotaryEncoder::LatchMode::TWO03);

  // Pullup Clock and Data Pin (Common Ground)
  pinMode(PIN_ENCODER1,INPUT_PULLUP);
  pinMode(PIN_ENCODER2,INPUT_PULLUP);
  
  // register interrupt routine for encoder
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER2), checkPosition, CHANGE);
  pinMode(PIN_BUTTON, INPUT);
}

void initSensors() {
  sensors.begin();
  while(!sensors.getAddress(tempSensor, 0)) {
    PrintSensorError();
  }
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);
}

void initPID() {
  digitalWrite(RelayPin, LOW);  // make sure Relay is off
  
  LoadParameters();
  myPID.SetTunings(Kp,Ki,Kd);

  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);

  // Set it to "Manual" (whatever that means). No Automatic updates.
  myPID.SetMode(MANUAL);

  // Run timer2 interrupt every 15 ms 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= 1<<TOIE2;
}

// ************************************************
// Errors
// ************************************************

void PrintSensorError() {
  lcd.clear();
  lcd.print("Sensor Error");
  delay(10);
}

// ************************************************
// Encoder
// ************************************************

int getEncoderRotation() {
  encoder->tick(); // check the encoder state
  int newPos = encoder->getPosition(); // get encoder Position

  if(newPos != 0) {
    //save new direction of rotation in array, move the last direction in the array
    encoderDirection[0] = encoderDirection[1];
    if(newPos > 0) {
      encoderDirection[1] = true;
    }
    else if(newPos < 0) {
      encoderDirection[1] = false;
    }

    encoder->setPosition(0); // reset encoder position
    return newPos;
  }
  
  return 0;
}

bool getEncoderClick() {
  encoderClick[0] = encoderClick[1];
  encoderClick[1] = digitalRead(PIN_BUTTON);
  
  if(encoderClick[1] == true && encoderClick[0] == false) { // if buttons last state was not klicked and is now clicked
    // clicked
    return true;
  }
  return false;
}

/**
 * Sum numbers in a vector.
 *
 * @param float maxAcceleration    maximum Acceleration.
 * @param longCutoff               Time Between Encoder Ticks for Normal Acceleration (factor 1)
 * @param float shortCutoff        Time Between Encoder Ticks for Maximum Acceleration (factor maxAcceleration)
 * 
 */
float getEncoderRotationAccelerated(float maxAcceleration, float longCutoff, float shortCutoff, int maxEncoderRepeatsMultiplier) {
  int newPos = getEncoderRotation();
  
  if(newPos != 0) {
    //Serial.print("pos: ");
    //Serial.println(newPos);

    unsigned long ms = encoder->getMillisBetweenRotations();

    if (ms < longCutoff) {
      // limit to maximum acceleration
      if (ms < shortCutoff) {
        ms = shortCutoff;
      }

      encoderRepeatsMultiplier++; // add one repeat

      if(encoderDirection[0] != encoderDirection[1]) { // reset multiplier if direction changes
        encoderRepeatsMultiplier = 0;
      }
      
      if(encoderRepeatsMultiplier >= maxEncoderRepeatsMultiplier) { // don't let repeats multiplier get bigger than max setting
        encoderRepeatsMultiplier = maxEncoderRepeatsMultiplier;
      }

      float pitch = -1 / (longCutoff - shortCutoff); // pitch 
      float b = -1 * pitch * longCutoff;
      float speedMultiplier = pitch * ms + b;
      //Serial.print("speedMultiplier: ");
      //Serial.println(speedMultiplier);
      //Serial.print("encoderRepeatsMultiplier: ");
      //Serial.println(encoderRepeatsMultiplier + 1);

      float acceleratedValue = newPos + speedMultiplier * (encoderRepeatsMultiplier + 1) * maxAcceleration * newPos; // detected tick + speed multiplier * how many time the encoder was repeated accelerated * max Acceleration * detected tick
      return acceleratedValue;
    }
    else {
      encoderRepeatsMultiplier = 0; // reset repeats multiplier
    }
  }
  return newPos;
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune() {
   // Remember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune() {
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters() {
  if (Setpoint != EEPROM_readDouble(SpAddress)) {
    EEPROM_writeDouble(SpAddress, Setpoint);
  }
  if (Kp != EEPROM_readDouble(KpAddress)) {
    EEPROM_writeDouble(KpAddress, Kp);
  }
  if (Ki != EEPROM_readDouble(KiAddress)) {
    EEPROM_writeDouble(KiAddress, Ki);
  }
  if (Kd != EEPROM_readDouble(KdAddress)) {
    EEPROM_writeDouble(KdAddress, Kd);
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters() {
  // Load from EEPROM
  Setpoint = EEPROM_readDouble(SpAddress);
  Kp = EEPROM_readDouble(KpAddress);
  Ki = EEPROM_readDouble(KiAddress);
  Kd = EEPROM_readDouble(KdAddress);
   
  // Use defaults if EEPROM values are invalid
  if (isnan(Setpoint)) {
    Setpoint = 60;
  }
  if (isnan(Kp)) {
    Kp = 850;
  }
  if (isnan(Ki)) {
    Ki = 0.5;
  }
  if (isnan(Kd)) {
    Kd = 0.1;
  }  
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************

void EEPROM_writeDouble(int address, double value) {
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address++, *p++);
  }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************

double EEPROM_readDouble(int address) {
  double value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

// ************************************************
// Log CSV to Serial
// ************************************************

void logToSerial() {
    if (millis() - lastLogTime >= logInterval) {
      lastLogTime = millis();
      Serial.println("Autotune");
      Serial.print(Input);
      Serial.print(",");
      Serial.println(Output);
    }
}
