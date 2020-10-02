//This program runs a feedback thermocycle assay on two peltier arrays with the format A-B-A
// A = Temp1 and Time1
// B = Temp2 and Time2 

#include <DallasTemperature.h> //One wire temperature sensors 
#include <OneWire.h>
#include <LiquidCrystal.h>

#include <Wire.h>
#include <Adafruit_MLX90614.h> //IR temperature sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

unsigned long startTime; //store time at which the program is started after 1st click 
unsigned long currentTime; //store time that the program has been running after 1st click 

//thermalcycle variables
int tempPercent; //percent difference from sensor and target temp 
int absTempPercent;
int rateAdjust; //adjusted rate of PWM power based
int constRateAdjust; // contrained to 0-255 

//Rotary menu 
const int clickMenu1 = 0; //part A Temp
const int clickMenu2 = 1; //part A Time
const int clickMenu3 = 2; //part B Temp
const int clickMenu4 = 3; //part B Time
const int clickMenu5 = 4; //cancel 
const int menuLast = clickMenu5;
int progAbort = 0;

//Rotary variables for setting assay 
int partATemp = 25; //part A is the first and last period
int partBTemp = 25; //part B is the middle period 

int partAMin = 1; // Minutes
int partBMin = 2; 

unsigned long partAMilli;   // Milli seconds
unsigned long partBMilli;

//PWM array variables 
byte M1ArrayPower = 0; //Motor1 Array of Peltiers 
byte M2ArrayPower = 0;
char peltierPower[4];
bool cycle_end = false; //triggered when cycle ends 

#define ROT_PORT PORTC
#define ROT_PIN PINC
#define ROT_A _BV(1)
#define ROT_B _BV(2)
#define ROT_BTN _BV(3)
#define ROT_DELAY 110
#define ROT_BTN_DELAY 255

#define ROT_LED_R A4
#define ROT_LED_G A5
#define ROT_LED_B 2
#define ROT_LED_ON 0
#define ROT_LED_OFF 1

const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

#define URC10_MOTOR_1_DIR 4 // set motor for direction control
#define URC10_MOTOR_1_PWM 5 // set PWM for power control

#define URC10_MOTOR_2_DIR 7 // set motor for direction control
#define URC10_MOTOR_2_PWM 6 // set PWM for power control

#define COOL 1       // define direction for cooling effect 
#define HEAT 0       // define direction for heating effect 

OneWire  ds(14);  // on pin 10 (a 4.7K resistor is necessary)

void setup() { //////////////////////////////////////////////////////////////////////////////////////
  //Initialize serial and wait for port to open:
  Serial.begin(38400);//);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("12\t30\t\n"); //set temp range before dynamic 
  Serial.print(" ");

  mlx.begin(); // IR temperature sensor
  
  lcd.begin(20, 4);
  lcd.cursor();
  
  pinMode(14, OUTPUT);
  
  pinMode(URC10_MOTOR_1_DIR, OUTPUT);
  digitalWrite(URC10_MOTOR_1_DIR, HEAT);
  pinMode(URC10_MOTOR_1_PWM, OUTPUT);  

  pinMode(URC10_MOTOR_2_DIR, OUTPUT);
  digitalWrite(URC10_MOTOR_2_DIR, HEAT);
  pinMode(URC10_MOTOR_2_PWM, OUTPUT);  
  
  // rotory encoder 
  OCR2A = ROT_DELAY;
  TCCR2B = 0;
  TCCR2A = _BV(WGM20);
  TIFR2 |= _BV(OCF2A);
  TIMSK2 |= _BV(OCIE2A);

  ROT_PORT |= ROT_A | ROT_B;
  PCMSK1 |= ROT_A | ROT_B | ROT_BTN;
  PCIFR |= _BV(PCIF1);
  PCICR |= _BV(PCIE1);

  pinMode(ROT_LED_R, OUTPUT);
  pinMode(ROT_LED_G, OUTPUT);
  pinMode(ROT_LED_B, OUTPUT);
  digitalWrite(ROT_LED_G, ROT_LED_OFF);
}

//reading rotory encoder 
byte clamp255(int v) { 
  if (v < 0)
    return 0;
  else if (v > 255)
    return 255;
  else
    return v;
}

volatile int _b = 0;
volatile byte _lastRot = 0;
volatile char _sign = 0;

// multi click rotary option
int rotButton(){
  return _b % (menuLast + 1);
}

// Impliment rotary click with multi click menu
ISR(TIMER2_COMPA_vect) {
  TCCR2B = 0;

  if (_sign == 0)
    _b++;
  else {
    if (rotButton() == clickMenu1) 
      partATemp = clamp255(partATemp + _sign);
    else if (rotButton() == clickMenu2) 
      partAMin = abs(clamp255(partAMin + _sign));
    else if (rotButton() == clickMenu3) 
      partBTemp = clamp255(partBTemp + _sign);
    else if (rotButton() == clickMenu4) 
      partBMin = abs(clamp255(partBMin + _sign));
    else if (rotButton() == clickMenu5) 
      progAbort = _sign;
  }
}

ISR(PCINT1_vect) {
  byte _rot = ROT_PIN;
  if ((_lastRot ^ _rot) & ROT_A && (_rot & ROT_B) == 0) {
    if (_rot & ROT_A) {
      TCCR2B = 0;
    } else {
      _sign = -1;
      setDelay(ROT_DELAY);
    }
  }
  if ((_lastRot ^ _rot) & ROT_B && (_rot & ROT_A) == 0) {
    if (_rot & ROT_B) {
      TCCR2B = 0;
    } else {
      _sign = 1;
      setDelay(ROT_DELAY);
    }
  }
  if ((_lastRot ^ _rot) & ROT_BTN) {
    if (_rot & ROT_BTN) {
      TCCR2B = 0;
    } else {
      _sign = 0;
      setDelay(ROT_BTN_DELAY);
    }
  }

  _lastRot = _rot;
}

void setDelay(byte dly) {
  OCR2A = dly;
  TCNT2 = 1;
  TCCR2B = _BV(CS22) | _BV(CS20) | _BV(WGM22);
}

int nTempSensor = 0;

void loop(void) {//////////////////////////////////////////////////////////////////////////////////////
  if (millis() < 1000) {
    while (_b == 0){
      lcd.setCursor(0, 0); 
      lcd.print("Press to start");
      digitalWrite(ROT_LED_R, ROT_LED_OFF);
      digitalWrite(ROT_LED_B, ROT_LED_OFF);
      digitalWrite(ROT_LED_G, ROT_LED_ON);
      delay(250);
      digitalWrite(ROT_LED_G, ROT_LED_OFF);
      delay(250);
    }
    lcd.setCursor(0, 0);
    lcd.print("               "); 
    lcd.setCursor(0, 0);
    lcd.print("Peltier Go!");
    delay(1000);
    lcd.setCursor(0, 0); 
    lcd.print("            ");
    startTime = millis();
    _b = 0;   
  }
  
  //set LCD menu display 
  lcd.setCursor(0, 0); 
  lcd.print("A)");
  lcd.setCursor(2, 0); 
  lcd.print("t:");
  lcd.print(partATemp);
  lcd.setCursor(7, 0);
  lcd.print("m:");
  lcd.print(partAMin);
  
  lcd.setCursor(0, 1); 
  lcd.print("B)");
  lcd.setCursor(2, 1);
  lcd.print("t:");
  lcd.print(partBTemp);
  lcd.setCursor(7, 1);
  lcd.print("m:");
  lcd.print(partBMin);
  
  lcd.setCursor(19, 1); 
  lcd.print("*"); //menu point for canceling program

  if (rotButton() == clickMenu1) { // multi click rotary option
    digitalWrite(ROT_LED_R, ROT_LED_OFF);
    digitalWrite(ROT_LED_B, ROT_LED_OFF);
    digitalWrite(ROT_LED_G, ROT_LED_ON);
    lcd.setCursor(5, 0);
  } else if (rotButton() == clickMenu2) {
    digitalWrite(ROT_LED_R, ROT_LED_OFF);
    digitalWrite(ROT_LED_B, ROT_LED_ON);
    digitalWrite(ROT_LED_G, ROT_LED_OFF);
    lcd.setCursor(9, 0);
  } else if (rotButton() == clickMenu3) {
    digitalWrite(ROT_LED_R, ROT_LED_ON);
    digitalWrite(ROT_LED_B, ROT_LED_OFF);
    digitalWrite(ROT_LED_G, ROT_LED_ON);
    lcd.setCursor(5, 1);
  } else if (rotButton() == clickMenu4) {
    digitalWrite(ROT_LED_R, ROT_LED_OFF);
    digitalWrite(ROT_LED_B, ROT_LED_ON);
    digitalWrite(ROT_LED_G, ROT_LED_ON);
    lcd.setCursor(9, 1);
  } else if (rotButton() == clickMenu5) {
    digitalWrite(ROT_LED_R, ROT_LED_ON);
    digitalWrite(ROT_LED_B, ROT_LED_OFF);
    digitalWrite(ROT_LED_G, ROT_LED_OFF);
    lcd.setCursor(19, 1);
  }
  
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius_onewire;
  float celsius_IRambient;
  float celsius_IRobject;
  
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    Serial.println();////
    ds.reset_search();
    delay(250);
    nTempSensor = 0;
    return;
  }
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  
   for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
   }
  
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }


  celsius_onewire = (float)raw / 16.0;
  Serial.print(celsius_onewire);
  Serial.print("\t");
  lcd.setCursor((nTempSensor) * 4, 2);
  lcd.print(":");
  lcd.print(celsius_onewire,0);
  nTempSensor++;

  celsius_IRambient = mlx.readAmbientTempC();
  celsius_IRobject = mlx.readObjectTempC();
  lcd.setCursor(0, 3);
  lcd.print("Object");
  lcd.print(":");
  lcd.print(celsius_IRobject,1);

  partAMilli = partAMin * 60000;
  partBMilli = partBMin * 60000;  
  
  if (progAbort != 0) { //Determine if program is aborted 
    tempPercent = 0;
    lcd.setCursor(11, 2);
    lcd.print("cAncElLeD");
    cycle_end = true
    digitalWrite(ROT_LED_R, ROT_LED_OFF);
    digitalWrite(ROT_LED_B, ROT_LED_OFF);
    delay(250);
    
  } else { //determine the amount and direction of off target based on the time the assay is running 
    currentTime = millis() - startTime; 
    if(currentTime < partAMilli) {
      tempPercent = ((partATemp - celsius_IRobject)/partATemp) * 100;
      lcd.setCursor(11, 1);
      lcd.print("trgt:");
      lcd.print(partATemp);
    } else if((currentTime > partAMilli) && (currentTime < (partBMilli + partAMilli))) {
      tempPercent = ((partBTemp - celsius_IRobject)/partBTemp) * 100;
      lcd.setCursor(11, 1);
      lcd.print("trgt:");
      lcd.print(partBTemp);
    } else if((currentTime > partAMilli + partBMilli) && (currentTime < (partBMilli + (partAMilli * 2)))) {
      //tempPercent = ((partATemp - celsius_IRobject)/partATemp) * 100;
      tempPercent = ((partATemp - celsius_IRobject)/partATemp) * 100;
      lcd.setCursor(11, 1);
      lcd.print("trgt:");
      lcd.print(partATemp);
    } else {
      tempPercent = 0;
      lcd.setCursor(11, 1);
      lcd.print("End    ");
      cycle_end = true;
    }
  }
  
  lcd.setCursor(11, 0);
  lcd.print("Cnt");
  lcd.print(currentTime / (1000)); //report seconds of program run time 
  
  //set Peltier direction based on direction off target 
  //Calculate rate adjusted power by polynomial scaling 
  absTempPercent = abs(tempPercent);
  if (tempPercent > 0) {
    digitalWrite(URC10_MOTOR_1_DIR, HEAT);
    digitalWrite(URC10_MOTOR_2_DIR, HEAT);
    lcd.setCursor(19, 0);
    lcd.print("+"); 
    rateAdjust = (0.38 * (absTempPercent * absTempPercent)) + (0.21 * absTempPercent) - 10; // pow() works creates float variables  
  } else if (tempPercent <= 0) {
    digitalWrite(URC10_MOTOR_1_DIR, COOL);
    digitalWrite(URC10_MOTOR_2_DIR, COOL); 
    lcd.setCursor(19, 0);
    lcd.print("-");
    rateAdjust = (-0.75 * (absTempPercent * absTempPercent)) + (28 * absTempPercent) + 20; // pow() works creates float variables         
  } 

  if(rateAdjust > 255) { //constrain scaling to 255 
    constRateAdjust = 255; 
  } else {
    constRateAdjust = rateAdjust;
  } 

  if(cycle_end == true){ // keeps rateadjust intercept from running peliters constantly when the cycles are over 
    constRateAdjust = 0;
  }

  //set PWM byte from poynomial scaling 
  M1ArrayPower = (byte) constRateAdjust;
  M2ArrayPower = (byte) constRateAdjust;
  Serial.print(M1ArrayPower);
  Serial.print("  ");
  Serial.print(currentTime);
  
  //send PWM value to Peltiers 
  analogWrite(URC10_MOTOR_1_PWM, M1ArrayPower);
  analogWrite(URC10_MOTOR_2_PWM, M2ArrayPower);
}
