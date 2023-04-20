//**************************************************************//
//  Name    : POV1                                              //
//  Author  : Isaac Assegai <isaac@chicosystems.com             //
//  Date    : 4/19/23                                           //
//  Version : 0.0.1                                             //
//  Notes   : A Persistance of Vision Display                   //
//****************************************************************
//Pin connected to ST_CP of 74HC595
int latchPin = 8;
//Pin connected to SH_CP of 74HC595
int clockPin = 12;
////Pin connected to DS of 74HC595
int dataPin = 11;
//holders for information you're going to pass to shifting function
byte dataTOP;
byte dataBOTTOM;
byte dataArrayRED[8];

short dataArray[17];

#include <Wire.h>
#include <math.h>

#define SCALE 2  // accel full-scale, should be 2, 4, or 8

/* LSM303 Address definitions */
#define LSM303_MAG  0x1E  // assuming SA0 grounded
#define LSM303_ACC  0x18  // assuming SA0 grounded

#define X 0
#define Y 1
#define Z 2

/* LSM303 Register definitions */
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define HP_FILTER_RESET_A 0x25
#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define INT1_CFG_A 0x30
#define INT1_SOURCE_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define CRA_REG_M 0x00
#define CRB_REG_M 0x01
#define MR_REG_M 0x02
#define OUT_X_H_M 0x03
#define OUT_X_L_M 0x04
#define OUT_Y_H_M 0x05
#define OUT_Y_L_M 0x06
#define OUT_Z_H_M 0x07
#define OUT_Z_L_M 0x08
#define SR_REG_M 0x09
#define IRA_REG_M 0x0A
#define IRB_REG_M 0x0B
#define IRC_REG_M 0x0C

int text_array[475] = {
0x00,0x00,0x00,0x00,0x00,/*space*/ // is 32 in ASCII
0x00,0xF6,0xF6,0x00,0x00,/*!*/
0x00,0xE0,0x00,0xE0,0x00,/*"*/
0x28,0xFE,0x28,0xFE,0x28,/*#*/
0x00,0x64,0xD6,0x54,0x08,/*$*/
0xC2,0xCC,0x10,0x26,0xC6,/*%*/
0x4C,0xB2,0x92,0x6C,0x0A,/*&*/
0x00,0x00,0xE0,0x00,0x00,/*'*/
0x00,0x38,0x44,0x82,0x00,/*(*/
0x00,0x82,0x44,0x38,0x00,/*)*/
0x88,0x50,0xF8,0x50,0x88,/***/
0x08,0x08,0x3E,0x08,0x08,/*+*/
0x00,0x00,0x05,0x06,0x00,/*,*/
0x08,0x08,0x08,0x08,0x08,/*-*/
0x00,0x00,0x06,0x06,0x00,/*.*/
0x02,0x0C,0x10,0x60,0x80,/*/*/
0x7C,0x8A,0x92,0xA2,0x7C,/*0*/
0x00,0x42,0xFE,0x02,0x00,/*1*/
0x42,0x86,0x8A,0x92,0x62,/*2*/
0x44,0x82,0x92,0x92,0x6C,/*3*/
0x10,0x30,0x50,0xFE,0x10,/*4*/
0xE4,0xA2,0xA2,0xA2,0x9C,/*5*/
0x3C,0x52,0x92,0x92,0x0C,/*6*/
0x80,0x86,0x98,0xE0,0x80,/*7*/
0x6C,0x92,0x92,0x92,0x6C,/*8*/
0x60,0x92,0x92,0x94,0x78,/*9*/
0x00,0x00,0x36,0x36,0x00,/*:*/
0x00,0x00,0x35,0x36,0x00,/*;*/
0x10,0x28,0x44,0x82,0x00,/*<*/
0x28,0x28,0x28,0x28,0x28,/*=*/
0x00,0x82,0x44,0x28,0x10,/*>*/
0x40,0x80,0x8A,0x90,0x60,/*?*/
0x7C,0x82,0xBA,0xBA,0x62,/*@*/
0x3E,0x48,0x88,0x48,0x3E,/*A*/
0xFE,0x92,0x92,0x92,0x6C,/*B*/
0x7C,0x82,0x82,0x82,0x44,/*C*/
0xFE,0x82,0x82,0x82,0x7C,/*D*/
0xFE,0x92,0x92,0x92,0x82,/*E*/
0xFE,0x90,0x90,0x90,0x80,/*F*/
0x7C,0x82,0x82,0x8A,0x4E,/*G*/
0xFE,0x10,0x10,0x10,0xFE,/*H*/
0x82,0x82,0xFE,0x82,0x82,/*I*/
0x84,0x82,0xFC,0x80,0x80,/*J*/
0xFE,0x10,0x28,0x44,0x82,/*K*/
0xFE,0x02,0x02,0x02,0x02,/*L*/
0xFE,0x40,0x20,0x40,0xFE,/*M*/
0xFE,0x60,0x10,0x0C,0xFE,/*N*/
0x7C,0x82,0x82,0x82,0x7C,/*O*/
0xFE,0x90,0x90,0x90,0x60,/*P*/
0x7C,0x82,0x82,0x86,0x7E,/*Q*/
0xFE,0x90,0x98,0x94,0x62,/*R*/
0x64,0x92,0x92,0x92,0x4C,/*S*/
0x80,0x80,0xFE,0x80,0x80,/*T*/
0xFC,0x02,0x02,0x02,0xFC,/*U*/
0xF8,0x04,0x02,0x04,0xF8,/*V*/
0xFC,0x02,0x0C,0x02,0xFC,/*W*/
0xC6,0x28,0x10,0x28,0xC6,/*X*/
0xC0,0x20,0x1E,0x20,0xC0,/*Y*/
0x86,0x8A,0x92,0xA2,0xC2,/*Z*/
0x00,0x00,0xFE,0x82,0x00,/*[*/
0x00,0x00,0x00,0x00,0x00,/*this should be / */
0x80,0x60,0x10,0x0C,0x02,/*]*/
0x20,0x40,0x80,0x40,0x20,/*^*/
0x01,0x01,0x01,0x01,0x01,/*_*/
0x80,0x40,0x20,0x00,0x00,/*`*/
0x04,0x2A,0x2A,0x2A,0x1E,/*a*/
0xFE,0x12,0x22,0x22,0x1C,/*b*/
0x1C,0x22,0x22,0x22,0x14,/*c*/
0x1C,0x22,0x22,0x12,0xFE,/*d*/
0x1C,0x2A,0x2A,0x2A,0x18,/*e*/
0x10,0x7E,0x90,0x80,0x40,/*f*/
0x18,0x25,0x25,0x25,0x1E,/*g*/
0xFE,0x10,0x10,0x10,0x0E,/*h*/
0x00,0x12,0x5E,0x02,0x00,/*i*/
0x02,0x01,0x01,0x11,0x5E,/*j*/
0xFE,0x08,0x08,0x14,0x22,/*k*/
0x00,0x82,0xFE,0x02,0x00,/*l*/
0x3E,0x20,0x1C,0x20,0x1E,/*m*/
0x3E,0x20,0x20,0x20,0x1E,/*n*/
0x1C,0x22,0x22,0x22,0x1C,/*o*/
0x3F,0x24,0x24,0x24,0x18,/*p*/
0x18,0x24,0x24,0x3F,0x01,/*q*/
0x3E,0x10,0x20,0x20,0x10,/*r*/
0x12,0x2A,0x2A,0x2A,0x04,/*s*/
0x00,0x10,0x3C,0x12,0x04,/*t*/
0x3C,0x02,0x02,0x02,0x3E,/*u*/
0x30,0x0C,0x02,0x0C,0x30,/*v*/
0x38,0x06,0x18,0x06,0x38,/*w*/
0x22,0x14,0x08,0x14,0x22,/*x*/
0x38,0x05,0x05,0x05,0x3E,/*y*/
0x22,0x26,0x2A,0x32,0x22,/*z*/
0x00,0x10,0x6C,0x82,0x82,/*{*/
//0x00,0x00,0xFF,0x00,0x00,/*|*/
0x04,0x02,0xFF,0x02,0x04,/*|, arrow*/
0x82,0x82,0x6C,0x10,0x00,/*}*/
0x08,0x10,0x18,0x08,0x10/*~*/
};

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here

void setup() {
  //set pins to output because they are addressed in the main loop
  pinMode(latchPin, OUTPUT);
  Serial.begin(9600);

  Wire.begin();  // Start up I2C, required for LSM303 communication
  initLSM303(SCALE);  // Initialize the LSM303, using a SCALE full-scale range

  //Arduino doesn't seem to have a way to write binary straight into the code
  //so these values are in HEX.  Decimal would have been fine, too.
  dataArrayRED[0] = 0xFF; //11111111
  dataArrayRED[1] = 0xFE; //11111110
  dataArrayRED[2] = 0xFC; //11111100
  dataArrayRED[3] = 0xF8; //11111000
  dataArrayRED[4] = 0xF0; //11110000
  dataArrayRED[5] = 0xE0; //11100000
  dataArrayRED[6] = 0xC0; //11000000
  dataArrayRED[7] = 0x80; //10000000
  dataArrayRED[8] = 0x00; //00000000

  dataArray[0] =  0xFFFF;  //1111 1111 1111 1111
  dataArray[1] =  0xFFFE;  //1111 1111 1111 1110
  dataArray[2] =  0xFFFC;  //1111 1111 1111 1100
  dataArray[3] =  0xFFF8;  //1111 1111 1111 1000
  dataArray[4] =  0xFFF0;  //1111 1111 1111 0000
  dataArray[5] =  0xFFE0;  //1111 1111 1110 0000
  dataArray[6] =  0xFFC0;  //1111 1111 1100 0000
  dataArray[7] =  0xFF80;  //1111 1111 1000 0000
  dataArray[8] =  0xFF00;  //1111 1111 0000 0000
  dataArray[9] =  0xFE00;  //1111 1110 0000 0000
  dataArray[10] = 0xFC00;  //1111 1100 0000 0000
  dataArray[11] = 0xF800;  //1111 1000 0000 0000
  dataArray[12] = 0xF000;  //1111 0000 0000 0000
  dataArray[13] = 0xE000;  //1110 0000 0000 0000
  dataArray[14] = 0xC000;  //1100 0000 0000 0000
  dataArray[15] = 0x8000;  //1000 0000 0000 0000
  dataArray[16] = 0x0000;  //1000 0000 0000 0000

//11111111
//11111111
//11
//11111111
//11111111
//11
//11111111
//11111111


  //function that blinks all the LEDs
  //gets passed the number of blinks and the pause time
  blinkAll_2Bytes(3,500);
}


/**
 * Displays the character given on the leds
 */
void display(char c, bool isForward){

  

  // Setup our output data
  byte outputData = 0x00;

  // Translate from the input character to the index of our text_array
	int i = (c - 32)*5;

  if(isForward){
    // Loop through the characters invidual scan lines.
    for ( int temp = i + 5; temp >= i; temp-- ) {

      // Set the output data
      outputData= text_array[temp]; 

      dataBOTTOM = 0x00;
      dataTOP = outputData;

      //ground latchPin and hold low for as long as you are transmitting
      digitalWrite(latchPin, 0);
      //move 'em out
      shiftOut(dataPin, clockPin, dataBOTTOM);
      shiftOut(dataPin, clockPin, dataTOP);
      //return the latch pin high to signal chip that it
      //no longer needs to listen for information
      digitalWrite(latchPin, 1);

      // Delay for 500 microseconds
      delayMicroseconds(300);
      dataBOTTOM = 0x00;
      dataTOP = 0x00;

      //ground latchPin and hold low for as long as you are transmitting
      digitalWrite(latchPin, 0);
      //move 'em out
      shiftOut(dataPin, clockPin, dataBOTTOM);
      shiftOut(dataPin, clockPin, dataTOP);
      //return the latch pin high to signal chip that it
      //no longer needs to listen for information
      digitalWrite(latchPin, 1);
      delay(2);
    }
  }else{
    // Loop through the characters invidual scan lines.
    for ( int temp = i; temp < i + 5; temp++ ) {

      // Set the output data
      outputData= text_array[temp]; 

      dataBOTTOM = 0x00;
      dataTOP = outputData;

      //ground latchPin and hold low for as long as you are transmitting
      digitalWrite(latchPin, 0);
      //move 'em out
      shiftOut(dataPin, clockPin, dataBOTTOM);
      shiftOut(dataPin, clockPin, dataTOP);
      //return the latch pin high to signal chip that it
      //no longer needs to listen for information
      digitalWrite(latchPin, 1);

      // Delay for 500 microseconds
      delayMicroseconds(300);
      dataBOTTOM = 0x00;
      dataTOP = 0x00;

      //ground latchPin and hold low for as long as you are transmitting
      digitalWrite(latchPin, 0);
      //move 'em out
      shiftOut(dataPin, clockPin, dataBOTTOM);
      shiftOut(dataPin, clockPin, dataTOP);
      //return the latch pin high to signal chip that it
      //no longer needs to listen for information
      digitalWrite(latchPin, 1);
      delay(2);
    }
  }
  

	dataBOTTOM = 0x00;
  dataTOP = 0x00;

  //ground latchPin and hold low for as long as you are transmitting
  digitalWrite(latchPin, 0);
  //move 'em out
  shiftOut(dataPin, clockPin, dataBOTTOM);
  shiftOut(dataPin, clockPin, dataTOP);
  //return the latch pin high to signal chip that it
  //no longer needs to listen for information
  digitalWrite(latchPin, 1); 
  delay(2);
}

void loop() {
  getLSM303_accel(accel);  // get the acceleration values and store them in the accel array
  while(!(LSM303_read(SR_REG_M) & 0x01))
    ;  // wait for the magnetometer readings to be ready
  getLSM303_mag(mag);  // get the magnetometer values, store them in mag
  //printValues(mag, accel);  // print the raw accel and mag values, good debugging
  
  for (int i=0; i<3; i++){
    realAccel[i] = accel[i] / pow(2, 15) * SCALE;  // calculate real acceleration values, in units of g
  }

  bool isForward = true;


  if(realAccel[1] < 0){
    isForward = false;
  }


  if(isForward){
    display('I', isForward); // Paaaaraaammm...
    display('S', isForward);
    display('A', isForward);
    display('A', isForward);
    display('C', isForward);
  }else{
    display('C', isForward); // Paaaaraaammm...
    display('A', isForward);
    display('A', isForward);
    display('S', isForward);
    display('I', isForward);
  }
	

  delay(60); // Wait a moment before starting
  


  /*for(int i = 0; i < 17; i++){
    tiltHeading = getTiltHeading(mag, realAccel);

    Serial.print("TiltHeading: ");

    Serial.println(tiltHeading, 3);  // see how awesome tilt compensation is?!

    dataBOTTOM = dataArray[i]>>8;

    dataTOP = dataArray[i];

    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //move 'em out
    shiftOut(dataPin, clockPin, dataBOTTOM);
    shiftOut(dataPin, clockPin, dataTOP);
    //return the latch pin high to signal chip that it
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    delay(300);   
    

  }*/ 
  /*for (int j = 0; j < 10; j++) {
    //load the light sequence you want from array
    dataRED = dataArrayRED[0];
    dataGREEN = dataArrayRED[0];
    //ground latchPin and hold low for as long as you are transmitting
    digitalWrite(latchPin, 0);
    //move 'em out
    shiftOut(dataPin, clockPin, dataGREEN);
    shiftOut(dataPin, clockPin, dataRED);
    //return the latch pin high to signal chip that it
    //no longer needs to listen for information
    digitalWrite(latchPin, 1);
    delay(300);
  }
  */
}

// the heart of the program
void shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first,
  //on the rising edge of the clock,
  //clock idles low
  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);
  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);
  //for each bit in the byte myDataOut&#xFFFD;
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights.
  for (i=7; i>=0; i--)  {
  digitalWrite(myClockPin, 0);
  //if the value passed to myDataOut and a bitmask result
  // true then... so if we are at i=6 and our value is
  // %11010100 it would the code compares it to %01000000
  // and proceeds to set pinState to 1.
  if ( myDataOut & (1<<i) ) {
    pinState= 1;
  }else {
    pinState= 0;
  }
  //Sets the pin to HIGH or LOW depending on pinState
  digitalWrite(myDataPin, pinState);
  //register shifts bits on upstroke of clock pin
  digitalWrite(myClockPin, 1);
  //zero the data pin after shift to prevent bleed through
  digitalWrite(myDataPin, 0);
  }
  //stop shifting
  digitalWrite(myClockPin, 0);
}




//blinks the whole register based on the number of times you want to
//blink "n" and the pause between them "d"
//starts with a moment of darkness to make sure the first blink
//has its full visual effect.
void blinkAll_2Bytes(int n, int d) {
  digitalWrite(latchPin, 0);
  shiftOut(dataPin, clockPin, 0);
  shiftOut(dataPin, clockPin, 0);
  digitalWrite(latchPin, 1);
  delay(200);
  for (int x = 0; x < n; x++) {
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 255);
    shiftOut(dataPin, clockPin, 255);
    digitalWrite(latchPin, 1);
    delay(d);
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, 0);
    shiftOut(dataPin, clockPin, 0);
    digitalWrite(latchPin, 1);
    delay(d);
  }
}

void initLSM303(int fs)
{
  LSM303_write(0x27, CTRL_REG1_A);  // 0x27 = normal power mode, all accel axes on
  if ((fs==8)||(fs==4))
    LSM303_write((0x00 | (fs-fs/2-1)<<4), CTRL_REG4_A);  // set full-scale
  else
    LSM303_write(0x00, CTRL_REG4_A);
  LSM303_write(0x14, CRA_REG_M);  // 0x14 = mag 30Hz output rate
  LSM303_write(0x00, MR_REG_M);  // 0x00 = continouous conversion mode
}

void printValues(int * magArray, int * accelArray)
{
  /* print out mag and accel arrays all pretty-like */
  //Serial.print("accelX:");
  Serial.print(accelArray[X], DEC);
  Serial.print("\t");
  Serial.print(accelArray[Y], DEC);
  Serial.print("\t");
  Serial.print(accelArray[Z], DEC);
  Serial.print("\t\t");
  
  Serial.print(magArray[X], DEC);
  Serial.print("\t");
  Serial.print(magArray[Y], DEC);
  Serial.print("\t");
  Serial.print(magArray[Z], DEC);
  Serial.println();
}

float getHeading(int * magValue)
{
  // see section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI;  // assume pitch, roll are 0
  
  if (heading <0)
    heading += 360;
  
  return heading;
}

float getTiltHeading(int * magValue, float * accelValue)
{
  // see appendix A in app note AN3192 
  float pitch = asin(-accelValue[X]);
  float roll = asin(accelValue[Y]/cos(pitch));
  
  float xh = magValue[X] * cos(pitch) + magValue[Z] * sin(pitch);
  float yh = magValue[X] * sin(roll) * sin(pitch) + magValue[Y] * cos(roll) - magValue[Z] * sin(roll) * cos(pitch);
  float zh = -magValue[X] * cos(roll) * sin(pitch) + magValue[Y] * sin(roll) + magValue[Z] * cos(roll) * cos(pitch);

  float heading = 180 * atan2(yh, xh)/PI;
  if (yh >= 0)
    return heading;
  else
    return (360 + heading);
}

void getLSM303_mag(int * rawValues)
{
  Wire.beginTransmission(LSM303_MAG);
  Wire.write(OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(LSM303_MAG, 6);
  for (int i=0; i<3; i++)
    rawValues[i] = (Wire.read() << 8) | Wire.read();
}

void getLSM303_accel(int * rawValues)
{
  rawValues[Z] = ((int)LSM303_read(OUT_X_L_A) << 8) | (LSM303_read(OUT_X_H_A));
  rawValues[X] = ((int)LSM303_read(OUT_Y_L_A) << 8) | (LSM303_read(OUT_Y_H_A));
  rawValues[Y] = ((int)LSM303_read(OUT_Z_L_A) << 8) | (LSM303_read(OUT_Z_H_A));  
  // had to swap those to right the data with the proper axis
}

byte LSM303_read(byte address)
{
  byte temp;
  
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  
  if (address >= 0x20)
    Wire.requestFrom(LSM303_ACC, 1);
  else
    Wire.requestFrom(LSM303_MAG, 1);
  while(!Wire.available())
    ;
  temp = Wire.read();
  Wire.endTransmission();
  
  return temp;
}

void LSM303_write(byte data, byte address)
{
  if (address >= 0x20)
    Wire.beginTransmission(LSM303_ACC);
  else
    Wire.beginTransmission(LSM303_MAG);
    
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}