/*

    Autonomous Robotic Pacer arduino program


    Created by Yorai Shaoul. The program is open source and can be used for
    personal applications. The use of this code for profit is prohibited
    unless otherwise specified by the creator.
    I can be reached at yorai[at]mit[dot]edu

    The physical setup required for this program to function can be studied at yoraish.com.


    Keep on making!
    Yorai :)
*/
// for seven sensors, from left to right


//libraries
#include <PID_v1.h>

#include <Wire.h>

#include <Servo.h>

#include <Adafruit_TCS34725.h>

#include <Encoder.h>

#include <SoftReset.h>

#include <EEPROM.h>


//serial to float vars
String inString;
float floatOut;
bool gotFloat = false;

//sensor defintion

#define TCAADDR 0x70

uint8_t rightAddress = 1;
uint8_t leftAddress = 2;

uint8_t xrightAddress = 5;
uint8_t xleftAddress = 4;

uint8_t xxrightAddress = 0;
uint8_t xxleftAddress = 6;

uint8_t middleAddress = 3;


uint16_t rRight, gRight, bRight, cRight, colorTempRight, luxRight;
uint16_t rLeft, gLeft, bLeft, cLeft, colorTempLeft, luxLeft;

uint16_t rXRight, gXRight, bXRight, cXRight, colorTempXRight, luxXRight;
uint16_t rXLeft, gXLeft, bXLeft, cXLeft, colorTempXLeft, luxXLeft;


uint16_t rXXRight, gXXRight, bXXRight, cXXRight, colorTempXXRight, luxXXRight;
uint16_t rXXLeft, gXXLeft, bXXLeft, cXXLeft, colorTempXXLeft, luxXXLeft;


uint16_t rMiddle, gMiddle, bMiddle, cMiddle, colorTempMiddle, luxMiddle;



// variables for color detection, including threshold

float threshold = 0.015;
//--------change from color check ---------------------


float RrLine = 0.3687;
float RgLine = 0.3176;
float RbLine = 0.3137;
float rTotal; float rRpart, rGpart, rBpart;

float LrLine = 0.3744;
float LgLine = 0.3153;
float LbLine = 0.3103;
float lTotal; float lRpart, lGpart, lBpart;

float XRrLine = 0.3676;
float XRgLine = 0.3186;
float XRbLine = 0.3137;
float xrTotal; float xrRpart, xrGpart, xrBpart;

float XLrLine = 0.3839;
float XLgLine = 0.3080;
float XLbLine = 0.3080;
float xlTotal; float xlRpart, xlGpart, xlBpart;

float XXRrLine = 0.3733;
float XXRgLine = 0.3162;
float XXRbLine = 0.3105;
float xxrTotal; float xxrRpart, xxrGpart, xxrBpart;

float XXLrLine = 0.3860;
float XXLgLine = 0.3114;
float XXLbLine = 0.3026;
float xxlTotal; float xxlRpart, xxlGpart, xxlBpart;

float MrLine = 0.3731;
float MgLine = 0.3109;
float MbLine = 0.3161;
float mTotal; float mRpart, mGpart, mBpart;



//--------- up to here ----------------


float rRc, gRc, bRc;
float rLc, gLc, bLc;

float rXRc, gXRc, bXRc;
float rXLc, gXLc, bXLc;

float rXXRc, gXXRc, bXXRc;
float rXXLc, gXXLc, bXXLc;

float rMc, gMc, bMc;




int lastPosition = 8;


// steering and esc declarations

Servo steerServo;

#define servoPin 9

float steerAngle = 84;

Servo esc;

#define escPin 8

int escSpeed = 80;

//i2c multiplexer function

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}



/* RIGHT Initialise with specific int time and gain values */
Adafruit_TCS34725 rightSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/* LEFT Initialise with specific int time and gain values */
Adafruit_TCS34725 leftSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/* Extreme RIGHT Initialise with specific int time and gain values */
Adafruit_TCS34725 xrightSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/*Extreme LEFT Initialise with specific int time and gain values */
Adafruit_TCS34725 xleftSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);


/* Extreme Extreme LEFT Initialise with specific int time and gain values */
Adafruit_TCS34725 xxleftSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/* Extreme Extreme RIGHT Initialise with specific int time and gain values */
Adafruit_TCS34725 xxrightSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

/*MIDDLE Initialise with specific int time and gain values */
Adafruit_TCS34725 middleSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);



//position vars

int positionArray[8] = {8, 0, 0, 0, 0, 0, 0, 0};

unsigned long positionNumber = 0;

int linePos; // the output from linePosition function
//pid vars

double pidInput;
double pidOutput;
double pidSetpoint = 8;

PID pid(&pidInput, &pidOutput, &pidSetpoint, 0.62 , 0.16 , 0.139, DIRECT); // Modes  [1]  [2]  [3] [4] [5] [6] [7] [8] [9]  1.3 0.5 0.15 || 0.9 ,0.5 ,0.2 || 0.85 ,0.32 ,0.13

//speed measurment

volatile int hzCount = 0;
double kph = 0;
unsigned long lastmillis = 0;

//spid vars - for controlling speed

double spidInput;
double spidOutput;
double spidSetpoint = 8; // setpoint in kph

PID spid(&spidInput, &spidOutput, &spidSetpoint, 0.7 , 1 , 0, DIRECT);

//duration vars

float duration = 0; // the duration of the session in MINUTES
unsigned long durationMillis = 0; //  the same duration of the session in MILLISECONDS
unsigned long startMillis = 0;

//calibration vars
#define calibrationPin 10

//==========================================================================================
//--------------------------------------SETUP---------------------------------------------
//==========================================================================================




void setup() {

  Serial.begin(9600);

  delay(500);

  //if calibration buttin is pressed, calibrate sensors
  pinMode(calibrationPin, INPUT);
  if (digitalRead(calibrationPin) == HIGH) {
    sensorCalibration();
  }


  steerServo.attach(servoPin);
  steerServo.write(steerAngle); //servo middle position

  delay(100);

  esc.attach(escPin);
  //esc.write(escSpeed); //esc stopped/ running at contant speed


  //pid
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-9, 9);


  //spid
  spid.SetMode(AUTOMATIC);
  spid.SetOutputLimits(99, 120); // limits the speed the speed. 110 is about 15 km/h

  //rpm interrupt
  attachInterrupt(0, rpm_drive, FALLING);//interrupt cero (0) is on pin two(2).


  //get pace from BT
  Serial.println("Insert pace (in kph)");

  do {
    serialToFloat();
  }
  while (gotFloat == false);

  Serial.println("got pace!");
  gotFloat = false;
  spidSetpoint = floatOut; // set the speed to the one received (in kph)

  //get duration
  Serial.println("Insert duration (in minutes)");

  do {
    serialToFloat();
  }
  while (gotFloat == false);

  duration = floatOut;
  durationMillis = duration * 60000;
  //durationMillis = durationMillis*60000; // the duration in milliseconds

  Serial.print(durationMillis);

  Serial.println(" --- got millis duration!");
  gotFloat = false;

  // no reason to get distance, because we can rely on speed*time = distance


  /*sensorCalibration();

    Serial.print("XLeft: "); Serial.print("R: "); Serial.print(XLrLine); Serial.print(" G: "); Serial.print(XLgLine); Serial.print(" B: "); Serial.print(XLbLine); Serial.println("");
  */

  startMillis = millis();
  Serial.println("end of setup");
}


//==========================================================================================
//------------------------LOOP--------------------------------------------------------------
//==========================================================================================




void loop() {
  //pid for steering
  //Serial.println("loop");

  linePosition();
  pidInput = linePos;
  Serial.println(pidInput);

  pid.Compute();

  steerAngle = 84 + pidOutput;
  steerServo.write(round(steerAngle));


  //checks every 1/2 second
  if (millis() - lastmillis >= 500) {

    //speed calculation

    calculateSpeed();//Uptade every one half second, this will be equal to half the reading frequency (Hz).
    //pid for speed
    spidInput = kph;
    spid.Compute();
    //spidOutput = map(spidOutput,96,106,0,255);
    escSpeed = round(spidOutput);
    esc.write(escSpeed); //esc stopped/ running at contant speed
    //Serial.println(escSpeed);


    //check if time has passed, stop program
    if (millis() - startMillis > durationMillis) {
      Serial.println("END after");
      Serial.println(durationMillis);
      esc.detach();
      soft_restart(); //call reset
      while (1) {}
    }

    //check if stop has been pressed on phone
    if (Serial.available() > 0) {
      if (Serial.read() == '0') {
        Serial.println("END after");
        Serial.println(lastmillis);
        esc.detach();
        soft_restart(); //call reset
        while (1) {};
      }
    }

  }


}


//====================================================================================


int rightSensing() {

  tcaselect(rightAddress); //get info from right sensor

  rightSensor.getRawData(&rRight, &gRight, &bRight, &cRight);


  rTotal = rRight + gRight + bRight;
  rRpart = rRight / rTotal;
  rGpart = gRight / rTotal;
  rBpart = bRight / rTotal;


  rRc = rRpart - RrLine;
  gRc = rGpart - RgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bRc = rBpart - RbLine;



  //&& abs(gRc) < 100 && abs(bRc) < 100
  if (abs(rRc) < threshold && abs(gRc) < threshold && abs(bRc) < threshold) {


    return 1;

  }
  else {

    return 0;


  }

}
int leftSensing() {

  tcaselect(leftAddress); //get info from left sensor


  leftSensor.getRawData(&rLeft, &gLeft, &bLeft, &cLeft);

  lTotal = rLeft + gLeft + bLeft;
  lRpart = rLeft / lTotal;
  lGpart = gLeft / lTotal;
  lBpart = bLeft / lTotal;





  rLc = lRpart - LrLine;
  gLc = lGpart - LgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bLc = lBpart - LbLine;







  //
  if (abs(rLc) < threshold && abs(gLc) < threshold && abs(bLc) < threshold ) {


    return 1;


  }
  else {


    return 0;


  }

}



//===================================EXTREMES=================================================


int xrightSensing() {

  tcaselect(xrightAddress); //get info from right sensor

  xrightSensor.getRawData(&rXRight, &gXRight, &bXRight, &cXRight);


  xrTotal = rXRight + gXRight + bXRight;
  xrRpart = rXRight / xrTotal;
  xrGpart = gXRight / xrTotal;
  xrBpart = bXRight / xrTotal;


  rXRc = xrRpart - XRrLine;
  gXRc = xrGpart - XRgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bXRc = xrBpart - XRbLine;



  //&& abs(gRc) < 100 && abs(bRc) < 100
  if (abs(rXRc) < threshold && abs(gXRc) < threshold && abs(bXRc) < threshold) {


    return 1;

  }
  else {

    return 0;


  }

}
int xleftSensing() {

  tcaselect(xleftAddress); //get info from left sensor


  xleftSensor.getRawData(&rXLeft, &gXLeft, &bXLeft, &cXLeft);

  xlTotal = rXLeft + gXLeft + bXLeft;
  xlRpart = rXLeft / xlTotal;
  xlGpart = gXLeft / xlTotal;
  xlBpart = bXLeft / xlTotal;





  rXLc = xlRpart - XLrLine;
  gXLc = xlGpart - XLgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bXLc = xlBpart - XLbLine;







  //
  if (abs(rXLc) < threshold && abs(gXLc) < threshold && abs(bXLc) < threshold ) {
    return 1;


  }
  else {


    return 0;


  }

}




//===================================EXTREMES EXTREMES=================================================


int xxrightSensing() {

  tcaselect(xxrightAddress); //get info from right sensor

  xxrightSensor.getRawData(&rXXRight, &gXXRight, &bXXRight, &cXXRight);


  xxrTotal = rXXRight + gXXRight + bXXRight;
  xxrRpart = rXXRight / xxrTotal;
  xxrGpart = gXXRight / xxrTotal;
  xxrBpart = bXXRight / xxrTotal;


  rXXRc = xxrRpart - XXRrLine;
  gXXRc = xxrGpart - XXRgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bXXRc = xxrBpart - XXRbLine;



  //&& abs(gRc) < 100 && abs(bRc) < 100
  if (abs(rXXRc) < threshold && abs(gXXRc) < threshold && abs(bXXRc) < threshold) {


    return 1;

  }
  else {

    return 0;


  }

}

int xxleftSensing() {

  tcaselect(xxleftAddress); //get info from left sensor


  xxleftSensor.getRawData(&rXXLeft, &gXXLeft, &bXXLeft, &cXXLeft);

  xxlTotal = rXXLeft + gXXLeft + bXXLeft;
  xxlRpart = rXXLeft / xxlTotal;
  xxlGpart = gXXLeft / xxlTotal;
  xxlBpart = bXXLeft / xxlTotal;





  rXXLc = xxlRpart - XXLrLine;
  gXXLc = xxlGpart - XXLgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bXXLc = xxlBpart - XXLbLine;







  //
  if (abs(rXXLc) < threshold && abs(gXXLc) < threshold && abs(bXXLc) < threshold ) {
    return 1;


  }
  else {


    return 0;


  }

}

//================middle==============

int middleSensing() {

  tcaselect(middleAddress); //get info from left sensor


  middleSensor.getRawData(&rMiddle, &gMiddle, &bMiddle, &cMiddle);

  mTotal = rMiddle + gMiddle + bMiddle;
  mRpart = rMiddle / mTotal;
  mGpart = gMiddle / mTotal;
  mBpart = bMiddle / mTotal;





  rMc = mRpart - MrLine;
  gMc = mGpart - MgLine;       // calculations cannot take place inside abs() - doing them outside and then abs(gRc);
  bMc = mBpart - MbLine;


  //
  if (abs(rMc) < threshold && abs(gMc) < threshold && abs(bMc) < threshold ) {
    return 1;


  }
  else {


    return 0;


  }

}

//==

void linePosition() { // where is the line, when looking from the robot's point of view
  //Serial.println("position function");
  positionArray[0] = 8; //first number of array is non zero


  positionArray[1] = xxleftSensing(); //xleft
  //Serial.println("of middle position function");

  positionArray[2] = xleftSensing(); // left

  positionArray[3] = leftSensing(); // right

  positionArray[4] = middleSensing(); // right

  positionArray[5] = rightSensing(); // right

  positionArray[6] = xrightSensing(); // xright

  positionArray[7] = xxrightSensing(); // right

  arrayToNumber(positionArray);

  switch (positionNumber) {

    case 80001000 :

      lastPosition = 8;
      linePos = 8;
      break;

    case 80011000 :

      lastPosition = 7;
      linePos = 7;
      break;

    case 80010000 :

      lastPosition = 6;
      linePos = 6;
      break;

    case 80110000 :

      lastPosition = 5;
      linePos = 5;
      break;

    case 80100000 :

      lastPosition = 4;
      linePos = 4;
      break;

    case 81100000 :

      lastPosition = 3;
      linePos = 3;
      break;

    case 81000000 :

      lastPosition = 2;
      linePos = 2;
      break;

    case 80001100 :

      lastPosition = 9;
      linePos = 9;
      break;

    case 80000100 :

      lastPosition = 10;
      linePos = 10;
      break;

    case 80000110 :

      lastPosition = 11;
      linePos = 11;
      break;

    case 80000010 :

      lastPosition = 12;
      linePos = 12;
      break;

    case 80000011 :

      lastPosition = 13;
      linePos = 13;
      break;

    case 80000001 :

      lastPosition = 14;
      linePos = 14;
      break;

    case 80000000 :

      if (lastPosition == 2 || lastPosition == 1) {
        linePos = 1;
      }
      else if (lastPosition == 14 || lastPosition == 15) {
        linePos = 15;
      }
      break;

    default:


      linePos = lastPosition;

  }
  // Serial.println("end of position function");

}








//==============================SPEED MEASURMENTS===================================
void calculateSpeed() {

  detachInterrupt(0);    //Disable interrupt when calculating

  //v = RPM × r(meters) × 0.10472

  //     (counts per second - two counts per rotation)*(differential gear ratio)*(60 seconds) = RPM *(Radius in meters)*Constant
  kph = ((hzCount / 2) * (0.3421) * 60 * 2) * (0.0675) * (0.10472); /* Convert frequency to kph, note: this works for one interruption per half rotation. For two interrups per full rotation use rpmcount * 30.*/

  //Serial.print("KPH = "); //print the word "KPH" and tab.
  //Serial.println(kph); // print the KPH value.

  hzCount = 0; // Restart the RPM counter
  lastmillis = millis(); // Uptade lasmillis
  attachInterrupt(0, rpm_drive, FALLING); //enable interrupt
}


//rpm interrupt adds to counter

void rpm_drive() { /* this code will be executed every time the interrupt 0 (pin2) gets low.*/
  hzCount++;
  //Serial.println(hzCount);
}




//=================ARRAY TO NUMBER==============

unsigned long arrayToNumber (int array[]) {
  positionNumber = 0;

  for ( int i = 0; i < 8 ; i ++) {


    positionNumber = positionNumber * 10 + array[i];
  }
  //Serial.println(positionNumber);

  return positionNumber;
}



void sensorCalibration() {

  RrLine = 0;  RgLine = 0;   RbLine = 0;

  LrLine = 0;  LgLine = 0; LbLine = 0;

  float counts = 0;
  float RrSum = 0, LrSum = 0, XRrSum = 0, XLrSum = 0, XXRrSum = 0, XXLrSum = 0, MrSum = 0 ;
  float RgSum = 0, LgSum = 0, XRgSum = 0, XLgSum = 0, XXRgSum = 0, XXLgSum = 0, MgSum = 0;
  float RbSum = 0, LbSum = 0, XRbSum = 0, XLbSum = 0, XXRbSum = 0, XXLbSum = 0, MbSum = 0;



  for (int i = 0; i < 9; i++) { // sensing every sensor and adding to its corresponding sum

    rightSensing();
    RrSum += rRpart; delay(10);
    RgSum += rGpart; delay(10);
    RbSum += rBpart; delay(10);
    leftSensing();
    LrSum += lRpart;
    LgSum += lGpart;
    LbSum += lBpart;
    delay(10);


    xrightSensing();
    XRrSum += xrRpart;
    XRgSum += xrGpart;
    XRbSum += xrBpart;

    delay(10);

    xleftSensing();
    XLrSum += (xlRpart);
    XLgSum += (xlGpart);
    XLbSum += (xlBpart);
    // Serial.print(XLgSum);
    delay(10);

    xxrightSensing();
    XXRrSum += xxrRpart;
    XXRgSum += xxrGpart;
    XXRbSum += xxrBpart;
    delay(10);

    xxleftSensing();
    XXLrSum += xxlRpart;
    XXLgSum += xxlGpart;
    XXLbSum += xxlBpart;


    delay(10);

    middleSensing();
    MrSum += mRpart;
    MgSum += mGpart;
    MbSum += mBpart;

    delay(10);

    counts ++;
  }

  //calculating the average of all counts in the calibration's time period

  RrLine = RrSum / counts;
  RgLine = RgSum / counts;
  RbLine = RbSum / counts;

  LrLine = (LrSum / counts);
  LgLine = LgSum / counts;
  LbLine = LbSum / counts;

  XRrLine = XRrSum / counts;
  XRgLine = XRgSum / counts;
  XRbLine = XRbSum / counts;

  XLrLine = (XLrSum / counts);
  XLgLine = (XLgSum / counts);
  XLbLine = (XLbSum / counts);

  XXRrLine = XXRrSum / counts;
  XXRgLine = XXRgSum / counts;
  XXRbLine = XXRbSum / counts;

  XXLrLine = XXLrSum / counts;
  XXLgLine = XXLgSum / counts;
  XXLbLine = XXLbSum / counts;

  MrLine = MrSum / counts;
  MgLine = MgSum / counts;
  MbLine = MbSum / counts;


  Serial.println("DONE CAL");



  delay(300);

}




float serialToFloat() {
  if (Serial.available() > 0) {
    int inChar;

    inChar = Serial.read();

    if (inChar != 'a') {

      // As long as the incoming byte
      // is not a 'a',
      // convert the incoming byte to a char
      // and add it to the string

      inString += (char)inChar;
      delay(100);

    }
    // if you get a 'a', print the string value as a float:

    else {
      floatOut = inString.toFloat();
      Serial.print("This is the float:");  Serial.println(floatOut);
      inString = "";
      delay(50);
      Serial.flush();
      gotFloat = true;
      return floatOut;

    }
  }
}
