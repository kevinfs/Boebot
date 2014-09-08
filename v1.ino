#include <Servo.h>
#include <math.h>


// LEDS
int pinLed = 3;
int pinLedLightLeft = 11;
int pinLedLightRight = 10;

// Servos
Servo rightServo;
Servo leftServo;
int pinRightServo = 13;
int pinLeftServo = 12;

// RightServo
int stopSign = 96;
int forwardSignRight = 51;
int forwardQuickSignRight = 0;
//int forwardQuickerSignRight = 544;
int backwardSignRight = 140;
int backwardQuickSignRight = 180;
//int backwardQuickerSignRight = 543;

// LeftServo
int forwardSignLeft = 140;
int forwardQuickSignLeft = 180;
//int forwardQuickerSignLeft = 543;
int backwardSignLeft = 51;
int backwardQuickSignLeft = 0;
//int backwardQuickerSignLeft = 544;
//phototransistor
int phototransistorLeft = A1;
int phototransistorRight = A0;

// PID
float error;
float lastError;
float kP = 1; // 0.5
float kD = 1; // 1
float pV;




void setup() {
  // initialize the digital pin as an output.
  Serial.begin(9600);
  pinMode (pinLed, OUTPUT);
  turnOnLights();

  rightServo.attach(pinRightServo);
  leftServo.attach(pinLeftServo); 

  ledBlink(3, 100);
  //  turnLeft();
  //  ledBlink(3, 100);
  //  turnRight();
  //  ledBlink(3, 100);

  /*moveForward(2000);
   ledBlink(3, 200);
   moveBackward(2000);
   ledBlink(3, 200);*/
  //  rightServo.write(94); 
  //   leftServo.write(100);

  //moveForwardCM(30.0);
//   uTurn();
//  moveToFirstBase(1,1,-20,1);
}

void loop() {
  //digitalWrite(3,HIGH);
  //  outputPhotoTransistorValues();
 followLine();

  // v = 11 cm/s en normal
  // v = 12 cm/s en quick

  //  rightServo.write(forwardQuickSignRight); 
  //    leftServo.write(forwardQuickSignLeft);

}

// followLine with PID mechanics
void followLine() {

  error = (float) linePosition();

  // too much left or too much right
  if(error > 0.13) {
    turnRight();
  } 
  else if(error < -0.13){
    turnLeft();
  } 

  pV = kP * error + kD * (error - lastError);
  lastError = error;
  Serial.print("Pv: ");
  Serial.println(pV);
  moveForwardAngle(pV);

}

float linePosition() {
  float output = voltPhotoRight()-voltPhotoLeft();
  Serial.print("LinePosition: ");
  Serial.println(output);
  return output; 
}

float voltPhotoLeft() {
  return volts(phototransistorLeft); 
}

float voltPhotoRight() {
  return volts(phototransistorRight); 
}

// ledBlink
void ledBlink(int times, int duration) {
  for(int i=times;i-->0;) {
    digitalWrite(pinLed, HIGH);
    delay(duration);
    digitalWrite(pinLed, LOW);
    delay(duration);
  }
}

// moveForward
void moveForward(int duration) {
  rightServo.write(forwardQuickSignRight);
  leftServo.write(forwardQuickSignLeft);
  delay(duration);
  rightServo.write(stopSign);
  leftServo.write(stopSign);
}

// moveForwardAngle
void moveForwardAngle(float pV) {

  if(pV>0.02) {
    rightServo.write(94); // 94
    leftServo.write(100);
  } 
  else if(pV<-0.02) {
    rightServo.write(90); 
    leftServo.write(98);
  } 
  else {
    rightServo.write(forwardSignRight); 
    leftServo.write(forwardSignLeft);
  }
  /*
  rightServo.write(forwardQuickSignRight-pV);
   leftServo.write(forwardQuickSignLeft+pV);
   Serial.print("Right: ");
   Serial.println(forwardQuickSignRight-pV);
   Serial.print("Left: ");
   Serial.println(forwardQuickSignLeft+pV);*/
}

void moveForwardCM(float length) {
  rightServo.write(forwardQuickSignRight);
  leftServo.write(forwardQuickSignLeft);
  float duration = (float) length / 11 * 1000;
  delay(duration);
  rightServo.write(stopSign);
  leftServo.write(stopSign);
}

// moveBackward
void moveBackward(int duration) {
  rightServo.write(backwardQuickSignRight);
  leftServo.write(backwardQuickSignLeft);
  delay(duration);
  rightServo.write(stopSign);
  leftServo.write(stopSign);
}

// turnRight
void turnRight() {
  leftServo.write(forwardQuickSignLeft);
  rightServo.write(backwardQuickSignRight);
  delay(400);
  leftServo.write(stopSign);
  rightServo.write(stopSign);
}

// turnLeft
void turnLeft() {
  leftServo.write(backwardQuickSignLeft);
  rightServo.write(forwardQuickSignRight);
  delay(400);
  leftServo.write(stopSign);
  rightServo.write(stopSign);
}

// U-turn
void uTurn() {
  leftServo.write(backwardQuickSignLeft);
  rightServo.write(forwardQuickSignRight);
  delay(1400);
  leftServo.write(stopSign);
  rightServo.write(stopSign);
}

// turnAngle
void turnAngle(float angle) {
  leftServo.write(forwardQuickSignLeft);
  rightServo.write(backwardQuickSignRight);
  delay(angle/118*1000);
  leftServo.write(stopSign);
  rightServo.write(stopSign);
}

void moveToFirstBase(float x1, float y1, float x2, float y2) {
  float alpha = atan(fabs(y2-y1)/fabs(x2-x1))*180/M_PI;
  if(x2>=x1 && y2>=y1){
    alpha = 90-alpha;
  } else if (x2>=x1 && y2<y1){
    alpha = 90 + alpha;
  }  else if (x2<x1 && y2<y1){
    alpha = 360 - (90 + alpha);
  }  else if (x2<x1 && y2>=y1){
    alpha = 360 - (90 - alpha);
  }
  float distance = sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
  turnAngle(alpha);
  moveForwardCM(distance);
  Serial.print("alpha = ");
  Serial.println(alpha);
  Serial.print("distance = ");
  Serial.println(distance);
}

/*long rcTime(int pin) {
 pinMode(pin, OUTPUT);
 digitalWrite(pin, HIGH);
 delay(1)*/

void turnOnLights() {
  pinMode(pinLedLightLeft, OUTPUT);
  pinMode(pinLedLightRight, OUTPUT);
  digitalWrite(pinLedLightLeft, HIGH);
  digitalWrite(pinLedLightRight, HIGH);
}

float volts(int adPin) {
  return float(analogRead(adPin))*5.0/1024.0;
}

void outputPhotoTransistorValues() {
  Serial.print(voltPhotoLeft());
  Serial.print("  ");
  Serial.println(voltPhotoRight());
  delay(10);
}




