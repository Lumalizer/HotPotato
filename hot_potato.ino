#include <math.h>
#include <LedControl.h>

// LED Matrix
LedControl lc = LedControl(13,11,12,0);

// Touch Sensor
const int touchSensor = 2;

// Sound Output
const int beeper = 6;

// Drive Control
const int motorSpeed = 3;
const int rightMotorBackward = 8;
const int rightMotorForward = 7;
const int leftMotorForward = 4;
const int leftMotorBackward = 5;

// Ultrasonic Sensor
const int ultrasonicTrig = 9;
const int ultrasonicEcho = 10;

// Variables
int requiredAttempts = 3; //amount of defusion attempts until the timer successfully stops
int attemptTimeAddition = 2000; // amount of time added for each defusion attempt (ms)
int attempts = 0; // current amount of attempts
int totalTime = 20000; // total available starting time until explosion
int timeRemaining;
int randomTarget;
//boolean hasTarget = false;
int previousDistanceMeasurement;
boolean chosenTarget = false;
int targetStartTime;
boolean returningToStart = false;
int timeToReturnToStart;
int forwardStepTracker = 0;

// States
const int INACTIVE = 0;
const int INITIATING = 1;
const int PLAYING = 2;
int state = INACTIVE;



// -------------------------------- Setup



void setup() {
  pinMode(motorSpeed, OUTPUT);
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(touchSensor, INPUT);
  pinMode(beeper, OUTPUT);
  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  analogWrite(motorSpeed, 255);
  lc.shutdown(0,false);
  lc.setIntensity(0,6);
  lc.clearDisplay(0);
  Serial.begin(9600);
}



// ----------------------- Helper functions



uint8_t Logrithmic(uint8_t num) {
  int output = log(num+1)/log(1000)*255;
  return output;
}

boolean buttonPressed() {
  return digitalRead(touchSensor) == HIGH;
}



// ----------------------- Ultrasonic sensor



long ultrasonic_duration;
int ultrasonic_distance;

int getDistance() {
  digitalWrite(ultrasonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonicTrig, LOW);
  ultrasonic_duration = pulseIn(ultrasonicEcho, HIGH);
  ultrasonic_distance = ultrasonic_duration * 0.034 / 2;
  return ultrasonic_distance;
//  Serial.print("Distance: ");
//  Serial.println(ultrasonic_distance);
}



// ----------------------------- Sound output



void beep(int freq, int delay1, int delay2, int repetitions = 1) {
  for (int i = 0; i < repetitions; i++){
    tone(beeper, freq);
    delay(delay1);
    noTone(beeper);
    delay(delay2);
  }
}



// ----------------------------- Drive control



void forward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW); 
}

void backward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH); 
}

void halt() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW); 
}

void startRotation() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void spin(int milliseconds) {
  startRotation();
  delay(milliseconds);
  halt();
}

void forwardBackward(int milliseconds, int wait = 100) {
  forward();
  delay(milliseconds/2);
  halt();
  delay(wait);
  backward();
  delay(milliseconds/2);
  halt();
}



// --------------------- LED Matrix



byte smile[8] = {0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};
byte heart1[8] = {0x00,0x00,0x18,0x3C,0x18,0x18,0x00,0x00};
byte heart2[8] = {0x00,0x18,0x3C,0x7E,0x3C,0x3C,0x18,0x00};
byte heart3[8] = {0x00,0x7E,0x7E,0xFF,0x7E,0x7E,0x3C,0x18};
byte heart4[8] = {0x66,0xFF,0xFF,0xFF,0xFF,0x7E,0x3C,0x18};
byte mine1[8] = {0xEF,0xEF,0xC3,0xD3,0x00,0xC3,0xEF,0xEF};
byte mine2[8] = {0xEF,0xAD,0xC3,0xD3,0x00,0xC3,0xED,0xEF};
byte mine3[8] = {0x6E,0xAD,0xC3,0xD7,0x00,0xC3,0xAD,0xEE};
byte mine4[8] = {0x76,0xAD,0x43,0xDD,0x18,0xC3,0xAD,0x6E};
byte questionMark[8] = {0x3C,0x7E,0x66,0x66,0x0E,0x18,0x00,0x18};
byte numberThree[8] = {0x18,0x24,0x04,0x18,0x04,0x24,0x18,0x00};
byte numberTwo[8] = {0x18,0x24,0x04,0x18,0x20,0x20,0x3C,0x00};
byte numberOne[8] = {0x18,0x38,0x08,0x08,0x08,0x08,0x08,0x00};
byte backwardArrow[8] = {0x18,0x3C,0x7E,0x18,0x18,0x18,0x18,0x18};
byte forwardArrow[8] = {0x18,0x18,0x18,0x18,0x18,0x7E,0x3C,0x18};
byte rightwardArrow[8] = {0x04,0x06,0x7F,0xFF,0xC6,0xC4,0x70,0x38};


void printByte(byte character []) {
  for(int i=0;i<8;i++)
  {
    lc.setRow(0,i,character[i]);
  }
}

void animateExplosion(int repetitions = 1) {
  for (int i = 1; i <= repetitions; i++) {
    if (i < 15) {
      lc.setIntensity(0,i);
    } else {
      lc.setIntensity(0,15);
    }
    printByte(mine4);
    delay(100);
    printByte(mine3);
    delay(100);
    printByte(mine2);
    delay(100);
    printByte(mine1);
    delay(100);
  }
  lc.clearDisplay(0);  
}

void animateHeart(int repetitions = 1) {
  for (int i = 0; i < repetitions; i++) {
    lc.setIntensity(0,15);
    printByte(heart1);
    delay(400);
    printByte(heart2);
    delay(400);
    printByte(heart3);
    delay(400);
    printByte(heart4);
    delay(1000);
  }  
  lc.clearDisplay(0);  
}



// ---------------------- Timing functions



// the less time there is left, the shorter the delay, with upper and lower limits
int getDelayTime() {
  float ratio = (float(timeRemaining) / float(totalTime));
  int delayed = 20 + min(int(ratio*ratio * totalTime / 5), min(totalTime/10, 1500));
  return delayed;
}

void extendTime() {
  timeRemaining += attemptTimeAddition;
  tone(beeper, 900);
  delay(50);
  timeRemaining -= 50;
  noTone(beeper);
}

unsigned long startMillis; 
unsigned long currentMillis;
const unsigned long period = 1000;

void soundTimer() {
//  tone(beeper, 600);
//  delay(50);
//  timeRemaining -= 50;
//  noTone(beeper);
//  int delayed = getDelayTime();
//  delay(delayed);
//  timeRemaining -= delayed;

    currentMillis = millis();
    int delayed = getDelayTime();
    timeRemaining -= 50;
    delay(50);
    if (currentMillis - startMillis >= delayed) {
      beep(600, 150, 50);
      timeRemaining -= delayed;
      startMillis = currentMillis;
    }
    
}



// --------------------------- Game functions



void start() {
  lc.setIntensity(0,15);
  printByte(numberThree);
  beep(1000, 150, 50);
  delay(800);
  printByte(numberTwo);
  beep(1000, 150, 50);
  delay(800);
  printByte(numberOne);
  beep(1000, 150, 50);
  delay(800);
  lc.clearDisplay(0);  
  timeRemaining = totalTime;
  chooseRandomTarget();
  state = PLAYING;
}

void explode() {
  state = INACTIVE;

  int Hz = 1000;
  animateExplosion();
  for(int i = 0; i < 20; i++) {
    Hz += i*30;
    printByte(mine1);
    tone(beeper, Hz);
    delay(Logrithmic(i));
    printByte(mine2);
    noTone(beeper);
    delay(Logrithmic(i));
    printByte(mine3);
  }
  beep(100, 400, 0);
}

void defuse() {
  state = INACTIVE;

  attempts = 0;
  beep(800, 60, 200, 5);
  beep(700, 90, 300, 4);
  beep(500, 240, 800, 2);
  beep(400, 440, 800, 1);
}

void chooseRandomTarget() {
  randomTarget = random(10, 30);
}

void lookForTarget() {
//  lc.clearDisplay(0);
//  int dist = getDistance();
//  Serial.print("Distance: ");
//  Serial.println(dist);
//  if (dist < 60 && (abs(previousDistanceMeasurement - dist) > 60)) {
//    Serial.print("Target found! Remaining: ");
//    Serial.println(randomTarget);
//    printByte(heart2);
//    if (randomTarget != 0) {
//      randomTarget = randomTarget - 1;
//    }
//    previousDistanceMeasurement = dist;
//  }

    randomTarget -= 1;
}


void listenForTargetAction() {
  if (buttonPressed()) {

    chooseRandomTarget();
//    chosenTarget = false;
//    returningToStart = true;


//    timeToReturnToStart = (currentMillis - targetStartTime);
//    Serial.print(timeToReturnToStart);
  }
}

void initialize() {
//    spin(2000);
//    delay(2000);
//    spin(1500);
//    beep(100, 50, 50);
//    spin(400);
//    delay(2000);
//    readUltrasonic();
    
//    animateHeart(3);
    state = INITIATING;
}



// ------------------------------- Main Loop



void loop() {
    if (state == INACTIVE) {
      printByte(questionMark);
      if (buttonPressed()) {initialize();}
    } 
    else if (state == INITIATING) {
      start();
    }
    else if (state == PLAYING) {
      Serial.print("time: ");
      Serial.print(timeRemaining);
        if (timeRemaining < 1) {explode();}
        else {
          currentMillis = millis();
          
          
//          if (returningToStart) 
//          
//          {
//            if (forwardStepTracker > 0) {
//              backward();
//              forwardStepTracker -= 1;
//            } else {
//              returningToStart = false;
//              forwardStepTracker = 0;
//              chooseRandomTarget();
//            }
//            printByte(backwardArrow);
//          } 
//          else
          if (randomTarget == 0) 
          
          {
            // rotated to and now facing the chosen target
//            if (!chosenTarget) {
//              chosenTarget = true;
//              targetStartTime = millis();
//            }
//            if (currentMillis - targetStartTime < 3000) {
//              forward();
//              forwardStepTracker += 1;
//            }
            printByte(forwardArrow);
            listenForTargetAction();
          } 
          
          else 
          
          {
            // continue rotating to the target     
            startRotation();
            printByte(rightwardArrow);
            lookForTarget();
          }
          
          soundTimer();
          halt();
          
//          if (buttonPressed()){
//            attempts += 1;
//            if (attempts >= requiredAttempts) {defuse();}
//            else {extendTime();}
//          }
        }
      }
}
