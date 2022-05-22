#include <math.h>

//https://www.instructables.com/How-to-use-the-L293D-Motor-Driver-Arduino-Tutorial/
const int leftMotorForward  = 5;  // Pin 14 of L293
const int leftMotorBackward  = 6;  // Pin 10 of L293
const int rightMotorForward  = 10; // Pin  7 of L293
const int rightMotorBackward  = 9;  // Pin  2 of L293

const int touchSensor = 2;
const int beeper = 8;

int requiredAttempts = 3; //amount of defusion attempts until the timer successfully stops
int attemptTimeAddition = 2000; // amount of time added for each defusion attempt (ms)
int attempts = 0; // current amount of attempts
int totalTime = 20000; // total available starting time until explosion
int timeRemaining;

const int INACTIVE = 0;
const int INITIATING = 1;
const int PLAYING = 2;
int state = INACTIVE;

void setup() {
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(touchSensor, INPUT);
  pinMode(beeper, OUTPUT);
  Serial.begin(9600);
}

boolean buttonPressed() {
  return digitalRead(touchSensor) == HIGH;
}

void beep(int freq, int delay1, int delay2, int repetitions = 1) {
  for (int i = 0; i < repetitions; i++){
    tone(beeper, freq);
    delay(delay1);
    noTone(beeper);
    delay(delay2);
  }
}

void initialize() {
  state = INITIATING;

  // TEST MOTORS
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  delay(1000); 

  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void start() {
  state = PLAYING;
  beep(1000, 200, 50, 3);
  timeRemaining = totalTime;
}

uint8_t Logrithmic(uint8_t num){
  int output = log(num+1)/log(1000)*255;
  return output;
}

void explode() {
  state = INACTIVE;

  int Hz = 1000;
  for(int i = 0; i < 20; i++) {
    Hz += i*30;
    tone(beeper, Hz);
    delay(Logrithmic(i));
    noTone(beeper);
    delay(Logrithmic(i));
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

// the less time there is left, the shorter the delay, with upper and lower limits
int getDelayTime(){
  float ratio = (float(timeRemaining) / float(totalTime));
  int delayed = 20 + min(int(ratio*ratio * totalTime / 5), min(totalTime/10, 1500));
  return delayed;
}

void extendTime(){
  timeRemaining += attemptTimeAddition;
  tone(beeper, 900);
  delay(50);
  timeRemaining -= 50;
  noTone(beeper);
}

void soundTimer(){
  tone(beeper, 600);
  delay(50);
  timeRemaining -= 50;
  noTone(beeper);
  int delayed = getDelayTime();
  delay(delayed);
  timeRemaining -= delayed;
}

void loop() {
    if (state == INACTIVE) {
      if (buttonPressed()) {initialize();}
    } 
    else if (state == INITIATING) {
      //robot would be spinning around here
      start();
    }
    else if (state == PLAYING) {
        if (timeRemaining < 1) {explode();}
        else {
          soundTimer();
          if (buttonPressed()){
            attempts += 1;
            if (attempts >= requiredAttempts) {defuse();}
            else {extendTime();}
          }
        }
      }
}
