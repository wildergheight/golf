#include <Arduino.h>
#include <Servo.h>
#include "sms812.h"
#include "Ultrasonic.h"
#include "arduinoFFT.h"
#include <Wire.h>
#include "Adafruit_VL6180X.h"

// FFT Definitions and Constants
#define SAMPLES 64             
#define SAMPLING_FREQ 10000    
#define ANALOG_PIN A1          
#define LED_PIN LED_BUILTIN    
#define MAGNITUDE_THRESHOLD 35 

double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

TwoWire myWire(20, 21); // SDA = GP0, SCL = GP1
Adafruit_VL6180X vl = Adafruit_VL6180X();

// Define the LED pin
const int led = 25;  // Pico's built-in LED

// Define the motor control pins
const int motorPWM = 13;  // Changed to different PWM pin to avoid conflict with LED
const int motorINA = 11;    // INA pin
const int motorINB = 28;    // INB pin
const int motorENA = 12;  //enable
const int motorCS = 14; //current sense
const int motorENB = 15;

//6V Buck Converter
const int buckPG = 6;
const int buckEN = 7;

//Relays
const int relay1 = 4;
const int relay2 = 5;

//ButtonPCB
const int start_button = 0;
const int kill_motor_button = 1;
const int beam_breakLED = 2;
const int pico_readyLED = 3;

//Servo
Servo myservo;
const int servo = 8;

//Beam Break Sensors
const int beam_data = 9;
const int beam_data2 = 10; 

//ultrasonic Sensor
Ultrasonic ultrasonic(10);

//Dist Sensor
const int dist_data = 26;

//Extra
const int gp17 = 17;
const int gp16 = 16;
const int gp18 = 18;
const int gp19 = 19;
const int gp20 = 20;
const int gp21 = 21;
const int gp22 = 22;
// const int gp26 = 26;
const int gp27 = 27;

//beam states
int sensorState = 0, lastState=0;

//initial button states
int start_flag = 1;
int kill_motor_flag = 1;
int main_button = 1;
unsigned long lastServoTime = 0;

//distance sensor init
// SharpDistSensor mySensor(GP2Y0A41SK0F, dist_data);

void initializeRelays(){
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);

  digitalWrite(relay1, HIGH);
  sleep_ms(100);
  digitalWrite(relay1, LOW);

  sleep_ms(500);

  digitalWrite(relay2, HIGH);
  sleep_ms(100);
  digitalWrite(relay2, LOW);
}

void cycleRelays(){
  digitalWrite(relay2, HIGH);
  sleep_ms(500);
  digitalWrite(relay2, LOW);
  sleep_ms(500);
  digitalWrite(relay1, HIGH);
  sleep_ms(500);
  digitalWrite(relay1, LOW);
}

void initializeMotor(){
  // Configure motor control pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorINA, OUTPUT);
  pinMode(motorINB, OUTPUT);
  pinMode(motorENA, OUTPUT);
  pinMode(motorENB, OUTPUT);
  pinMode(motorCS, INPUT);  // Current sense pin as input

  // Set initial motor direction (for example)
  digitalWrite(motorINA, HIGH);
  digitalWrite(motorINB, LOW);
  digitalWrite(motorENA, HIGH);    // Enable motor driver
  digitalWrite(motorENB, HIGH);    // Enable motor driver

}

void initializeBuck(){
  //Configure and enable 6v buck converter
  pinMode(buckEN, OUTPUT);
  pinMode(buckPG, INPUT_PULLUP);   //Power Good Input

  digitalWrite(buckEN, HIGH);
}

void initializeBeam(){
  pinMode(beam_data, INPUT_PULLUP);
  pinMode(beam_data2, INPUT_PULLUP);
  // digitalWrite(beam_data, HIGH); // turn on the pullup
}

void initializeUltrasonic(){
  // Initialize the distance sensor
  // mySensor.begin();
  pinMode(beam_data2, INPUT_PULLUP);
}

void initializeTOF(){
  myWire.begin();

  Serial.println("Adafruit VL6180x test!");
  if (!vl.begin(&myWire)) {
    Serial.println("Failed to find sensor");
  }
  Serial.println("Sensor found!");
}

// void checkTOF(){
//   float lux = vl.readLux(VL6180X_ALS_GAIN_5);

//   uint8_t range = vl.readRange();
//   uint8_t status = vl.readRangeStatus();

//   if (status == VL6180X_ERROR_NONE) {
//     Serial.print("Range: "); Serial.println(range);

//     if (range < 50) {
//       digitalWrite(LED_PIN, HIGH);
//     } else {
//       digitalWrite(LED_PIN, LOW);
//     }
//   } else {
//     digitalWrite(LED_PIN, LOW); // turn off LED on error
//   }

//   // Print error messages
//   if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
//     Serial.println("System error");
//   }
//   else if (status == VL6180X_ERROR_ECEFAIL) {
//     Serial.println("ECE failure");
//   }
//   else if (status == VL6180X_ERROR_NOCONVERGE) {
//     Serial.println("No convergence");
//   }
//   else if (status == VL6180X_ERROR_RANGEIGNORE) {
//     Serial.println("Ignoring range");
//   }
//   else if (status == VL6180X_ERROR_SNR) {
//     Serial.println("Signal/Noise error");
//   }
//   else if (status == VL6180X_ERROR_RAWUFLOW) {
//     Serial.println("Raw reading underflow");
//   }
//   else if (status == VL6180X_ERROR_RAWOFLOW) {
//     Serial.println("Raw reading overflow");
//   }
//   else if (status == VL6180X_ERROR_RANGEUFLOW) {
//     Serial.println("Range reading underflow");
//   }
//   else if (status == VL6180X_ERROR_RANGEOFLOW) {
//     Serial.println("Range reading overflow");
//   }

//   delay(50);
// }

bool ballRequested(){
    int sensorValue = analogRead(dist_data);  // Read raw ADC value
    float voltage = sensorValue * (3.3 / 1023.0);  // Convert to voltage (assuming 10-bit ADC)
    
    // Convert voltage to distance using the sensor's characteristic curve (datasheet)
    float distance = 27.86 / (voltage - 0.42); // Example formula, may need adjustment
    
    if (distance > 5 && distance <= 13) {  // Typical range for GP2Y0A41SK0F is 4-30 cm
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      return true;
    } else {
      Serial.println("Out of range");
      return false;
    }  
}

void initializeLED(){
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);  // Turn on the LED
}

void initializeServo(){
  myservo.attach(servo);
}

void initializeButtonBox(){
  pinMode(start_button, INPUT_PULLUP);
  pinMode(kill_motor_button, INPUT_PULLUP);
  pinMode(gp21, INPUT_PULLUP);
  pinMode(beam_breakLED, OUTPUT);
  pinMode(pico_readyLED, OUTPUT);

  // digitalWrite(beam_breakLED, HIGH);
  // digitalWrite(pico_readyLED, HIGH);
}

void blinkLED(int cycles, int duration_ms){
  digitalWrite(led, LOW);
  for(int i=0;i<cycles;i++){
    digitalWrite(led, HIGH);
    sleep_ms(duration_ms);
    digitalWrite(led, LOW);
    sleep_ms(duration_ms);
  }
}

void runMotor(float percentage){
  int dutyCycle = 255*percentage;
  analogWrite(motorPWM, dutyCycle);
}

void stopMotor(){
  analogWrite(motorPWM, 0);
}

void smoothServoMove(Servo &servo, int startAngle, int endAngle, int stepDelay) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      servo.write(angle);
      delay(stepDelay);  // Delay between each step
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      servo.write(angle);
      delay(stepDelay);  // Delay between each step
    }
  }
}

void servoRelease(){
  myservo.write(45);
  // int currentAngle = myservo.read();  // Get the current position of the servo
  // smoothServoMove(myservo, currentAngle, 40, 10);  // Move smoothly to 45 degrees
}

void servoArm(){
  myservo.write(85);
  // int currentAngle = myservo.read();  // Get the current position of the servo
  // smoothServoMove(myservo, currentAngle, 80, 10);  // Move smoothly to 80 degrees
}

void servoCycleBall(){
  servoRelease();
  delay(1000);
  servoArm();
  delay(1000);
}

bool isKillMotor(){
  sensorState = digitalRead(kill_motor_button);
  if (sensorState == LOW) {     
    return true;
  } 
  else{
    return false;
  }
}

bool isStart(){
  sensorState = digitalRead(start_button);
  if (sensorState == LOW) {     
    return true;
  } 
  else{
    return false;
  }
}

// bool isGolfBall(){


//   long RangeInCentimeters;
//   RangeInCentimeters = ultrasonic.read();
//   if (RangeInCentimeters > 0 && RangeInCentimeters <=6){
//     digitalWrite(beam_breakLED, HIGH);
//     return true;
//   }
//   else{
//     digitalWrite(beam_breakLED, LOW);
//     return false;
//   }

// }
#define HISTORY_SIZE 1000

bool isGolfBall() {
  // static bool lastState = false;
  // static bool readingHistory[HISTORY_SIZE] = {false};
  // static int index = 0;
  // static int passCount = 0;

  // // Take a reading
  // long range = ultrasonic.read();
  // bool isPass = (range > 0 && range <= 3);

  // // Update the history ring buffer
  // if (readingHistory[index]) passCount--;       // remove old value from count
  // if (isPass) passCount++;                      // add new value to count
  // readingHistory[index] = isPass;
  // index = (index + 1) % HISTORY_SIZE;

  // // Determine new state based on majority
  // bool newState = (passCount > HISTORY_SIZE / 2);

  // // Only change the LED if state changes
  // if (newState != lastState) {
  //   digitalWrite(beam_breakLED, newState ? HIGH : LOW);
  //   lastState = newState;
  // }

  // return lastState;

  // float lux = vl.readLux(VL6180X_ALS_GAIN_5);
  bool isBallDetected = false;

  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);

    if (range <= 50) {
      // digitalWrite(LED_PIN, HIGH);
      isBallDetected = true; // Ball detected if range is less than or equal to 50
    } else {
      // digitalWrite(LED_PIN, LOW);
      isBallDetected = false; // No ball detected if range is greater than 50
    }
  } else {
    // digitalWrite(LED_PIN, LOW); // turn off LED on error
    isBallDetected = false; // No ball detected if range is greater than 50
  }

  // Print error messages
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }

  return isBallDetected;

}


bool isServoButton(){     //Remove when confirmed not needed
  sensorState = digitalRead(gp21);
  if (sensorState == LOW) {     
    return true;
  } 
  else{
    return false;
  }
}

bool isBallHit(){
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(ANALOG_PIN);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQ);
  }

  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  bool hitDetected = false;
  double hitFrequency = 0;
  double hitMagnitude = 0;

  for (int i = 2; i < SAMPLES / 2; i++) {
    double freq = (i * 1.0 * SAMPLING_FREQ) / SAMPLES;

    if (freq >= 2000 && freq <= 3000 && vReal[i] > MAGNITUDE_THRESHOLD) {
      hitFrequency = freq;
      hitMagnitude = vReal[i];
      hitDetected = true;

      // Serial.print("Freq: ");
      // Serial.print(freq);
      // Serial.print("  vReal: ");
      // Serial.println(vReal[i]);
      return true;
    }
  }
  return false;
}
void testDistance(){
  while(1){
    if (ballRequested()){
      digitalWrite(led, HIGH);
    }
    else{
      digitalWrite(led, LOW);
    }
    delay(20);
  }
}

void setup() {
  // Start serial communication
  Serial.begin(115200);
  // while (!Serial && millis() < 10000UL);
  // Serial.println("started");
  initializeRelays();
  initializeMotor();
  initializeBuck();
  // initializeBeam();
  initializeUltrasonic();
  initializeServo();
  initializeLED();
  initializeButtonBox();
  initializeTOF();
  servoArm();

  // blinkLED(4, 200);
  Serial.println("Setup Complete");

  digitalWrite(pico_readyLED, HIGH);
  while(!isStart()){
    // delay(20);
    Serial.println("Waiting for Start Button");
    main_button = digitalRead(gp21);
    blinkLED(1, 50);
    // Serial.println(main_button);
    // start_flag = digitalRead(start_button);
    // Serial.println(start_flag);
  }
  Serial.println("Button Pressed. Starting Motor");
  runMotor(1);
  // start_flag = 0;
  delay(1000);

  // testDistance();
}



void loop() {
  if (isGolfBall()) {
    stopMotor();
  } else {
    runMotor(1);
  }

  // Check if a ball is requested and enough time has passed since last servo cycle
  if (ballRequested()) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastServoTime >= 3000) {
      digitalWrite(pico_readyLED, LOW);
      servoCycleBall();
      lastServoTime = currentMillis;
      digitalWrite(pico_readyLED, HIGH);
    }
  }

  // if (isBallHit()) {
  //   digitalWrite(pico_readyLED, LOW);
  //   servoCycleBall();
  //   digitalWrite(pico_readyLED, HIGH);
  // }

  if (isKillMotor()) {
    stopMotor();
    delay(1000);
    while (!isKillMotor()) {
      if (ballRequested() || isBallHit()) {
        digitalWrite(pico_readyLED, LOW);
        servoCycleBall();
        digitalWrite(pico_readyLED, HIGH);
      }
    }
    digitalWrite(pico_readyLED, LOW);
    delay(2000);
    digitalWrite(pico_readyLED, HIGH);
  }

  if (isStart()) {
    stopMotor();
    delay(1000);
    while (!isStart()) {
      if (isBallHit()) {
        blinkLED(4, 50);
      }
    }
    digitalWrite(pico_readyLED, LOW);
    delay(2000);
    digitalWrite(pico_readyLED, HIGH);
  }
}
