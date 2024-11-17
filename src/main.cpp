#include <Arduino.h>
#include <Servo.h>

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
const int beam_data2 = 10; //Not Used

//Extra
const int gp17 = 17;
const int gp16 = 16;
const int gp18 = 18;
const int gp19 = 19;
const int gp20 = 20;
const int gp21 = 21;
const int gp22 = 22;
const int gp26 = 26;
const int gp27 = 27;

//beam states
int sensorState = 0, lastState=0;

//initial button states
int start_flag = 1;
int kill_motor_flag = 1;

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
  // digitalWrite(beam_data, HIGH); // turn on the pullup
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

void servoRelease(){
  myservo.write(45);
}

void servoArm(){
  myservo.write(80);
}

void servoCycleBall(){
  servoRelease();
  delay(1000);
  servoArm();
  delay(1000);
}

bool isGolfBall(){
  sensorState = digitalRead(beam_data);
  if (sensorState == LOW) {     
    return true;
  } 
  else{
    return false;
  }
  // // read the state of the pushbutton value:
  // sensorState = digitalRead(beam_data);
  // // check if the sensor beam is blocked
  // // if it is, the sensorState is LOW:
  // if (sensorState == LOW) {     
  //   // turn LED on:
  //   digitalWrite(led, HIGH);  
  // } 
  // else {
  //   // turn LED off:
  //   digitalWrite(led, LOW); 
  // }
  
  // if (sensorState && !lastState) {
  //   Serial.println("Unbroken");
  // } 
  // if (!sensorState && lastState) {
  //   Serial.println("Broken");
  // }
  // lastState = sensorState;
}

void setup() {
  // Start serial communication
  Serial.begin(115200);
  initializeRelays();
  initializeMotor();
  initializeBuck();
  initializeBeam();
  initializeServo();
  initializeLED();
  initializeButtonBox();
  servoArm();

  

  blinkLED(4, 200);
  Serial.println("Setup Complete");

  digitalWrite(pico_readyLED, HIGH);
  Serial.println(start_flag);
  while(start_flag!=0){
    delay(20);
    start_flag = digitalRead(start_button);
    Serial.println(start_flag);
  }
  runMotor(1);
  start_flag = 0;
  delay(1000);
}



void loop() {
  
  if (isGolfBall()){
    digitalWrite(beam_breakLED, HIGH);
    stopMotor();

  }
  else{
    digitalWrite(beam_breakLED, LOW);
    runMotor(1);
  }
  
  
  if (kill_motor_flag == 0 && isGolfBall()){
    servoCycleBall();
  }

  if (start_flag == 0){
    delay(1000);
    start_flag = 1;
    while (start_flag == 1){
      start_flag = digitalRead(start_button);
      delay(20);
    }
    delay(400);
  }
  
  delay(20);
  kill_motor_flag = digitalRead(kill_motor_button);
  start_flag = digitalRead(start_button);
}