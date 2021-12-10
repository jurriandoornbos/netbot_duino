 
#ifndef AWESOME_LIB_H
#define AWESOME_LIB_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h> //1.4.1
#include <Adafruit_PWMServoDriver.h> //2.4.0
#include <QTRSensors.h> // 4.0.0

#define SERVOMIN  88  // This is the 'minimum' pulse length count (out of 4096) (org=150)
#define SERVOMAX  320 // This is the 'maximum' pulse length count (out of 4096) (org=600)
#define USMIN  600    // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 (org=600)
#define USMAX  2400   // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600(org=2400)
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates (org=50)

#define WHEEL_STABLE 340
#define WHEEL_FRONT 230  
#define WHEEL_BACK -230 

#define NUM_FOLLOW_LINE             4  // number of sensors used
#define NUM_SAMPLES_PER_FOLLOW_LINE  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             4  // emitter is controlled by digital pin 4

#define echoPin 3     // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 2     //attach pin D3 Arduino to pin Trig of HC-SR04


const int buzzer = 9; //buzzer to arduino pin 9

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// defines variables
//long duration; // variable for the duration of sound wave travel
//int distance; // variable for the distance measurement
uint8_t L_servo = 0; // our servo # counter
uint8_t R_servo = 1; // our servo # counter

QTRSensors qtra;
unsigned int sensorValues[NUM_FOLLOW_LINE];


void default_config_setup() {
  Serial.begin(9600);
  Serial.println("Starting Serial Communication.");

  // Config buzzer
  pinMode(buzzer, OUTPUT);

  if (tcs.begin()) {
    Serial.println("Found color sensor");
  } else {
    Serial.println("No color sensor found ... check your connections");
  }

  //Start PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
  Serial.println("Starting PWM PCA9685.");

  // Initialize sonar pins
  pinMode(trigPin, OUTPUT);    // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);     // Sets the echoPin as an INPUT
  Serial.println("Initialising HC-SR04 trig and echo pins.");

  
  Serial.println("Starting follow line calibration. Get ready");
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){0, 1, 2, 3}, NUM_FOLLOW_LINE);
  qtra.setEmitterPin(EMITTER_PIN);
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_FOLLOW_LINE; i++)
  {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_FOLLOW_LINE; i++)
  {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

// time in seconds
void wait(float time) {
  delay(time*1000);
}

// time in seconds 
void buzzer_sound(float time) {
  tone(buzzer, 1000); // Send 1KHz sound signal...
  wait(time);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  wait(time);        // ...for 1sec
}


float* read_rgb_sensor() {
  float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED
  static float color[3];
  color[0] = red;
  color[1] = green;
  color[2] = blue;
  return color;
}

bool is_red(float* color) {
  return false;
}

bool is_blue(float* color) {
  return false;
}
  
bool is_green(float* color) {
  return false;
}

bool is_purple(float* color) {
  return false;
}


float get_sonar_distance(){
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement
  
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  return duration * 0.034 / 2;       // Speed of sound wave divided by 2 (go and back)
}

void stop_robot(void) {
  pwm.setPWM(0, 0, WHEEL_STABLE);
  pwm.setPWM(1, 0, WHEEL_STABLE);
}

void go_to_front(void) {
  pwm.setPWM(0, 0, WHEEL_STABLE + WHEEL_FRONT);
  pwm.setPWM(1, 0, WHEEL_STABLE + WHEEL_FRONT);
}

void go_to_back(void) {
  pwm.setPWM(0, 0, WHEEL_STABLE + WHEEL_BACK);
  pwm.setPWM(1, 0, WHEEL_STABLE + WHEEL_BACK);
}

void go_to_right(void) {
  pwm.setPWM(0, 0, WHEEL_STABLE + WHEEL_FRONT);
  pwm.setPWM(1, 0, WHEEL_STABLE + WHEEL_BACK);
}

void go_to_left(void) {
  pwm.setPWM(0, 0, WHEEL_STABLE + WHEEL_BACK);
  pwm.setPWM(1, 0, WHEEL_STABLE + WHEEL_FRONT);
}


uint16_t read_line_black_position(void) {
  return qtra.readLineBlack(sensorValues);
}

void follow_line(void) {




}



#endif  // AWESOME_LIB_H
