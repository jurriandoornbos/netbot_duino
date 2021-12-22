 
#ifndef AWESOME_LIB_H
#define AWESOME_LIB_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include <QTRSensors.h>


#define SERVOMIN  88  // This is the 'minimum' pulse length count (out of 4096) (org=150)
#define SERVOMAX  320 // This is the 'maximum' pulse length count (out of 4096) (org=600)
#define USMIN  600    // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150 (org=600)
#define USMAX  2400   // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600(org=2400)
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates (org=50)

#define DEFAULT_WHEEL_STABLE 340 // Value of stable point if none found
#define WHEEL_FRONT 230  
#define WHEEL_BACK -230 
int left_wheel_front_dir {1}; // Dir of the wheel when going to front. Change to -1 if opposite dir
int right_wheel_front_dir {-1}; // Dir of the wheel when going to front. Change to -1 if opposite dir
int left_wheel_stable_point {DEFAULT_WHEEL_STABLE}; //350  robot 15 bieng a very special boy
int right_wheel_stable_point {DEFAULT_WHEEL_STABLE}; //270 robot 15  (340 is stable, but too quick for the left motor to keep up)
volatile int left_wheel_count = 0; // Count for the odometry
volatile int right_wheel_count = 0; // Count for the odometry
volatile int left_wheel_dir = 0; // Direction for the odometry
volatile int right_wheel_dir = 0; // Direction for the odometry


#define BLACK_LINE_THRESHOLD 800 // Value of black line treshold
#define NUM_FOLLOW_LINE             4  // number of sensors used
#define NUM_SAMPLES_PER_FOLLOW_LINE  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             4  // emitter is controlled by digital pin 4

#define PROX_SENSOR_LEFT 3   // Pin for the odometry sensor in the left wheel 
#define PROX_SENSOR_RIGHT 2    // Pin for the odometry sensor in the right wheel 

#define echoPin 6     // attach pin D6 Arduino to pin Echo of HC-SR04
#define trigPin 5     //attach pin D5 Arduino to pin Trig of HC-SR04

const int buzzer = 7; //buzzer to arduino pin 9




bool call_calibrate_rgb_bit_camera = false;
bool call_calibrate_motors = false;

bool proximity_sensor_left_connected {true};
bool proximity_sensor_right_connected {true};


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// defines variables
//long duration; // variable for the duration of sound wave travel
//int distance; // variable for the distance measurement
uint8_t L_servo = 0; // our servo # counter
uint8_t R_servo = 1; // our servo # counter

QTRSensors qtra;
unsigned int sensorValues[NUM_FOLLOW_LINE];

void count_prox_sensor_left(void) {
  left_wheel_count += left_wheel_dir;
}

void count_prox_sensor_right(void) {
  right_wheel_count += right_wheel_dir;
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
  pwm.setPWM(L_servo, 0, left_wheel_stable_point);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point);
}

void go_to_front(void) {
  int8_t l_fw = left_wheel_stable_point + left_wheel_front_dir*WHEEL_FRONT;
  int8_t r_fw = right_wheel_stable_point + right_wheel_front_dir*WHEEL_FRONT;

  pwm.setPWM(L_servo, 0, l_fw);
  pwm.setPWM(R_servo, 0, r_fw);
  left_wheel_dir = 1;
  right_wheel_dir = 1;
}

void go_to_back(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir*WHEEL_BACK);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir*WHEEL_BACK);
  left_wheel_dir = -1;
  right_wheel_dir = -1;  
}

void go_to_right(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir*WHEEL_FRONT);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir*WHEEL_BACK);
  left_wheel_dir = 1;
  right_wheel_dir = -1;
}

void go_to_left(void) {
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + left_wheel_front_dir*WHEEL_BACK);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + right_wheel_front_dir*WHEEL_FRONT);
  left_wheel_dir = -1;
  right_wheel_dir = 1;
}


uint16_t read_line_black_position(void) {
  return qtra.readLineBlack(sensorValues);
}

//some function to give array of true false values and black line position
bool* black_line_array(void) {
  static bool line_array[NUM_FOLLOW_LINE];
  
  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    line_array[i] = sensorValues[i] > BLACK_LINE_THRESHOLD;
  }
  return line_array;
}


void follow_line(int error) {
  int Kp = 3;

  int adjust = Kp*error;

  float L_speed =  left_wheel_front_dir * (WHEEL_FRONT - adjust);
  float R_speed =  right_wheel_front_dir * (WHEEL_FRONT - adjust);
  
  pwm.setPWM(L_servo, 0, left_wheel_stable_point + L_speed);
  pwm.setPWM(R_servo, 0, right_wheel_stable_point + R_speed);
  delay(50);

}


int read_prox_sensor_left(void) {
  return digitalRead(PROX_SENSOR_LEFT);
}

int read_prox_sensor_right(void) {
  return digitalRead(PROX_SENSOR_RIGHT);
}

void overwrite_ir_calibration(void){
  // robot calibration is quite irreliable, so we overwrite it with some default values
  // only works after calibration is executed
  for (int i = 0; i < NUM_FOLLOW_LINE; i++) {
    qtra.calibrationOn.minimum[i] = 600;
    qtra.calibrationOn.maximum[i] = 900;
  }
}


void calibrate_motors(void) {
  delay(500);
  left_wheel_stable_point = left_wheel_stable_point / 1.75;
  right_wheel_stable_point = right_wheel_stable_point / 1.75;
  left_wheel_dir = 1;
  right_wheel_dir = 1;
  bool stop_calibrating_left {false};
  bool stop_calibrating_right {false};
  const int number_zeros_read_stop_calib {5};
  int zeros_left {0};
  int zeros_right {0};

  for (int i = 0; i < 400; i++) {
    stop_robot();

    
    if ( left_wheel_count > 0) {
      proximity_sensor_left_connected = true;
      zeros_left = 0;
    } else if ( left_wheel_count == 0) {
      zeros_left++;
      if (number_zeros_read_stop_calib < zeros_left) {
        stop_calibrating_left = true;
        
      }
    }
    
    if ( right_wheel_count > 0) {
      proximity_sensor_right_connected = true;
      zeros_right = 0;
    } else if ( right_wheel_count == 0) {
      zeros_right++;
      if (number_zeros_read_stop_calib < zeros_right) {
        stop_calibrating_right = true;
      }
    }     

    if (!stop_calibrating_left) {
      left_wheel_stable_point += abs(left_wheel_count);
    } 
    left_wheel_count = 0;
      
    if (!stop_calibrating_right) {
      right_wheel_stable_point += abs(right_wheel_count);
    }
    right_wheel_count = 0;  

    if (stop_calibrating_right && stop_calibrating_left) {
      break;
    }
        
    delay(100);
  }
  if ( proximity_sensor_left_connected) {
    Serial.println("Found left proximity sensor");    
    Serial.print("The stable point for the left wheel is: ");
    Serial.println( left_wheel_stable_point);
  } else {
    Serial.println("Left proximity sensor is not connected!!!");
    left_wheel_stable_point = DEFAULT_WHEEL_STABLE;
  }
  
  if ( proximity_sensor_right_connected) {
    Serial.println("Found right proximity sensor"); 
    Serial.print("The stable point for the right wheel is: ");
    Serial.println(right_wheel_stable_point);    
  } else {
    Serial.println("Right proximity sensor is not connected!!!");
    right_wheel_stable_point = DEFAULT_WHEEL_STABLE;
  }
  
  left_wheel_dir = 0;
  right_wheel_dir = 0;
}

void calibrate_rgb_bit_camera(void) {
    Serial.println("Starting follow line calibration. Get ready");
    delay(500);
    pinMode(13, OUTPUT);
    
    digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
    for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
    {
      qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
    }
    digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  
  
    // print the calibration minimum values measured when emitters were on
    Serial.print("Min:");
    for (int i = 0; i < NUM_FOLLOW_LINE; i++)
    {
      
      Serial.print(qtra.calibrationOn.minimum[i]);
      Serial.print(' ');
    }
    Serial.println();
  
    // print the calibration maximum values measured when emitters were on
    Serial.print("Max:");
    for (int i = 0; i < NUM_FOLLOW_LINE; i++)
    {
      
      Serial.print(qtra.calibrationOn.maximum[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
}




void default_config_setup() {
  Serial.begin(9600);
  Serial.println("Starting Serial Communication.");

  // Proximity sensor
  pinMode(PROX_SENSOR_LEFT,INPUT);
  pinMode(PROX_SENSOR_RIGHT,INPUT);  

  delay(500);
  Serial.println("Starting Odometry");
  attachInterrupt(digitalPinToInterrupt(2), count_prox_sensor_left, RISING);
  attachInterrupt(digitalPinToInterrupt(3), count_prox_sensor_right, RISING);

  
  if (call_calibrate_motors) {
    calibrate_motors();
  }

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
  Serial.println("Initialising HC-SR04 trig and echo pins.");
  pinMode(trigPin, OUTPUT);    // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);     // Sets the echoPin as an INPUT

  // Initialize follow line
  Serial.println("Initialising Follow line pins.");
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]){0, 1, 2, 3}, NUM_FOLLOW_LINE);
  qtra.setEmitterPin(EMITTER_PIN);
  if (call_calibrate_rgb_bit_camera) {
    calibrate_rgb_bit_camera();
  }  
}




#endif  // AWESOME_LIB_H
