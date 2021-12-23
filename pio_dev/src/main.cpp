  /*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <awesome_lib.h>

void setup(){
  // calibration functions:

  // calibrate all but wheels and camera
  call_calibrate_rgb_bit_camera = true;
  call_calibrate_motors = false;
  default_config_setup(); 

  // Hacks to get overwrite the calibration data for the IR line:
  overwrite_ir_calibration();

  // Overwriting the stable_points for the wheels:
  int left_wheel_stable_point{350};
  int right_wheel_stable_point{DEFAULT_WHEEL_STABLE};
 
};

  // all implemented functions in aweseome_lib.h:
  // helpers:
void loop(){

  uint16_t black = read_line_black_position();
  int16_t err = black-1500;


  // Pcontrol(err);

  float* colours = read_rgb_sensor();

  float R = colours[0];
  float G = colours[1];
  float B = colours[2];
  Serial.print("R: ");
  Serial.print(R);
  Serial.print(" ");
  Serial.print(" G: ");
  Serial.print(G);
  Serial.print(" ");
  Serial.print(" B: ");
  Serial.print(B);
  Serial.println(" ");

  if (R>150 && G <100 && B<100) {
    Serial.println("red");
  }

  delay(200);





  
};


