// Header Files
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// Defines Servo Limits via PWM
#define SERVOMIN 120 //0 This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  510 //550 This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


// Variables
uint8_t servonum = 1;
char operation; // Holds operation (R, W, ...)
char mode; // Holds the mode (D, A)
int pin_number; // Holds the pin number
int digital_value; // Holds the digital value
int analog_value; // Holds the analog value
int value_to_write; // Holds the value that we want to write
int wait_for_transmission = 5; // Delay in ms in order to receive the serial data


// Servo Number Instantiation
//Servo SERVO1; 
int SERVO1_PIN = 1; // this is where the servo is attached to on our board
int SERVO2_PIN = 2;
int SERVO3_PIN = 3;
int SERVO4_PIN = 4;
int SERVO5_PIN = 5;
int SERVO6_PIN = 6;


void setup() {
  Serial.begin(9600); // Serial Port at 9600 baud
  Serial.setTimeout(500); // Instead of the default 1000ms, in order to speed up the Serial.parseInt() 
  pwm.begin();
  
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}



void servo_write(int pin_number, int servo_value){
    /*
     * Performs a servo write on pin_number with the servo_value
     * The value must be 0 to 180 (might change depending on servo)
     */
     
     if (pin_number==SERVO1_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       //SERVO1.write(servo_value);
       pwm.setPWM(SERVO1_PIN, 0, pulse);
       //delay(10);
     }

     else if (pin_number==SERVO2_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(SERVO2_PIN, 0, pulse);
       //delay(10);
     }

     else if (pin_number==SERVO3_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(SERVO3_PIN, 0, pulse);
       //delay(10);
     }

     else if (pin_number==SERVO4_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(SERVO4_PIN, 0, pulse);
       //delay(10);
     }
     
     else if (pin_number==SERVO5_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(SERVO5_PIN, 0, pulse);
       //delay(10);
     } 
     
     else if (pin_number==SERVO6_PIN)
     {
      int angle = servo_value;
      int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
       pwm.setPWM(SERVO6_PIN, 0, pulse);
       //delay(10);
     } 

     
//  int angle = 45;
//  int pulse = map(angle, 0 , 180, SERVOMIN, SERVOMAX);
//  // Drive each servo one at a time using setPWM()
//  pwm.setPWM(0 , 0, pulse);
//  delay(1000);
//  
}



void loop() {

    // Check if characters available in the buffer
    if (Serial.available() > 0) 
    {
        // parse information
        // courtesy of lekum 
        operation = Serial.read();
        delay(wait_for_transmission); // If not delayed, second character is not correctly read
        mode = Serial.read();
        pin_number = Serial.parseInt(); // Waits for an int to be transmitted
        
        if (Serial.read()==':')
        {
            value_to_write = Serial.parseInt(); // Collects the value to be written
        }

        // if we recieve proper input write servo
        if (operation == 'W')
        {
            if (mode == 'S')
            {
                servo_write(pin_number, value_to_write);
            }
        }
        
    }







   

}
