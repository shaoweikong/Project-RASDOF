#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();



// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// 0,300 = 90
// 0,500 = 180


#define SERVOMIN 120 //0 This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  510 //550 This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;



void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}





void loop() {


  int angle2 =180;
  int pulse2 = map(angle2, 0 , 180, SERVOMIN, SERVOMAX);
  // Drive each servo one at a time using setPWM()
  pwm.setPWM(2 , 0, pulse2);
  delay(500);

  int angle3 =160;
  int pulse3 = map(angle3, 0 , 180, SERVOMIN, SERVOMAX);
  // Drive each servo one at a time using setPWM()
  pwm.setPWM(3 , 0, pulse3);

  



  



}
