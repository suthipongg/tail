#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define N_MOTOR 3
#define PWMFreq 60
#define OscillatorFrequency 27000000
#define CLOCK 400000

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int pulse = 0;

void setup() {
   Serial.begin(9600);
   Serial.println("16 channel PWM test!");

   pwm.begin();
   //pwm.setOscillatorFrequency(OscillatorFrequency);
   pwm.setPWMFreq(PWMFreq);
   //Wire.setClock(CLOCK);
}

void loop() {
   pulse = Serial.parseInt();
   if (Serial.available()) {
      Serial.println("=====");
      for (uint8_t pwmnum=0; pwmnum < N_MOTOR; pwmnum++) {
         pwm.setPWM(pwmnum, 0, pulse);
      }
      Serial.println(pulse);
   }
}
