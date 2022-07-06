#include <IBusBM.h>
#include <Servo.h>

Servo myservo;
IBusBM ibus;
// 'en' for speed, 'a' and 'b' for direction
// 'xPin' for controlling pendulum angle
int en = 6, a = 7, b = 8, xPin = A0;
int CH1 = 0; // tilt
int CH3 = 0; // Acceleration
int CH6 = 0; // direction
int d = 0, throttle = 0, tilt = 0;

void setup() {

  Serial.begin(115200);
  ibus.begin(Serial);
  myservo.attach(9);
  myservo.write(100);
  pinMode(a, OUTPUT); pinMode(b, OUTPUT);
  
}

void loop() {

  int reading = analogRead(xPin);
  CH1 = ibus.readChannel(0);
  CH3 = ibus.readChannel(2);
  CH6 = ibus.readChannel(5);
  Serial.print(reading);
  tilt = CH1;
  Serial.print(" tilt = ");
  Serial.print(tilt);

  throttle = CH3;
  Serial.print(" throttle = ");
  Serial.print(throttle);

  d = CH6;
  Serial.print(" direction = ");
  Serial.println(d);

}
