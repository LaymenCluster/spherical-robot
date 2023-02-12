#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define WIFI_SSID "****"
#define WIFI_PASSWORD "****"

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
int packetSize;

int en = D1;
const int motor1pin1  = D2;
const int motor1pin2  = D3;
Servo servo;
const int xPin = A0;

int offset = 0, len, w = 0;

void setup() {
  Serial.begin(9600); /* begin serial for debug */

  servo.attach(2);
  servo.write(100);
  pinMode(en, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort);
}

void loop() {
  int reading = analogRead(xPin);
  servo_control(offset, reading);
  packetSize = Udp.parsePacket();
  if (packetSize)
  {
    len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
      w = atoi(incomingPacket);
    }

    if (w == 0) {
      motor_control();
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW);
    }

    if (w == 1) {
      analogWrite(en, 450);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
    }

    if (w == 2) {
      analogWrite(en, 450);
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, LOW);
    }

    if (w == 3) {
      offset = 0;
    }

    if (w == 4) {
      offset = 35;
    }

    if (w == 5) {
      offset = -35;
    }
  }
}


void servo_control(int offset, int reading) {
  int x;
  if ((reading-470) < -20) {
    x = 5;
  }
  else if ((reading-470) > 20) {
    x = -5;
  }
  else {
    x = 0;
  }
  Serial.println(reading);
  servo.write(100 + offset + x);
  delay(100);
}

void motor_control() {
  for (int i = 400; i > 50; i-=10) {
    analogWrite(en, i);
    delay(30);
  }
  analogWrite(en, 0);
}
