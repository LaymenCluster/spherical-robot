#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "12345678"
#define hmc5883l_address  0x1E

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
int packetSize;

int en1 = D1;
const int motor1pin1  = D2;
const int motor1pin2  = D3;
Servo servo;

int offset=0, len, w;

void setup() {
  Serial.begin(9600); /* begin serial for debug */
  Wire.begin(D6, D5); /* join i2c bus with SDA=D6 and SCL=D5 of NodeMCU */
  hmc5883l_init();
  servo.attach(2);
  servo.write(90);
  pinMode(en1, OUTPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  Serial.begin(9600);

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
  Serial.println(hmc5883l_GetHeading());
  servo_control(hmc5883l_GetHeading(), offset);
  packetSize = Udp.parsePacket();
  if (packetSize)
  {
    len = Udp.read(incomingPacket, 255);
    if (len > 0)
    {
      incomingPacket[len] = 0;
    }

    w = atoi(incomingPacket);

    if (w == 0) {
      analogWrite(en1, 0);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, LOW);
    }

    if (w == 1) {
      analogWrite(en1, 520);
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, LOW);
    }

    if (w == 2) {
      analogWrite(en1, 520);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
    }

    if (w == 3) {
      offset = 0;
    }

    if (w == 4) {
      offset = -40;
    }

    if (w == 5) {
      offset = 40;
    }
    
  }
  //delay(150);
}


void servo_control(int orientation, int offset){
  servo.write((orientation-90) + offset);
}

void hmc5883l_init(){   /* Magneto initialize function */
  Wire.beginTransmission(hmc5883l_address);
  Wire.write(0x00);
  Wire.write(0x70); //8 samples per measurement, 15Hz data output rate, Normal measurement 
  Wire.write(0xA0); //
  Wire.write(0x00); //Continuous measurement mode
  Wire.endTransmission();
  delay(500);
}

int hmc5883l_GetHeading(){
  int16_t x, y, z;
  double Heading;
  Wire.beginTransmission(hmc5883l_address);
  Wire.write(0x03);
  Wire.endTransmission();
  /* Read 16 bit x,y,z value (2's complement form) */
  Wire.requestFrom(hmc5883l_address, 6);
  x = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());
  z = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());
  y = (((int16_t)Wire.read()<<8) | (int16_t)Wire.read());
  
  Heading = atan2((double)y, (double)z) + PI/2 + PI/8;
  if (Heading>2*PI) /* Due to declination check for >360 degree */
   Heading = Heading - 2*PI;
  if (Heading<0)    /* Check for sign */
   Heading = Heading + 2*PI;
  return (Heading* 180 / PI);/* Convert into angle and return */
}
