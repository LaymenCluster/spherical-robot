#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Servo.h>

#define WIFI_SSID "****"
#define WIFI_PASSWORD "****"

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = D5;
const uint8_t sda = D6;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 1638;
const uint16_t GyroScaleFactor = 13;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
int packetSize;

int en = D1;
const int motor1pin1  = D2;
const int motor1pin2  = D3;
Servo servo;

int offset = 0, len, w = 0;
int ary[] = {0, 0};

void setup() {
  Serial.begin(9600); /* begin serial for debug */
  Wire.begin(sda, scl);
  MPU6050_Init();

  servo.attach(2);
  servo.write(100);
  pinMode(en, OUTPUT);
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
  mpu6050_getAngle();
  servo_control(offset);
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
      analogWrite(en, 300);
      digitalWrite(motor1pin1, LOW);
      digitalWrite(motor1pin2, HIGH);
    }

    if (w == 2) {
      analogWrite(en, 300);
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


void servo_control(int offset, int startAngle) {
  int x;
  if (ary[1] < -2) {
    x = -20;
  }
  else if (ary[1] > 2) {
    x = 20;
  }
  else {
    x = 0;
  }

  servo.write(100 + offset + t);

}

void motor_control() {
  for (int i = 300; i > 5; i--) {
    analogWrite(en, i);
    delay(30);
  }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}


void mpu6050_getAngle() {
  double Ax, Ay;

  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  //divide each with their sensitivity scale factor
  Ax = (double)AccelX / AccelScaleFactor;
  Ay = (double)AccelY / AccelScaleFactor;
  //  Az = (double)AccelZ/AccelScaleFactor;
  //  T = (double)Temperature/340+36.53; //temperature formula
  //  Gx = (double)GyroX/GyroScaleFactor;
  //  Gy = (double)GyroY/GyroScaleFactor;
  //  Gz = (double)GyroZ/GyroScaleFactor;
  ary[0] = Ax;
  ary[1] = Ay;
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print("\n");
  delay(200);
  //  Serial.print(" Az: "); Serial.print(Az);
  //  Serial.print(" T: "); Serial.print(T);
  //  Serial.print(" Gx: "); Serial.print(Gx);
  //  Serial.print(" Gy: "); Serial.print(Gy);
  //  Serial.print(" Gz: "); Serial.println(Gz);
}
