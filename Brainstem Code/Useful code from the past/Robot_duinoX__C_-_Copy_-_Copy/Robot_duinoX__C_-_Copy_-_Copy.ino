#ifndef MPU6050_TOCKN_H
#define MPU6050_TOCKN_H

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42

#include <SoftwareSerial.h>
#define tx 3
#define rx 2
#define ThighF 4
#define ThighB 5
#define ThighFBAnalogue 6
#define ThighL 7
#define ThighR 8
#define ThighLRAnalogue 9
#define ShinFBAnalogue 10
#define ShinF 12
#define ShinB 13
//#define FootF 0
//#define FootB 0
//#define ShinCW 0
//#define ShinACW 0
#define mpuTorso A0
#define mpu2 A1
#define mpu3 A2
#define mpu4 A3

class MPU6050{
  public:

  MPU6050(TwoWire &w);
  MPU6050(TwoWire &w, float aC, float gC);

  void begin();

  void writeMPU6050(byte reg, byte data);

  int16_t getRawAccX1(){ return rawAccX1; }; int16_t getRawAccX2(){ return rawAccX2; }; int16_t getRawAccX3(){ return rawAccX3; }; int16_t getRawAccX4(){ return rawAccX4; };
  int16_t getRawAccY1(){ return rawAccY1; }; int16_t getRawAccY2(){ return rawAccY2; }; int16_t getRawAccY3(){ return rawAccY3; }; int16_t getRawAccY4(){ return rawAccY4; };
  int16_t getRawAccZ1(){ return rawAccZ1; }; int16_t getRawAccZ2(){ return rawAccZ2; }; int16_t getRawAccZ3(){ return rawAccZ3; }; int16_t getRawAccZ4(){ return rawAccZ4; };

  int16_t getRawGyroX1(){ return rawGyroX1; }; int16_t getRawGyroX2(){ return rawGyroX2; }; int16_t getRawGyroX3(){ return rawGyroX3; }; int16_t getRawGyroX4(){ return rawGyroX4; };
  int16_t getRawGyroY1(){ return rawGyroY1; }; int16_t getRawGyroY2(){ return rawGyroY2; }; int16_t getRawGyroY3(){ return rawGyroY3; }; int16_t getRawGyroY4(){ return rawGyroY4; };
  int16_t getRawGyroZ1(){ return rawGyroZ1; }; int16_t getRawGyroZ2(){ return rawGyroZ2; }; int16_t getRawGyroZ3(){ return rawGyroZ3; }; int16_t getRawGyroZ4(){ return rawGyroZ4; };

  float getAccX1(){ return accX1; }; float getAccX2(){ return accX2; }; float getAccX3(){ return accX3; }; float getAccX4(){ return accX4; };
  float getAccY1(){ return accY1; }; float getAccY2(){ return accY2; }; float getAccY3(){ return accY3; }; float getAccY4(){ return accY4; };
  float getAccZ1(){ return accZ1; }; float getAccZ2(){ return accZ2; }; float getAccZ3(){ return accZ3; }; float getAccZ4(){ return accZ4; };

  float getGyroX1(){ return gyroX1; }; float getGyroX2(){ return gyroX2; }; float getGyroX3(){ return gyroX3; }; float getGyroX4(){ return gyroX4; };
  float getGyroY1(){ return gyroY1; }; float getGyroY2(){ return gyroY2; }; float getGyroY3(){ return gyroY3; }; float getGyroY4(){ return gyroY4; };
  float getGyroZ1(){ return gyroZ1; }; float getGyroZ2(){ return gyroZ2; }; float getGyroZ3(){ return gyroZ3; }; float getGyroZ4(){ return gyroZ4; };

  float getGyroXoffset1(){ return gyroXoffset1; }; float getGyroXoffset2(){ return gyroXoffset2; }; float getGyroXoffset3(){ return gyroXoffset3; }; float getGyroXoffset4(){ return gyroXoffset4; };
  float getGyroYoffset1(){ return gyroYoffset1; }; float getGyroYoffset2(){ return gyroYoffset2; }; float getGyroYoffset3(){ return gyroYoffset3; }; float getGyroYoffset4(){ return gyroYoffset4; };
  float getGyroZoffset1(){ return gyroZoffset1; }; float getGyroZoffset2(){ return gyroZoffset2; }; float getGyroZoffset3(){ return gyroZoffset3; }; float getGyroZoffset4(){ return gyroZoffset4; };

  void update();

  float getAccAngleX1(){ return angleAccX1; }; float getAccAngleX2(){ return angleAccX2; }; float getAccAngleX3(){ return angleAccX3; }; float getAccAngleX4(){ return angleAccX4; };
  float getAccAngleY1(){ return angleAccY1; }; float getAccAngleY2(){ return angleAccY2; }; float getAccAngleY3(){ return angleAccY3; }; float getAccAngleY4(){ return angleAccY4; };

  float getGyroAngleX1(){ return angleGyroX1; }; float getGyroAngleX2(){ return angleGyroX2; }; float getGyroAngleX3(){ return angleGyroX3; }; float getGyroAngleX4(){ return angleGyroX4; };
  float getGyroAngleY1(){ return angleGyroY1; }; float getGyroAngleY2(){ return angleGyroY2; }; float getGyroAngleY3(){ return angleGyroY3; }; float getGyroAngleY4(){ return angleGyroY4; };
  float getGyroAngleZ1(){ return angleGyroZ1; }; float getGyroAngleZ2(){ return angleGyroZ2; }; float getGyroAngleZ3(){ return angleGyroZ3; }; float getGyroAngleZ4(){ return angleGyroZ4; };

  float getAngleX1(){ return angleX1; }; float getAngleX2(){ return angleX2; }; float getAngleX3(){ return angleX3; }; float getAngleX4(){ return angleX4; };
  float getAngleY1(){ return angleY1; }; float getAngleY2(){ return angleY2; }; float getAngleY3(){ return angleY3; }; float getAngleY4(){ return angleY4; };
  float getAngleZ1(){ return angleZ1; }; float getAngleZ2(){ return angleZ2; }; float getAngleZ3(){ return angleZ3; }; float getAngleZ4(){ return angleZ4; };

  private:

  TwoWire *wire;

  int16_t rawAccX1, rawAccY1, rawAccZ1, rawTemp1,
  rawGyroX1, rawGyroY1, rawGyroZ1;
  int16_t rawAccX2, rawAccY2, rawAccZ2, rawTemp2,
  rawGyroX2, rawGyroY2, rawGyroZ2;
  int16_t rawAccX3, rawAccY3, rawAccZ3, rawTemp3,
  rawGyroX3, rawGyroY3, rawGyroZ3;
  int16_t rawAccX4, rawAccY4, rawAccZ4, rawTemp4,
  rawGyroX4, rawGyroY4, rawGyroZ4;

  float gyroXoffset1, gyroYoffset1, gyroZoffset1;
  float gyroXoffset2, gyroYoffset2, gyroZoffset2;
  float gyroXoffset3, gyroYoffset3, gyroZoffset3;
  float gyroXoffset4, gyroYoffset4, gyroZoffset4;

  float temp1, accX1, accY1, accZ1, gyroX1, gyroY1, gyroZ1;
  float temp2, accX2, accY2, accZ2, gyroX2, gyroY2, gyroZ2;
  float temp3, accX3, accY3, accZ3, gyroX3, gyroY3, gyroZ3;
  float temp4, accX4, accY4, accZ4, gyroX4, gyroY4, gyroZ4;

  float angleGyroX1, angleGyroY1, angleGyroZ1,
  angleAccX1, angleAccY1, angleAccZ1;
  float angleGyroX2, angleGyroY2, angleGyroZ2,
  angleAccX2, angleAccY2, angleAccZ2;
  float angleGyroX3, angleGyroY3, angleGyroZ3,
  angleAccX3, angleAccY3, angleAccZ3;
  float angleGyroX4, angleGyroY4, angleGyroZ4,
  angleAccX4, angleAccY4, angleAccZ4;

  float angleX1, angleY1, angleZ1;
  float angleX2, angleY2, angleZ2;
  float angleX3, angleY3, angleZ3;
  float angleX4, angleY4, angleZ4;

  float interval;
  long preInterval;

  float accCoef, gyroCoef;
};

#endif



MPU6050::MPU6050(TwoWire &w){
  wire = &w;
  accCoef = 0.02f;
  gyroCoef = 0.98f;
}

MPU6050::MPU6050(TwoWire &w, float aC, float gC){
  wire = &w;
  accCoef = aC;
  gyroCoef = gC;
}

void MPU6050::begin(){
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x00);
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX1 = 0;
  angleGyroY1 = 0;
  angleGyroX2 = 0;
  angleGyroY2 = 0;
  angleGyroX3 = 0;
  angleGyroY3 = 0;
  angleGyroX4 = 0;
  angleGyroY4 = 0;
  angleX1 = this->getAccAngleX1();
  angleY1 = this->getAccAngleY1();
  angleX2 = this->getAccAngleX2();
  angleY2 = this->getAccAngleY2();
  angleX3 = this->getAccAngleX3();
  angleY3 = this->getAccAngleY3();
  angleX4 = this->getAccAngleX4();
  angleY4 = this->getAccAngleY4();
  preInterval = millis();
}

void MPU6050::writeMPU6050(byte reg, byte data){
  
  digitalWrite(mpuTorso, LOW);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();

  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, LOW);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
  
  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, LOW);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();

  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, LOW);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(reg);
  wire->write(data);
  wire->endTransmission();
  
}

void MPU6050::update(){
  
  digitalWrite(mpuTorso, LOW);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX1 = wire->read() << 8 | wire->read();
  rawAccY1 = wire->read() << 8 | wire->read();
  rawAccZ1 = wire->read() << 8 | wire->read();
  rawTemp1 = wire->read() << 8 | wire->read();
  rawGyroX1 = wire->read() << 8 | wire->read();
  rawGyroY1 = wire->read() << 8 | wire->read();
  rawGyroZ1 = wire->read() << 8 | wire->read();

  temp1 = (rawTemp1 + 12412.0) / 340.0;

  accX1 = ((float)rawAccX1) / 16384.0;
  accY1 = ((float)rawAccY1) / 16384.0;
  accZ1 = ((float)rawAccZ1) / 16384.0;

  angleAccX1 = atan2(accY1, sqrt(accZ1 * accZ1 + accX1 * accX1)) * 360 / 2.0 / PI;
  angleAccY1 = atan2(accX1, sqrt(accZ1 * accZ1 + accY1 * accY1)) * 360 / -2.0 / PI;

  gyroX1 = ((float)rawGyroX1) / 65.5;
  gyroY1 = ((float)rawGyroY1) / 65.5;
  gyroZ1 = ((float)rawGyroZ1) / 65.5;

  gyroX1 -= gyroXoffset1;
  gyroY1 -= gyroYoffset1;
  gyroZ1 -= gyroZoffset1;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX1 += gyroX1 * interval;
  angleGyroY1 += gyroY1 * interval;
  angleGyroZ1 += gyroZ1 * interval;

  angleX1 = (gyroCoef * (angleX1 + gyroX1 * interval)) + (accCoef * angleAccX1);
  angleY1 = (gyroCoef * (angleY1 + gyroY1 * interval)) + (accCoef * angleAccY1);
  angleZ1 = angleGyroZ1;

  preInterval = millis();
  
/*
 * 
 */
 
  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, LOW);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX2 = wire->read() << 8 | wire->read();
  rawAccY2 = wire->read() << 8 | wire->read();
  rawAccZ2 = wire->read() << 8 | wire->read();
  rawTemp2 = wire->read() << 8 | wire->read();
  rawGyroX2 = wire->read() << 8 | wire->read();
  rawGyroY2 = wire->read() << 8 | wire->read();
  rawGyroZ2 = wire->read() << 8 | wire->read();

  temp2 = (rawTemp2 + 12412.0) / 340.0;

  accX2 = ((float)rawAccX2) / 16384.0;
  accY2 = ((float)rawAccY2) / 16384.0;
  accZ2 = ((float)rawAccZ2) / 16384.0;

  angleAccX2 = atan2(accY2, sqrt(accZ2 * accZ2 + accX2 * accX2)) * 360 / 2.0 / PI;
  angleAccY2 = atan2(accX2, sqrt(accZ2 * accZ2 + accY2 * accY2)) * 360 / -2.0 / PI;

  gyroX2 = ((float)rawGyroX2) / 65.5;
  gyroY2 = ((float)rawGyroY2) / 65.5;
  gyroZ2 = ((float)rawGyroZ2) / 65.5;

  gyroX2 -= gyroXoffset2;
  gyroY2 -= gyroYoffset2;
  gyroZ2 -= gyroZoffset2;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX2 += gyroX2 * interval;
  angleGyroY2 += gyroY2 * interval;
  angleGyroZ2 += gyroZ2 * interval;

  angleX2 = (gyroCoef * (angleX2 + gyroX2 * interval)) + (accCoef * angleAccX2);
  angleY2 = (gyroCoef * (angleY2 + gyroY2 * interval)) + (accCoef * angleAccY2);
  angleZ2 = angleGyroZ2;

  preInterval = millis();
  
/*
 * 
 */
 
  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, LOW);
  digitalWrite(mpu4, HIGH);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX3 = wire->read() << 8 | wire->read();
  rawAccY3 = wire->read() << 8 | wire->read();
  rawAccZ3 = wire->read() << 8 | wire->read();
  rawTemp3 = wire->read() << 8 | wire->read();
  rawGyroX3 = wire->read() << 8 | wire->read();
  rawGyroY3 = wire->read() << 8 | wire->read();
  rawGyroZ3 = wire->read() << 8 | wire->read();

  temp3 = (rawTemp3 + 12412.0) / 340.0;

  accX3 = ((float)rawAccX3) / 16384.0;
  accY3 = ((float)rawAccY3) / 16384.0;
  accZ3 = ((float)rawAccZ3) / 16384.0;

  angleAccX3 = atan2(accY3, sqrt(accZ3 * accZ3 + accX3 * accX3)) * 360 / 2.0 / PI;
  angleAccY3 = atan2(accX3, sqrt(accZ3 * accZ3 + accY3 * accY3)) * 360 / -2.0 / PI;

  gyroX3 = ((float)rawGyroX3) / 65.5;
  gyroY3 = ((float)rawGyroY3) / 65.5;
  gyroZ3 = ((float)rawGyroZ3) / 65.5;

  gyroX3 -= gyroXoffset3;
  gyroY3 -= gyroYoffset3;
  gyroZ3 -= gyroZoffset3;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX3 += gyroX3 * interval;
  angleGyroY3 += gyroY3 * interval;
  angleGyroZ3 += gyroZ3 * interval;

  angleX3 = (gyroCoef * (angleX3 + gyroX3 * interval)) + (accCoef * angleAccX3);
  angleY3 = (gyroCoef * (angleY3 + gyroY3 * interval)) + (accCoef * angleAccY3);
  angleZ3 = angleGyroZ3;

  preInterval = millis();
  
/*
 * 
 */
  digitalWrite(mpuTorso, HIGH);
  digitalWrite(mpu2, HIGH);
  digitalWrite(mpu3, HIGH);
  digitalWrite(mpu4, LOW);
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX4 = wire->read() << 8 | wire->read();
  rawAccY4 = wire->read() << 8 | wire->read();
  rawAccZ4 = wire->read() << 8 | wire->read();
  rawTemp4 = wire->read() << 8 | wire->read();
  rawGyroX4 = wire->read() << 8 | wire->read();
  rawGyroY4 = wire->read() << 8 | wire->read();
  rawGyroZ4 = wire->read() << 8 | wire->read();

  temp4 = (rawTemp4 + 12412.0) / 340.0;

  accX4 = ((float)rawAccX4) / 16384.0;
  accY4 = ((float)rawAccY4) / 16384.0;
  accZ4 = ((float)rawAccZ4) / 16384.0;

  angleAccX4 = atan2(accY4, sqrt(accZ4 * accZ4 + accX4 * accX4)) * 360 / 2.0 / PI;
  angleAccY4 = atan2(accX4, sqrt(accZ4 * accZ4 + accY4 * accY4)) * 360 / -2.0 / PI;

  gyroX4 = ((float)rawGyroX4) / 65.5;
  gyroY4 = ((float)rawGyroY4) / 65.5;
  gyroZ4 = ((float)rawGyroZ4) / 65.5;

  gyroX4 -= gyroXoffset4;
  gyroY4 -= gyroYoffset4;
  gyroZ4 -= gyroZoffset4;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX4 += gyroX4 * interval;
  angleGyroY4 += gyroY4 * interval;
  angleGyroZ4 += gyroZ4 * interval;

  angleX4 = (gyroCoef * (angleX4 + gyroX4 * interval)) + (accCoef * angleAccX4);
  angleY4 = (gyroCoef * (angleY4 + gyroY4 * interval)) + (accCoef * angleAccY4);
  angleZ4 = angleGyroZ4;

  preInterval = millis();
  
}

MPU6050 mpu6050(Wire);

SoftwareSerial Bluetooth(rx, tx); // RX, TX

#include <stdlib.h>


void setup(){
  Wire.begin();
  Serial.begin(9600);
  Serial.print("Serial started");

  Bluetooth.begin(9600);
  pinMode(tx, OUTPUT);
  pinMode(rx, INPUT);
  Serial.write("Bluetooth started");
  
  pinMode(ThighF, OUTPUT);
  digitalWrite(ThighF, LOW);
  pinMode(ThighB, OUTPUT);
  digitalWrite(ThighB, LOW);
  pinMode(ThighL, OUTPUT);
  digitalWrite(ThighL, LOW);
  pinMode(ThighR, OUTPUT);
  digitalWrite(ThighR, LOW);
  pinMode(ThighF, OUTPUT);
  digitalWrite(ThighF, LOW);
  pinMode(ThighB, OUTPUT);
  digitalWrite(ThighB, LOW);
  pinMode(ThighL, OUTPUT);
  digitalWrite(ThighL, LOW);
  pinMode(ThighR, OUTPUT);
  digitalWrite(ThighR, LOW);

  pinMode(ThighFBAnalogue, OUTPUT);
  analogWrite(ThighFBAnalogue, 0);
  pinMode(ThighLRAnalogue, OUTPUT);
  analogWrite(ThighLRAnalogue, 0);
  pinMode(ShinFBAnalogue, OUTPUT);
  analogWrite(ShinFBAnalogue, 0);
  
  pinMode(mpuTorso, OUTPUT);
  digitalWrite(mpuTorso, HIGH);
  pinMode(mpu2, OUTPUT);
  digitalWrite(mpu2, HIGH);
  pinMode(mpu3, OUTPUT);
  digitalWrite(mpu3, HIGH);
  pinMode(mpu4, OUTPUT);
  digitalWrite(mpu4, LOW);
  
  mpu6050.begin();
  
}

int grabbluetooth1(){
  
  //Serial.println("BLE 1 entered");
  int finalmessage = 0;

  while ((Bluetooth.available()) and (Bluetooth.peek() != 97)){
    Bluetooth.read(); //Waits for the a
  }
  //Serial.println("Found the A");
  Bluetooth.read(); //Clears the a

  while (Bluetooth.available() == 0){
  } //Waits till bluetooth is received
  
  //Serial.println("Yes, there is bluetooth");
  finalmessage = Bluetooth.parseInt();
  
  //Serial.print("Final message1: ");
  //Serial.println(finalmessage);

  return finalmessage;
}

int grabbluetooth2(){
  
  //Serial.println("BLE 2 entered");
  int finalmessage = 0;

  while ((Bluetooth.available()) and (Bluetooth.peek() != 98)){
    Bluetooth.read(); //Waits for the b
  }
  //Serial.println("Found the B");
  Bluetooth.read(); //Clears the b

  while (Bluetooth.available() == 0){
  } //Waits till bluetooth is received
  
  //Serial.println("Yes, there is bluetooth");
  finalmessage = Bluetooth.parseInt();
  
  //Serial.print("Final message1: ");
  //Serial.println(finalmessage);

  return finalmessage;
}

int grabbluetooth3(){
  
  //Serial.println("BLE 3 entered");
  int finalmessage = 0;

  while ((Bluetooth.available()) and (Bluetooth.peek() != 99)){
    Bluetooth.read(); //Waits for the c
  }
  //Serial.println("Found the C");
  Bluetooth.read(); //Clears the c

  while (Bluetooth.available() == 0){
  } //Waits till bluetooth is received
  
  //Serial.println("Yes, there is bluetooth");
  finalmessage = Bluetooth.parseInt();
  
  //Serial.print("Final message1: ");
  //Serial.println(finalmessage);

  return finalmessage;
}

int grabbluetooth4(){
  
  //Serial.println("BLE 4 entered");
  int finalmessage = 0;

  while ((Bluetooth.available()) and (Bluetooth.peek() != 100)){
    Bluetooth.read(); //Waits for the d
  }
  //Serial.println("Found the D");
  Bluetooth.read(); //Clears the d

  while (Bluetooth.available() == 0){
  } //Waits till bluetooth is received
  
  //Serial.println("Yes, there is bluetooth");
  finalmessage = Bluetooth.parseInt();
  
  //Serial.print("Final message1: ");
  //Serial.println(finalmessage);

  return finalmessage;
}

int grabbluetooth5(){
  
  //Serial.println("BLE 5 entered");
  int finalmessage = 0;

  while ((Bluetooth.available()) and (Bluetooth.peek() != 101)){
    Bluetooth.read(); //Waits for the e
  }
  //Serial.println("Found the E");
  Bluetooth.read(); //Clears the e

  while (Bluetooth.available() == 0){
  } //Waits till bluetooth is received
  
  //Serial.println("Yes, there is bluetooth");
  finalmessage = Bluetooth.parseInt();
  
  //Serial.print("Final message1: ");
  //Serial.println(finalmessage);

  return finalmessage;
}

void loop(){ 
  int locker = 0;
  int ThighXComparator = 0;
  int ThighYComparator = 0;
  int ShinXComparator = 0;
  int ShinZComparator = 0;
  int FootXComparator = 0;
  int ThighXComparatorAcc = 0;
  int ThighYComparatorAcc = 0;
  int ShinXComparatorAcc = 0;
  int ShinZComparatorAcc = 0;
  int FootXComparatorAcc = 0;
  int ThighXTimer = 1;
  int ThighYTimer = 1;
  int ShinXTimer = 1;
  int ShinZTimer = 1;
  int FootXTimer = 1;
  
  int TorsoFB = 0;
  int TorsoLR = 0;
  int TorsoTWIST = 0;
  int TorsoFBAcc = 0;
  int TorsoLRAcc = 0;
  int TorsoTWISTAcc = 0;

  int ThighFB = 0;
  int ThighLR = 0;
  int ThighFBAcc = 0;
  int ThighLRAcc = 0;

  int ShinFB = 0;
  int ShinFBAcc = 0;

  int FootFB = 0;
  int FootTWIST = 0;
  int FootFBAcc = 0;
  int FootTWISTAcc = 0;

  int ThighX = 0;
  int ThighY = 0;
  int ShinX = 0;
  int ShinZ = 0;
  int FootX = 0;

  int ThighXAcc = 0;
  int ThighYAcc = 0;
  int ShinXAcc = 0;
  int ShinZAcc = 0;
  int FootXAcc = 0;

  int ThighXPWM = 1;
  int ThighYPWM = 1;
  int ShinXPWM = 1;
  int ShinZPWM = 1;
  int FootXPWM = 1;

  float ThighMass = 0.194;
  float ShinMass = 0.11;
  float FootMass = 0.054;

  int ThighDistance = 7;
  int ShinDistance = 6;
  int FootDistance = 3;

  int SpeedMultiplier = 1;
  float MaxTorque = 1.15;

  int message1 = grabbluetooth1();
  int message3 = grabbluetooth2();
  int message5 = grabbluetooth3();
  int message7 = grabbluetooth4();
  int message9 = grabbluetooth5();

  
  while (locker == 0){ //Everything before this was kinda a setup. This is the real loop
   //Serial.println("Locker entered");
    if (Bluetooth.available()){
      //Serial.println("Bluetooth Detected inside locker");
      int messagex = grabbluetooth1();
      if ((messagex > 0) and (messagex < 361)){
        if (messagex > 180){
          message1 = messagex - 360;
        }
        else {
          message1 = messagex;
        }
      }
      //Serial.print("ThighX:  ");
      //Serial.println(message1);
      //delay(1);
      int message2 = 30000;
      //Serial.println(message2);
      //delay(1);
      messagex = grabbluetooth2();
      if ((messagex > 999) and (messagex < 1361)){
        if (messagex > 1180){
          message3 = messagex - 1360;
        }
        else {
          message3 = messagex - 1000;
        }
      }
      //Serial.print("ThighY:  ");
      //Serial.println(message3);
      //delay(1);
      int message4 = 30000;
      //Serial.println(message4);
      //delay(1);
      messagex = grabbluetooth3();
      if ((messagex > 1999) and (messagex < 2361)){
        if (messagex > 2180){
          message5 = messagex - 2360;
        }
        else {
          message5 = messagex - 2000;
        }
      }
      //Serial.print("ShinX:  ");
      //Serial.println(message5);
      //delay(1);
      int message6 = 30000;
      //Serial.println(message6);
      //delay(1);
      messagex = grabbluetooth4();
      if ((messagex > 2999) and (messagex < 3361)){
        if (messagex > 3180){
          message7 = messagex - 3360;
        }
        else {
          message7 = messagex - 3000;
        }
      }
      //Serial.print("ShinZ:  ");
      //Serial.println(message7);
      //delay(1);
      int message8 = 30000;
      //Serial.println(message8);
      //delay(1);
      messagex = grabbluetooth5();
      if ((messagex > 3999) and (messagex < 4361)){
        if (messagex > 4180){
          message9 = messagex - 4360;
        }
        else {
          message9 = messagex - 4000;
        }
      }
      //Serial.print("FootX:  ");
      //Serial.println(message9);
      //delay(1);
      int message10 = 30000;
      //Serial.println(message10);
      //delay(1);

      //Serial.println("messages done");
    
        int CThighX = 1;
        int CThighY = 1;
        int CShinX = 1;
        int CShinZ = 1;
        int CFootX = 1;
  
        int CThighXAcc = 1;
        int CThighYAcc = 1;
        int CShinXAcc = 1;
        int CShinZAcc = 1;
        int CFootXAcc = 1;




        
        CThighX = message1; //Offload to new variables
        CThighXAcc = message2 - 30000;

        mpu6050.update();

        int TorsoFB = mpu6050.getAccAngleX1(); //Grab raw values
        int TorsoFBAcc = mpu6050.getAccX1();
    
        int ThighFB = mpu6050.getAccAngleX2(); //Grab raw values
        int ThighFBAcc = mpu6050.getAccX2();
    
        int ThighX = ThighFB - TorsoFB; //Local limb angle
  
        int ThighXAcc = (TorsoFBAcc - ThighFBAcc)/1; //Local limb speed
    
        ThighXComparator = ThighX - CThighX; //Comparison Angle
  
        ThighXComparatorAcc = ThighXAcc - CThighXAcc;//Comparison Speed
  
        if (ThighXComparatorAcc < 0){ //Speed always needs to be +ve
          ThighXComparatorAcc = ThighXComparatorAcc * -1;
        }

        //int ThighXMultiplier = ThighXAcc / CThighXAcc; //To correct the existing pwm

        float tempangle = sin((TorsoFB * 3.14159 / 180) + (CThighX * 3.14159 / 180));
        float ThighXTorqueIntermediate = ThighMass * ThighDistance;
        float ThighXTorque = ThighXTorqueIntermediate * tempangle;
        //ThighXPWM = 255 * ThighXTorque * SpeedMultiplier / MaxTorque;
        ThighXPWM = 255 * tempangle;
        
        //Serial.print("TorsoFB:  ");
        //Serial.println(TorsoFB);
        //Serial.print("CThighX:  ");
        //Serial.println(CThighX);
        //Serial.print("sin(TorsoFB + CThighX):  ");
        //Serial.println(tempangle);
        Serial.print("ThighXPWM:  ");
        Serial.println(ThighXPWM);
        
      
        //if (ThighXComparator < 0){ //if negative
        if (ThighXPWM < 0){ //if negative
          ThighXPWM = ThighXPWM * -1;
          ThighXPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ThighFBAnalogue, ThighXPWM);
          digitalWrite(ThighF, HIGH);
          digitalWrite(ThighB, LOW);
          //Serial.println("Thigh moving forward");
        }
        else { //if positive
          ThighXPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ThighFBAnalogue, ThighXPWM);
          digitalWrite(ThighF, LOW);
          digitalWrite(ThighB, HIGH);
          //Serial.println("Thigh moving backward");
        }
          

          
      




        
        CThighY = message3; //Offload to new variables
        CThighXAcc = message4 - 30000;

        mpu6050.update();

        int TorsoLR = mpu6050.getAngleZ1(); //Grab raw values
        int TorsoLRAcc = mpu6050.getAccZ1();
    
        int ThighLR = mpu6050.getAngleZ2(); //Grab raw values
        int ThighLRAcc = mpu6050.getAccZ2();
    
        int ThighY = TorsoLR - ThighLR; //Local limb angle
  
        int ThighYAcc = (TorsoLRAcc - ThighLRAcc)/1; //Local limb speed
    
        ThighYComparator = ThighY - CThighY; //Comparison Angle
  
        ThighYComparatorAcc = ThighYAcc - CThighYAcc;//Comparison Speed
  
        if (ThighYComparatorAcc < 0){ //Speed always needs to be +ve
          ThighYComparatorAcc = ThighYComparatorAcc * -1;
        }

        //int ThighYMultiplier = ThighYAcc / CThighYAcc; //To correct the existing pwm

        tempangle = sin((TorsoLR * 3.14159 / 180) + (CThighY * 3.14159 / 180));
        float ThighYTorqueIntermediate = ThighMass * ThighDistance;
        float ThighYTorque = ThighYTorqueIntermediate * tempangle;
        //ThighYPWM = 255 * ThighYTorque * SpeedMultiplier / MaxTorque;
        ThighYPWM = 255 * tempangle;
        
        //Serial.print("TorsoLR:  ");
        //Serial.println(TorsoLR);
        //Serial.print("CThighY:  ");
        //Serial.println(CThighY);
        //Serial.print("sin(TorsoLR + CThighY):  ");
        //Serial.println(tempangle);
        //Serial.print("ThighYPWM:  ");
        //Serial.println(ThighYPWM);
        
      
        //if (ThighYComparator < 0){ //if negative
        if (ThighYPWM < 0){ //if negative
          ThighYPWM = ThighYPWM * -1;
          ThighYPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ThighLRAnalogue, ThighYPWM);
          digitalWrite(ThighL, HIGH);
          digitalWrite(ThighR, LOW);
          //Serial.println("Thigh moving left");
        }
        else { //if positive
          ThighYPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ThighLRAnalogue, ThighYPWM);
          digitalWrite(ThighL, LOW);
          digitalWrite(ThighR, HIGH);
          //Serial.println("Thigh moving right");
        }
          
          





        CShinX = message5 - 2000; //Offload to new variables
        CShinXAcc = message6 - 30000;

        mpu6050.update();

        ThighFB = mpu6050.getAccAngleX2(); //Grab raw values
        ThighFBAcc = mpu6050.getAccX2(); //No int required, it's been declared before
    
        int ShinFB = mpu6050.getAccAngleX3(); //Grab raw values
        int ShinFBAcc = mpu6050.getAccX3();
    
        int ShinX = ThighFB - ShinFB; //Local limb angle
  
        int ShinXAcc = (ThighFBAcc - ShinFBAcc)/1; //Local limb speed
    
        ShinXComparator = ShinX - CShinX; //Comparison angle
  
        ShinXComparatorAcc = ShinXAcc - CShinXAcc; //Comparison speed
  
        if (ShinXComparatorAcc < 0){ //Speed always needs to be +ve
          ShinXComparatorAcc = ShinXComparatorAcc * -1;
        }

        //int ShinXMultiplier = ShinXAcc / CShinXAcc; //To correct the existing pwm

        tempangle = sin((ThighFB * 3.14159 / 180) + (CShinX * 3.14159 / 180));
        float ShinTorqueIntermediate = ShinMass * ShinDistance;
        float ShinTorque = ShinTorqueIntermediate * tempangle;
        //ShinXPWM = 255 * ShinTorque * SpeedMultiplier / MaxTorque;
        ShinXPWM = 255 * tempangle;
        
        //Serial.print("ThighFB:  ");
        //Serial.println(ThighFB);
        //Serial.print("CShinX:  ");
        //Serial.println(CShinX);
        //Serial.print("sin(ThighFB + CShinX):  ");
        //Serial.println(tempangle);
        //Serial.print("ShinXPWM:   ");
        //Serial.println(ShinXPWM);
        
      
        //if (ShinXComparator < 0){ //if negative
        if (ShinXPWM < 0){ //if negative
          ShinXPWM = ShinXPWM * -1;
          ShinXPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ShinFBAnalogue, ShinXPWM);
          digitalWrite(ShinF, HIGH);
          digitalWrite(ShinB, LOW);
          //Serial.println("Shin moving forward");
        }
        else { //if positive
          ShinXPWM += 31; //Adding on the bit it loses to the motor unit (2V)
          analogWrite(ShinFBAnalogue, ShinXPWM);
          digitalWrite(ShinF, LOW);
          digitalWrite(ShinB, HIGH);
          //Serial.println("Shin moving backward");
        }

          
      




        /*
        if ((message7 < 3000) and (message7 > 3180)){
          CShinZ = message7 - 3000;
          CShinZAcc = message8 - 30000;

        mpu6050.update();
        int TorsoTWIST = mpu6050.getAccAngleY1();
        int TorsoTWISTAcc = mpu6050.getAccY1();
    
        int FootTWIST = mpu6050.getAccAngleY4();
        int FootTWISTAcc = mpu6050.getAccY4();
    
        int ShinZ = TorsoTWIST - FootTWIST;
  
        int ShinZAcc = (TorsoTWISTAcc - FootTWISTAcc)/1;
    
        ShinZComparator = ShinZ - CShinZ;
  
        ShinZComparatorAcc = ShinZAcc - CShinZAcc;

        if (ShinZComparatorAcc < 0){
          ShinZComparatorAcc = ShinZComparatorAcc * -1;
        }
  
        int ShinZMultiplier = ShinZComparatorAcc / CShinZAcc;
  
        ShinZTimer = ShinZMultiplier * ShinZTimer;
  
        if (ShinZTimer > 20){
          ShinZTimer = 20;
        }
        

      
      if ((ShinZComparator > 3) or (ShinZComparator < -3)){
        if (ShinZComparator < 0){
          digitalWrite(ShinCW, HIGH);
          digitalWrite(ShinACW, LOW);
          //Serial.println("Illegal shin moving CW");
          delay(ShinZTimer);
          //Serial.println("Shin done");
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, LOW);
        }
        else {
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, HIGH);
          //Serial.println("Illegal shin moving ACW");
          delay(ShinZTimer);
          //Serial.println("Shin done");
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, LOW);
        }
      }
      //Serial.println("J'ai fini");
          
        }


        if ((message7 < 3180) and (message7 > 3360)){
          CShinZ = message7 - 3360;
          CShinZAcc = message8 - 30000;

        mpu6050.update();
        int TorsoTWIST = mpu6050.getAccAngleY1();
        int TorsoTWISTAcc = mpu6050.getAccY1();
    
        int FootTWIST = mpu6050.getAccAngleY4();
        int FootTWISTAcc = mpu6050.getAccY4();
    
        int ShinZ = TorsoTWIST - FootTWIST;
  
        int ShinZAcc = (TorsoTWISTAcc - FootTWISTAcc)/1;
    
        ShinZComparator = ShinZ - CShinZ;
  
        ShinZComparatorAcc = ShinZAcc - CShinZAcc;

        if (ShinZComparatorAcc < 0){
          ShinZComparatorAcc = ShinZComparatorAcc * -1;
        }
  
        int ShinZMultiplier = ShinZComparatorAcc / CShinZAcc;
  
        ShinZTimer = ShinZMultiplier * ShinZTimer;
  
        if (ShinZTimer > 20){
          ShinZTimer = 20;
        }
        

      
      if ((ShinZComparator > 3) or (ShinZComparator < -3)){
        if (ShinZComparator < 0){
          digitalWrite(ShinCW, HIGH);
          digitalWrite(ShinACW, LOW);
          //Serial.println("Illegal shin moving CW");
          delay(ShinZTimer);
          //Serial.println("Shin done");
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, LOW);
        }
        else {
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, HIGH);
          //Serial.println("Illegal shin moving ACW");
          delay(ShinZTimer);
          //Serial.println("Shin done");
          digitalWrite(ShinCW, LOW);
          digitalWrite(ShinACW, LOW);
        }
      }
      //Serial.println("J'ai fini");
          
        }*/



/*
        
        if ((message9 < 4000) and (message9 > 4180)){
          CFootX = message9 - 4000;
          CFootXAcc = message10 - 30000;

        mpu6050.update();
    
        int ShinFB = mpu6050.getAccAngleX3();
        int ShinFBAcc = mpu6050.getAccX3();
    
        int FootFB = mpu6050.getAccAngleX4();
        int FootFBAcc = mpu6050.getAccX4();
    
        int FootX = ShinFB - FootFB;
  
        int FootXAcc = (ShinFBAcc - FootFBAcc)/1;
    
        FootXComparator = FootX - CFootX;
  
        FootXComparatorAcc = FootXAcc - CFootXAcc;
  
        if (FootXComparatorAcc < 0){
          FootXComparatorAcc = FootXComparatorAcc * -1;
        }
  
        int FootXMultiplier = FootXComparatorAcc / CFootXAcc;
  
        FootXTimer = FootXMultiplier * FootXTimer;
  
        if (FootXTimer > 20){
          FootXTimer = 20;
        }
        
      

      if ((FootXComparator > 3) or (FootXComparator < -3)){
        if (FootXComparator < 0){
          digitalWrite(FootF, HIGH);
          digitalWrite(FootB, LOW);
          Serial.println("Foot moving forward");
        }
        else {
          digitalWrite(FootF, LOW);
          digitalWrite(FootB, HIGH);
          Serial.println("Foot moving backward");
        }
      }
      }
      if (FootX != 0){
        if (FootXComparator < 0){
          FootXTimer = 10;
          digitalWrite(FootF, HIGH);
          digitalWrite(FootB, LOW);
        }
        else{
          FootXTimer = 10;
          digitalWrite(FootF, LOW);
          digitalWrite(FootB, HIGH);
        }
        Serial.println("Foot Sustenance");
      }
      //Serial.println("J'ai fini");
          
      




        
        if ((message9 < 4180) and (message9 > 4360)){
          CFootX = message9 - 4360;
          CFootXAcc = message10 - 30000;

        mpu6050.update();
    
        int ShinFB = mpu6050.getAccAngleX3();
        int ShinFBAcc = mpu6050.getAccX3();
    
        int FootFB = mpu6050.getAccAngleX4();
        int FootFBAcc = mpu6050.getAccX4();
    
        int FootX = ShinFB - FootFB;
  
        int FootXAcc = (ShinFBAcc - FootFBAcc)/1;
    
        FootXComparator = FootX - CFootX;
  
        FootXComparatorAcc = FootXAcc - CFootXAcc;
  
        if (FootXComparatorAcc < 0){
          FootXComparatorAcc = FootXComparatorAcc * -1;
        }
  
        int FootXMultiplier = FootXComparatorAcc / CFootXAcc;
  
        FootXTimer = FootXMultiplier * FootXTimer;
  
        if (FootXTimer > 20){
          FootXTimer = 20;
        }
        
      

      if ((FootXComparator > 3) or (FootXComparator < -3)){
        if (FootXComparator < 0){
          digitalWrite(FootF, HIGH);
          digitalWrite(FootB, LOW);
          Serial.println("Foot moving forward");
        }
        else {
          digitalWrite(FootF, LOW);
          digitalWrite(FootB, HIGH);
          Serial.println("Foot moving backward");
        }
      }
      }
      if (FootX != 0){
        if (FootXComparator < 0){
          FootXTimer = 10;
          digitalWrite(FootF, HIGH);
          digitalWrite(FootB, LOW);
        }
        else{
          FootXTimer = 10;
          digitalWrite(FootF, LOW);
          digitalWrite(FootB, HIGH);
        }
        Serial.println("Foot Sustenance");
      }
      //Serial.println("J'ai fini");
          



      while (currenttime < 20){
        currenttime = millis() - basetime;
        if (currenttime > ThighXTimer){
          digitalWrite(ThighF, LOW);
          digitalWrite(ThighB, LOW);
        }
        if (currenttime > ThighYTimer){
          digitalWrite(ThighL, LOW);
          digitalWrite(ThighR, LOW);
        }
        if (currenttime > ShinXTimer){
          digitalWrite(ShinF, LOW);
          digitalWrite(ShinB, LOW);
        }
        if (currenttime > FootXTimer){
          digitalWrite(FootF, LOW);
          digitalWrite(FootB, LOW);
        }
      }


        

      }*/
    }
  }
}
