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
#define mpuTorso A3
#define mpu2 A2
#define mpu3 A1
#define mpu4 A0

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

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.print("Serial started");

  Bluetooth.begin(9600);
  Serial.println("Bluetooth started");

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

void loop() {
  int locker = 0;
  while (locker == 0){
    
    mpu6050.update();
    
    int TorsoFB = mpu6050.getAccAngleX1();
    int TorsoLR = mpu6050.getAngleZ1();
    int TorsoTWIST = mpu6050.getAccAngleY1();
    int TorsoFBAcc = mpu6050.getAccX1();
    int TorsoLRAcc = mpu6050.getAccZ1();
    int TorsoTWISTAcc = mpu6050.getAccY1();

    int ThighFB = mpu6050.getAccAngleX2();
    int ThighLR = mpu6050.getAngleZ2();
    int ThighFBAcc = mpu6050.getAccX2();
    int ThighLRAcc = mpu6050.getAccZ2();

    int ShinFB = mpu6050.getAccAngleX3();
    int ShinFBAcc = mpu6050.getAccX3();

    int FootFB = mpu6050.getAccAngleX4();
    int FootTWIST = mpu6050.getAccAngleY4();
    int FootFBAcc = mpu6050.getAccX4();
    int FootTWISTAcc = mpu6050.getAccY4();

    int ThighX = ThighFB - TorsoFB;
    int ThighY = ThighLR - TorsoLR;
    int ShinX = ThighFB - ShinFB;
    int ShinZ = TorsoTWIST - FootTWIST;
    int FootX = ShinFB - FootFB;

    int ThighXAcc = (TorsoFBAcc - ThighFBAcc)/1;
    int ThighYAcc = (TorsoLRAcc - ThighLRAcc)/1;
    int ShinXAcc = (ThighFBAcc - ShinFBAcc)/1;
    int ShinZAcc = (TorsoTWISTAcc - FootTWISTAcc)/1;
    int FootXAcc = (ShinFBAcc - FootFBAcc)/1;

    if (ThighX < 0){
      ThighX = ThighX + 360;
    }
    if (ThighXAcc < 0){
      ThighXAcc = ThighXAcc * -1;
    }
    ThighXAcc = ThighXAcc + 30000;

    String ThighXS = String(ThighX);
    String ThighXAccS = String(ThighXAcc);
    
    Bluetooth.print(ThighXS);
    //Bluetooth.print(".");
    Serial.println(ThighXS);
    //delay(2);
    //Bluetooth.print(ThighXAccS);
    Bluetooth.print("b");
    //Serial.println(ThighXAccS);
    //delay(2);
    
    if (ThighY < 0){
      ThighY = ThighY + 360;
    }
    if (ThighYAcc < 0){
      ThighYAcc = ThighYAcc * -1;
    }
    ThighY = ThighY + 1000;
    ThighYAcc = ThighYAcc + 30000;

    String ThighYS = String(ThighY);
    String ThighYAccS = String(ThighYAcc);
    
    Bluetooth.print(ThighYS);
    Serial.println(ThighYS);
    //Bluetooth.print(".");
    //delay(2);
    //Bluetooth.print(ThighYAccS);
    //Serial.println(ThighYAccS);
    Bluetooth.print("c");
    //delay(2);
    
    if (ShinX < 0){
      ShinX = ShinX + 360;
    }
    if (ShinXAcc < 0){
      ShinXAcc = ShinXAcc * -1;
    }
    ShinX = ShinX + 2000;
    ShinXAcc = ShinXAcc + 30000;

    String ShinXS = String(ShinX);
    String ShinXAccS = String(ShinXAcc);
    Bluetooth.print(ShinXS);
    Serial.println(ShinXS);
    //Bluetooth.print(".");
    //delay(2);
    //Bluetooth.print(ShinXAccS);
    //Serial.println(ShinXAccS);
    Bluetooth.print("d");
    //delay(2);
    
    if (ShinZ < 0){
      ShinZ = ShinZ + 360;
    }
    if (ShinZAcc < 0){
      ShinZAcc = ShinZAcc * -1;
    }
    ShinZ = ShinZ + 3000;
    ShinZAcc = ShinZAcc + 30000;

    String ShinZS = String(ShinZ);
    String ShinZAccS = String(ShinZAcc);
    Bluetooth.print(ShinZS);
    Serial.println(ShinZS);
    //Bluetooth.print(".");
    //delay(2);
    //Bluetooth.print(ShinZAccS);
    //Serial.println(ShinZAccS);
    Bluetooth.print("e");
    //delay(2);
    
    if (FootX < 0){
      FootX = FootX + 360;
    }
    if (FootXAcc < 0){
      FootXAcc = FootXAcc * -1;
    }
    FootX = FootX + 4000;
    FootXAcc = FootXAcc + 30000;

    String FootXS = String(FootX);
    String FootXAccS = String(FootXAcc);
    Bluetooth.print(FootXS);
    Serial.println(FootXS);
    //Bluetooth.print(".");
    //delay(2);
    //Bluetooth.print(FootXAccS);
    //Serial.println(FootXAccS);
    //Bluetooth.print(".");
    Bluetooth.print("a");
    Serial.println("a");
    //delay(2);
    
  }

}
