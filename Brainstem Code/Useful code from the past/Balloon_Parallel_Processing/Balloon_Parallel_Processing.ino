#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23
#define DIO0 26
#define seaLevelPressure_hPa 1013.25
#define GPS_SERIAL_RX 35
#define GPS_SERIAL_TX 12
#define Heater 12
#define LiPo 25
#define BAND 868E6
#define customMOSI 15
#define customMISO 2
#define customSCK 14
#define customCS 13
#define MPU_SDA 23
#define MPU_SCL 19
#define INTERRUPT_PIN 4

Adafruit_BMP085 bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
MPU6050 mpu;
File AccelerometerFile, GPSFile, AltimeterFile, BatteryFile;

bool dmpReady = false;
uint8_t mpuIntStatus, devStatus;
uint16_t packetSize, fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorInt16 aa, aaReal, aaWorld;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;

double voltage, percentage;
float latitude, longitude, altitudee;
int dayy, hourr, minutee, secondd;
float speedmph, bearing;
int satellitess;
float angleX, angleY, angleZ;
float AccelerationX, AccelerationY, AccelerationZ;
float temperature, pressure, altitudeee;
int cycle;
int heaterStatus;

SemaphoreHandle_t batteryMutex; // Mutex for battery-related variables
SemaphoreHandle_t gpsMutex;     // Mutex for GPS-related variables
SemaphoreHandle_t accelMutex;    // Mutex for accelerometer-related variables
SemaphoreHandle_t spiMutex; // Mutex for SPI bus

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_SERIAL_RX, GPS_SERIAL_TX);

    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DIO0);
    while (!LoRa.begin(BAND)) {
        Serial.println("LoRa isn't starting up :(");
    }
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }

    pinMode(LiPo, INPUT);
    pinMode(Heater, OUTPUT);
    digitalWrite(Heater, LOW);

    SPI.end();
    delay(200);
    SPI.begin(customSCK, customMISO, customMOSI, customCS);
    if (!SD.begin(customCS)) {
        Serial.println("Card failed, or not present");
        while (1) {}
    }

    initializeSD();
    initializeMPU();

    // Create mutexes
    batteryMutex = xSemaphoreCreateMutex();
    gpsMutex = xSemaphoreCreateMutex();
    accelMutex = xSemaphoreCreateMutex();
    spiMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        taskSendLoRaData,
        "SendLoRaData",
        65536,
        NULL,
        1,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        taskSensorReadings,
        "SensorReadings",
        65536,
        NULL,
        1,
        NULL,
        0
    );

}

void initializeSD() {
    AltimeterFile = SD.open("/Altimeter.txt", FILE_APPEND);
    AltimeterFile.println("-------------------------------------------------------------");
    AltimeterFile.println("------------------------- NEW REFRESH -----------------------");
    AltimeterFile.println("-------------------------------------------------------------");
    AltimeterFile.println("");
    AltimeterFile.println("Cycle\tTemperature/C\tPressure/Pa\t\tAltitude/m");
    AltimeterFile.close();

    GPSFile = SD.open("/GPS.txt", FILE_APPEND);
    GPSFile.println("-------------------------------------------------------------");
    GPSFile.println("------------------------- NEW REFRESH -----------------------");
    GPSFile.println("-------------------------------------------------------------");
    GPSFile.println("");
    GPSFile.println("Cycle\tLatitude\t\tLongitude\t\tAltitude/m\t\tDay:Hour:Minute:Second\t\tSpeed/mph\t\tBearing/Deg\t\tNumber of satellites");
    GPSFile.close();

    AccelerometerFile = SD.open("/Accelerometer.txt", FILE_APPEND);
    AccelerometerFile.println("-------------------------------------------------------------");
    AccelerometerFile.println("------------------------- NEW REFRESH -----------------------");
    AccelerometerFile.println("-------------------------------------------------------------");
    AccelerometerFile.println("");
    AccelerometerFile.println("Cycle\tAngleX\t\tAngleY\t\tAngleZ\t\tAccX\t\tAccY\t\tAccZ");
    AccelerometerFile.close();

    BatteryFile = SD.open("/Battery.txt", FILE_APPEND);
    BatteryFile.println("-------------------------------------------------------------");
    BatteryFile.println("------------------------- NEW REFRESH -----------------------");
    BatteryFile.println("-------------------------------------------------------------");
    BatteryFile.println("");
    BatteryFile.println("Cycle\tPercentage");
    BatteryFile.println("(Unlike the other data, this is updated every 60 cycles, not every cycle)");
    BatteryFile.close();
}

void initializeMPU() {
    Wire.begin(MPU_SDA, MPU_SCL);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    if (mpu.testConnection()) {
        devStatus = mpu.dmpInitialize();
        if (devStatus == 0) {
            mpu.setXGyroOffset(220);
            mpu.setYGyroOffset(76);
            mpu.setZGyroOffset(-85);
            mpu.setZAccelOffset(1688); //1788 is the default
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            mpu.setDMPEnabled(true);
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            Serial.print("DMP Initialization failed (code ");
            Serial.print(devStatus);
            Serial.println(")");
        }
    } else {
        Serial.println("MPU6050 connection failed");
    }
}

void loop() {
    // Empty loop as tasks are handled by FreeRTOS
}

void taskSendLoRaData(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(30000)); // Send a message every 30 seconds
        
        // Obtain mutex before accessing shared variables
        xSemaphoreTake(batteryMutex, portMAX_DELAY);
        double tempvoltage = voltage;
        double temppercentage = percentage;
        xSemaphoreGive(batteryMutex);
        
        xSemaphoreTake(gpsMutex, portMAX_DELAY);
        float templatitude = latitude;
        float templongitude = longitude;
        float tempaltitudee = altitudee;
        xSemaphoreGive(gpsMutex);
        
        xSemaphoreTake(accelMutex, portMAX_DELAY);
        float tempangleX = angleX;
        float tempangleY = angleY;
        float tempangleZ = angleZ;
        float tempAccelerationZ = AccelerationZ;
        xSemaphoreGive(accelMutex);

        xSemaphoreTake(spiMutex, portMAX_DELAY);
        SPI.end();
        SPI.begin(SCK, MISO, MOSI, SS); // LoRa SPI settings
        
        LoRa.beginPacket();
        Serial.print("-------------------------------Sending LoRa: ");///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Serial.println(millis());
    
        LoRa.print("LiPo Volt=");
        LoRa.println(tempvoltage, 4);
        LoRa.print("LiPo%=");
        LoRa.println(temppercentage, 4);
    
        LoRa.print("Temperature=");
        LoRa.println(temperature);
        LoRa.print("Pressure=");
        LoRa.println(pressure);
        LoRa.print("BMP Altitude=");
        LoRa.println(altitudeee);
    
        LoRa.print("Lat=");
        LoRa.println(templatitude, 6);
        LoRa.print("Lon=");
        LoRa.println(templongitude, 6);
        LoRa.print("GPS Altitude=");
        LoRa.println(tempaltitudee, 2);
    
        LoRa.print("Accelerometer(x,y,z)= (");
        LoRa.print(tempangleX);
        LoRa.print(",");
        LoRa.print(tempangleY);
        LoRa.print(",");
        LoRa.print(tempangleZ);
        LoRa.println(")");
        if (tempAccelerationZ > 6){
          LoRa.println("Balloon is likely falling");
        }
    
        if (heaterStatus == 0){
          LoRa.println("Heater off");
        }
        else if (heaterStatus == 1){
          LoRa.println("Heater on");
        }
        else{
          LoRa.println("Heater error");
        }
  
        LoRa.endPacket();

        SPI.end();
        xSemaphoreGive(spiMutex);
  
        Serial.print("--------------------------------LoRa sent");////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Serial.println(millis());

    }
}

void taskSensorReadings(void *pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(1000)); // Read sensors every second
        cycle++;

        readBattery(cycle);
        readAltimeter(cycle);
        controlHeater();
        readGPS(cycle);
        readAccelerometer(cycle);
        adjustOffsets();
        
        Serial.print("LiPo Volt=");
        Serial.println(voltage, 4);
        Serial.print("LiPo%=");
        Serial.println(percentage, 4);
        Serial.print("Temperature=");
        Serial.println(temperature);
        Serial.print("Pressure=");
        Serial.println(pressure);
        Serial.print("BMP Altitude=");
        Serial.println(altitudeee);
        Serial.print("Lat=");
        Serial.println(latitude, 6);
        Serial.print("Lon=");
        Serial.println(longitude, 6);
        Serial.print("GPS Altitude=");
        Serial.println(altitudee, 2);
        Serial.print("Accelerometer(x,y,z)= (");
        Serial.print(angleX);
        Serial.print(",");
        Serial.print(angleY);
        Serial.print(",");
        Serial.print(angleZ);
        Serial.println(")");
    }
}

// The rest of your code remains unchanged...



void readBattery(int cycle) {
  //if (cycle > 58){
    xSemaphoreTake(batteryMutex, portMAX_DELAY);
    
    double rawreading = analogRead(LiPo);
    voltage = rawreading * 3.3 / 4095 * 10 / 5.1 / 2;
    double round1 = voltage / 3.7;
    double round2 = pow((1 + pow(round1, 80)), 0.165);
    double round3 = 123 / round2;
    percentage = 123 - round3;

    xSemaphoreGive(batteryMutex);

    BatteryFile = SD.open("/Battery.txt", FILE_APPEND);
    BatteryFile.print(cycle);
    BatteryFile.print("\t");
    BatteryFile.println(percentage);
    BatteryFile.close();
  //}
}

void readAltimeter(int cycle) {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    altitudeee = bmp.readAltitude();

    AltimeterFile = SD.open("/Altimeter.txt", FILE_APPEND);
    AltimeterFile.print(cycle);
    AltimeterFile.print("\t");
    AltimeterFile.print(temperature);
    AltimeterFile.print("\t\t");
    AltimeterFile.print(pressure);
    AltimeterFile.print("\t\t");
    AltimeterFile.println(altitudeee);
    AltimeterFile.close();
}

void controlHeater() {
    temperature = bmp.readTemperature();
    if (temperature < 10) {
        digitalWrite(Heater, HIGH);
        heaterStatus = 1;
    } else {
        digitalWrite(Heater, LOW);
        heaterStatus = 0;
    }
}

void readGPS(int cycle) {
    while (GPSSerial.available()) {
        gps.encode(GPSSerial.read());
    }
    if (gps.location.isUpdated()) {
        xSemaphoreTake(gpsMutex, portMAX_DELAY);
        
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        altitudee = gps.altitude.meters();
        dayy = gps.date.day();
        hourr = gps.time.hour();
        minutee = gps.time.minute();
        secondd = gps.time.second();
        speedmph = gps.speed.mph();
        bearing = gps.course.deg();
        satellitess = gps.satellites.value();

        xSemaphoreGive(gpsMutex);

        GPSFile = SD.open("/GPS.txt", FILE_APPEND);
        GPSFile.print(cycle);
        GPSFile.print("\t");
        GPSFile.print(latitude, 6);
        GPSFile.print("\t\t");
        GPSFile.print(longitude, 6);
        GPSFile.print("\t\t");
        GPSFile.print(altitudee, 6);
        GPSFile.print("\t\t");
        GPSFile.print(dayy);
        GPSFile.print(":");
        GPSFile.print(hourr);
        GPSFile.print(":");
        GPSFile.print(minutee);
        GPSFile.print(":");
        GPSFile.print(secondd);
        GPSFile.print("\t\t");
        GPSFile.print(speedmph);
        GPSFile.print("\t\t");
        GPSFile.print(bearing);
        GPSFile.print("\t\t");
        GPSFile.println(satellitess);
        GPSFile.close();
    }
}

void readAccelerometer(int cycle) {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        return;
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

        xSemaphoreTake(accelMutex, portMAX_DELAY);
        angleX = ypr[0] * 180 / M_PI;
        angleY = ypr[1] * 180 / M_PI;
        angleZ = ypr[2] * 180 / M_PI;
        AccelerationX = aaWorld.x;
        AccelerationY = aaWorld.y;
        AccelerationZ = aaWorld.z;
        xSemaphoreGive(accelMutex);

        AccelerometerFile = SD.open("/Accelerometer.txt", FILE_APPEND);
        AccelerometerFile.print(cycle);
        AccelerometerFile.print("\t");
        AccelerometerFile.print(angleX);
        AccelerometerFile.print("\t\t");
        AccelerometerFile.print(angleY);
        AccelerometerFile.print("\t\t");
        AccelerometerFile.print(angleZ);
        AccelerometerFile.print("\t\t");
        AccelerometerFile.print(AccelerationX);
        AccelerometerFile.print("\t\t");
        AccelerometerFile.print(AccelerationY);
        AccelerometerFile.print("\t\t");
        AccelerometerFile.println(AccelerationZ);
        AccelerometerFile.close();
    }
}

void adjustOffsets() {
    // Example function to adjust offsets when the module is stable
    if (isModuleStable()) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        AccelerometerFile = SD.open("/Accelerometer.txt", FILE_APPEND);
        AccelerometerFile.println("------------------------");
        AccelerometerFile.println("----Mini calibration----");
        AccelerometerFile.println("------------------------");
        AccelerometerFile.close();
    }
}

bool isModuleStable() {
    // Example logic to determine if the module is stable
    // You can replace this with actual checks for variance in sensor readings
    static int stableCount = 0;
    static int unstableCount = 0;
    const int stableThreshold = 10;
    const int unstableThreshold = 3;

    VectorInt16 accel;
    VectorInt16 gyro;
    mpu.getMotion6(&accel.x, &accel.y, &accel.z, &gyro.x, &gyro.y, &gyro.z);

    if (abs(accel.x) < stableThreshold && abs(accel.y) < stableThreshold && abs(accel.z) < stableThreshold &&
        abs(gyro.x) < stableThreshold && abs(gyro.y) < stableThreshold && abs(gyro.z) < stableThreshold) {
        stableCount++;
        unstableCount = 0;
    } else {
        unstableCount++;
        stableCount = 0;
    }

    return stableCount > 1000;  // Adjust threshold as needed
}
