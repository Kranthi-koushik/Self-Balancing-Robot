#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// ===== Motor Pins =====
#define IN1 6
#define IN2 9
#define IN3 10
#define IN4 12
#define ENA 5
#define ENB 11

// ===== PID VALUES =====
double setpoint = 186.55;
double Kp = 19;
double Kd = 0.8;
double Ki = 140;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ===== MPU Variables =====
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {

    Serial.begin(115200);
    Wire.begin();

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // Gyro offsets
    mpu.setXGyroOffset(87);
    mpu.setYGyroOffset(-22);
    mpu.setZGyroOffset(6);
    mpu.setZAccelOffset(1572);

    if (devStatus == 0) {

        mpu.setDMPEnabled(true);
        attachInterrupt(1, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);

    } 
    else {
        Serial.println("DMP Initialization failed");
        while (1);
    }

    // Motor Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void loop() {

    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {

        pid.Compute();

        if (input > 150 && input < 200) {

            if (output > 0)
                Forward();
            else if (output < 0)
                Reverse();
        } 
        else {
            Stop();
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    }

    else if (mpuIntStatus & 0x02) {

        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Calculate robot tilt angle
        input = ypr[1] * 180 / M_PI + 180;

        // ===== DATA FOR PERFORMANCE GRAPH =====
        // Plot Setpoint and Measured Angle
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(input);
    }
}

void Forward() {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, abs(output));
    analogWrite(ENB, abs(output));
}

void Reverse() {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    analogWrite(ENA, abs(output));
    analogWrite(ENB, abs(output));
}

void Stop() {

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
