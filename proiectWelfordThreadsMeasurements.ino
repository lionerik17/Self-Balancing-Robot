#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>

#define SAMPLE_TIME 0.005
#define RAD_TO_DEG (180 / M_PI)
#define GYRO_SENSITIVITY 131.0
#define alpha (0.75 / (0.75 + 0.005))
#define SPEED_OF_SOUND 0.0343
#define NO_OF_MEASUREMENTS 40

#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_ZOUT_H 0x47

#define Kp 1.75
#define Ki 0.015
#define Kd 0.45
#define TARGET_ANGLE -90.0

const int trigPin = 12;
const int echoPin = 3;

volatile double accAngle = 0.0, gyroAngle = 0.0;
volatile double currentAngle = 0.0, previousAngle = 0.0;

double currentDistance = 0.0;

int16_t accX = 0, accY = 0, gyroZ = 0;
int countAngle = 0, countDistance = 0;

double angleMean = 0.0, angleM2 = 0.0, angleVariance = 0.0;
double distanceMean = 0.0, distanceM2 = 0.0, distanceVariance = 0.0;

double delta = 0.0, duration = 0.0;

double error = 0.0, prevError = 0.0, integral = 0.0, derivative = 0.0;
volatile double pidOutput = 0.0;

int currentTime = 0;
int index = 0;

AccelStepper stepper1(AccelStepper::FULL4WIRE, 11, 9, 10, 8);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 7, 5, 6, 4);
int dir = 1;
double bound = 100.0;

struct Thread {
  int time;
  int count;  
  void (*task)(void);     
};

struct Measurement {
  unsigned long totalExecutionTime;
  int count;
};

Measurement measurementData = {0, 0};

const long measurementSteps[] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};
const long numSteps = sizeof(measurementSteps) / sizeof(measurementSteps[0]);
long currentStepIndex = 0;

void readMPUTask();
void readDistanceTask();
void updateWelfordAngleTask();
void updateWelfordDistanceTask();
void controlMotorsTask();

Thread threads[] = {
    {NO_OF_MEASUREMENTS, 0, readMPUTask},
    {NO_OF_MEASUREMENTS, 0, readDistanceTask},
    {NO_OF_MEASUREMENTS, 0, updateWelfordAngleTask},
    {NO_OF_MEASUREMENTS, 0, updateWelfordDistanceTask},
    {1, 0, controlMotorsTask},
};

const int numThreads = sizeof(threads) / sizeof(threads[0]);

void readMPUTask() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 4, true);

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  gyroZ = (Wire.read() << 8) | Wire.read();
}

void readDistanceTask() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  currentDistance = (duration / 2) * SPEED_OF_SOUND;
}

void updateWelfordAngleTask() {
  countAngle++;
  delta = currentAngle - angleMean;
  angleMean += delta / countAngle;
  angleM2 += delta * (currentAngle - angleMean);
}

void updateWelfordDistanceTask() {
  countDistance++;
  delta = currentDistance - distanceMean;
  distanceMean += delta / countDistance;
  distanceM2 += delta * (currentDistance - distanceMean);
}

void controlMotorsTask() {
  accAngle = atan2(accY, accX) * RAD_TO_DEG;
  gyroAngle = (gyroZ / GYRO_SENSITIVITY) * SAMPLE_TIME;

  currentAngle = alpha * (previousAngle + gyroAngle) + (1 - alpha) * accAngle;
  previousAngle = currentAngle;

  error = currentAngle - TARGET_ANGLE;
  integral += error * SAMPLE_TIME;
  derivative = (error - prevError) / SAMPLE_TIME;
  prevError = error;

  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  pidOutput = constrain(pidOutput, -1500, 1500);

  if (distanceVariance > bound) {
    dir = -1;
  } else {
    dir = 1;
  }

  bound = bound * alpha + (1 - alpha) * bound;

  stepper1.setSpeed(-pidOutput * dir);
  stepper2.setSpeed(-pidOutput * dir);

  stepper1.runSpeed();
  stepper2.runSpeed();
}

void scheduler() {
  unsigned long startTime = micros();
  for (index = 0; index < numThreads; ++index) {
    ++threads[index].count;
    if (threads[index].count >= threads[index].time) {
      threads[index].count = 0;
      threads[index].task();
    }
  }
  unsigned long executionTime = micros() - startTime;

  measurementData.totalExecutionTime += executionTime;
  measurementData.count++;

   if (measurementData.count >= measurementSteps[currentStepIndex]) {
    Serial.print("Measurements: ");
    Serial.println(measurementSteps[currentStepIndex]);

    Serial.print("Average Execution Time: ");
    Serial.print(measurementData.totalExecutionTime / measurementData.count);
    Serial.println(" us");

    Serial.println("--------------------------------------");

    measurementData.totalExecutionTime = 0;
    measurementData.count = 0;

    if (currentStepIndex < numSteps - 1) {
      currentStepIndex++;
    } else {
      Serial.println("All measurements completed!");
      while (true);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  stepper1.setMaxSpeed(1500);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(1500);
  stepper2.setAcceleration(1000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  scheduler();
}
