#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>

#define SAMPLE_TIME 0.005
#define RAD_TO_DEG (180 / M_PI)
#define GYRO_SENSITIVITY 131.0
#define alpha (0.75 / (0.75 + 0.005))
#define SPEED_OF_SOUND 0.034
#define NO_OF_MEASUREMENTS 10

#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_ZOUT_H 0x47

#define Kp 1.75
#define Ki 0.005
#define Kd 0.5
#define TARGET_ANGLE -90.0

const int trigPin = 12;
const int echoPin = 3;

volatile double accAngle = 0.0, gyroAngle = 0.0;
volatile double currentAngle = 0.0, previousAngle = 0.0;

double duration = 0.0, distance = 0.0;
double currentDistance = 0.0;
double delta = 0.0;

int16_t accX = 0, accY = 0, gyroZ = 0;
int countAngle = 0, countDistance = 0;

double angleMean = 0.0, angleM2 = 0.0, angleVariance = 0.0;
double distanceMean = 0.0, distanceM2 = 0.0, distanceVariance = 0.0;
double bound = 100.0;

double error = 0.0, prevError = 0.0, integral = 0.0, derivative = 0.0;
volatile double pidOutput = 0.0;

AccelStepper stepper1(AccelStepper::FULL4WIRE, 11, 9, 10, 8);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 7, 5, 6, 4);
int dir = 1;

void initTimer() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 9999; // 5 ms interval
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS11);  // Prescaler = 8
  TIMSK1 |= (1 << OCIE1A); // Timer1 compare match A
  sei();
}

void setupMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void readMPU6050Registers() {
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

double readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * SPEED_OF_SOUND;
  return distance;
}

void updateWelford(double newValue, double &mean, double &M2, int &count) {
  count++;
  delta = newValue - mean;
  mean += delta / count;
  M2 += delta * (newValue - mean);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupMPU6050();

  stepper1.setMaxSpeed(1500);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(1500);
  stepper2.setAcceleration(1000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  initTimer();
}

void loop() {
  readMPU6050Registers();

  updateWelford(currentAngle, angleMean, angleM2, countAngle);

  currentDistance = readDistance();
  updateWelford(currentDistance, distanceMean, distanceM2, countDistance);

  if (countAngle % NO_OF_MEASUREMENTS == 0 && countDistance % NO_OF_MEASUREMENTS == 0) {
    angleVariance = angleM2 / countAngle;
    distanceVariance = distanceM2 / countDistance;

    // Serial.print("angleVariance:");
     Serial.print(angleVariance);
    // Serial.print(" distanceVariance:");
    // Serial.println(distanceVariance);

    if (distanceVariance > bound ) {
      dir = -1;
    } else {
      dir = 1;
    }

    bound = bound * alpha + (1 - alpha) * distanceVariance;

    countAngle = 0;
    countDistance = 0;
    angleMean = 0.0;
    angleM2 = 0.0;
    distanceMean = 0.0;
    distanceM2 = 0.0;
  }
}

ISR(TIMER1_COMPA_vect) {
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

  stepper1.setSpeed(-pidOutput * dir);
  stepper2.setSpeed(-pidOutput * dir);

  stepper1.runSpeed();
  stepper2.runSpeed();
}
