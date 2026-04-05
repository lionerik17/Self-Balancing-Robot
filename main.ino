#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>

/* --- Configuration and Constants --- */
#define SAMPLE_TIME 0.005              // Control loop period (5ms)
#define RAD_TO_DEG (180 / M_PI)        // Constant to convert radians to degrees
#define GYRO_SENSITIVITY 131.0         // Sensitivity scale factor for MPU6050 gyroscope
#define alpha (0.75 / (0.75 + 0.005))  // Complementary filter coefficient
#define SPEED_OF_SOUND 0.0343          // Speed of sound in cm/us for ultrasonic sensor
#define NO_OF_MEASUREMENTS 40          // Number of measurements for Welford's algorithm tasks

/* --- MPU6050 Register Addresses --- */
#define MPU_ADDR 0x68                  // I2C address of the MPU6050
#define ACCEL_XOUT_H 0x3B              // Starting register for accelerometer data
#define GYRO_ZOUT_H 0x47               // Starting register for gyroscope data (Z-axis)

/* --- PID Controller Constants --- */
#define Kp 1.75                        // Proportional gain
#define Ki 0.015                       // Integral gain
#define Kd 0.45                        // Derivative gain
#define TARGET_ANGLE -90.0             // Desired vertical balance angle

/* --- Pin Definitions --- */
const int trigPin = 12;                // Ultrasonic sensor Trigger pin
const int echoPin = 3;                 // Ultrasonic sensor Echo pin

/* --- Global Variables --- */
volatile double accAngle = 0.0, gyroAngle = 0.0;
volatile double currentAngle = 0.0, previousAngle = 0.0;

double currentDistance = 0.0;

int16_t accX = 0, accY = 0, gyroZ = 0;
int countAngle = 0, countDistance = 0;

// Variables for Welford's algorithm (online mean/variance calculation)
double angleMean = 0.0, angleM2 = 0.0, angleVariance = 0.0;
double distanceMean = 0.0, distanceM2 = 0.0, distanceVariance = 0.0;

double delta = 0.0, duration = 0.0;

// PID state variables
double error = 0.0, prevError = 0.0, integral = 0.0, derivative = 0.0;
volatile double pidOutput = 0.0;

int currentTime = 0;

/* --- Motor Drivers Initialization --- */
// Using FULL4WIRE mode for stepper motors
AccelStepper stepper1(AccelStepper::FULL4WIRE, 11, 9, 10, 8);
AccelStepper stepper2(AccelStepper::FULL4WIRE, 7, 5, 6, 4);
int dir = 1;
double bound = 100.0;

/* --- Task Scheduler Definitions --- */
struct TaskStruct {
  int time;                     // Interval (in scheduler ticks) between task executions
  int count;                    // Current tick counter
  void (*taskFunction)(void);   // Pointer to the function to execute
};

// Task function prototypes
void readMPUTask();
void readDistanceTask();
void updateWelfordAngleTask();
void updateWelfordDistanceTask();
void controlMotorsTask();

// Array of tasks to be managed by the scheduler
TaskStruct tasks[] = {
    {NO_OF_MEASUREMENTS, 0, readMPUTask},           // Read MPU data periodically
    {NO_OF_MEASUREMENTS, 0, readDistanceTask},      // Read distance from ultrasonic sensor
    {NO_OF_MEASUREMENTS, 0, updateWelfordAngleTask}, // Statistical updates for angle
    {NO_OF_MEASUREMENTS, 0, updateWelfordDistanceTask}, // Statistical updates for distance
    {1, 0, controlMotorsTask},                      // Core PID control loop (runs every tick)
};

const int numTasks = sizeof(tasks) / sizeof(tasks[0]);

/* --- Task Implementation: Read MPU6050 Data --- */
void readMPUTask() {
  // Request accelerometer data (X and Y axes)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 4, true);

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();

  // Request gyroscope data (Z axis)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  gyroZ = (Wire.read() << 8) | Wire.read();
}

/* --- Task Implementation: Read Ultrasonic Distance --- */
void readDistanceTask() {
  // Trigger the ultrasonic sensor pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the duration of the echo pulse
  duration = pulseIn(echoPin, HIGH);
  // Calculate distance in cm
  currentDistance = (duration / 2) * SPEED_OF_SOUND;
}

/* --- Task Implementation: Update Statistics for Angle --- */
void updateWelfordAngleTask() {
  countAngle++;
  delta = currentAngle - angleMean;
  angleMean += delta / countAngle;
  angleM2 += delta * (currentAngle - angleMean);
  // Variance can be computed as: angleM2 / countAngle
}

/* --- Task Implementation: Update Statistics for Distance --- */
void updateWelfordDistanceTask() {
  countDistance++;
  delta = currentDistance - distanceMean;
  distanceMean += delta / countDistance;
  distanceM2 += delta * (currentDistance - distanceMean);
}

/* --- Task Implementation: Core Control Logic --- */
void controlMotorsTask() {
  // 1. Calculate tilt angle from accelerometer
  accAngle = atan2(accY, accX) * RAD_TO_DEG;
  
  // 2. Calculate angular velocity from gyroscope
  gyroAngle = (gyroZ / GYRO_SENSITIVITY) * SAMPLE_TIME;

  // 3. Complementary Filter: Combine high-pass (gyro) and low-pass (acc) filters
  // This reduces drift from gyro and noise from accelerometer
  currentAngle = alpha * (previousAngle + gyroAngle) + (1 - alpha) * accAngle;
  previousAngle = currentAngle;

  // 4. PID Controller Calculation
  error = currentAngle - TARGET_ANGLE;
  integral += error * SAMPLE_TIME;
  derivative = (error - prevError) / SAMPLE_TIME;
  prevError = error;

  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  // Constrain output to valid motor speeds
  pidOutput = constrain(pidOutput, -1500, 1500);

  // 5. Basic obstacle avoidance logic (using variance/bounds)
  if (distanceVariance > bound) {
    dir = -1;
  } else {
    dir = 1;
  }

  // Smoothly update the boundary threshold
  bound = bound * alpha + (1 - alpha) * bound;

  // 6. Set motor speeds based on PID output and direction
  stepper1.setSpeed(-pidOutput * dir);
  stepper2.setSpeed(-pidOutput * dir);

  // Step the motors
  stepper1.runSpeed();
  stepper2.runSpeed();
}

/* --- Scheduler Logic --- */
void scheduler() {
  for (int i = 0; i < numTasks; ++i) {
    ++tasks[i].count;
    if (tasks[i].count >= tasks[i].time) {
      tasks[i].count = 0;
      tasks[i].taskFunction();
    }
  }
}

/* --- Arduino Setup --- */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050 (Wake up from sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1 register
  Wire.write(0x00); // Wake up
  Wire.endTransmission(true);

  // Initialize Stepper Motor parameters
  stepper1.setMaxSpeed(1500);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(1500);
  stepper2.setAcceleration(1000);

  // Initialize Ultrasonic Sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

/* --- Arduino Main Loop --- */
void loop() {
  // The loop simply drives the task scheduler
  scheduler();
}
