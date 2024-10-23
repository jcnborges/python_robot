#include <QuickPID.h>
#include <Wire.h>
#include <TimerOne.h>

// Define pin connections
const int motor1_pwm = 11; // Motor 1 PWM pin
const int motor1_dir1 = 12; // Motor 1 Direction pin 1 (forward)
const int motor1_dir2 = 13; // Motor 1 Direction pin 2 (reverse)
const int motor2_pwm = 5; // Motor 2 PWM pin
const int motor2_dir1 = 6; // Motor 2 Direction pin 1 (forward)
const int motor2_dir2 = 7; // Motor 2 Direction pin 2 (reverse)
const int encoder1_A = 2; // Encoder 1 Channel A
const int encoder2_A = 3; // Encoder 2 Channel A
const int adc_channel = A0;

const unsigned long timer_interval = 50; // Timer interval in milliseconds
float time_factor = 1000.00d / timer_interval; // Time factor for Pulse Rate calculation

// PID Controller variables
float setpoint1 = 0, setpoint2 = 0; // Desired speed (pulses per second)
float input1, input2, output1, output2; // Inputs and outputs for PID
float Kp[] = {5, 5}; // PID constants
float Ki[] = {20, 30};
float Kd[] = {0.001, 0.001}; 

QuickPID myPID1(&input1, &output1, &setpoint1, Kp[0], Ki[0], Kd[0], QuickPID::pMode::pOnError, 
  QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct); // PID for motor 1

QuickPID myPID2(&input2, &output2, &setpoint2, Kp[1], Ki[1], Kd[1], QuickPID::pMode::pOnError, 
  QuickPID::dMode::dOnMeas, QuickPID::iAwMode::iAwCondition, QuickPID::Action::direct); // PID for motor 2

// float for number of slots in encoder disk
float diskslots = 20.00f;  // Change to match value of encoder disk
volatile unsigned int encoder1_count = 0; // Encoder 1 counts
volatile unsigned int encoder2_count = 0; // Encoder 2 counts

unsigned long previousMillis = 0;

// Robot parameters
const float wheel_radius = 0.034f; // Wheel radius in meters
const float wheel_base = 0.107f; // Distance between wheels in meters

// Differential Drive Control Variables
float linear_velocity = 0.0f; // Desired linear velocity in m/s
float angular_velocity = 0.0f; // Desired angular velocity in rad/s
float wheel2_speed, wheel1_speed; // Calculated wheel speeds in rad/s

void setup() {
  Serial.begin(9600);

  // Set pin modes
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir1, OUTPUT);
  pinMode(motor1_dir2, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir1, OUTPUT);
  pinMode(motor2_dir2, OUTPUT);

  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);

  // Attach interrupt handlers for encoders (using RISING mode for single-pin)
  attachInterrupt(digitalPinToInterrupt(encoder1_A), encoder1_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_A), encoder2_interrupt, RISING);
  Timer1.initialize(timer_interval * 1000);
  Timer1.attachInterrupt(calculatePulseRate);

  // Initialize PID controllers
  myPID1.SetMode(QuickPID::Control::automatic);
  myPID1.SetOutputLimits(-255, 255); // Limit output to PWM range
  myPID1.SetSampleTimeUs(timer_interval * 1000);
  myPID1.Initialize();

  myPID2.SetMode(QuickPID::Control::automatic);
  myPID2.SetOutputLimits(-255, 255);
  myPID2.SetSampleTimeUs(timer_interval * 1000);
  myPID2.Initialize();

  // Set I2C slave
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  // Print values
  if (millis() - previousMillis > timer_interval) {
    previousMillis = millis();
    Serial.print("SP1:");
    Serial.print(setpoint1);
    Serial.print(",");
    Serial.print("PPS1:"); 
    Serial.print(input1);
    Serial.print(",");
    Serial.print("DC1:"); 
    Serial.print(output1);
    Serial.print(",");
    Serial.print("SP2:"); 
    Serial.print(setpoint2);
    Serial.print(",");
    Serial.print("PPS2:"); Serial.print(input2);
    Serial.print(",");
    Serial.print("DC2:"); Serial.print(output2);
    Serial.println();
  }
}

// Calculate Pulse Rate (pulses per second) on encoder count and time
float calculatePulseRate() {
  input1 = (encoder1_count / diskslots) * time_factor;  // calculate Pulse Rate for Motor 1
  encoder1_count = 0;  //  reset counter to zero

  input2 = (encoder2_count / diskslots) * time_factor;  // calculate Pulse Rate for Motor 2
  encoder2_count = 0;  //  reset counter to zero
  
  // Update PID controllers
  myPID1.Compute();
  myPID2.Compute();
  
  // Control motors based on PID outputs
  controlMotor(motor1_pwm, motor1_dir1, motor1_dir2, output1);
  controlMotor(motor2_pwm, motor2_dir1, motor2_dir2, output2);
}

// Interrupt handlers for encoder readings (using single-pin)
void encoder1_interrupt() {
  encoder1_count++; // Increment count on any change
}

void encoder2_interrupt() {
  encoder2_count++; // Increment count on any change
}

void receiveEvent(int howMany) {
  if (howMany == 8) { // Check for 4 bytes (linear_velocity, angular_velocity)
      float linear_velocity = receivefloat();
      float angular_velocity = receivefloat();
      if (!isnan(linear_velocity) && !isnan(angular_velocity)) {
        setLinearVelocity(linear_velocity);
        setAngularVelocity(angular_velocity);
        calculateWheelSpeeds();
        refreshSetpoints();
      }
  }
}

float receivefloat() {
  byte bytes[4]; // Array to store the 4 bytes
  for (int i = 0; i < 4; i++) {
    bytes[i] = Wire.read(); 
  }

  // Combine the bytes into a float using a union
  union {
    float f;
    byte b[4];
  } data;
  
  memcpy(data.b, bytes, 4); // Copy the bytes into the union
  return data.f;
}

void setLinearVelocity(float velocity) {
  linear_velocity = velocity;
  calculateWheelSpeeds(); // Update wheel speeds
}

void setAngularVelocity(float velocity) {
  angular_velocity = velocity;
  calculateWheelSpeeds(); // Update wheel speeds
}

// Function to calculate wheel speeds from linear and angular velocities
void calculateWheelSpeeds() {
  wheel1_speed = (linear_velocity + (angular_velocity * wheel_base / 2)) / wheel_radius;
  wheel2_speed = (linear_velocity - (angular_velocity * wheel_base / 2)) / wheel_radius;    
}

void refreshSetpoints() {
  setpoint1 = (wheel1_speed * diskslots) / (2 * PI);
  setpoint2 = (wheel2_speed * diskslots) / (2 * PI);

  if (myPID1.GetDirection() == 1 && setpoint1 >= 0) {
    myPID1.SetControllerDirection(QuickPID::Action::direct);
    myPID1.Reset();
  } else if (myPID1.GetDirection() == 0 && setpoint1 < 0) {
    myPID1.SetControllerDirection(QuickPID::Action::reverse);
    myPID1.Reset();
  }

  if (myPID2.GetDirection() == 1 && setpoint2 >= 0) {
    myPID2.SetControllerDirection(QuickPID::Action::direct);
    myPID2.Reset();
  } else if (myPID2.GetDirection() == 0 && setpoint2 < 0) {
    myPID2.SetControllerDirection(QuickPID::Action::reverse);
    myPID2.Reset();
  }

  setpoint1 = abs(setpoint1);
  setpoint2 = abs(setpoint2);
}

// Control motor direction and speed based on PID output
void controlMotor(int pwm, int dir1, int dir2, float output) {
  if (output > 0) {
    analogWrite(pwm, abs(output)); // Set PWM duty cycle
    digitalWrite(dir1, HIGH); // Set direction to forward
    digitalWrite(dir2, LOW);
  } else {
    analogWrite(pwm, abs(output)); // Set PWM duty cycle
    digitalWrite(dir1, LOW); // Set direction to reverse
    digitalWrite(dir2, HIGH);
  }
}

void requestEvent() {
  int adc_level = analogRead(adc_channel);
  byte byte_array[2];
  
  // Send the ADC value back to Raspberry Pi
  byte_array[0] = (adc_level >> 8) & 0xFF; // Send high byte
  byte_array[1] = adc_level & 0xFF; // Send low byte   

  Serial.println(adc_level);

  Wire.write(byte_array, 2);
}