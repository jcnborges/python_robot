#include <PID_v1.h>
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
const unsigned long timer_interval = 100; // Timer interval in milliseconds
double time_factor = 60000.00d / timer_interval; // Time factor for speed calculation

// PID Controller variables
double setpoint1 = 0, setpoint2 = 0; // Desired speed (pulses per second)
double input1, input2, output1, output2; // Inputs and outputs for PID
double Kp[] = {0.03, 0.03}; // PID constants
double Ki[] = {0.3, 0.3};
double Kd[] = {0.0005, 0.0005};
PID myPID1(&input1, &output1, &setpoint1, Kp[0], Ki[0], Kd[0], DIRECT); // PID for motor 1
PID myPID2(&input2, &output2, &setpoint2, Kp[1], Ki[1], Kd[1], DIRECT); // PID for motor 2

// Float for number of slots in encoder disk
float diskslots = 20.00f;  // Change to match value of encoder disk
unsigned int encoder1_count = 0; // Encoder 1 counts
unsigned int encoder2_count = 0; // Encoder 2 counts

unsigned long previousMillis = 0;

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
  Timer1.attachInterrupt(calculateSpeed);

  // Initialize PID controllers
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(0, 255); // Limit output to PWM range
  myPID1.SetSampleTime(timer_interval);
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(0, 255);
  myPID1.SetSampleTime(timer_interval);

  // Set I2C slave
  Wire.begin(4);
  Wire.onReceive(receiveEvent);
}

void loop() {
  // Print values
  if (millis() - previousMillis > timer_interval) {
    previousMillis = millis();
    Serial.print("SP1:");
    Serial.print(setpoint1);
    Serial.print(",");
    Serial.print("RPM1:"); 
    Serial.print(input1);
    Serial.print(",");
    Serial.print("DC1:"); 
    Serial.print(output1);
    Serial.print(",");
    Serial.print("SP2:"); 
    Serial.print(setpoint2);
    Serial.print(",");
    Serial.print("RPM2:"); Serial.print(input2);
    Serial.print(",");
    Serial.print("DC2:"); Serial.print(output2);
    Serial.println();
  }
}

// Calculate speed based on encoder count and time
double calculateSpeed() {
  Timer1.detachInterrupt();  // Stop the timer
  
  input1 = (encoder1_count / diskslots) * time_factor;  // calculate RPM for Motor 1
  encoder1_count = 0;  //  reset counter to zero
  input2 = (encoder2_count / diskslots) * time_factor;  // calculate RPM for Motor 2
  encoder2_count = 0;  //  reset counter to zero
  
  // Update PID controllers
  myPID1.Compute();
  myPID2.Compute();
  
  // Control motors based on PID outputs
  analogWrite(motor1_pwm, output1);
  analogWrite(motor2_pwm, output2);

  Timer1.attachInterrupt(calculateSpeed);  // Enable the timer
}

// Interrupt handlers for encoder readings (using single-pin)
void encoder1_interrupt() {
  encoder1_count++; // Increment count on any change
}

void encoder2_interrupt() {
  encoder2_count++; // Increment count on any change
}

void receiveEvent(int howMany) {
  if (howMany == 4) { // Check for 4 bytes (motor, direction, setpoint high, setpoint low)
    byte motor = Wire.read();
    byte direction = Wire.read();
    unsigned int setpoint = Wire.read() | Wire.read() << 8;    
    switch (motor) {
      case 1: {
        if (direction == 1) {
          // Set motor 1 forward
          digitalWrite(motor1_dir1, HIGH);
          digitalWrite(motor1_dir2, LOW);
        } else if (direction == 0) {
          // Set motor 1 reverse
          digitalWrite(motor1_dir1, LOW);
          digitalWrite(motor1_dir2, HIGH);          
        } 
        setpoint1 = setpoint;  // Apply setpoint directly
        break;
      }
      case 2: {
        if (direction == 1) {
          // Set motor 2 forward
          digitalWrite(motor2_dir1, HIGH);
          digitalWrite(motor2_dir2, LOW);
        } else if (direction == 0) {
          // Set motor 2 reverse
          digitalWrite(motor2_dir1, LOW);
          digitalWrite(motor2_dir2, HIGH);          
        }
        setpoint2 = setpoint; 
        break;
      }
    }
  }
}