#include <ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <Arduino.h>

// ros serial node handler
ros::NodeHandle nh;

/// DEFINE MOTOR PINS ///

#define STEP_J1 44  // Step signal (PUL-)
#define DIR_J1 42   // Direction signal (DIR-)
#define EN_J1 40    // Enable signal (ENA-), optional

#define STEP_J2 54  // X-axis step pin
#define DIR_J2 55   // X-axis direction pin
#define EN_J2 38    // Enable pin (low = active)

#define STEP_J3 60  // Y-axis step pin
#define DIR_J3 61   // Y-axis direction pin
#define EN_J3 56    // Y-axis enable pin

#define STEP_J4 46  // Z-axis step pin
#define DIR_J4 48   // Z-axis direction pin
#define EN_J4 62    // Z-axis enable pin

#define STEP_J5 36  // E0 step pin
#define DIR_J5 34   // E0 direction pin
#define EN_J5 30    // E0 enable pin

#define STEP_J6 26  // E1 step pin
#define DIR_J6 28   // E1 direction pin
#define EN_J6 24    // E1 enable pin

#define LIMIT_SWITCH_J1 3  // Limit switch pin for X-
#define LIMIT_SWITCH_J2 2  // Limit switch pin for Y-
#define LIMIT_SWITCH_J3 14  // Limit switch pin for Z-
#define LIMIT_SWITCH_J4 15 // Limit switch pin for E0-
#define LIMIT_SWITCH_J5 18  // Limit switch pin for E1-
#define LIMIT_SWITCH_J6 19 // Limit switch pin for DigStep-

Servo gripperServo;
unsigned long gripperStartTime = 0;
bool gripperActive = false;

struct StepperMotor {
  int stepPin;
  int dirPin;
  int enPin;
  float stepsPerDeg;
  float gearRatio;
  int delayBetweenSteps;
  int stepsToMove;
  int currentStep;
  bool moving;
  float targetPosition;
  float currentPosition;
};

// maps motors to array with they step pin, dir pin, en pin, step_angle, gear ratio, delay
StepperMotor motors[6] = {
  {STEP_J1, DIR_J1, EN_J1, 1.8, 100.0 / 16.0, 500, 0, 0, false, 0.0, 0.0},  // J1
  {STEP_J2, DIR_J2, EN_J2, 0.35, 100.0 / 16.0, 500, 0, 0, false, 0.0, 0.0}, // J2
  {STEP_J3, DIR_J3, EN_J3, 1.8, 100.0 / 16.0, 500, 0, 0, false, 0.0, 0.0},  // J3
  {STEP_J4, DIR_J4, EN_J4, 1.8, 60.0 / 16.0, 500, 0, 0, false, 0.0, 0.0},   // J4
  {STEP_J5, DIR_J5, EN_J5, 0.9, 1.0, 500, 0, 0, false, 0.0, 0.0},           // J5
  {STEP_J6, DIR_J6, EN_J6, 1.8, 1.0, 500, 0, 0, false, 0.0, 0.0}            // J6
};

void moveMotor(int motorNumber) {
  StepperMotor &motor = motors[motorNumber];
  float stepper_degrees = abs(motor.targetPosition - motor.currentPosition) * motor.gearRatio;
  motor.stepsToMove = int(stepper_degrees / motor.stepsPerDeg);
  motor.currentStep = 0;
  motor.moving = true;

  if (motor.targetPosition > motor.currentPosition) {
      digitalWrite(motor.dirPin, HIGH); // forward
  } else {
      digitalWrite(motor.dirPin, LOW); // reverse
  }
  // nh.loginfo("Executing motor movement");
}

void updateMotors() {
  for (int i = 0; i < 6; i++) {
    StepperMotor &motor = motors[i];
    if (motor.moving) {
      // nh.loginfo("Motor moving");
      if (motor.currentStep < motor.stepsToMove) {
        digitalWrite(motor.stepPin, HIGH);
        delayMicroseconds(motor.delayBetweenSteps);
        digitalWrite(motor.stepPin, LOW);
        delayMicroseconds(motor.delayBetweenSteps);
        motor.currentStep++;
        motor.currentPosition += (motor.targetPosition > motor.currentPosition) ? motor.stepsPerDeg / motor.gearRatio : -motor.stepsPerDeg / motor.gearRatio;
      } else {
        motor.moving = false;
        // nh.loginfo("Goal reached");
      }
    }
  }
}

void updateGripper(){
  if (gripperActive && (millis() - gripperStartTime >= 5000)) {
    gripperServo.detach();  // Stop the servo
    gripperActive = false;
    nh.loginfo("Gripper stopped");
  }
}

/// CALLBACK FUNCTIONS ///
void goalStateCb(const sensor_msgs::JointState& msg) {
  nh.loginfo("Goal state received");
  for (int i = 0; i < static_cast<int>(msg.position_length); i++) {
    if (i < 6) {
      motors[i].targetPosition = msg.position[i];
      moveMotor(i);
    }
  }
}

void gripperCb(const std_msgs::String& msg) {
  nh.loginfo("Gripper command received");
  if (strcmp(msg.data, "close") == 0) {
    gripperServo.write(0);  // Close the gripper
    nh.loginfo("Gripper closing");
    gripperStartTime = millis();
    gripperActive = true;
  } else if (strcmp(msg.data, "open") == 0) {
    gripperServo.write(180);  // Open the gripper
    nh.loginfo("Gripper opening");
    gripperStartTime = millis();
    gripperActive = true;
  } else {
    nh.logwarn("Unknown gripper command");
  }
}

/// INTERUPT SERVICE REQUESTS ///
void limitSwitchJ1ISR() {
  motors[0].currentPosition = 0.0;  // Set J1 position to 0
}
void limitSwitchJ2ISR() {
  motors[1].currentPosition = 0.0;  // Set J2 position to 0
}
void limitSwitchJ3ISR() {
  motors[2].currentPosition = 0.0;  // Set J3 position to 0
}
void limitSwitchJ4ISR() {
  motors[3].currentPosition = 0.0;  // Set J4 position to 0
}
void limitSwitchJ5ISR() {
  motors[4].currentPosition = 0.0;  // Set J5 position to 0
}
void limitSwitchJ6ISR() {
  motors[5].currentPosition = 0.0;  // Set J6 position to 0
}

// SUBSCRIBERS
ros::Subscriber<sensor_msgs::JointState> goalStateSub("/goal_state", &goalStateCb);
ros::Subscriber<std_msgs::String> gripperSub("/gripper", &gripperCb);

// PUBLISHERS
sensor_msgs::JointState jointStateMsg;
ros::Publisher jointStatePub("/joint_states", &jointStateMsg);

void setup() {
  // start ROS node
  nh.initNode();
  nh.subscribe(goalStateSub);
  nh.subscribe(gripperSub);
  nh.advertise(jointStatePub);
  
  // Initialize stepper motor pins
  for (int i = 0; i < 6; i++) {
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    pinMode(motors[i].enPin, OUTPUT);
    digitalWrite(motors[i].enPin, LOW);  // Enable motor
    motors[i].currentPosition = 0.0;     // Initialize all motors to 0 position
  }

  // Initialize limit switch
  pinMode(LIMIT_SWITCH_J1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J1), limitSwitchJ1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J2), limitSwitchJ2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J3), limitSwitchJ3ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J4), limitSwitchJ4ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J5), limitSwitchJ5ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J6), limitSwitchJ6ISR, FALLING);

  // Initialize servo
  gripperServo.attach(4);  // Attach the servo to pin 4
  

  jointStateMsg.position_length = 6;
  jointStateMsg.position = new float[6];
  delay(1000);  // Add a delay to allow the node to initialize
}

void publishJointStates() {
  for (int i = 0; i < 6; i++) {
    jointStateMsg.position[i] = motors[i].currentPosition;
  }
  jointStatePub.publish(&jointStateMsg);
}

void loop() {
  nh.spinOnce();
  updateMotors();
  updateGripper();
  publishJointStates();
  delay(1);
}