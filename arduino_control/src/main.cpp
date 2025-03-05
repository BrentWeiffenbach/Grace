#include <ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <Servo.h>
#include <std_msgs/String.h>
#include <LinkedList.h>
#include <Arduino.h>

// PUBLISHERS
sensor_msgs::JointState jointStateMsg;
ros::Publisher jointStatePub("/grace/arm_joint_angles", &jointStateMsg);
std_msgs::String armStatusMsg;
ros::Publisher armStatusPub("/grace/arm_status", &armStatusMsg);

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
#define LIMIT_SWITCH_J2 2  // Limit switch pin for X+
#define LIMIT_SWITCH_J3 20  // Limit switch pin for I2C 20
#define LIMIT_SWITCH_J4 21 // Limit switch pin for I2C 21
#define LIMIT_SWITCH_J5 18  // Limit switch pin for D18 Z-
#define LIMIT_SWITCH_J6 19 // Limit switch pin for D19 Z+

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
  LinkedList<float> targetPosition;
  float currentPosition;
};

enum ArmStatus {
  WAITING,
  EXECUTING,
  COMPLETED
};

ArmStatus currentStatus = WAITING;

// maps motors to array with they step pin, dir pin, en pin, step_angle, gear ratio, delay
// TODO TUNE DELAY
StepperMotor motors[6] = {
  {STEP_J1, DIR_J1, EN_J1, 1.8, 100.0 / 16.0, 8000, 0, 0, false, LinkedList<float>(), 0.0},  // J1
  {STEP_J2, DIR_J2, EN_J2, 0.35, 100.0 / 16.0, 8000, 0, 0, false, LinkedList<float>(), 0.0}, // J2
  {STEP_J3, DIR_J3, EN_J3, 1.8, 100.0 / 16.0, 8000, 0, 0, false, LinkedList<float>(), 0.0},  // J3
  {STEP_J4, DIR_J4, EN_J4, 1.8, 60.0 / 16.0, 8000, 0, 0, false, LinkedList<float>(), 0.0},   // J4
  {STEP_J5, DIR_J5, EN_J5, 0.9, 32.0 / 16.0, 8000, 0, 0, false, LinkedList<float>(), 0.0},           // J5
  {STEP_J6, DIR_J6, EN_J6, 1.8, 1.0, 8000, 0, 0, false, LinkedList<float>(), 0.0}            // J6
};

void moveMotor(int motorNumber) {
  StepperMotor &motor = motors[motorNumber];
  char debugMsg[100];
  snprintf(debugMsg, 100, "target position for motor %d: %f", motorNumber, static_cast<double>(motor.targetPosition.get(0)));
  nh.loginfo(debugMsg);
  float stepper_degrees = abs(motor.targetPosition.get(0) - motor.currentPosition) * motor.gearRatio;
  motor.stepsToMove = int(stepper_degrees / motor.stepsPerDeg);
  motor.currentStep = 0;
  motor.moving = true;
  char logMsg[100];
  snprintf(logMsg, 100, "Moving motor %d, steps to move: %d", motorNumber, motor.stepsToMove);
  nh.loginfo(logMsg);

  if (motor.targetPosition.get(0) < motor.currentPosition) {
      digitalWrite(motor.dirPin, HIGH); // reverse
  } else {
      digitalWrite(motor.dirPin, LOW); // forward
  }
  // nh.loginfo("Executing motor movement");
}

/// UPDATE FUNCTIONS ///
void updateArmStatus() {
  switch (currentStatus) {
    case WAITING:
      armStatusMsg.data = "waiting";
      break;
    case EXECUTING:
      armStatusMsg.data = "executing";
      break;
    case COMPLETED:
      armStatusMsg.data = "completed";
      break;
    }
    armStatusPub.publish(&armStatusMsg);
}

void updateMotors() {
  bool allMotorsStopped = true;
  for (int i = 0; i < 6; i++) {
    StepperMotor &motor = motors[i];
    if (motor.moving) {
      allMotorsStopped = false;
      if (motor.currentStep < motor.stepsToMove) {
        digitalWrite(motor.stepPin, HIGH);
        delayMicroseconds(motor.delayBetweenSteps);
        digitalWrite(motor.stepPin, LOW);
        delayMicroseconds(motor.delayBetweenSteps);
        motor.currentStep++;
        motor.currentPosition += (motor.targetPosition.get(0) > motor.currentPosition) ? motor.stepsPerDeg / motor.gearRatio : -motor.stepsPerDeg / motor.gearRatio;
      } else {
        nh.loginfo("Reached trajectory point, moving to next");
        motor.targetPosition.remove(0);
        if (motor.targetPosition.size() == 0) {
          nh.loginfo("Stopping motor, no more target positions");
          motor.moving = false;
        }
      }
    }
  }
  if (allMotorsStopped && currentStatus == EXECUTING) {
    currentStatus = COMPLETED;
    updateArmStatus();
  }
}

void updateGripper(){
  if (gripperActive && (millis() - gripperStartTime >= 5000)) {
    if (gripperServo.read() == 180) {
      gripperServo.detach();  // Stop the servo if it was opened
    }
    gripperActive = false;
    nh.loginfo("Gripper stopped");
  }
}

/// CALLBACK FUNCTIONS ///
void goalStateCb(const sensor_msgs::JointState& msg) {
  nh.loginfo(("Trajectory point received"));
  // add point to target position list
  for (int i = 0; i < static_cast<int>(msg.position_length); i++) {
    if (i < 6) {
      motors[i].targetPosition.add(msg.position[i]);
    }
  }
  
  // check if it is final point in trajectory
  const char* header = msg.header.frame_id;
  const char* goal = "Goal";
  char debugMsg[100];
  snprintf(debugMsg, 100, "Received joint state: %f, header: %s, goal: %s", static_cast<double>(msg.position[0]), header, goal);
  nh.loginfo(debugMsg);
  if (strcmp(header, goal) == 0) {
    nh.loginfo("Received the final point in the trajectory");
    currentStatus = EXECUTING;
    updateArmStatus();
    for (int j = 0; j < 6; j++) {
      moveMotor(j);
    }
  
  } else {
    nh.loginfo("Point is not goal");
    currentStatus = WAITING;
    updateArmStatus();
  }
}

void gripperCb(const std_msgs::String& msg) {
  nh.loginfo("Gripper command received");
  gripperServo.attach(4);
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
volatile unsigned long lastDebounceTimeJ1 = 0;
volatile unsigned long lastDebounceTimeJ2 = 0;
volatile unsigned long lastDebounceTimeJ3 = 0;
volatile unsigned long lastDebounceTimeJ4 = 0;
volatile unsigned long lastDebounceTimeJ5 = 0;
volatile unsigned long lastDebounceTimeJ6 = 0;
const unsigned long debounceDelay = 50;  // 50 milliseconds debounce delay

void limitSwitchJ1ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ1) > debounceDelay) {
    motors[0].currentPosition = 0.0;  // Set J1 position to 0
    lastDebounceTimeJ1 = currentTime;
  }
}

void limitSwitchJ2ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ2) > debounceDelay) {
    motors[1].currentPosition = 0.0;  // Set J2 position to 0
    lastDebounceTimeJ2 = currentTime;
  }
}

void limitSwitchJ3ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ3) > debounceDelay) {
    motors[2].currentPosition = 0.0;  // Set J3 position to 0
    lastDebounceTimeJ3 = currentTime;
  }
}

void limitSwitchJ4ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ4) > debounceDelay) {
    motors[3].currentPosition = 0.0;  // Set J4 position to 0
    lastDebounceTimeJ4 = currentTime;
  }
}

void limitSwitchJ5ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ5) > debounceDelay) {
    motors[4].currentPosition = 0.0;  // Set J5 position to 0
    lastDebounceTimeJ5 = currentTime;
  }
}

void limitSwitchJ6ISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTimeJ6) > debounceDelay) {
    motors[5].currentPosition = 0.0;  // Set J6 position to 0
    lastDebounceTimeJ6 = currentTime;
  }
}

// SUBSCRIBERSrosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
ros::Subscriber<sensor_msgs::JointState> goalStateSub("/grace/arm_goal", &goalStateCb);
ros::Subscriber<std_msgs::String> gripperSub("/grace/gripper", &gripperCb);

void setup() {
  // start ROS node
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(goalStateSub);
  nh.subscribe(gripperSub);
  nh.advertise(jointStatePub);
  nh.advertise(armStatusPub);
  
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
  pinMode(LIMIT_SWITCH_J2, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_J3, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_J4, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_J5, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_J6, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J1), limitSwitchJ1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J2), limitSwitchJ2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J3), limitSwitchJ3ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J4), limitSwitchJ4ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J5), limitSwitchJ5ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_J6), limitSwitchJ6ISR, FALLING);
  
  // Initialize servo
  gripperServo.attach(4);  // Attach the servo to pin 4
  

  jointStateMsg.position_length = 6;
  static float jointPositions[6];
  jointStateMsg.position = jointPositions;
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