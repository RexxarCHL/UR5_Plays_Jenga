/**
 * End Effector Control
 * Provides a-la-carte functionalities for the Arduino based end-effector
 * Author: Chia-Hung Lin (clin110[AT]jhu[DOT]edu)
 * Date Created: 04/19/2018
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_VL6180X.h"
#include "HX711.h"

#include <ros.h>
#include <sensor_msgs/Range.h> // For range finder
#include <jenga_msgs/Probe.h> // For load cell probe
#include <jenga_msgs/EndEffectorControl.h> // For controlling the arduino 
#include <jenga_msgs/EndEffectorFeedback.h> // For feedbacks

#define GRIPPER_OPEN_WIDE_POSITION 2050
#define GRIPPER_OPEN_NARROW_POSITION 1500
#define GRIPPER_CLOSE_NARROW_POSITION 1235
#define GRIPPER_CLOSE_WIDE_POSITION 1915

void commandCallback(const jenga_msgs::EndEffectorControl& msg);
void publishFeedback(const uint8_t feedback_code);
void publishRangeMessage();
void publishProbeMessage();
void driveServo(int ms);

Adafruit_VL6180X range_finder = Adafruit_VL6180X();
Servo servo;
HX711 load_cell(A1, A0);

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
jenga_msgs::Probe probe_msg;
jenga_msgs::EndEffectorFeedback feedback_msg;
ros::Publisher range_publisher("/tool/range", &range_msg);
ros::Publisher probe_publisher("/tool/probe", &probe_msg);
ros::Publisher feedback_publisher("/tool/feedback", &feedback_msg);
ros::Subscriber<jenga_msgs::EndEffectorControl> command_subscriber("/tool/command", commandCallback);

bool probe_on, range_on;

void commandCallback(const jenga_msgs::EndEffectorControl& cmd)
{
  uint8_t command = cmd.command_code;
  
  char msg[24];
  sprintf(msg, "Received command: %d", command);
  nh.loginfo(msg);

  switch (command)
  {
    case jenga_msgs::EndEffectorControl::GRIPPER_OPEN_WIDE:
      driveServo(GRIPPER_OPEN_WIDE_POSITION);
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_OPENED);
      break;
    case jenga_msgs::EndEffectorControl::GRIPPER_OPEN_NARROW:
      driveServo(GRIPPER_OPEN_NARROW_POSITION);
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_OPENED);
      break;
    case jenga_msgs::EndEffectorControl::GRIPPER_CLOSE_WIDE:
      driveServo(GRIPPER_CLOSE_WIDE_POSITION);
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_CLOSED);
      break;
    case jenga_msgs::EndEffectorControl::GRIPPER_CLOSE_NARROW:
      driveServo(GRIPPER_CLOSE_NARROW_POSITION);
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_GRIPPER_CLOSED);
      break;
    case jenga_msgs::EndEffectorControl::PROBE_ON:
      probe_on = true;
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_PROBE_ON);
      break;
    case jenga_msgs::EndEffectorControl::PROBE_OFF:
      probe_on = false;
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_PROBE_OFF);
      break;
    case jenga_msgs::EndEffectorControl::RANGE_ON:
      range_on = true;
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_RANGE_ON);
      break;
    case jenga_msgs::EndEffectorControl::RANGE_OFF:
      range_on = false;
      publishFeedback(jenga_msgs::EndEffectorFeedback::ACK_RANGE_OFF);
      break;

    default:
      publishFeedback(jenga_msgs::EndEffectorFeedback::UNKNOWN_COMMAND);
  }
}

void publishFeedback(const uint8_t feedback_code)
{
  feedback_msg.header.stamp = nh.now();
  feedback_msg.feedback_code = feedback_code;
  feedback_publisher.publish(&feedback_msg);
}

void publishRangeMessage()
{
  if (!range_on)
    return;

  uint8_t range = range_finder.readRange();
  uint8_t status = range_finder.readRangeStatus();

  if(status == VL6180X_ERROR_NONE){
    range_msg.header.stamp = nh.now();
    range_msg.radiation_type = range_msg.INFRARED; // No laser enum...whatever
    range_msg.field_of_view = 0.43633231299; // about 25 deg
    range_msg.min_range = 0.005; // 5 mm
    range_msg.max_range = 0.2; // 200 mm
    range_msg.range = range;
    range_publisher.publish(&range_msg);
  }
}

void publishProbeMessage()
{
  if (!probe_on)
    return;

  load_cell.power_up();
  float force = load_cell.get_units(1);
  load_cell.power_down();

  probe_msg.header.stamp = nh.now();
  probe_msg.data = force;
  probe_publisher.publish(&probe_msg);
}

void driveServo(int ms)
{
  digitalWrite(13, HIGH); // Signal driving servo

  servo.writeMicroseconds(ms);

  long wait_timer = millis() + 1000;
  while(millis() < wait_timer)
  {
    nh.spinOnce(); // Wait 250ms for the servo to be in position
  }
  digitalWrite(13, LOW); // End signaling
}


void setup()
{
  /* Initialize node handle */
  nh.initNode();
  nh.getHardware()->setBaud(57600);

  /* Initialize subscribers and publishers */
  nh.advertise(range_publisher);
  nh.advertise(probe_publisher);
  nh.advertise(feedback_publisher);
  nh.subscribe(command_subscriber);

  /* Initialize global variables */
  probe_on = false;
  range_on = false;

  /* Initialize servo and sensors */
  nh.loginfo("Initializing sensors...");
  // Servo
  servo.attach(9);

  // Range finder
  while(!range_finder.begin())
  {
    nh.logfatal("Failed to find range finder");
  }

  // Load cell
  load_cell.set_scale(-12622.0); // Calibration value
  load_cell.tare();
}

void loop()
{
  // Try publish messages. Will return immediately if flags are not set
  publishRangeMessage();
  publishProbeMessage();

  nh.spinOnce();
}