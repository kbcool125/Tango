#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <tango_msgs/relays.h>
#include <tango_msgs/lights.h>
#include <tango_msgs/bumpers.h>
#include <tango_msgs/fallsensors.h>
#include <tango_msgs/ultrasonics.h>
#include <tango_msgs/infrareds.h>
#include <tango_msgs/motor_commands.h>
#include <tango_msgs/wheel_encoders.h>
#include <Encoder.h>
#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include <SharpIR.h>

#if defined(WIRE_T3)
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

#include "imu_configuration.h"

boolean arcRobot = true;
boolean useBumpers = true;
boolean useFallSensors = false;
boolean useIRSensors = true;
boolean useUSSensors = false;
boolean useBatterySensor = true;
boolean useIMU = true;

const byte nLowerRelays = 4;
const byte nUpperRelays = 4;
const byte nLightsUpper = 3;
const byte nLightsLower = 3;
const byte nIRSensors = 7;
const byte nMotors = 2;
const byte nBumpers = 3;
const byte nFallSensors = 4;

uint32_t last_time = 0;
uint8_t update_rate = 50; //Hz
bool is_first = true;

// Define Left Encoder Pins
// Right Encoder A-2, Right Encoder B-3
const byte rightEncoderPins[] = {2, 3};

// Define Digital Pin for Ultrasonics
const byte usDigitalPin = 4;

// Define Motor Serial Tx Pin
const byte motorPin = 10;

// Define Max485 (Ultrasonic) Serial Pins
// Ultrasonic Tx-14, Ultrasonic Rx-15
const byte ultrasonicSerial[] = {14, 15};

// Define Right Encoder Pins
// Left Encoder A-18, Left Encoder B-19
const byte leftEncoderPins[] = {18, 19};

// Define Lower Relay Pins
// Motor Relay-22, Unused-23
// Infrared Relay-24, IMU/Fall Sensor Relay-25
const byte lowerRelayPins[] = {22, 23, 24, 25};


// Define Upper Relay Pins
// Slam Device Relay-26, Upper Lights Relay-27
// Lower Lights Relay-28, Unused-29
const byte upperRelayPins[] = {26, 27, 28, 29};

// Define Bumper Pins
// Center Bumper-32, Left Bumper-33, Right Bumper-34
const byte bumperPins[] = {32, 33, 34};

// Define Upper R, G, B Pins
// Upper Red-38, Upper Green-39, Upper Blue-40
const byte lightsUpperPins[] = {38, 39, 40};

// Define Lower R, G, B Pins
// Lower Red-41, Lower Green-42, Lower Blue-43
const byte lightsLowerPins[] = {41, 42, 43};

// Define Bluetooth Serial Pins
// Bluetooth Rx-48, Bluetooth Tx-49
const byte btSerialPins[] = {48, 49};

// Define Rear IR Sensor Pins
// Rear Left IR-A0, Front Right Outer IR-A1, Front Center IR-A2
// Front Left Outer IR-A3, Rear Right IR-A4,
// Front Right Inner IR-A5, Front Left Inner IR-A6
const byte irSensorPins[] = {A0, A1, A2, A3, A4, A5, A6};

// Define Voltage Sensor Pin
const byte voltageSensorPin = A8;

// Define FallSensorPins
// Front Right Fall-A12, Rear Right Fall-A13
// Front Left Fall-A14, Rear Left Fall-A15
const byte fallSensorPins[] = {A12, A13, A14, A15}; 

Encoder leftEncoder(leftEncoderPins[0], leftEncoderPins[1]);
Encoder rightEncoder(rightEncoderPins[0], rightEncoderPins[1]);

SoftwareSerial SWSerial(NOT_A_PIN, motorPin);
Sabertooth ST(128,SWSerial);

SharpIR front_c_ir(GP2Y0A21YK0F, irSensorPins[2]);
SharpIR front_r_inner_ir(GP2Y0A21YK0F, irSensorPins[5]);
SharpIR front_r_outer_ir(GP2Y0A21YK0F, irSensorPins[1]);
SharpIR rear_r_ir(GP2Y0A21YK0F, irSensorPins[4]);
SharpIR rear_l_ir(GP2Y0A21YK0F, irSensorPins[0]);
SharpIR front_l_outer_ir(GP2Y0A21YK0F, irSensorPins[3]);
SharpIR front_l_inner_ir(GP2Y0A21YK0F, irSensorPins[6]);

byte bumperValue = 0;
byte previousBumperValue = 0;
byte fallSensorValue = 0;
byte previousFallSensorValue = 0;
byte bumperVals[nBumpers] = {0, 0, 0};
int fallSensorVals[nFallSensors] = {0, 0, 0, 0};
int irSensorVals[nIRSensors] = {0, 0, 0, 0, 0, 0, 0};

long lOldEncoderPos  = -999;
long rOldEncoderPos  = -999;

int v1;
float v2;
float vout;
float r1 = 30000.0;
float r2 = 7500.0;
char battOutput[8];
long previousBatteryMillis = 0;
long batteryPubRate = 10000;
long previousEncoderMillis = 0;
long encoderPubRate = 50;
long previousIRMillis = 0;
long irPubRate = 250; 
long previousUSMillis = 0;
long usPubRate = 50; 

ros::NodeHandle  nh;

tango_msgs::bumpers bumpers_msg;
tango_msgs::lights lights_msg;
tango_msgs::relays relays_msg;
tango_msgs::fallsensors fallsensors_msg;
tango_msgs::motor_commands motor_commands_msg;
tango_msgs::wheel_encoders wheel_encoders_msg;
tango_msgs::infrareds infrareds_msg;
tango_msgs::ultrasonics ultrasonics_msg;
sensor_msgs::BatteryState batteryState_msg;
ros_arduino_msgs::RawImu raw_imu_msg;

ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);
ros::Publisher rWheelEncoders("wheel_encoders", &wheel_encoders_msg);
ros::Publisher rBumpers("tango_msgs/bumpers", &bumpers_msg);
ros::Publisher rFallSensors("tango_msgs/fallsensors", &fallsensors_msg);
ros::Publisher rIRSensors("infrareds", &infrareds_msg);
//ros::Publisher rUSSensors("/tango_msgs/ultrasonics", &ultrasonics_msg);
ros::Publisher rBatteryState("battery", &batteryState_msg);

void relayOnOff(byte relayNum, byte on=0){
  switch (relayNum){
    case 1:
      nh.logdebug("Changing state of motor controller power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[0], LOW);
      }
      break;
    case 2:
      nh.logdebug("Changing state of unused power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[1], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[1], LOW);
      }
      break;
    case 3:
      nh.logdebug("Changing state of infrared sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[2], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[2], LOW);
      }
      break;
    case 4:
      nh.logdebug("Changing state of IMU and fall sensors power relay");
      if (on == 1){
        digitalWrite(lowerRelayPins[3], HIGH);
      } else if (on == 0){
        digitalWrite(lowerRelayPins[3], LOW);
      }
      break;
    case 5:
      nh.logdebug("Changing state of slam device power relay");
      if (on == 1){
        digitalWrite(upperRelayPins[0], HIGH);
      } else if (on == 0){
        digitalWrite(upperRelayPins[0], LOW);
      }
      break;
  }    
  if (arcRobot == false){
    switch (relayNum){
      case 6:
        nh.logdebug("Changing state of upper lights power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[1], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[1], LOW);
        }
        break;
      case 7:
        nh.logdebug("Changing state lower lights power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[2], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[2], LOW);
        }
        break;
      case 8:
        nh.logdebug("Changing state of unused power relay");
        if (on == 1){
          digitalWrite(upperRelayPins[3], HIGH);
        } else if (on == 0){
          digitalWrite(upperRelayPins[3], LOW);
        }
        break;
    }
  }
}

void rRelaysCb(const tango_msgs::relays& relays_msg){
  if (relays_msg.state == 1){
    if (arcRobot == false){
      relayOnOff(relays_msg.number, 1);
    } else if (arcRobot == true){
      relayOnOff(relays_msg.number, 0);
    }
  } else if (relays_msg.state == 0){
    if (arcRobot == false){
      relayOnOff(relays_msg.number, 0);
    } else if(arcRobot == true){
      relayOnOff(relays_msg.number, 1);
    }
  }  
}

void rLightsCb(const tango_msgs::lights& lights_msg){
  if (lights_msg.tiers == 1){
    switch (lights_msg.colour){
      case 0:
        nh.logdebug("Turning upper lights off");
        digitalWrite(lightsUpperPins[0], LOW);
        digitalWrite(lightsUpperPins[1], LOW);
        digitalWrite(lightsUpperPins[2], LOW);
        break;
      case 1:
        nh.logdebug("Changing upper lights to white");
        digitalWrite(lightsUpperPins[0], HIGH);
        digitalWrite(lightsUpperPins[1], HIGH);
        digitalWrite(lightsUpperPins[2], HIGH);
        break;
      case 2:
        nh.logdebug("Changing upper lights to red");
        digitalWrite(lightsUpperPins[0], HIGH);
        digitalWrite(lightsUpperPins[1], LOW);
        digitalWrite(lightsUpperPins[2], LOW);
        break;
      case 3:
        nh.logdebug("Changing upper lights to green");
        digitalWrite(lightsUpperPins[0], LOW);
        digitalWrite(lightsUpperPins[1], HIGH);
        digitalWrite(lightsUpperPins[2], LOW);
        break;
      case 4:
        nh.logdebug("Changing upper lights to blue");
        digitalWrite(lightsUpperPins[0], LOW);
        digitalWrite(lightsUpperPins[1], LOW);
        digitalWrite(lightsUpperPins[2], HIGH);
        break; 
    }
  } else if (lights_msg.tiers == 0){
    switch (lights_msg.colour){
      case 0:
        nh.logdebug("Turning lower lights off");
        digitalWrite(lightsLowerPins[0], LOW);
        digitalWrite(lightsLowerPins[1], LOW);
        digitalWrite(lightsLowerPins[2], LOW);
        break;
      case 1:
        nh.logdebug("Changing lower lights to white");
        digitalWrite(lightsLowerPins[0], HIGH);
        digitalWrite(lightsLowerPins[1], HIGH);
        digitalWrite(lightsLowerPins[2], HIGH);
        break; 
      case 2:
        nh.logdebug("Changing lower lights to red");
        digitalWrite(lightsLowerPins[0], HIGH);
        digitalWrite(lightsLowerPins[1], LOW);
        digitalWrite(lightsLowerPins[2], LOW);
        break;
      case 3:
        nh.logdebug("Changing lower lights to green");
        digitalWrite(lightsLowerPins[0], LOW);
        digitalWrite(lightsLowerPins[1], HIGH);
        digitalWrite(lightsLowerPins[2], LOW);
        break;
      case 4:
        nh.logdebug("Changing lower lights to blue");
        digitalWrite(lightsLowerPins[0], LOW);
        digitalWrite(lightsLowerPins[1], LOW);
        digitalWrite(lightsLowerPins[2], HIGH);
        break;  
    }
  }
}

void rMotorCmdsCb(const tango_msgs::motor_commands& motor_commands_msg){
  ST.motor(1, (int)motor_commands_msg.l_motor);
  ST.motor(2, (int)motor_commands_msg.r_motor);
}

ros::Subscriber<tango_msgs::relays> rRelays("/tango_msgs/relays", &rRelaysCb);
ros::Subscriber<tango_msgs::lights> rLights("/tango_msgs/lights", &rLightsCb);
ros::Subscriber<tango_msgs::motor_commands> rMotorCommands("/motor_commands", &rMotorCmdsCb);

void readBatteryVoltage(){
  if((millis()-previousBatteryMillis) > batteryPubRate){
    v1 = analogRead(voltageSensorPin);
    v2 = ((v1*5.0)/1024.0);
    vout = v2/(r2/(r1+r2));
    dtostrf(vout, 6, 2, battOutput);
    nh.logdebug("Battery Voltage = ");
    nh.logdebug(battOutput);
    if (vout < 11.2){
      nh.logwarn("Battery voltage is below 11.2V. Please charge battery soon");
      nh.logwarn("Consider shutting down tango to charge the main battery");
    } else if (vout < 10.8){
      nh.logerror("Battery voltage critial. Stopping all operations!");
      nh.logerror("Please shutdown tango and charge the main battery");
    }
    batteryState_msg.voltage = vout;
    batteryState_msg.header.stamp = nh.now();
    rBatteryState.publish(&batteryState_msg);
    previousBatteryMillis = millis();
  }
}

void readUSSensors(){
  if((millis()-previousUSMillis) > usPubRate){
//    ultrasonics_msg.front_c_us.range = (float)front_c_ir.getDistance()/100.0;
//    ultrasonics_msg.front_c_us.header.stamp = nh.now();
//    ultrasonics_msg.front_r_us.range = (float)front_r_inner_ir.getDistance()/100.0;
//    ultrasonics_msg.front_r_us.header.stamp = nh.now();
//    ultrasonics_msg.rear_r_us.range = (float)front_r_outer_ir.getDistance()/100.0;
//    ultrasonics_msg.rear_r_us.header.stamp = nh.now();
//    ultrasonics_msg.rear_c_us.range = (float)rear_r_ir.getDistance()/100.0;
//    ultrasonics_msg.rear_c_us.header.stamp = nh.now();
//    ultrasonics_msg.rear_l_us.range = (float)rear_l_ir.getDistance()/100.0;
//    ultrasonics_msg.rear_l_us.header.stamp = nh.now();
//    ultrasonics_msg.front_l_us.range = (float)rear_l_ir.getDistance()/100.0;
//    ultrasonics_msg.front_l_us.header.stamp = nh.now();
//    rUSSensors.publish(&ultrasonics_msg);
//    previousUSMillis = millis();
  }
}

void readIRSensors(){
  if((millis()-previousIRMillis) > irPubRate){
    infrareds_msg.front_c_ir.range = (float)front_c_ir.getDistance()/100.0;
    infrareds_msg.front_c_ir.header.stamp = nh.now();
    infrareds_msg.front_r_inner_ir.range = (float)front_r_inner_ir.getDistance()/100.0;
    infrareds_msg.front_r_inner_ir.header.stamp = nh.now();
    infrareds_msg.front_r_outer_ir.range = (float)front_r_outer_ir.getDistance()/100.0;
    infrareds_msg.front_r_outer_ir.header.stamp = nh.now();
    infrareds_msg.rear_r_ir.range = (float)rear_r_ir.getDistance()/100.0;
    infrareds_msg.rear_r_ir.header.stamp = nh.now();
    infrareds_msg.rear_l_ir.range = (float)rear_l_ir.getDistance()/100.0;
    infrareds_msg.rear_l_ir.header.stamp = nh.now();
    infrareds_msg.front_l_outer_ir.range = (float)front_l_outer_ir.getDistance()/100.0;
    infrareds_msg.front_l_outer_ir.header.stamp = nh.now();
    infrareds_msg.front_l_inner_ir.range = (float)front_l_inner_ir.getDistance()/100.0;
    infrareds_msg.front_l_inner_ir.header.stamp = nh.now();
    rIRSensors.publish(&infrareds_msg);
    previousIRMillis = millis();
  }
}

void readFallSensors(){
  fallSensorValue = 0;
  for (int i=0; i<nFallSensors; i++){
    fallSensorVals[i] = digitalRead(fallSensorPins[i]);
    if (fallSensorVals[i] == 1){
      haltRobot();
    }
  }
  if (fallSensorVals[0] == 1)
    fallSensorValue += 1;
  if (fallSensorVals[1] == 1)
    fallSensorValue += 2;
  if (fallSensorVals[2] == 1)
    fallSensorValue += 4;
  if (fallSensorVals[3] == 1)
    fallSensorValue += 8;
  fallsensors_msg.state = fallSensorValue;
  if (fallSensorValue != previousFallSensorValue){
    rFallSensors.publish(&fallsensors_msg);
    if (fallSensorValue == 1)
      nh.loginfo("Front right fall sensor activated!");
    if (fallSensorValue == 2)
      nh.loginfo("Rear right fall sensor activated!");
    if (fallSensorValue == 4)
      nh.loginfo("Front left fall sensor activated!");
    if (fallSensorValue == 8)
      nh.loginfo("Rear left fall sensor activated!");
  }
  previousFallSensorValue = fallSensorValue;
}

void readEncoders(){
  long lNewEncoderPos = leftEncoder.read();
  long rNewEncoderPos = rightEncoder.read();
  if (lNewEncoderPos != lOldEncoderPos){
    lOldEncoderPos = lNewEncoderPos;
    wheel_encoders_msg.l_count = (int)lNewEncoderPos;
  }
  if (rNewEncoderPos != rOldEncoderPos){
    rOldEncoderPos = rNewEncoderPos;
    wheel_encoders_msg.r_count = (int)rNewEncoderPos;
  }
  if((millis()-previousEncoderMillis) > encoderPubRate){
    rWheelEncoders.publish(&wheel_encoders_msg);
    previousEncoderMillis = millis();
  }
}

void checkBumpers(){
  bumperValue = 0;
  for (int i=0; i<nBumpers; i++){
    bumperVals[i] = digitalRead(bumperPins[i]); 
    if (bumperVals[i] == 0){
      haltRobot();
    }
  }
  if (bumperVals[0] == 0)
    bumperValue += 1;
  if (bumperVals[1] == 0)
    bumperValue += 2;
  if (bumperVals[2] == 0)
    bumperValue += 4;
  bumpers_msg.state = bumperValue;
  if (bumperValue != previousBumperValue){
    rBumpers.publish(&bumpers_msg);
    if (bumperValue == 1)
      nh.loginfo("Center bumper activated!");
    if (bumperValue == 2)
      nh.loginfo("Left bumper activated!");
    if (bumperValue == 4)
      nh.loginfo("Right bumper activated!");
  }
  previousBumperValue = bumperValue;
}

void haltRobot(){
  ST.stop();
}

void setupInfraredSensorsRos(){
  nh.loginfo("Populating infrared sensor message parameters");
  infrareds_msg.front_c_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.front_c_ir.header.frame_id =  "front_c_ir_link";
  infrareds_msg.front_c_ir.field_of_view = 0.15;  // fake
  infrareds_msg.front_c_ir.min_range = 0.1;
  infrareds_msg.front_c_ir.max_range = 0.8;
  infrareds_msg.front_r_inner_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.front_r_inner_ir.header.frame_id =  "front_r_inner_ir_link";
  infrareds_msg.front_r_inner_ir.field_of_view = 0.15;  // fake
  infrareds_msg.front_r_inner_ir.min_range = 0.1;
  infrareds_msg.front_r_inner_ir.max_range = 0.8;
  infrareds_msg.front_r_outer_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.front_r_outer_ir.header.frame_id =  "front_r_outer_ir_link";
  infrareds_msg.front_r_outer_ir.field_of_view = 0.15;  // fake
  infrareds_msg.front_r_outer_ir.min_range = 0.1;
  infrareds_msg.front_r_outer_ir.max_range = 0.8;
  infrareds_msg.rear_r_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.rear_r_ir.header.frame_id =  "rear_r_ir_link";
  infrareds_msg.rear_r_ir.field_of_view = 0.15;  // fake
  infrareds_msg.rear_r_ir.min_range = 0.1;
  infrareds_msg.rear_r_ir.max_range = 0.8; 
  infrareds_msg.rear_l_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.rear_l_ir.header.frame_id =  "rear_l_ir_link";
  infrareds_msg.rear_l_ir.field_of_view = 0.15;  // fake
  infrareds_msg.rear_l_ir.min_range = 0.1;
  infrareds_msg.rear_l_ir.max_range = 0.8;
  infrareds_msg.front_l_outer_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.front_l_outer_ir.header.frame_id =  "front_l_outer_ir_link";
  infrareds_msg.front_l_outer_ir.field_of_view = 0.15;  // fake
  infrareds_msg.front_l_outer_ir.min_range = 0.1;
  infrareds_msg.front_l_outer_ir.max_range = 0.8;
  infrareds_msg.front_l_inner_ir.radiation_type = sensor_msgs::Range::INFRARED;
  infrareds_msg.front_l_inner_ir.header.frame_id =  "front_l_inner_ir_link";
  infrareds_msg.front_l_inner_ir.field_of_view = 0.15;  // fake
  infrareds_msg.front_l_inner_ir.min_range = 0.1;
  infrareds_msg.front_l_inner_ir.max_range = 0.8; 
}

void setupUltrasonicSensorsRos(){
  nh.loginfo("Populating ultrasonic sensor message parameters");
  ultrasonics_msg.front_c_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.front_c_us.header.frame_id =  "front_c_us_link";
  ultrasonics_msg.front_c_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.front_c_us.min_range = 0.04;
  ultrasonics_msg.front_c_us.max_range = 5.0;
  ultrasonics_msg.front_r_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.front_r_us.header.frame_id =  "front_r_us_link";
  ultrasonics_msg.front_r_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.front_r_us.min_range = 0.04;
  ultrasonics_msg.front_r_us.max_range = 5.0;
  ultrasonics_msg.rear_r_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.rear_r_us.header.frame_id =  "rear_r_us_link";
  ultrasonics_msg.rear_r_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.rear_r_us.min_range = 0.04;
  ultrasonics_msg.rear_r_us.max_range = 5.0;
  ultrasonics_msg.rear_c_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.rear_c_us.header.frame_id =  "rear_c_us_link";
  ultrasonics_msg.rear_c_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.rear_c_us.min_range = 0.04;
  ultrasonics_msg.rear_c_us.max_range = 5.0; 
  ultrasonics_msg.rear_l_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.rear_l_us.header.frame_id =  "rear_l_us_link";
  ultrasonics_msg.rear_l_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.rear_l_us.min_range = 0.04;
  ultrasonics_msg.rear_l_us.max_range = 5.0;
  ultrasonics_msg.front_l_us.radiation_type = sensor_msgs::Range::ULTRASOUND;
  ultrasonics_msg.front_l_us.header.frame_id =  "front_l_us_link";
  ultrasonics_msg.front_l_us.field_of_view = 0.3;  // fake
  ultrasonics_msg.front_l_us.min_range = 0.04;
  ultrasonics_msg.front_l_us.max_range = 5.0; 
}

void setupBatterySensorRos(){
  nh.loginfo("Populating battery message parameters");
  batteryState_msg.capacity = 6.9;
  batteryState_msg.design_capacity = 7;
  batteryState_msg.power_supply_technology = 0;
  batteryState_msg.present = "true";
}

void setupMotorController(){
  nh.loginfo("Initialising motor controller");
  if (arcRobot == false){
   relayOnOff(1,1);
  } else if (arcRobot == true){
   relayOnOff(1,0);
  }
  delay(2000);
  SWSerial.begin(38400);
  haltRobot();
  nh.loginfo("Motor controller initialisation complete");
}

void setupIRSensorsRelay(){
  if (useIRSensors == true){
    nh.loginfo("Setting up infrared sensors");
    delay(1000);
    if (arcRobot == false){
      relayOnOff(2, 1);
    } else if(arcRobot == true){
      relayOnOff(3, 0);
    }
  }
}

void setupFallSensorsRelay(){
  if (useFallSensors == true){
    nh.loginfo("Setting up fall sensors");
    delay(1000);
    if (arcRobot == false){
      relayOnOff(3, 1);
    } else if(arcRobot == true){
      relayOnOff(2, 0);
    }
  }  
}

void readIMU(){
  if (nh.connected()){
    if (is_first){ 
      raw_imu_msg.accelerometer = check_accelerometer();
      raw_imu_msg.gyroscope = check_gyroscope();
      raw_imu_msg.magnetometer = check_magnetometer();
      if (!raw_imu_msg.accelerometer){
        nh.logerror("Accelerometer NOT FOUND!");
      }
      if (!raw_imu_msg.gyroscope){
        nh.logerror("Gyroscope NOT FOUND!");
      }
      if (!raw_imu_msg.magnetometer){
        nh.logerror("Magnetometer NOT FOUND!");
      }
      is_first = false;
    }
    else if (millis() - last_time >= 1000/update_rate)
    {
      raw_imu_msg.header.stamp = nh.now();
      raw_imu_msg.header.frame_id = "imu_link";
      if (raw_imu_msg.accelerometer){
        measure_acceleration();
        raw_imu_msg.raw_linear_acceleration = raw_acceleration;
      }
      if (raw_imu_msg.gyroscope){
        measure_gyroscope();
        raw_imu_msg.raw_angular_velocity = raw_rotation;
      }
      if (raw_imu_msg.magnetometer){
        measure_magnetometer();
        raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
      }
      raw_imu_pub.publish(&raw_imu_msg);
      last_time = millis();
    }
  }
}

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(rRelays);
  nh.subscribe(rLights);
  nh.subscribe(rMotorCommands);
  nh.advertise(rBumpers);
  nh.advertise(rFallSensors);
  nh.advertise(rIRSensors);
  //nh.advertise(rUSSensors);
  nh.advertise(rWheelEncoders);
  nh.advertise(rBatteryState);
  nh.advertise(raw_imu_pub);
  while(!nh.connected()) {
    nh.spinOnce();
  }
  nh.loginfo("ROS Arduino IMU started.");
  #if defined(WIRE_T3)
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  #else
    Wire.begin();
  #endif
  delay(1000);
  for (byte i=0; i<nBumpers; i++){
    pinMode(bumperPins[i], INPUT_PULLUP);
  }
  for (byte i=0; i<nFallSensors; i++){
    pinMode(fallSensorPins[i], INPUT);
  }
  for (byte i=0; i<nIRSensors; i++){
    pinMode(irSensorPins[i], INPUT);
  }
  for (byte i=0; i<nLowerRelays; i++){
    pinMode(lowerRelayPins[i], OUTPUT);
    if (arcRobot == false){
      relayOnOff(i, 0);
    } else if (arcRobot == true){
      relayOnOff(i, 1);
    }
  }
  for (byte i=0; i<nUpperRelays; i++){
    pinMode(upperRelayPins[i], OUTPUT);
    if (arcRobot == false){
      relayOnOff(i+nUpperRelays, 0);
    } else if (arcRobot == true){
      relayOnOff(i+nUpperRelays, 1);
    }
  }
  if (arcRobot == false){
    for (byte i=0; i<nLightsUpper; i ++){
      pinMode(lightsUpperPins[i], OUTPUT);
      pinMode(lightsLowerPins[i], OUTPUT);
      digitalWrite(lightsUpperPins[i], LOW);
      digitalWrite(lightsLowerPins[i], LOW);
    }
  }
  pinMode(voltageSensorPin, INPUT);
  nh.loginfo("Initialising tango robot");
  setupIRSensorsRelay();
  setupInfraredSensorsRos();
  setupFallSensorsRelay();
  //setupUltrasonicSensorsRos();
  setupBatterySensorRos();
  setupMotorController();
  nh.loginfo("Tango robot initialisation complete");
}

void loop()
{ 
  if (useBumpers == true){
    checkBumpers();
  }
  if (useFallSensors == true){
    readFallSensors();
  }
  if (useIMU == true){
    readIMU();
  }
  if (useIRSensors == true){
    readIRSensors();
  }
  if (useUSSensors == true){
    readUSSensors();
  }
  if (useBatterySensor == true){
    readBatteryVoltage();
  }
  readEncoders();
  nh.spinOnce();
}


