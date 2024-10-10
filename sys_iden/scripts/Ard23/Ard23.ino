//Includes
//Generic includes
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <math.h>

//Pressure sensor includes
#include <Wire.h>
#include "MS5837.h"

//IMU includes (Also Wire.h above)
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//ROS includes
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>

//Thruster includes
#include <Servo.h>

//Objects
//Pressure Sensor object
MS5837 ps;
//IMU object
Adafruit_BNO055 bno = Adafruit_BNO055();
//Thrusters object
Servo thrustObj[8];

//Global variables
const float to_rad = (22/7.0) / 180.0; //degree to rad conversion constant
float roll, pitch, yaw ;
const unsigned int thrustPins[8] = {2, 3, 4, 5, 10, 11, 12, 13}; //thrusters pin arrangement
unsigned int thrust[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //thruster pwm values (in order)
uint16_t SENSOR_DELAY = 100; //delay between readings

//ROS handle
ros::NodeHandle  nh;

//Sensors topic initialization
sensor_msgs::FluidPressure pressure_msg;
ros::Publisher pressure0("pressure0", &pressure_msg);
sensor_msgs::Imu imu_msg;
ros::Publisher imu0("imu0", &imu_msg);

//Call back function to subscribe from control node
void pwmCallback(const std_msgs::UInt16MultiArray& pwm_msg) {
  //Write new PWMs to thrusters (using a thrust array loop)
  for (int i = 0; i < 8; i++) {
      thrust[i] = pwm_msg.data[i];
  }
}
ros::Subscriber<std_msgs::UInt16MultiArray> sub_thrust("pwm", pwmCallback);

void setup() {
  // put your setup code here, to run once:
  
  //Pressure sensor setup
  Wire.begin();
  if (!ps.init())
  {
    while (1);
  }
  ps.setModel(MS5837::MS5837_02BA);
  ps.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  
  //IMU setup
  if (!bno.begin())
  {
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  //Thruster setup
  for (int i = 0; i < 8; i++) {
    //Initialize vertical thrusters
    thrustObj[i].attach(thrustPins[i]);
    thrustObj[i].writeMicroseconds(1500);
  }
  
  //ROS setup
  nh.initNode();
  nh.advertise(pressure0);
  nh.advertise(imu0);
  nh.subscribe(sub_thrust);

  //Initial code run
  //Pressure sensor reading update
  ps.read(); 
  pressure_msg.header.stamp = nh.now(); //pressure reading time stamp
  pressure_msg.header.frame_id = "pressure0"; //frame id
  pressure_msg.fluid_pressure = ps.pressure() * 100.0; //pressure in pascals
  pressure_msg.variance = 4444.444; //pressure reading variance
  
  //IMU reading update
  //Euler angles -> quaternion
  sensors_event_t event;
  bno.getEvent(&event);
  imu_msg.header.stamp = nh.now(); //imu reading time stamp
  imu_msg.header.frame_id = "imu0"; //frame id
  roll = event.orientation.x * to_rad; //roll reading
  pitch = event.orientation.y * to_rad; //pitch reading
  yaw = event.orientation.z * to_rad; //yaw reading
  imu_msg.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  imu_msg.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  imu_msg.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  imu_msg.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
  //Angular velocity
  imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); //read raw gyroscope data
  imu_msg.angular_velocity.x = ang_vel.x(); //angular velocity about x (rad/sec.)
  imu_msg.angular_velocity.y = ang_vel.y(); //angular velocity about y (rad/sec.)
  imu_msg.angular_velocity.z = ang_vel.z(); //angular velocity about y (rad/sec.)
  imu_msg.angular_velocity_covariance[0] = 0.00000305;//gyroscope variance
  imu_msg.angular_velocity_covariance[4] = 0.00000305;//gyroscope variance
  imu_msg.angular_velocity_covariance[8] = 0.00000305;//gyroscope variance
  //Linear acceleration
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //read raw linear acceleration data (without gravity)
  imu_msg.linear_acceleration.x = lin_acc.x(); //linear acceleration along x (m/sec2)
  imu_msg.linear_acceleration.y = lin_acc.y(); //linear acceleration along y (m/sec2)
  imu_msg.linear_acceleration.z = lin_acc.z(); //linear acceleration along z (m/sec2)
  imu_msg.linear_acceleration_covariance[0] = 0.00006753; //linear acceleration covariance
  imu_msg.linear_acceleration_covariance[4] = 0.00006753; //linear acceleration covariance
  imu_msg.linear_acceleration_covariance[8] = 0.00006753; //linear acceleration covariance
  delay(SENSOR_DELAY);
}

void loop() {
  //Pressure sensor reading update
  ps.read(); 
  pressure_msg.header.stamp = nh.now(); //pressure reading time stamp
  pressure_msg.fluid_pressure = ps.pressure() * 100.0; //pressure in pascals

  //IMU reading update
  //Euler angles -> quaternion
  sensors_event_t event;
  bno.getEvent(&event);
  imu_msg.header.stamp = nh.now(); //imu reading time stamp
  roll = event.orientation.x * to_rad; //roll reading
  pitch = event.orientation.y * to_rad; //pitch reading
  yaw = event.orientation.z * to_rad; //yaw reading
  imu_msg.orientation.x = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
  imu_msg.orientation.y = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
  imu_msg.orientation.z = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
  imu_msg.orientation.w = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
  //Angular velocity
  imu::Vector<3> ang_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); //read raw gyroscope data
  imu_msg.angular_velocity.x = ang_vel.x(); //angular velocity about x (rad/sec.)
  imu_msg.angular_velocity.y = ang_vel.y(); //angular velocity about y (rad/sec.)
  imu_msg.angular_velocity.z = ang_vel.z(); //angular velocity about y (rad/sec.)
  //Linear acceleration
  imu::Vector<3> lin_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); //read raw linear acceleration data (without gravity)
  imu_msg.linear_acceleration.x = lin_acc.x(); //linear acceleration along x (m/sec2)
  imu_msg.linear_acceleration.y = lin_acc.y(); //linear acceleration along y (m/sec2)
  imu_msg.linear_acceleration.z = lin_acc.z(); //linear acceleration along z (m/sec2)
  delay(SENSOR_DELAY);

  //Thruster PWM update
  for (int i = 0; i < 8; i++) {
    thrustObj[i].writeMicroseconds(thrust[i]);
  }
  
  //Construct the publisher message
  pressure0.publish( &pressure_msg );
  imu0.publish( &imu_msg );
  
  //spin
  nh.spinOnce();
  delay(SENSOR_DELAY);
}
