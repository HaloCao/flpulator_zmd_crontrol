#include <ros/ros.h>
#include <flypulator_physical/offb_parameterConfig.h>
#include <flypulator_common_msgs/RotorVelStamped.h>
#include <dynamic_reconfigure/server.h>
#include <motors.hpp>

// handle for all motors
Motors *motors_ptr;

bool flag_in_control = false;

void controlMsgCallback(const flypulator_common_msgs::RotorVelStamped msg)
{
  float input[6] = {0};

  if (motors_ptr->getMotorsState() == Motors::MOTOR_ARMED)
    return;

  flag_in_control = true;

  for (int i = 0; i < 6; ++i)
    input[i] = msg.velocity[i];

  motors_ptr->setMotorsVel(input);

  if (motors_ptr->writeMotors() != Motors::WRITE_OK)
    printf(PRED "writeMotor error!" PRST);
}

void dynamicParamCallback(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
  if (config.drone_armed)
  {
    // arm motors
    if (motors_ptr->getMotorsState() != Motors::MOTOR_ARMED)
      motors_ptr->armMotors();

    flag_in_control = false;
  }
  else
  {
    // disarm motors
    if (motors_ptr->getMotorsState() != Motors::MOTOR_DISARMED)
      motors_ptr->disarmMotors();
  }

  if (!flag_in_control)
  {
    float input[6]={0};
    input[0] = config.motor2_speed; // drone motor 2
    input[1] = config.motor5_speed; // drone motor 5
    input[2] = config.motor1_speed; // drone motor 1
    input[3] = config.motor4_speed; // drone motor 4
    input[4] = config.motor6_speed; // drone motor 6
    input[5] = config.motor3_speed; // drone motor 3

    motors_ptr->setMotorsVel(input);
  }

  float upper_limit = config.upper_limit;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "i2c_to_esc_node");
  ros::NodeHandle nh;

  motors_ptr = new Motors();

  // add subscribers for topics state and control output
  ros::Subscriber control_sub = nh.subscribe<flypulator_common_msgs::RotorVelStamped>(
      "/drone/rotor_cmd", 10, controlMsgCallback);

  // set up callbacks for dynamic reconfigure
  dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig> param_srv;
  dynamic_reconfigure::Server<flypulator_mavros::offb_parameterConfig>::CallbackType cb;
  cb = boost::bind(&dynamicParamCallback, _1, _2);
  param_srv.setCallback(cb);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (!flag_in_control)
      if (motors_ptr->writeMotors() != Motors::WRITE_OK)
        printf(PRED "writeMotor error!" PRST);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
