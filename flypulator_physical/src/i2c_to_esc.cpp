#include <ros/ros.h>
#include <flypulator_physical/offb_parameterConfig.h>
#include <flypulator_common_msgs/RotorVelStamped.h>
#include <dynamic_reconfigure/server.h>
#include <motors.hpp>

// handle for all motors
Motors *motors_ptr;
float g_upper_limit = 6000;

bool flag_in_control = false;

void controlMsgCallback(const flypulator_common_msgs::RotorVelStamped msg)
{
  float input[6] = {0};

  if (motors_ptr->getMotorsState() != Motors::MOTOR_ARMED)
    return;

  if(!flag_in_control){
    flag_in_control = true;
    ROS_WARN_STREAM("Controller ON!");
  }

  for (int i = 0; i < 6; ++i)
    input[i] = msg.velocity[i]>g_upper_limit?g_upper_limit:msg.velocity[i];

  motors_ptr->setMotorsVel(input);

  if (motors_ptr->writeMotors() != Motors::WRITE_OK)
    printf(PRED "writeMotor error!" PRST);
}

void dynamicParamCallback(flypulator_mavros::offb_parameterConfig &config, uint32_t level)
{
  if (config.drone_armed)
  {
    // arm motors
    if (motors_ptr->getMotorsState() != Motors::MOTOR_ARMED){
      motors_ptr->armMotors();
      ROS_WARN_STREAM("Motor ARMED!");
    }
  }
  else
  {
    // disarm motors
    if (motors_ptr->getMotorsState() != Motors::MOTOR_DISARMED){
      motors_ptr->disarmMotors();
      ROS_WARN_STREAM("Motor DISARMED!");
    }

    flag_in_control = false;
    ROS_WARN_STREAM("Controller OFF!");
  }

  g_upper_limit = config.upper_limit;

  if ((!flag_in_control)&&(motors_ptr->getMotorsState() != Motors::MOTOR_DISARMED))
  {
    float input[6]={0};
    input[0] = config.motor1_speed; 
    input[1] = config.motor2_speed;
    input[2] = config.motor3_speed; 
    input[3] = config.motor4_speed;
    input[4] = config.motor5_speed; 
    input[5] = config.motor6_speed; 

    for(int i=0; i<6; ++i)
      input[i] = input[i]>g_upper_limit?g_upper_limit:input[i];

    motors_ptr->setMotorsVel(input);
    ROS_INFO_STREAM("Set Motors speed!");
  }
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

  ros::Rate loop_rate(200);
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
