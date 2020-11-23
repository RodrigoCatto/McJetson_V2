#include <mcjetson_v2/MYRobot_hardware_interface.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#define _USE_MATH_DEFINES
#include <cmath>

​

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {

 

// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

​

MyRobot::~MyRobot() {
}

​

void MyRobot::init() {
        
// Create joint_state_interface for rear_wheel_joint
    hardware_interface::JointStateHandle jointStateHandle("rear_wheel_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle);
// Create velocity joint interface as rear_wheel_joint accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    velocity_joint_interface_.registerHandle(jointVelocityHandle);
// Create Joint Limit interface for rear_wheel_joint
    joint_limits_interface::getJointLimits("rear_wheel_joint", nh_, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
    velocityJointSaturationInterface.registerHandle(jointVelocityHandle);   
   
    
// Create joint_state_interface for front_steer_joint
    hardware_interface::JointStateHandle jointStateHandle("front_steer_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle);
// Create position joint interface as front_steer_joint accepts position command.
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandle);
// Create Joint Limit interface for front_steer_joint
    joint_limits_interface::getJointLimits("front_steer_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle);    


// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocityJointSaturationInterface); 
    registerInterface(&positionJointSaturationInterface);    
}

​

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

​

void MyRobot::read() {​

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]
  double motor_speed = _motor_speed;
  motor_speed = motor_speed*120/1000;
  double  wheel_radius = 0.043 /*Meters*/
  double  convert_rpm_ms =  (2*M_PI*wheel_radius)/60
  joint_velocity_[0] = motor_speed * convert_rpm_ms

  double servo_angle = _servo_angle;
  joint_position_[1] = (54.5*servo_value-27.25) * (pi/180)  #rad




}

​

void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
  positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_position_command_ for JointC.

}

ros::Subscriber motor_speed_sub_;
ros::Subscriber servo_angle_sub_;

​void motorSpeedCallback(const std_msgs::VescStateStamped& msg) {
  _motor_speed = msg.state.speed;
}

void servoAngleCallback(const std_msgs::Float64& msg) {
  _servo_angle = msg.data;
}


int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_inerface_node");
    ros::NodeHandle nh;

    motor_speed_sub_ = nh.subscribe("/sensors/core", 1, &MyRobotHWInterface::motorSpeedCallback, this);
    servo_angle_sub_ = nh.subscribe("/commands/servo/position", 1, &MyRobotHWInterface::servoAngleCallback, this);
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedspinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}
