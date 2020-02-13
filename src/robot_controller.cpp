#include <ros_tutorial_src/robot_controller.h>

RobotController::RobotController(ros::NodeHandle nh) : nh_(nh)
{
  // ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  
  // ### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
  // ### When a message is received, call RobotController::odometryCB
  odom_sub_ = nh_.subscribe("/odom", 1, &RobotController::odometryCB, this);
  
  // ### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
  // ### When a message is received, call RobotController::goalCB
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &RobotController::goalCB, this);
}

RobotController::~RobotController()
{
}

void RobotController::odometryCB(const nav_msgs::Odometry &msg)
{
  robot_odom_ = msg;
  robot_curr_x_ = msg.pose.pose.position.x;
  robot_curr_y_ = msg.pose.pose.position.y;
  robot_curr_th_ = tf::getYaw(msg.pose.pose.orientation);
  msg.pose.pose.orientation;
}

void RobotController::goalCB(const geometry_msgs::PoseStamped &msg)
{
  goal_pose_ = msg;
  goal_available_ = true;
  ROS_INFO("Goal CallBack!");
}

void RobotController::publishVelocity(const float linear_vel, const float angular_vel)
{
  // # Create twist message
  geometry_msgs::Twist robot_twist;
  // # Linear velocity
  robot_twist.linear.x = linear_vel;
  robot_twist.linear.y = 0.0;
  robot_twist.linear.z = 0.0;
  // # Angular velocity
  robot_twist.angular.x = 0.0;
  robot_twist.angular.y = 0.0;
  robot_twist.angular.z = angular_vel;
  // # Send command
  velocity_pub_.publish(robot_twist);
}

float RobotController::normalizeAngle(const float value)
{
  float result;
  if (value > 0.0)
    result = std::fmod(value + M_PI, 2 * M_PI) - M_PI;
  else
    result = std::fmod(value - M_PI, 2 * M_PI) + M_PI;
  return result;
}

float RobotController::calcAngleDiff(const float ang_1, const float ang_2)
{
  float norm_ang_1, norm_ang_2;
  norm_ang_1 = normalizeAngle(ang_1);
  norm_ang_2 = normalizeAngle(ang_2);

  return normalizeAngle(norm_ang_1 - norm_ang_2);
}

void RobotController::driveRobot(const float distance, const float linear_velocity)
{
  //  # Save current position
  geometry_msgs::Point init_position;
  init_position.x = robot_curr_x_;
  init_position.y = robot_curr_y_;
  
  // # Go straight
  publishVelocity(linear_velocity, 0.0);

  // # Keep going until the distance is not within the tolerance
  while (isReached(init_position, distance))
    ros::Duration(SLEEP_TIME).sleep();

  // # Stop
  publishVelocity(0.0, 0.0);
}

void RobotController::rotateRobot(const float angle, const float angular_velocity)
{
  // # Calculate limit angle after which we must stop
  float limit_angle = calcAngleDiff(angle, ANGLE_TOLERANCE);

  // # Save the initial orientation
  float init_th = robot_curr_th_;

  // # Rotate
  publishVelocity(0.0, std::copysign(angular_velocity, angle));

  // # Keep going until we're within angle tolerance
  while (std::fabs(calcAngleDiff(robot_curr_th_, init_th)) < std::fabs(limit_angle))
    ros::Duration(SLEEP_TIME).sleep();

  // # Stop
  publishVelocity(0.0, 0.0);
}

void RobotController::goToGoal()
{
  // Set variable false to enable resetting goal position during run.
  goal_available_ = false;

  // # Save target pose
  float goal_x = goal_pose_.pose.position.x;
  float goal_y = goal_pose_.pose.position.y;
  geometry_msgs::Quaternion goal_quat = goal_pose_.pose.orientation;
  float goal_th = tf::getYaw(goal_quat);

  // # Rotate to face target
  ROS_INFO("Facing direction.");
  float vec_angle = calcVecAngle(goal_x - robot_curr_x_, goal_y - robot_curr_y_);
  float face_th = calcAngleDiff(vec_angle, robot_curr_th_);
  rotateRobot(face_th, ANGULAR_SPEED);

  // # Drive straight to target
  ROS_INFO("Driving Robot.");
  float dist = calcVecLength(goal_x - robot_curr_x_, goal_y - robot_curr_y_);
  driveRobot(dist, LINEAR_SPEED);
  // smoothDriveRobot(dist, LINEAR_SPEED);

  // # Rotate to final pose
  ROS_INFO("Orienting Robot");
  float final_th = calcAngleDiff(goal_th, robot_curr_th_);
  rotateRobot(final_th, ANGULAR_SPEED);

  ROS_INFO("Done Moving Robot");
}

void RobotController::smoothDriveRobot(const float distance, const float linear_velocity)
{
  // # Save initial position and time
  geometry_msgs::Point init_position;
  init_position.x = robot_curr_x_;
  init_position.y = robot_curr_y_;

  // # Fix the length of phases 1 and 3 to 20% of the total drive distance
  //       # Phase 0 is when we're the farthest
  //       # Phase 1 is when we drive at max speed
  //       # Phase 2 is when we're almost at the target
  //       # Store end of each phase

  std::vector<float> phases;
  phases.resize(3);
  phases.at(0) = distance;
  phases.at(1) = distance*0.8;
  phases.at(2) = distance*0.2;

  bool reached = false;

  // # Phase 1: ramp speed up
  while (!reached)
  {
    // # Get current distance
    float vec_length = calcVecLength(init_position.x - robot_curr_x_, init_position.y - robot_curr_y_);
    float curr_dist = std::fabs(distance - vec_length);

    // # Identify in which phase we are
    int phase = 0;
    if(curr_dist < phases.at(2))
      phase = 2;
    else if (curr_dist < phases.at(1))
      phase = 1;

    // # Check if we arrived to destination
    if(isWithinTolerance(curr_dist))
    {
      reached = true;
      break;
    }

    // # Calculate the linear speed depending on the phase
    float ls = linear_velocity;
    if (phase == 0)
      ls = linear_velocity * (1.0 - (curr_dist - phases.at(1)) / (phases.at(2) - phases.at(1)));
    else if(phase == 2)
      ls = linear_velocity * ((curr_dist + 0.01) / phases.at(2));

    // # Set speed
    publishVelocity(ls, 0.0);

    // # Sleep a little
    ros::Duration(SLEEP_TIME).sleep();
  }

  // # Stop robot
  publishVelocity(0.0, 0.0);
}

int main(int argc, char **argv)
{
  // ### Initialize node, name it 'robot_controller'
  ros::init(argc, argv, "robot_controller");

  // # Create Nodehandle 
  ros::NodeHandle nh;

  ros::Rate rate(500);
  // #spinners are needed for subscribers
  ros::AsyncSpinner spinner(1);

  spinner.start();

  // # Create object of controller class.
  RobotController control(nh);
  ros::Duration(1.0).sleep();

  while (ros::ok())
 {
  //  # Check if goal has published.
   if (control.goal_available_)
   {
    //  # if goal is available, move the robot
     ROS_INFO("Moving to Goal");
     control.goToGoal();
   }
  //  # Sleep for some time
   rate.sleep();
  }

  spinner.stop();
  ros::Duration(1.0).sleep();
  return 0;
}
