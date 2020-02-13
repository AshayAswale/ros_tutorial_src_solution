#include <ros_tutorial_src/robot_controller.h>

RobotController::RobotController(ros::NodeHandle nh) : nh_(nh)
{
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  odom_sub_ = nh_.subscribe("/odom", 1, &RobotController::odometryCB, this);
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
  geometry_msgs::Twist robot_twist;
  robot_twist.linear.x = linear_vel;
  robot_twist.linear.y = 0.0;
  robot_twist.linear.z = 0.0;
  robot_twist.angular.x = 0.0;
  robot_twist.angular.y = 0.0;
  robot_twist.angular.z = angular_vel;
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
  geometry_msgs::Point init_position;
  init_position.x = robot_curr_x_;
  init_position.y = robot_curr_y_;

  publishVelocity(linear_velocity, 0.0);

  while (isReached(init_position, distance))
    ros::Duration(SLEEP_TIME).sleep();

  publishVelocity(0.0, 0.0);
}

void RobotController::rotateRobot(const float angle, const float angular_velocity)
{
  float limit_angle = calcAngleDiff(angle, ANGLE_TOLERANCE);
  float init_th = robot_curr_th_;

  publishVelocity(0.0, std::copysign(angular_velocity, angle));

  while (std::fabs(calcAngleDiff(robot_curr_th_, init_th)) < std::fabs(limit_angle))
    ros::Duration(SLEEP_TIME).sleep();

  publishVelocity(0.0, 0.0);
}

void RobotController::goToGoal()
{
  goal_available_ = false;

  float goal_x = goal_pose_.pose.position.x;
  float goal_y = goal_pose_.pose.position.y;
  geometry_msgs::Quaternion goal_quat = goal_pose_.pose.orientation;
  float goal_th = tf::getYaw(goal_quat);

  ROS_INFO("Facing direction.");
  float vec_angle = calcVecAngle(goal_x - robot_curr_x_, goal_y - robot_curr_y_);
  float face_th = calcAngleDiff(vec_angle, robot_curr_th_);
  rotateRobot(face_th, ANGULAR_SPEED);

  ROS_INFO("Driving Robot.");
  float dist = calcVecLength(goal_x - robot_curr_x_, goal_y - robot_curr_y_);
  driveRobot(dist, LINEAR_SPEED);

  ROS_INFO("Orienting Robot");
  float final_th = calcAngleDiff(goal_th, robot_curr_th_);
  rotateRobot(final_th, ANGULAR_SPEED);

  ROS_INFO("Done Moving Robot");
}

void RobotController::arcTo()
{
  float goal_x = goal_pose_.pose.position.x;
  float goal_y = goal_pose_.pose.position.y;
  
  geometry_msgs::Quaternion goal_quat = goal_pose_.pose.orientation;
  float goal_th = tf::getYaw(goal_quat);

  bool is_reached = false;

  float vec_to_target_x, vec_to_target_y, ang_to_target;
  float curr_dist;
  float robot_vel_x, robot_ang_vel;

  while (!is_reached)
  {
    vec_to_target_x = goal_x - robot_curr_x_;
    vec_to_target_y = goal_y - robot_curr_y_;
    curr_dist = calcVecLength(vec_to_target_x, vec_to_target_y);
    if (isWithinTolerance(curr_dist))
    {
      is_reached = true;
      break;
    }
    ang_to_target = calcVecAngle(vec_to_target_x, vec_to_target_y);
    robot_vel_x = copysign(std::min(std::fabs(curr_dist), LINEAR_SPEED), curr_dist);
    robot_ang_vel = copysign(std::min(std::fabs(ang_to_target), ANGULAR_SPEED), ang_to_target);

    publishVelocity(robot_vel_x, robot_ang_vel);
    ros::Duration(SLEEP_TIME).sleep();
  }
  // rotateRobot(goal_th, ANGULAR_SPEED);
  publishVelocity(0.0, 0.0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle nh;
  ros::Rate rate(500);
  ros::AsyncSpinner spinner(1);

  spinner.start();
  RobotController control(nh);
  ros::Duration(1.0).sleep();

  // control.rotateRobot(M_PI / 4, 0.5);
  // ros::Duration(1.0).sleep();

  while (ros::ok())
  {
    if (control.goal_available_)
    {
      ROS_INFO("Moving to Goal");
      // control.goToGoal();
      control.arcTo();
    }
    // ROS_INFO("Sleeping...");
    rate.sleep();
  }

  ros::Duration(1.0).sleep();
  spinner.stop();
  return 0;
}
