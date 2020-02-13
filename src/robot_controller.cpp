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
  // driveRobot(dist, LINEAR_SPEED);
  smoothDriveRobot(dist, LINEAR_SPEED);

  ROS_INFO("Orienting Robot");
  float final_th = calcAngleDiff(goal_th, robot_curr_th_);
  rotateRobot(final_th, ANGULAR_SPEED);

  ROS_INFO("Done Moving Robot");
}

void RobotController::smoothDriveRobot(const float distance, const float linear_velocity)
{
  geometry_msgs::Point init_position;
  init_position.x = robot_curr_x_;
  init_position.y = robot_curr_y_;

  std::vector<float> phases;
  phases.resize(3);
  phases.at(0) = distance;
  phases.at(1) = distance*0.8;
  phases.at(2) = distance*0.2;

  bool reached = false;
  while (!reached)
  {
    float vec_length = calcVecLength(init_position.x - robot_curr_x_, init_position.y - robot_curr_y_);
    float curr_dist = std::fabs(distance - vec_length);

    int phase = 0;
    if(curr_dist < phases.at(2))
      phase = 2;
    else if (curr_dist < phases.at(1))
      phase = 1;

    if(isWithinTolerance(curr_dist))
    {
      reached = true;
      break;
    }

    float ls = linear_velocity;
    if (phase == 0)
      ls = linear_velocity * (1.0 - (curr_dist - phases.at(1)) / (phases.at(2) - phases.at(1)));
    else if(phase == 2)
      ls = linear_velocity * ((curr_dist + 0.01) / phases.at(2));

    publishVelocity(ls, 0.0);
    ros::Duration(SLEEP_TIME).sleep();
  }
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
      control.goToGoal();
    }
    // ROS_INFO("Sleeping...");
    rate.sleep();
  }

  ros::Duration(1.0).sleep();
  spinner.stop();
  return 0;
}
