#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>

class RobotController
{
private:
  ros::NodeHandle nh_;

  float LINEAR_SPEED = 0.1;                   // m/s
  float DISTANCE_TOLERANCE = 0.05;            // m
  float ANGULAR_SPEED = 5.0 * M_PI / 180.0;   // rad/s
  float ANGLE_TOLERANCE = 3.0 * M_PI / 180.0; // rad
  float SLEEP_TIME = 0.05;                    // s

  float robot_curr_x_, robot_curr_y_, robot_curr_th_;

  nav_msgs::Odometry robot_odom_;
  geometry_msgs::PoseStamped goal_pose_;

  ros::Publisher velocity_pub_;
  ros::Subscriber odom_sub_, goal_sub_;

  inline float calcVecLength(const float x, const float y)
  {
    return std::sqrt(x * x + y * y);
  }

  inline float calcVecAngle(const float x, const float y)
  {
    return std::atan2(y, x);
  }

  inline bool isWithinTolerance(const float value)
  {
    return (std::fabs(value))<DISTANCE_TOLERANCE;
  }

  inline float convQuatToEuler(const geometry_msgs::Quaternion &msg_quat)
  {
    return tf::getYaw(msg_quat);
  }

  inline bool isReached(const geometry_msgs::Point &init_pose, const float distance)
  {
    float travel_dist = calcVecLength(init_pose.x - robot_curr_x_, init_pose.y - robot_curr_y_);
    float desired_travel_dist = distance - DISTANCE_TOLERANCE;
    return travel_dist < desired_travel_dist;
  }

  void odometryCB(const nav_msgs::Odometry &msg);
  void goalCB(const geometry_msgs::PoseStamped &msg);

  float normalizeAngle(const float value);
  float calcAngleDiff(const float ang_1, const float ang_2);
  void driveRobot(const float distanec, const float linear_velocity);
  void smoothDriveRobot(const float distance, const float linear_velocity);

public:
  RobotController(ros::NodeHandle nh);
  ~RobotController();
  void runController();
  bool goal_available_ = false;
  void publishVelocity(const float linear_vel, const float angular_vel);
  void rotateRobot(const float angle, const float angular_velocity);
  void goToGoal();
};
