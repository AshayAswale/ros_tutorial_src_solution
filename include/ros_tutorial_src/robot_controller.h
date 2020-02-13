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

  /**
   * @brief Calculates the length of a vector.
   * 
   * @param x   [float] The X coordinate.
   * @param y   [float] The Y coordinate.
   * @return float      The length.
   */
  inline float calcVecLength(const float x, const float y)
  {
    return std::sqrt(x * x + y * y);
  }

  /**
   * @brief Calculates the angle of a vector in radians normalized between -pi and pi.
   * 
   * @param x    [float] The X coordinate.
   * @param y    [float] The Y coordinate.
   * @return float [rad] The angle.
   */
  inline float calcVecAngle(const float x, const float y)
  {
    return std::atan2(y, x);
  }

  /**
   * @brief Checks whether a certain value is within a given tolerance.
   *        The passed value can be any sign. Internally, the check is
   *        performed on the absolute value of x.
   * 
   * @param value     [float] The value to check.
   * @return true     True if within tolerance, False otherwise
   * @return false 
   */
  inline bool isWithinTolerance(const float value)
  {
    return (std::fabs(value))<DISTANCE_TOLERANCE;
  }

  /**
   * @brief Converts input msg to return Yaw
   * 
   * @param msg_quat  [geometry_msgs::Quaternion] Input quaternion msg 
   * @return float    Yaw
   */
  inline float convQuatToEuler(const geometry_msgs::Quaternion &msg_quat)
  {
    return tf::getYaw(msg_quat);
  }

  /**
   * @brief Checks if the robot has covered the distance
   * 
   * @param init_pose     [geometry_msgs::Point] Initial position of the robot 
   * @param distance      [float] Distance to be covered
   * @return true         When robot has covered the distance.
   * @return false 
   */
  inline bool isReached(const geometry_msgs::Point &init_pose, const float distance)
  {
    float travel_dist = calcVecLength(init_pose.x - robot_curr_x_, init_pose.y - robot_curr_y_);
    float desired_travel_dist = distance - DISTANCE_TOLERANCE;
    return travel_dist < desired_travel_dist;
  }

  /**
   * @brief Updates the current pose of the robot.
   *        This method is a callback bound to a Subscriber.
   * 
   * @param msg [Odometry] The current odometry information.
   */
  void odometryCB(const nav_msgs::Odometry &msg);
  
  /**
   * @brief Updates the Goal Poisition for robot
   *        This method is a callback bound to a Subscriber.
   * 
   * @param msg [PoseStamped] The target pose.
   */
  void goalCB(const geometry_msgs::PoseStamped &msg);

  /**
   * @brief  Normalizes the given angle between -pi and pi.
   * 
   * @param value     [float] [rad] The angle.
   * @return float    [float] [rad] The normalized value.
   */
  float normalizeAngle(const float value);
  
  /**
   * @brief Calculates the difference between two angles.
   *        The return value is normalized between -pi and pi.
   * 
   * @param ang_1   [float] [rad] The first angle.
   * @param ang_2   [float] [rad] The second angle.
   * @return float  [float] [rad] The difference between the angles.
   */
  float calcAngleDiff(const float ang_1, const float ang_2);
  
  /**
   * @brief  Drives the robot in a straight line.
   * 
   * @param distance         [float] [m]   The distance to cover.
   * @param linear_velocity  [float] [m/s] The forward linear speed.
   */
  void driveRobot(const float distance, const float linear_velocity);

  /**
   * @brief Drives the robot in a straight line by changing the actual speed smoothly.
   * 
   * @param distance         [float] [m]   The distance to cover.
   * @param linear_velocity  [float] [m/s] The maximum forward linear speed.
   */
  void smoothDriveRobot(const float distance, const float linear_velocity);



public:
  
  /**
   * @brief Construct a new Robot Controller:: Robot Controller object
   * 
   * @param nh 
   */
  RobotController(ros::NodeHandle nh);

  /**
   * @brief Destroy the Robot Controller:: Robot Controller object
   *        Delete pointers here, if used
   * 
   */
  ~RobotController();
  
  // Set true when goal is available on topic: /move_base_simple/goal
  bool goal_available_ = false;
  
  /**
   * @brief Sends the speeds to the motors.
   * 
   * @param linear_vel  [float] [m/s]   The forward linear speed.
   * @param angular_vel [float] [rad/s] The angular speed for rotating around the body center.
   */
  void publishVelocity(const float linear_vel, const float angular_vel);
  
  /**
   * @brief Rotates the robot around the body center by the given angle.
   * 
   * @param angle             [float] [rad]   The distance to cover.
   * @param angular_velocity  [float] [rad/s] The angular speed.
   */
  void rotateRobot(const float angle, const float angular_velocity);
  
  /**
   * @brief Calls rotate(), drive(), and rotate() to attain a given pose.
   * 
   */
  void goToGoal();
};
