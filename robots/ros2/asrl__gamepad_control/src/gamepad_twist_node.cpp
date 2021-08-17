// Used for absolute value
#include <math.h>

// ros
#include <ros/ros.h>

// Definitions of the twist and joystick messages
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

// Some useful functions for running nodes
#include <asrl/rosutil/node_utilities.hpp>
#include <asrl/rosutil/param.hpp>


namespace asrl {

  //* GamepadTwist
  /**
   * The GamepadTwist node connects to a joystick node and receives joy
   * messages. The messages are interpreted as desired velocities and a
   * appropriate twist command is created and published.
   */
  class GamepadTwist
  {
  public:
    /*!
     * \brief The constructor, initialize the node
     *
     * In the constructor, set parameters from the parameter server (if
     * available), subscribe to joystick messages, set up the publisher of
     * twist messages.
     */
    GamepadTwist(const ros::NodeHandle & nh);

    /*!
     * \brief Spin the node, handle joy callbacks
     */
    void spin(void);

    /*!
     * \brief Make sure the speed limit is enforced
     *
     * Return a legal speed based on the request (command) and the speed limit
     */
    float getLegalSpeed(float command, float limit);


  private:
    /*!
     * \brief Handle joystick messages by publishing twist messages
     *
     * The joystick message callback. When a joystick message is received,
     * check if the deadman button is pressed and then publish the appropriate
     * twist message.
     */
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nodeHandle_;  /**< \brief A ros node handle for the service*/

    int linearForwardAxis_; /**< \brief The axis index for the forward linear velocity*/
    int linearReverseAxis_; /**< \brief The axis index for the reverse linear velocity*/
    int linearAscendAxis_;
    int linearDescendAxis_;
    int linearStrafeLeftAxis_;
    int linearStrafeRightAxis_;
    int angularAxis_; /**< \brief The axis index for the angular velocity*/
    int deadmanButton_; /**< \brief The button index for the deadman button*/
    int robotFunction1Button_; /**< \brief The button index for robot function 1 (w/ deadman interlock) button*/
    int robotFunction2Button_; /**< \brief The button index for robot function 2 (w/o deadman interlock) button*/
    int robotKill1Button_; /**< \brief The button index for robot kill 1 (w/o deadman interlock) button*/
    int robotKill2Button_; /**< \brief The button index for robot kill 2 (w/o deadman interlock) button*/    

    int curvatureOverRideOne_; /**< \brief The one of the two botton used for overriding curvature contraint*/
    int curvatureOverRideTwo_; /**< \brief The other botton used for overriding curvature contraint*/

    double linearScale_; /**< \brief The joystick to twist scale factor for linear velocity */
    double angularScale_; /**< \brief The joystick to twist scale factor for angular velocity*/

    bool isLinearForwardActive_; /**< \brief Identify if the linear forward axis is active*/
    bool isLinearReverseActive_; /**< \brief Identify if the linear reverse axis is active*/
    bool isLinearAscendActive_;
    bool isLinearDescendActive_;
    bool isLinearStrafeLeftActive_;
    bool isLinearStrafeRightActive_;
    bool isDeadmanPressed_; /**< \brief Identify if the deadman button is pressed*/
    bool curvatureConstraintEnabled_; /**< \brief Identify curvature contraint is to be applied*/

    ros::Publisher twistPublisher_; /**< \brief The publisher for twist messages*/
    ros::Publisher robotFunction1Publisher_; /**< \brief The publisher for robot function 1*/
    ros::Publisher robotFunction2Publisher_; /**< \brief The publisher for robot function 2*/
    ros::Publisher robotKillPublisher_; /**< \brief The publisher for robot kill*/

    ros::Subscriber joySubscriber_; /**< \brief The subscriber for joystick messages*/

    double linearMaxSpeedMetersPerSecond_; /**< \brief Maximum commandable linear speed (m/s)*/
    double angularMaxSpeedRadsPerSecond_; /**< \brief Maximum commandable angular speed (rad/s)*/
    double linearToAngularVelocityRatioMinimum_; /**< \brief A curvature constraint. Zero disables */

  };


  GamepadTwist::GamepadTwist(const ros::NodeHandle & nh) : nodeHandle_(nh)
  {
    // Look for the gamepad control parameters in the local namespace on the
    // parameter server
    nodeHandle_.param("axis_linear_forward", linearForwardAxis_, 4);
    nodeHandle_.param("axis_linear_reverse", linearReverseAxis_, 5);
    nodeHandle_.param("axis_linear_ascend", linearAscendAxis_, 3);
    nodeHandle_.param("axis_linear_descend", linearDescendAxis_, 3);
    nodeHandle_.param("axis_linear_strafeLeft", linearStrafeLeftAxis_,2);
    nodeHandle_.param("axis_linear_strafeRight", linearStrafeRightAxis_,2);
    nodeHandle_.param("axis_angular", angularAxis_, 0);
    nodeHandle_.param("scale_angular", angularScale_, 0.5);
    nodeHandle_.param("scale_linear", linearScale_, 0.5);
    nodeHandle_.param("deadman_button", deadmanButton_, 0);
    nodeHandle_.param("robotFunction1_button", robotFunction1Button_, 2);
    nodeHandle_.param("robotFunction2_button", robotFunction2Button_, 3);
    nodeHandle_.param("robotKill1_button", robotKill1Button_, 6);
    nodeHandle_.param("robotKill2_button", robotKill2Button_, 7);   

    nodeHandle_.param("curvature_over_ride_1", curvatureOverRideOne_, 4);
    nodeHandle_.param("curvature_over_ride_2", curvatureOverRideTwo_, 5);

    asrl::rosutil::param(nodeHandle_, "base/linear_max_speed_meters_per_second", linearMaxSpeedMetersPerSecond_, 1.0 );
    asrl::rosutil::param(nodeHandle_, "base/angular_max_speed_rads_per_second", angularMaxSpeedRadsPerSecond_, 2.0 );
    asrl::rosutil::param(nodeHandle_, "base/linear_to_angular_velocity_ratio_minimum", linearToAngularVelocityRatioMinimum_, 0.0);
      

    // Check for a potential problem. If a wireless gamepad loses connection
    // with autorepeat on (a joy node setting), the robot cannot be stopped
    // and the robot may try to escape through a wall.
    double joyRepeat = -1.0;
    nodeHandle_.param("joy/autorepeat_rate", joyRepeat, -1.0);
    if (joyRepeat > 0.0) {
      ROS_WARN("Joy/autorepeat_rate > 0.0, beware wireless gamepads with this setting! "
               "Use only those that return zero when communication is lost.");
    } else {
      ROS_WARN("Beware wireless gamepads with autorepeat_rate > 0! "
               "Use only those that return zero when communication is lost.");
    }

    // Setup the subscriber to joy messages, and the publisher of twist
    // messages
    joySubscriber_ = nodeHandle_.subscribe<sensor_msgs::Joy>("/joy", 10, &GamepadTwist::joyCallback, this);
    twistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("out/twist", 1);
    robotFunction1Publisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotFunction1_withDeadman", 1);
    robotFunction2Publisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotFunction2_withoutDeadman", 1);
    robotKillPublisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotKill_withoutDeadman", 1);

    // Set the axes as not being active. This is used to solve a problem where
    // some axis remapping happens under ros. Discussed in more detail later
    // on.
    isLinearForwardActive_ = false;
    isLinearReverseActive_ = false;
    isLinearAscendActive_ = false;
    isLinearDescendActive_ = false;
    isLinearStrafeLeftActive_ = false;
    isLinearStrafeRightActive_ = false;

    // Set the deadman as not pressed.
    isDeadmanPressed_ = false;
  }


  void GamepadTwist::spin(void)
  {
    ros::spin();
    return;
  }

  // Get legal speed
  float GamepadTwist::getLegalSpeed(float command, float limit)
  {
    if (fabs(command) >= limit) {
      if (command > 0.0) {
        command = limit;
      } else {
        command = -limit;
      }
    }

    return command;
  }



  void GamepadTwist::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {

    // Process RobotKill IMPORTANT: ONCE THE KILL BUTTON IS PRESSED THERE IS NO WAY TO STOP OR INVERT THE KILL FUNCTION
    if ( (joy->buttons[robotKill1Button_] == 1) && (joy->buttons[robotKill2Button_] == 1) )
    {
      robotKillPublisher_.publish(true); 
    } 
    else 
    { 
      robotKillPublisher_.publish(false);
    }

    geometry_msgs::Twist twist;
    try {
      ASRL_ASSERT_GE_LT(std::runtime_error, linearForwardAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_forward\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearReverseAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_reverse\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearAscendAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_ascend\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearDescendAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_descend\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearStrafeLeftAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_strafeLeft\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearStrafeRightAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_strafeRight\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, angularAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_angular\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, deadmanButton_, 0, (int)joy->buttons.size(), "Parameter \"deadmam_button\" is out of range for this gamepad");
    }
    catch(const std::runtime_error & e)
      {
        ROS_ERROR_STREAM(e.what());
        ros::shutdown();
        return;
      }

   // Process Robot Function 1 Button!
    if ( (joy->buttons[robotFunction1Button_] == 1) && (joy->buttons[deadmanButton_] == 1) )
    { 
      robotFunction1Publisher_.publish(true);
    } 
    else
    {
      robotFunction1Publisher_.publish(false);
    }

    // Process Robot Function 2 Button!
    if (joy->buttons[robotFunction2Button_] == 1){
        robotFunction2Publisher_.publish(true);
    } else {
        robotFunction2Publisher_.publish(false);
    }

    float linearForwardValue = joy->axes[linearForwardAxis_];
    float linearReverseValue = joy->axes[linearReverseAxis_];
    float linearAscendValue = joy->axes[linearAscendAxis_];
    float linearDescendValue = joy->axes[linearDescendAxis_];
    float linearStrafeLeftValue = joy->axes[linearStrafeLeftAxis_];
    float linearStrafeRightValue = joy->axes[linearStrafeRightAxis_];

    // Produce twist messages
    if (joy->buttons[deadmanButton_] == 1) {
      isDeadmanPressed_ = true;
    
      if (joy->buttons[curvatureOverRideOne_] == 1 && joy->buttons[curvatureOverRideTwo_] ==1)
        curvatureConstraintEnabled_ = false;
      else
        curvatureConstraintEnabled_ = true;

        twist.angular.z = angularScale_*joy->axes[angularAxis_];

      // If the triggers on the xBox controller are not mapped to the linear
      // velocity we don't have to use an offset on the axis input. We assume
      // that all triggers are going to act the same in ROS. The way we
      // identify if triggers are used is whether the forward and reverse axes
      // are the same
      if ( linearForwardAxis_ == linearReverseAxis_ ) {
        twist.linear.x = linearScale_*linearForwardValue;

        // The triggers are being used for linear speed. We need to account
        // for the input offset and the remapping problem
      } else {
        // The axis range in ROS seems to change once the triggers have been
        // touched so we must watch for the first time each trigger is used
        // (from 0.0 when not pressed to 1.0 when not pressed).
        if ( !isLinearForwardActive_ ) {
          if (fabs(linearForwardValue) < 0.5f) {
            linearForwardValue = 1.0f;
          } else {
            isLinearForwardActive_ = true;
            ROS_DEBUG("Forward active");
          }
        }

        if ( !isLinearReverseActive_ ) {
          if (fabs(linearReverseValue) < 0.5f) {
            linearReverseValue = 1.0f;
          } else {
            isLinearReverseActive_ = true;
            ROS_DEBUG("Reverse active");
          }
        }

        ROS_DEBUG("Linear velocity: [%f, %f, %f], Angular velocity: [%f]",
                  linearForwardValue, linearReverseValue,
                  (-(linearForwardValue-1.0f) + (linearReverseValue-1.0f) ),
                  joy->axes[angularAxis_]);

        // For the linear velocity use the combined inputs of both triggers
        twist.linear.x = linearScale_*(-(linearForwardValue-1.0f) + (linearReverseValue-1.0f) );


        // Now we limit the speed and curvature
        double linear = getLegalSpeed(twist.linear.x, linearMaxSpeedMetersPerSecond_);
        double maxAngular = angularMaxSpeedRadsPerSecond_;
        if(curvatureConstraintEnabled_ && linearToAngularVelocityRatioMinimum_ > 0)
          {
            maxAngular = std::min(maxAngular,fabs(linear)/linearToAngularVelocityRatioMinimum_);
          }


        double angular = getLegalSpeed(twist.angular.z, maxAngular);
        twist.linear.x = linear;
        twist.angular.z = angular;

      }


	// Compute twist.linear.z
	if ( linearAscendAxis_ == linearDescendAxis_ ) {
        twist.linear.z = linearScale_*linearAscendValue;

        // The triggers are being used for linear speed. We need to account
        // for the input offset and the remapping problem, see linearForward/Reverse
      } else {
        if ( !isLinearAscendActive_ ) {
          if (fabs(linearAscendValue) < 0.5f) {
            linearAscendValue = 1.0f;
          } else {
            isLinearAscendActive_ = true;
            ROS_DEBUG("Ascend active");
          }
        }

        if ( !isLinearDescendActive_ ) {
          if (fabs(linearDescendValue) < 0.5f) {
            linearDescendValue = 1.0f;
          } else {
            isLinearDescendActive_ = true;
            ROS_DEBUG("Descend active");
          }
        }

        // For the linear velocity use the combined inputs of both triggers
        twist.linear.z = linearScale_*(-(linearAscendValue-1.0f) + (linearDescendValue-1.0f) );
      }
	// Compute twist.linear.y, STRAFING!
	if ( linearStrafeLeftAxis_ == linearStrafeRightAxis_ ) {
        twist.linear.y = linearScale_*linearStrafeLeftValue;

        // The triggers are being used for linear speed. We need to account
        // for the input offset and the remapping problem, see linearForward/Reverse
      } else {
        if ( !isLinearStrafeLeftActive_ ) {
          if (fabs(linearStrafeLeftValue) < 0.5f) {
            linearStrafeLeftValue = 1.0f;
          } else {
            isLinearStrafeLeftActive_ = true;
            ROS_DEBUG("StafeLeft active");
          }
        }

        if ( !isLinearStrafeRightActive_ ) {
          if (fabs(linearStrafeRightValue) < 0.5f) {
            linearStrafeRightValue = 1.0f;
          } else {
            isLinearStrafeRightActive_ = true;
            ROS_DEBUG("StafeRight active");
          }
        }

        // For the linear velocity use the combined inputs of both triggers
        twist.linear.y = linearScale_*(-(linearStrafeLeftValue-1.0f) + (linearStrafeRightValue-1.0f) );
      }

      twistPublisher_.publish(twist);

    } else {
      // Only send one stop command once the deadman button is released. This
      // prevents conflict between other nodes that may be trying to command
      // the robot
      if (isDeadmanPressed_) {
        isDeadmanPressed_ = false;
        twist.angular.z = 0.0f;
        twist.linear.x = 0.0f;
        twist.linear.y = 0.0f;
        twist.linear.z = 0.0f;

        twistPublisher_.publish(twist);
      }
    }

    return;
  }

} // namespace asrl


int main(int argc, char** argv)
{
  return asrl::rosutil::spinNodeCatchException<asrl::GamepadTwist>(argc, argv, "gamepad_twist");
}

