// Used for absolute value
#include <math.h>

// ros
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

// Definitions of the twist and joystick messages
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/msg/twist.hpp>
// #include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
// #include <sensor_msgs/Joy.h>
#include <sensor_msgs/msg/joy.hpp>
#include <iostream>
#include <functional>

// Some useful functions for running nodes
//#include <asrl/rosutil/node_utilities.hpp>
//#include <asrl/rosutil/param.hpp>

namespace asrl {

  //* GamepadControl
  /**
   * The GamepadControl node connects to a joystick node and receives joy
   * messages. The messages are interpreted as desired velocities and a
   * appropriate twist command is created and published.
   */
  class GamepadControl
  {
  public:
    /*!
     * \brief The constructor, initialize the node
     *
     * In the constructor, set parameters from the parameter server (if
     * available), subscribe to joystick messages, set up the publisher of
     * twist messages.
     */
    GamepadControl(rclcpp::Node::SharedPtr node);

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
    void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy);
    void checkBounds(std::string parameter, double value,double lower_bound, double upper_bound);
    rclcpp::Node::SharedPtr node_;

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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher_; /**< \brief The publisher for twist messages*/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robotFunction1Publisher_; /**< \brief The publisher for robot function 1*/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robotFunction2Publisher_; /**< \brief The publisher for robot function 2*/
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robotKillPublisher_; /**< \brief The publisher for robot kill*/

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySubscriber_; /**< \brief The subscriber for joystick messages*/

    double linearMaxSpeedMetersPerSecond_; /**< \brief Maximum commandable linear speed (m/s)*/
    double angularMaxSpeedRadsPerSecond_; /**< \brief Maximum commandable angular speed (rad/s)*/
    double linearToAngularVelocityRatioMinimum_; /**< \brief A curvature constraint. Zero disables */
    double verticalMax_; // maximum allowable percentage of control_vz_max

  };


  GamepadControl::GamepadControl(rclcpp::Node::SharedPtr node) : node_(node)
  {
    // Look for the gamepad control parameters in the local namespace on the
    // parameter server
    node_->declare_parameter("axis_linear_forward", 4);
    node_->declare_parameter("axis_linear_reverse", 5);
    node_->declare_parameter("axis_linear_ascend", 3);
    node_->declare_parameter("axis_linear_descend", 3);
    node_->declare_parameter("axis_linear_strafeLeft", 2);
    node_->declare_parameter("axis_linear_strafeRight", 2);
    node_->declare_parameter("axis_angular", 0);
    node_->declare_parameter("scale_angular", 0.5);
    node_->declare_parameter("scale_linear", 0.5);
    node_->declare_parameter("deadman_button", 0);
    node_->declare_parameter("robotFunction1_button", 2);
    node_->declare_parameter("robotFunction2_button", 3);
    node_->declare_parameter("robotKill1_button", 6);
    node_->declare_parameter("robotKill2_button", 7);   

    node_->declare_parameter("curvature_over_ride_1", 4);
    node_->declare_parameter("curvature_over_ride_2", 5);
    
    node_->declare_parameter("base/linear_max_speed_meters_per_second", 1.0 );
    node_->declare_parameter("base/angular_max_speed_rads_per_second", 2.0 );
    node_->declare_parameter("base/vertical_max", 0.5 );
    node_->declare_parameter("base/linear_to_angular_velocity_ratio_minimum", 0.0);

    node_->get_parameter("axis_linear_forward", linearForwardAxis_);
    node_->get_parameter("axis_linear_reverse", linearReverseAxis_);
    node_->get_parameter("axis_linear_ascend", linearAscendAxis_);
    node_->get_parameter("axis_linear_descend", linearDescendAxis_);
    node_->get_parameter("axis_linear_strafeLeft", linearStrafeLeftAxis_);
    node_->get_parameter("axis_linear_strafeRight", linearStrafeRightAxis_);
    node_->get_parameter("axis_angular", angularAxis_);
    node_->get_parameter("scale_angular", angularScale_);
    node_->get_parameter("scale_linear", linearScale_);
    node_->get_parameter("deadman_button", deadmanButton_);
    node_->get_parameter("robotFunction1_button", robotFunction1Button_);
    node_->get_parameter("robotFunction2_button", robotFunction2Button_);
    node_->get_parameter("robotKill1_button", robotKill1Button_);
    node_->get_parameter("robotKill2_button", robotKill2Button_);   

    node_->get_parameter("curvature_over_ride_1", curvatureOverRideOne_);
    node_->get_parameter("curvature_over_ride_2", curvatureOverRideTwo_);
    
    node_->get_parameter("base/linear_max_speed_meters_per_second", linearMaxSpeedMetersPerSecond_);
    node_->get_parameter("base/angular_max_speed_rads_per_second", angularMaxSpeedRadsPerSecond_);
    node_->get_parameter("base/vertical_max", verticalMax_);
    node_->get_parameter("base/linear_to_angular_velocity_ratio_minimum", linearToAngularVelocityRatioMinimum_);
      

    // Check for a potential problem. If a wireless gamepad loses connection
    // with autorepeat on (a joy node setting), the robot cannot be stopped
    // and the robot may try to escape through a wall.
    double joyRepeat = -1.0;
    node_->get_parameter_or("joy/autorepeat_rate", joyRepeat, -1.0);
    if (joyRepeat > 0.0) {
      RCLCPP_WARN(node_->get_logger(), "Joy/autorepeat_rate > 0.0, beware wireless gamepads with this setting! "
               "Use only those that return zero when communication is lost.");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Beware wireless gamepads with autorepeat_rate > 0! "
               "Use only those that return zero when communication is lost.");
    }

    // Setup the subscriber to joy messages, and the publisher of twist
    // messages
    joySubscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&GamepadControl::joyCallback, this, std::placeholders::_1));
    twistPublisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("out/twist", 1);
    robotFunction1Publisher_ = node_->create_publisher<std_msgs::msg::Bool>("out/robotFunction1_withDeadman", 1);
    robotFunction2Publisher_ = node_->create_publisher<std_msgs::msg::Bool>("out/robotFunction2_withoutDeadman", 1);
    robotKillPublisher_ = node->create_publisher<std_msgs::msg::Bool>("out/robotKill_withoutDeadman", 1);
    // joySubscriber_ = nodeHandle_.subscribe<sensor_msgs::Joy>("/joy", 10, &GamepadControl::joyCallback, this);
    // twistPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("out/twist", 1);
    // robotFunction1Publisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotFunction1_withDeadman", 1);
    // robotFunction2Publisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotFunction2_withoutDeadman", 1);
    // robotKillPublisher_ = nodeHandle_.advertise<std_msgs::Bool>("out/robotKill_withoutDeadman", 1);

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


  void GamepadControl::spin(void)
  {
    rclcpp::spin(node_);
    return;
  }

  // Get legal speed
  float GamepadControl::getLegalSpeed(float command, float limit)
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

  void GamepadControl::checkBounds(std::string parameter, double value, double lower_bound, double upper_bound) {
    if(value < lower_bound || value > upper_bound) {
      std::string message = "Parameter " + parameter + " is out of range for this gamepad";
      RCLCPP_ERROR(node_->get_logger(), "%s",message.c_str());
      throw std::runtime_error(message);
    }
  }

  void GamepadControl::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy) {
    std_msgs::msg::Bool msg;

    // Process RobotKill IMPORTANT: ONCE THE KILL BUTTON IS PRESSED THERE IS NO WAY TO STOP OR INVERT THE KILL FUNCTION
    bool robotKill = (joy->buttons[robotKill1Button_] == 1) && (joy->buttons[robotKill2Button_] == 1);
    // Send the message
    msg.data = robotKill;
    robotKillPublisher_->publish(msg); 

    geometry_msgs::msg::Twist twist;
    try {
      checkBounds("axis_linear_forward", linearForwardAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_linear_reverse", linearReverseAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_linear_ascend", linearAscendAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_linear_descend", linearDescendAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_linear_strafeLeft", linearStrafeLeftAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_linear_strafeRight", linearStrafeRightAxis_, 0, (int)joy->axes.size());
      checkBounds("axis_angular", angularAxis_, 0, (int)joy->axes.size());
      checkBounds("deadman_button", deadmanButton_, 0, (int)joy->axes.size());
    }
    catch(const std::runtime_error & e) {
        RCLCPP_ERROR(node_->get_logger(), "%s",e.what());
        rclcpp::shutdown();
        return;
    }

    // Process Robot Function 1 Button!
    bool robotFunction1 = (joy->buttons[robotFunction1Button_] == 1) && (joy->buttons[deadmanButton_] == 1);
    msg.data = robotFunction1;
    robotFunction1Publisher_->publish(msg);

    // Process Robot Function 2 Button!
    bool robotFunction2 = (joy->buttons[robotFunction2Button_] == 1);
    msg.data = robotFunction2;
    robotFunction2Publisher_->publish(msg);

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
            RCLCPP_DEBUG(node_->get_logger(), "Forward active");
          }
        }

        if ( !isLinearReverseActive_ ) {
          if (fabs(linearReverseValue) < 0.5f) {
            linearReverseValue = 1.0f;
          } else {
            isLinearReverseActive_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "Reverse active");
          }
        }

        RCLCPP_DEBUG(node_->get_logger(), "Linear velocity: [%f, %f, %f], Angular velocity: [%f]",
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
        double maxVertical = verticalMax_;
	if ( linearAscendAxis_ == linearDescendAxis_ ) {
        twist.linear.z = getLegalSpeed(linearScale_*linearAscendValue,maxVertical);

        // The triggers are being used for linear speed. We need to account
        // for the input offset and the remapping problem, see linearForward/Reverse
      } else {
        if ( !isLinearAscendActive_ ) {
          if (fabs(linearAscendValue) < 0.5f) {
            linearAscendValue = 1.0f;
          } else {
            isLinearAscendActive_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "Ascend active");
          }
        }

        if ( !isLinearDescendActive_ ) {
          if (fabs(linearDescendValue) < 0.5f) {
            linearDescendValue = 1.0f;
          } else {
            isLinearDescendActive_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "Descend active");
          }
        }
        // For the linear velocity use the combined inputs of both triggers
        twist.linear.z = getLegalSpeed(linearScale_*(-(linearAscendValue-1.0f) + (linearDescendValue-1.0f) ),maxVertical);
      }
	// Compute twist.linear.y, STRAFING!

        double linear = linearMaxSpeedMetersPerSecond_;
	if ( linearStrafeLeftAxis_ == linearStrafeRightAxis_ ) {
        twist.linear.y = getLegalSpeed(linearScale_*linearStrafeLeftValue,linear);

        // The triggers are being used for linear speed. We need to account
        // for the input offset and the remapping problem, see linearForward/Reverse
      } else {
        if ( !isLinearStrafeLeftActive_ ) {
          if (fabs(linearStrafeLeftValue) < 0.5f) {
            linearStrafeLeftValue = 1.0f;
          } else {
            isLinearStrafeLeftActive_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "StafeLeft active");
          }
        }

        if ( !isLinearStrafeRightActive_ ) {
          if (fabs(linearStrafeRightValue) < 0.5f) {
            linearStrafeRightValue = 1.0f;
          } else {
            isLinearStrafeRightActive_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "StafeRight active");
          }
        }

        // For the linear velocity use the combined inputs of both triggers
        twist.linear.y = getLegalSpeed(linearScale_*(-(linearStrafeLeftValue-1.0f) + (linearStrafeRightValue-1.0f) ),linear);
      }

      twistPublisher_->publish(twist);

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

        twistPublisher_->publish(twist);
      }
    }

    return;
  }

} // namespace asrl


int main(int argc, char** argv)
{
  // ros::init(argc, argv, "gamepad_control");
  rclcpp::init(argc, argv);

  // ros::NodeHandle nh("~");
  auto node = rclcpp::Node::make_shared("gamepad_control");

  asrl::GamepadControl control_node(node);
  control_node.spin();
  //return asrl::rosutil::spinNodeCatchException<asrl::GamepadControl>(argc, argv, "gamepad_control");
}

