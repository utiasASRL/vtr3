// Used for absolute value
#include <math.h>

// ros
#include <ros/ros.h>

// Definitions of the twist and joystick messages
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

// Some useful functions for running nodes
#include <asrl/rosutil/node_utilities.hpp>
#include <asrl/rosutil/param.hpp>


namespace asrl {

  //* GamepadScalar
  /**
   * The GamepadScalar node connects to a joystick node and receives
   * joy messages. The messages are interpreted as a desired scalar
   * value and that scalar is published.
   */
  class GamepadScalar
  {
  public:
    /*!
     * \brief The constructor, initialize the node
     *
     * In the constructor, set parameters from the parameter server (if
     * available), subscribe to joystick messages, set up the publisher of
     * float messages.
     */
    GamepadScalar(const ros::NodeHandle & nh);

    /*!
     * \brief Spin the node, handle joy callbacks
     */
    void spin(void);


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
    
    int deadmanButton_; /**< \brief The button index for the deadman button*/

    double linearScale_; /**< \brief The joystick scale factor for linear velocity */

    bool isLinearForwardActive_; /**< \brief Identify if the linear forward axis is active*/
    bool isLinearReverseActive_; /**< \brief Identify if the linear reverse axis is active*/
    bool isDeadmanPressed_; /**< \brief Identify if the deadman button is pressed*/

    ros::Publisher floatPublisher_; /**< \brief The publisher for float messages*/
    ros::Subscriber joySubscriber_; /**< \brief The subscriber for joystick messages*/


  };


  GamepadScalar::GamepadScalar(const ros::NodeHandle & nh) : nodeHandle_(nh)
  {
    // Look for the gamepad control parameters in the local namespace on the
    // parameter server
    nodeHandle_.param("axis_linear_forward", linearForwardAxis_, 4);
    nodeHandle_.param("axis_linear_reverse", linearReverseAxis_, 5);
    nodeHandle_.param("scale_linear", linearScale_, 1.0);
    nodeHandle_.param("deadman_button", deadmanButton_, 0);      

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
    joySubscriber_ = nodeHandle_.subscribe<sensor_msgs::Joy>("/joy", 10, &GamepadScalar::joyCallback, this);
    floatPublisher_ = nodeHandle_.advertise<std_msgs::Float32>("out/scalar", 1);

    // Set the axes as not being active. This is used to solve a problem where
    // some axis remapping happens under ros. Discussed in more detail later
    // on.
    isLinearForwardActive_ = false;
    isLinearReverseActive_ = false;

    // Set the deadman as not pressed.
    isDeadmanPressed_ = false;
  }


  void GamepadScalar::spin(void)
  {
    ros::spin();
    return;
  }



  void GamepadScalar::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    float scalar = 0.0;
    try {
      ASRL_ASSERT_GE_LT(std::runtime_error, linearForwardAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_forward\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, linearReverseAxis_, 0, (int)joy->axes.size(), "Parameter \"axis_linear_reverse\" is out of range for this gamepad");
      ASRL_ASSERT_GE_LT(std::runtime_error, deadmanButton_, 0, (int)joy->buttons.size(), "Parameter \"deadmam_button\" is out of range for this gamepad");
    } catch(const std::runtime_error & e) {
      ROS_ERROR_STREAM(e.what());
      ros::shutdown();
      return;
    }

    float linearForwardValue = joy->axes[linearForwardAxis_];
    float linearReverseValue = joy->axes[linearReverseAxis_];
    
    // Figure out the appropriate scalar value to publish
    if (joy->buttons[deadmanButton_] == 1) {
      isDeadmanPressed_ = true;
    
      // If the triggers on the xBox controller are not mapped to the linear
      // velocity we don't have to use an offset on the axis input. We assume
      // that all triggers are going to act the same in ROS. The way we
      // identify if triggers are used is whether the forward and reverse axes
      // are the same
      if ( linearForwardAxis_ == linearReverseAxis_ ) {
        scalar = linearScale_*linearForwardValue;
        floatPublisher_.publish(scalar);
        ROS_DEBUG("Simple control: %f", scalar);

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
        
        ROS_DEBUG("Scalar: [%f, %f, %f]",
                  linearForwardValue, linearReverseValue,
                  (-(linearForwardValue-1.0f) + (linearReverseValue-1.0f) ));
        
        // For the linear velocity use the combined inputs of both triggers
        scalar = linearScale_*(-(linearForwardValue-1.0f) + (linearReverseValue-1.0f) );
        floatPublisher_.publish(scalar);
      }
      

    } else {
      // Only send one stop command once the deadman button is released. This
      // prevents conflict between other nodes that may be trying to command
      // the robot
      if (isDeadmanPressed_) {
        isDeadmanPressed_ = false;
        floatPublisher_.publish(0.0);
      }
    }  
    
    return;
  }

} // namespace asrl


int main(int argc, char** argv)
{
  return asrl::rosutil::spinNodeCatchException<asrl::GamepadScalar>(argc, argv, "gamepad_twist");
}

