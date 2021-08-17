// ros
#include <ros/ros.h>

// Definitions of the twist and joystick messages
//#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

// Some useful functions for running nodes
//#include <asrl/rosutil/node_utilities.hpp>
//#include <asrl/rosutil/param.hpp>


namespace asrl {

  //* GamepadTrigger
  /**
   * The GamepadControl node connects to a joystick node and receives joy
   * messages. It watches specific trigger button activity
   */
  class GamepadTrigger
  {
  public:
    /*!
     * \brief The constructor, initialize the node
     *
     * In the constructor, set parameters from the parameter server (if
     * available), subscribe to joystick messages, set up the publisher of
     * twist messages.
     */
    GamepadTrigger(const ros::NodeHandle & nh);

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

    bool isTriggerPressed_; /**< \brief Identify if the deadman button is pressed*/

    ros::Publisher triggerPublisher_; /**< \brief The publisher for twist messages*/
    ros::Subscriber joySubscriber_; /**< \brief The subscriber for joystick messages*/

    int triggerButton_; /**< \brief The button index for the deadman button*/

  };


  GamepadTrigger::GamepadTrigger(const ros::NodeHandle & nh) : nodeHandle_(nh)
  {
    nodeHandle_.param("trigger_button", triggerButton_, 10); //right analogue stick press map to 10, right bumper map to 5

    // Setup the subscriber to joy messages, and the publisher of twist
    // messages
    joySubscriber_ = nodeHandle_.subscribe<sensor_msgs::Joy>("/joy", 10, &GamepadTrigger::joyCallback, this);
    triggerPublisher_ = nodeHandle_.advertise<std_msgs::String>("/capture_trigger", 1);

    // Set the deadman as not pressed.
    isTriggerPressed_ = false;
  }


  void GamepadTrigger::spin(void)
  {
    ros::spin();
    return;
  }

  void GamepadTrigger::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    std_msgs::String triggerMsg;
    triggerMsg.data = "Gamepad Trigger Released";
    
    if (joy->buttons[triggerButton_] == 1) {
      isTriggerPressed_ = true;
    }
    else if(isTriggerPressed_){
      isTriggerPressed_ = false;
            triggerPublisher_.publish(triggerMsg);

    }
    return;
  }

} // namespace asrl


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "gamepad_trigger");
  ros::NodeHandle nh("~");
  asrl::GamepadTrigger control_node(nh);
  control_node.spin();
  //return asrl::rosutil::spinNodeCatchException<asrl::GamepadTrigger>(argc, argv, "gamepad_trigger");
}

