// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file state_machine.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp" 
#include "std_msgs/msg/string.hpp"
#include "vtr_mission_planning/state_machine/state_machine.hpp"

#include "vtr_mission_planning/state_machine/base_state.hpp"  // Idle
#include <string>
#include <typeinfo>

namespace vtr {
namespace mission_planning {

StateMachineInterface::StateMachineInterface(
    const StateMachineCallback::Ptr& callback)
    : callback_(callback) {}

StateMachineCallback::Ptr StateMachineInterface::callback() const {
  if (auto callback_acquired = callback_.lock())
    return callback_acquired;
  else {
    std::string err{"Callback has expired"};
    CLOG(ERROR, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

StateMachine::StateMachine(const Tactic::Ptr& tactic,
                           const RoutePlanner::Ptr& route_planner,
                           const PathPlanner::Ptr& path_planner,
                           const StateMachineCallback::Ptr& callback)
    : StateMachineInterface(callback),
      tactic_(tactic),
      route_planner_(route_planner),
      path_planner_(path_planner),
      node_(std::make_shared<rclcpp::Node>("state_machine_node")) {
  // initialize to idle state
  goals_.push_front(std::make_shared<Idle>());

  idle_instance.push_front(std::make_shared<Idle>()); // Same type as all other states, so we can use it to compare and find "idle"
  //
  thread_count_ = 1;
  process_thread_ = std::thread(&StateMachine::process, this);

  find_nearest_sub_ = node_->create_generic_subscription(
      "/vtr/find_nearest_vertex_2", "vtr_navigation_msgs/msg/FindNearest", rclcpp::QoS(10),
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      auto serialized_data = msg->get_rcl_serialized_message().buffer;

          if (serialized_data != nullptr && msg->size() >= 5) {
              size_t relevant_index = 4;  
              if (msg->size() > 5) {
                  relevant_index = 4;  
              }

              found_nearest = static_cast<bool>(serialized_data[relevant_index]);
              std::cout << "Found Nearest UP: " << (found_nearest ? "true" : "false") << std::endl;
              {
                  std::lock_guard<std::mutex> lock(msg_mutex_);
                  latest_find_nearest_msg_ = msg;  
              }
        } 
    });

  
}

StateMachine::~StateMachine() {
  UniqueLock lock(mutex_);
  // wait until no more event is being handled
  cv_empty_or_stop_.wait(lock, [this] { return stop_ || (event_ == nullptr); });
  // send stop signal
  stop_ = true;
  cv_set_or_stop_.notify_one();
  //
  cv_thread_finish_.wait(lock, [this] { return thread_count_ == 0; });
  if (process_thread_.joinable()) process_thread_.join();
}



void StateMachine::process() {


  el::Helpers::setThreadName("mission.state_machine");
  CLOG(INFO, "mission.state_machine") << "Starting the state machine thread.";

  tactic::VertexId v_initial(this->tactic()->getPersistentLoc().v);




  while (true) {
    UniqueLock lock(mutex_);

    cv_set_or_stop_.wait(lock, [this] { return stop_ || (event_ != nullptr); });

    if (stop_) {
      --thread_count_;
      CLOG(INFO, "mission.state_machine")
          << "Stopping the state machine thread.";
      cv_thread_finish_.notify_all();
      return;
    }

    CLOG(DEBUG, "mission.state_machine") << "Processing event " << *event_;

    auto curr_state = goals_.front();
    // acquire the tactic, route&path planner and callback so that they are not
    // expired during handling the event
    const auto tactic_acquired = tactic();
    const auto route_planner_acquired = route_planner();
    const auto path_planner_acquired = path_planner();
    const auto callback_acquired = callback();

    curr_state->processGoals(*this, *event_);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    executor.spin_some(); // Process callbacks once and then continue

    //We want to save the previous persistent localisation vertex, before it was overwritten with the scan match. In case the user clicks on using the scan match, proceedes to click cancel (wants actually no scan match), we can revert to the previous vertex. 
    if (save_pers_vertex) {
        prev_pers_loc = this->tactic()->getPersistentLoc().v;
    }
    if (found_nearest == false) {
        this->tactic()->setTrunk(prev_pers_loc);
    } else { 
        if (curr_state->name() == idle_instance.front()->name()) { //Confirming we are in Idle state

            // Register scan match
            this->tactic()->registerScanMatch();

            std::cout<<"SET SCAN MATCH IN STATE MACHINE" << std::endl;

            // Stop saving the vertex, since the save_pers_vertex keeps track of the vertex before overwriting it with scan match
            save_pers_vertex = false;
        }
    }


  // needs state transition
    if (curr_state != goals_.front()) {
      // perform all state transitions until we get to a state that is stable
      CLOG(INFO, "mission.state_machine") << "Lock the tactic pipeline.";
      auto pipeline_lock = tactic()->lockPipeline();
      while (curr_state != goals_.front()) {
        // The target state is always at the top of the stack
        auto new_state = goals_.front();

        CLOG(INFO, "mission.state_machine")
            << "Transitioning from " << curr_state->name() << " to "
            << new_state->name();

        // Invoke exit/entry logic for the old/new state
        curr_state->onExit(*this, *new_state);

        tactic()->setPipeline(new_state->pipeline());



        new_state->onEntry(*this, *curr_state);
        curr_state = new_state;

        CLOG(INFO, "mission.state_machine")
            << "In state " << curr_state->name();

        // Perform one processing step to see if the state will remain stable
        curr_state->processGoals(*this, Event());

      }
      CLOG(INFO, "mission.state_machine") << "Unlock the tactic pipeline.";
    }

    const auto state_success = trigger_success_;
    if (trigger_success_) trigger_success_ = false;
    event_ = nullptr;
    cv_empty_or_stop_.notify_all();

    CLOG(DEBUG, "mission.state_machine") << "Unlock the state machine.";
    lock.unlock();
    // mission server callback may try to acquire state machine lock within its
    // own lock, so we must release the state machine lock here to avoid
    // deadlock
    if (state_success) callback()->stateSuccess();
  }
  CLOG(INFO, "mission.state_machine") << "Stopping the state machine thread.";
}

void StateMachine::handle(const Event::Ptr& event, const bool block) {
  UniqueLock lock(mutex_, std::defer_lock);
  if (block) {
    lock.lock();
    cv_empty_or_stop_.wait(lock,
                           [this] { return stop_ || (event_ == nullptr); });
  } else {
    lock.try_lock();

    if (!lock.owns_lock()) {
      CLOG(WARNING, "mission.state_machine")
          << "Skipping event " << *event << " due to lock conflict.";
      return;
    }

    if (event_ != nullptr) {
      CLOG(WARNING, "mission.state_machine")
          << "Skipping event " << *event << " because there is already one.";
      return;
    }
  }

  if (stop_) {
    CLOG(WARNING, "mission.state_machine")
        << "Dropping event " << *event
        << " because the state machine is stopped.";
    return;
  }

  event_ = event;

  CLOG(DEBUG, "mission.state_machine") << "Handling event " << *event_;
  cv_set_or_stop_.notify_one();
}

void StateMachine::wait() const {
  UniqueLock lock(mutex_);
  cv_empty_or_stop_.wait(lock, [this] { return stop_ || (event_ == nullptr); });
}

std::string StateMachine::name() const {
  LockGuard lock(mutex_);
  return goals_.front()->name();
}

auto StateMachine::tactic() const -> Tactic::Ptr {
  if (auto tactic_acquired = tactic_.lock())
    return tactic_acquired;
  else {
    std::string err{"Tactic has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

auto StateMachine::route_planner() const -> RoutePlanner::Ptr {
  if (auto route_planner_acquired = route_planner_.lock())
    return route_planner_acquired;
  else {
    std::string err{"Route planner has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

auto StateMachine::path_planner() const -> PathPlanner::Ptr {
  if (auto path_planner_acquired = path_planner_.lock())
    return path_planner_acquired;
  else {
    std::string err{"Path planner has expired"};
    CLOG(WARNING, "mission.state_machine") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

}  // namespace mission_planning
}  // namespace vtr
