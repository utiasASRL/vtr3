import rclpy
from rclpy.node import Node

import numpy as np
import uuid
import os

from vtr_navigation_msgs.msg import GraphUpdate, GraphState, GraphVertex, MissionCommand, GoalHandle, RobotState

import pdb

class TorrentPlanner(Node):
    def __init__(self):
        super().__init__('torrent_planner')
        self.robot_name = os.getenv("ROBOT_NAME")
        self.robot_id = os.getenv("ROBOT_ID")
        print(f"ROBOT_NAME: {self.robot_name}")
        self.last_vtx = 0
        self.current_vtx = 0
        self.target_vtx = 0
        self.following_gap = int(self.robot_id) * 5 # vertices (order is 0:mr_green, 1:prof_plum, 2:col_mustard, 3:mrs_peacock)
        self.min_step = 5
        self.replan_margin = 3
        self.start_flag = True
    
        self.command_pub = self.create_publisher(
            MissionCommand,
            f'/{self.robot_name}/vtr/mission_command',
            10
        )
        
        self.graph_state_sub = self.create_subscription(
            GraphState, 
            f'{self.robot_name}/vtr/graph_state', 
            self.state_callback,
            10)
        self.graph_update_sub = self.create_subscription(
            GraphUpdate, 
            f'{self.robot_name}/vtr/graph_update', 
            self.update_callback,
            10)
        self.robot_state_sub = self.create_subscription(
            RobotState, 
            f'{self.robot_name}/vtr/robot_state', 
            self.robot_state_callback,
            10)

    def robot_state_callback(self, robot_state):
        # get id of current vertex were localized to
        self.current_vtx = robot_state.index
        self.try_advance()        

    def state_callback(self, graph_state):
        # get largest vid in the posegraph
        print('state callback')
        for vtx in graph_state.vertices:
            self.last_vtx = max(self.last_vtx, vtx.id)
        self.try_advance()        


    def update_callback(self, graph_update):
        # if were in the following gap, keep trying to queue next repeat
        print('update callback')
        self.last_vtx = max(self.last_vtx, graph_update.vertex_to.id)
        self.try_advance()        

    def try_advance(self):
        # if not self.current_vtx or not self.last_vtx:
        #     print('not localized')
        #     return # not localized
        command_vtx = self.last_vtx - self.following_gap
        if (command_vtx & 0xFFFF) < self.following_gap or command_vtx <= self.current_vtx:
            print('not inside following gap')
            return # near start, or inside following gap; wait
        if self.start_flag:
            if command_vtx - self.target_vtx < self.min_step:
                print('not enough new maps')
                return # not enough new maps
            if self.target_vtx - self.current_vtx > self.replan_margin:
                print(f'target: {self.target_vtx}, current: {self.current_vtx}')
                print('current goal is good')
                return # current goal still far
        self.start_flag = True
        self.target_vtx = command_vtx
        print(f"pub waypoint to {command_vtx} (current {self.current_vtx}, last {self.last_vtx})")
        self.pack_and_publish_msg(command_vtx)

    def pack_and_publish_msg(self, command_vtx):
        # wrap into a ADD_GOAL mission command
        goal_handle = GoalHandle(
            id=list(uuid.uuid4().bytes), # UUID
            type=2, # repeat
            pause_before=0,
            pause_after=0,
            waypoints=[command_vtx],
            controller_name=''
        )

        add_goal = MissionCommand(
            type=1, # add goal
            pause=False,
            goal_handle=goal_handle,
            vertex=command_vtx,
            # window=? don't set window
        )
        self.command_pub.publish(add_goal)

        # wrap into a BEGIN_GOAL mission command
        goal_handle = GoalHandle(
            id=list(uuid.uuid4().bytes), # UUID
            type=0, # repeat
            pause_before=0,
            pause_after=0,
            # waypoints=[self.last_vtx], # dont set waypoints
            controller_name=''
        )

        begin_goal = MissionCommand(
            type=3, # add goal
            pause=False,
            goal_handle=goal_handle,
            vertex=0,
            # window=? don't set window
        )
        self.command_pub.publish(begin_goal)

def main(args=None):
    rclpy.init(args=args)
    torrent_planner = TorrentPlanner()
    rclpy.spin(torrent_planner)
    torrent_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()