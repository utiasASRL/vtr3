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
        print(f"ROBOT_NAME: {self.robot_name}")
        self.last_vtx = 0
        self.current_vtx = 0
        self.following_gap = 5 # vertices
        self.first_pub = False # first pub comes from graphUpdate, subsequent from graphState
    
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
        print(f"localized to vertex {self.current_vtx}")

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

    def state_callback(self, graph_state):
        # get largest vid in the posegraph
        curr_vtx= self.last_vtx

        for vtx in graph_state.vertices:
            if vtx.id > self.last_vtx:
                self.last_vtx = vtx.id

        rid = (self.last_vtx >> 60) & 0xF
        major_id = (self.last_vtx >> 16) & 0xFFFFFFFFFFF
        minor_id = self.last_vtx & 0xFFFF
        print(f"vtx: {self.last_vtx}: robot_id {rid} | major_id {major_id} | minor_id {minor_id}")

        print("pub via graph state")
        # if at beginning or end of path (within following gap) do nothing
        if not(minor_id < self.following_gap or (minor_id - curr_vtx & 0xFFFF) < self.following_gap):
            command_vtx = self.last_vtx - self.following_gap
            print(f'waypoint to {command_vtx}')
            self.pack_and_publish_msg(command_vtx)

    def update_callback(self, graph_update):
        # if were in the following gap, keep trying to queue next repeat
        print('state callback')
        if self.last_vtx - self.current_vtx <= 2 * self.following_gap:
            # we are approaching following gap, queue next repeat

            command_vtx = graph_update.vertex_to.id - self.following_gap
            if command_vtx > self.current_vtx:
                print(f'waypoint to {command_vtx}')
                self.pack_and_publish_msg(command_vtx)

def main(args=None):
    rclpy.init(args=args)
    torrent_planner = TorrentPlanner()
    rclpy.spin(torrent_planner)
    torrent_planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()