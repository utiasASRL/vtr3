# vtr_mission_planning

## Integration tests

Test the (master) mission client:

- Launch the toy server node (written in python):

  ```bash
  ros2 run vtr_mission_planning mission_server_node.py
  ```

- In a separate terminal, launch the test mission client node (check the comment in test script)

  ```bash
  ros2 run vtr_mission_planning mission_client_tests.py
  ```

Test the mission server

- Launch the mission server with trivial tactic and planner

  ```bash
  ros2 run vtr_mission_planning mission_server_node
  ```

- In a separate terminal, launch the test mission client node

  ```bash
  ros2 run vtr_mission_planning mission_client_tests.py
  ```

Interact with the mission server

- Launch the mission server with trivial tactic and planner

  ```bash
  ros2 run vtr_mission_planning mission_server_node
  ```

- In a separate terminal, start the master mission client

  ```bash
  ros2 run vtr_mission_planning master_client_node.py
  ```

- In another terminal, use the vtr_mission script to send commands. (--help)

  ```bash
  ros2 run vtr_mission_planning vtr_mission.py --help
  ```
