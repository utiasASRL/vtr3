# vtr_mission_planning

Unit test included.

Launch a toy mission planning server written in python to test the client

```bash
ros2 run vtr_mission_planning mission_server_node.py
```

or launch a test mission planning server (the C++ one) to test the server

```bash
ros2 run vtr_mission_planning mission_server_node
```

Then, launch a test mission planning client

```bash
ros2 run vtr_mission_planning mission_client.py
```
