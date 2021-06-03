# vtr_interface

This package builds the UI interface (both frontend and backend) for the VT&R3 system.
The UI interface provides the ability to manage goals (TEACH, REPEAT, IDLE), as well the as ability to view and modify the pose graph and other localization actions.

## Table of Contents

- [vtr_interface](#vtr_interface)
  - [Table of Contents](#table-of-contents)
  - [User Interface (see vtr3.yaml)](#user-interface-see-vtr3yaml)
  - [User Interface (VTR2)](#user-interface-vtr2)
    - [Goal Panel](#goal-panel)
    - [Graph Map](#graph-map)
    - [Action Menu](#action-menu)
  - [System Structure](#system-structure)

## User Interface (see vtr3.yaml)

Launch the user interface:

```bash
ros2 run vtr_interface socket_client_node.py
ros2 run vtr_interface socket_server.py
```

Then go to [localhost:5201](http://localhost:5201)

## User Interface (VTR2)

After launching VT&R2, the interface can be accessed at [localhost:5000](http://localhost:5000)
The UI interface looks like the follows:
![alt text](https://github.com/utiasASRL/vtr2/blob/interface_tutorial/asrl__interface/UI.jpg)

The UI consists of 3 major components: the goal panel, the graph map, and the action menu.

### Goal Panel

The goal panel can be used to add 3 goals with the option to set a delay before and after the goal excecution. A form with the goal options will show up after the user clicks on the "+" button.

- **TEACH:** Add a new teach run by starting a new graph or branching off from the current graph.
- **REPEAT:** Add a new teach experience, the user can click on the graph map or type in "major:minor" vertex ids to set the waypints to be followed.
- **IDLE:**: Put the robot into idle state.

### Graph Map

The graph map view projects the locally topological pose graph on top of the Google satellite map. User can zoom in/out to investigate the graph and click on the pose graph to get vertex ids.

### Action Menu

The funcionality of each button is explaied below:

- **Graph Selector**: Some actions (merge, set localize position, edit graph) need to first select a subgraph or a vertex from the graph map to operate on. The graph selector is only activated during those actions. Nothing will happen if the graph selector button is clicked without the combination of those actions. There are 3 modes currectly implemented for the graph selector:
  - **Line Mode**: The line selector has 3 keypoints: start, middle, end. The keypoints are indicated by the arrows in the tool. User can grag on the keypoints to move and select the paths from the graph map. Note that selector forces the sequence of the arrows, which means for the selected path, the vertices are ordered by the breadth first search result of traversing through start, middle, end.
  - **Circle Mode**: The circle selector is shown as resizable circle in the graph map, the region covered by the circle will be selected.
  - **Point Mode**: The point selector is shown as a little circle icon pined on the pose graph. The point selected will always pin to its nearest neibour in the pose graph when being moved around on the map.
- **Merge Tool**: The merge tool is used for adding loop closure during TEACH. MERGE can only be triggered during TEACH and a search region must be selected by the graph selector to search for loop closure candiates. Clicking on the poped-up checkmark will trigger the merge candiate search. When a valid candidate (a SINGLE vertex) is found, a flashing green arrow will appear in the grpah map. Clicking the flashing green arrow (loop closing point candidate) will confirm the loop closure and trigger graph relaxation. The default graph selector for MERGE is the line selector, but the user can switch to other selector modes.
- **Set Localize Position**: this action is used for the purpose of placing the robot (yellow arrow) to a new place in the map, thus the VT&R2 system will try to localize against the new position at the beginning of the next run. The graph selector mode is forced to be point selector in this case. Clicking on the poped-up checkmark will place the robot in the selected position.
- **Force Relocalization**: this button is for the purpose of forcing relocalization during REPEAT, when localization is bad but certain, or if bad VO takes us off path.
- **Move Graph**: Clicking on this button will pop up a tool to move around and roate the pose graph. One the poped-up checkmark is clicked, the new postion and orientation of the pose graph will be logged in server (can't be undone).
- **Show Run Colors:** Show different runs in different colors.
- **Show Edge Weights:** Color the edges of the pose graph by its associated uncertainty.
- **Edit Pose Graph (Incomplete Development):**: 3 buttons will be poped-up with the ability to perform simple modifications on the pose graph (add junction, delete path, confirm). Adding junction will split the a run into 2 runs, delete path will delete a whole selected run. Hitting confirm will permanately change the graph (chunk files and graph protos), othweise the modifications will only be shown in the UI.

## System Structure

TODO
