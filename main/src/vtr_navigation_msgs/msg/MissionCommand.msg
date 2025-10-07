## Command Spec Constants
# server state
uint8 PAUSE = 0
# goal manipulation
uint8 ADD_GOAL = 1
uint8 CANCEL_GOAL = 2
uint8 BEGIN_GOALS = 3
# special commands
uint8 LOCALIZE = 4
uint8 START_MERGE = 5
uint8 CONFIRM_MERGE = 6
uint8 CONTINUE_TEACH = 7
uint8 SELECT_CONTROLLER = 8
uint8 type

## PAUSE
bool pause

## ADD_GOAL & CANCEL_GOAL - specify which the type of goal to start
GoalHandle goal_handle

## LOCALIZE - specify the vertex to localize
uint64 vertex

## START_MERGE - specify the window to merge
uint64[] window  # for START_MERGE

## SELECT_CONTROLLER - specify the controller name
string controller_name