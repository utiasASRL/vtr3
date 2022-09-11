uint8 EMPTY = 0
uint8 PROCESSING = 1
uint8 PENDING_PAUSE = 2
uint8 PAUSED = 3
uint8 server_state

uint8[16] current_goal_id

uint8 STARTING = 1
uint8 RUNNING = 2
uint8 FINISHING = 3
uint8 current_goal_state

GoalHandle[] goals