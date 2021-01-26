uint8 EMPTY = 0
uint8 PROCESSING = 1
uint8 PENDING_PAUSE = 2
uint8 PAUSED = 3
# Is the server currently active
uint8 status
# List of goals, in order
vtr_messages/GoalUUID[] mission_queue