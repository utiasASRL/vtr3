uint8[16] id

uint8 IDLE = 0
uint8 TEACH = 1
uint8 REPEAT = 2
uint8 type

# for repeat only
uint64 pause_before  # milliseconds
uint64 pause_after  # milliseconds
uint64[] waypoints  # vertex ids