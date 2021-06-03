uint8 TEMPORAL = 0
uint8 SPATIAL = 1

uint64 from_id   # Note: used to be "from, but it is a keyword in python
uint64 to_id
uint8 type
bool manual
geometry_msgs/Transform t_to_from