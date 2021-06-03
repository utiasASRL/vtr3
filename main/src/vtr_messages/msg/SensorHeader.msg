
# Sequence ID for this message
int32 sequence_id -1
# time originating from the sensor
vtr_messages/TimeStamp sensor_time_stamp
# time stamp smoothed by an algorithm
vtr_messages/TimeStamp smoothed_time_stamp
# time originating from the host
vtr_messages/TimeStamp host_time_stamp

string type_name

string frame