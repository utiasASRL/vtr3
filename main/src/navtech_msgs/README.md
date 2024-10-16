# navtech_msgs

The navtech_msgs ROS package contains the *custom message definitions*, as used in the accompanying ROS packages. For
further details of message fields see brief field descriptions in .msg files.

## navtech_msgs/msg

The msg folder contains the files which represent the *custom ROS message definitions*.

### navtech_msgs/msg/CameraConfigurationMessage.msg

This file contains the custom message which defines the *camera configuration*, the *field names and types*
are as detailed below.

| Data Type      | Name         |
| :------------- | :----------: |
| uint32         | height       |
| uint32         | width        |
| uint32         | channels     |
| uint32         | fps          |
	
### navtech_msgs/msg/RadarConfigurationMsg.msg

This file contains the custom message which defines the *radar configuration*, the *field names and types*
are as detailed below.

| Data Type      | Name                   |
| :------------- | :--------------------: |
| uint8[]        | azimuth_samples        |
| uint8[]        | encoder_size           |
| uint8[]        | bin_size               |
| uint8[]        | range_in_bins          |
| uint8[]        | expected_rotation_rate |
| uint8[]        | range_gain             |
| uint8[]        | range_offset           |


*Important*
*azimuth_samples* should be cast to a *uint16_t* data type in the endianess of your current system, but it
is represented as a network order uint8_t byte array

*encoder_size* should be cast to a *uint16_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

*bin_size* should be cast to a *double* data type in the endianess of your current system, but it is represented
as a network order uint8_t byte array

*range_in_bins* should be cast to a *uint16_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

*expected_rotation_rate* should be cast to a *uint16_t* data type in the endianess of your current system, but
it is represented as a network order uint8_t byte array

*range_gain* should be cast to a *uint64_t* data type in the endianess of your current system, but
it is represented as a network order uint8_t byte array

*range_offset* should be cast to a *uint64_t* data type in the endianess of your current system, but
it is represented as a network order uint8_t byte array

### navtech_msgs/msg/RadarFftDataMsg.msg

This file contains the custom message which defines the *radar fft data*, the *field names and types* are as
detailed below.

| Data Type      | Name              |
| :------------- | :---------------: |
| uint8[]        | angle             |
| uint8[]        | azimuth           |
| uint8[]        | sweep_counter     |
| uint8[]        | ntp_seconds       |
| uint8[]        | ntp_split_seconds |
| uint8[]        | data              |
| uint8[]        | data_length       |

*Important*
*angle* should be cast to a *double* data type in the endianess of your current system, but it is represented as
a network order uint8_t byte array

*azimuth* should be cast to a *uint16* data type in the endianess of your current system, but it is represented
as a network order uint8_t byte array

*sweep_counter* should be cast to a *uint16_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

*ntp_seconds* should be cast to a *uint32_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

*ntp_split_seconds* should be cast to a *uint32_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

*data* should be cast to a *uint8_t* data type in the endianess of your current system, but it is represented as
a network order uint8_t byte array

*data_length* should be cast to a *uint16_t* data type in the endianess of your current system, but it is
represented as a network order uint8_t byte array

## CMakeLists

This is the *build file* which defines how the camera_ros ROS package is built and installed.

## package

This file contains *properties* of the camera_ros package, such as package name, versions, authors etc.

Note - these properties are not intended to be edited by the user, these are *build related properties*