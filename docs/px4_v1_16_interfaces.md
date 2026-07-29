# PX4 v1.16 ROS 2 interfaces

Captured from ROS 2 Jazzy with `/home/xixi/px4_ros2_ws/install/px4_msgs` on 2026-07-29.

## px4_msgs/msg/OffboardControlMode

```text
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
bool thrust_and_torque
bool direct_actuator
```

## px4_msgs/msg/TrajectorySetpoint

```text
# Trajectory setpoint in NED frame
# Input to PID position controller.
# Needs to be kinematically consistent and feasible for smooth flight.
# setting a value to NaN means the state should not be controlled

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

# NED local world frame
float32[3] position # in meters
float32[3] velocity # in meters/second
float32[3] acceleration # in meters/second^2
float32[3] jerk # in meters/second^3 (for logging only)

float32 yaw # euler angle of desired attitude in radians -PI..+PI
float32 yawspeed # angular velocity around NED frame z-axis in radians/second
```

## px4_msgs/msg/VehicleCommand

```text
# Vehicle Command uORB message. Used for commanding a mission / action / etc.
# Follows the MAVLink COMMAND_INT / COMMAND_LONG definition

uint32 MESSAGE_VERSION = 0

uint64 timestamp					# time since system start (microseconds)

uint16 VEHICLE_CMD_CUSTOM_0 = 0				# test command
uint16 VEHICLE_CMD_CUSTOM_1 = 1				# test command
uint16 VEHICLE_CMD_CUSTOM_2 = 2				# test command
uint16 VEHICLE_CMD_NAV_WAYPOINT = 16			# Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_UNLIM = 17		# Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_TURNS = 18		# Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_LOITER_TIME = 19			# Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_RETURN_TO_LAUNCH = 20		# Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_NAV_LAND = 21			# Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_TAKEOFF = 22			# Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_PRECLAND = 23			# Attempt a precision landing
uint16 VEHICLE_CMD_DO_ORBIT = 34			# Start orbiting on the circumference of a circle defined by the parameters. |Radius [m] |Velocity [m/s] |Yaw behaviour |Empty |Latitude/X |Longitude/Y |Altitude/Z |
uint16 VEHICLE_CMD_DO_FIGUREEIGHT = 35        		# Start flying on the outline of a figure eight defined by the parameters. |Major Radius [m] |Minor Radius [m] |Velocity [m/s] |Orientation |Latitude/X |Longitude/Y |Altitude/Z |
uint16 VEHICLE_CMD_NAV_ROI = 80				# Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
uint16 VEHICLE_CMD_NAV_PATHPLANNING = 81		# Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|
uint16 VEHICLE_CMD_NAV_VTOL_TAKEOFF = 84		# Takeoff from ground / hand and transition to fixed wing |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_VTOL_LAND = 85			# Transition to MC and land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_NAV_GUIDED_LIMITS = 90		# set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|
uint16 VEHICLE_CMD_NAV_GUIDED_MASTER = 91		# set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_NAV_DELAY = 93				# Delay the next navigation command a number of seconds or until a specified time |Delay in seconds (decimal, -1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC)| Empty| Empty| Empty|
uint16 VEHICLE_CMD_NAV_LAST = 95			# NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_CONDITION_DELAY = 112		# Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_CONDITION_CHANGE_ALT = 113		# Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|
uint16 VEHICLE_CMD_CONDITION_DISTANCE = 114		# Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_CONDITION_YAW = 115			# Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|
uint16 VEHICLE_CMD_CONDITION_LAST = 159			# NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_CONDITION_GATE = 4501		# Wait until passing a threshold |2D coord mode: 0: Orthogonal to planned route | Altitude mode: 0: Ignore altitude| Empty| Empty| Lat| Lon| Alt|
uint16 VEHICLE_CMD_DO_SET_MODE = 176			# Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_JUMP = 177			# Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_CHANGE_SPEED = 178		# Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_SET_HOME = 179			# Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_DO_SET_PARAMETER = 180		# Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_SET_RELAY = 181			# Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_REPEAT_RELAY = 182		# Cycle a relay on and off for a desired number of cycles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_REPEAT_SERVO = 184		# Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_FLIGHTTERMINATION = 185		# Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_CHANGE_ALTITUDE = 186		# Set the vehicle to Loiter mode and change the altitude to specified value |Altitude| Frame of new altitude | Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_SET_ACTUATOR = 187		# Sets actuators (e.g. servos) to a desired value. |Actuator 1| Actuator 2| Actuator 3| Actuator 4| Actuator 5| Actuator 6| Index|
uint16 VEHICLE_CMD_DO_LAND_START = 189			# Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|
uint16 VEHICLE_CMD_DO_GO_AROUND = 191			# Mission command to safely abort an autonomous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_REPOSITION = 192			# Reposition to specific WGS84 GPS position. |Ground speed [m/s] |Bitmask |Loiter radius [m] for planes |Yaw	[deg] |Latitude	|Longitude |Altitude |
uint16 VEHICLE_CMD_DO_PAUSE_CONTINUE = 193
uint16 VEHICLE_CMD_DO_SET_ROI_LOCATION = 195		# Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Latitude| Longitude| Altitude|
uint16 VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196 	# Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| pitch offset from next waypoint| roll offset from next waypoint| yaw offset from next waypoint|
uint16 VEHICLE_CMD_DO_SET_ROI_NONE = 197		# Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Empty| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_CONTROL_VIDEO = 200		# Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_SET_ROI = 201			# Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|
uint16 VEHICLE_CMD_DO_DIGICAM_CONTROL=203
uint16 VEHICLE_CMD_DO_MOUNT_CONFIGURE=204		# Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_MOUNT_CONTROL=205			# Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|
uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST=206		# Mission command to set TRIG_DIST for this flight |Camera trigger distance (meters)| Shutter integration time (ms)| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_FENCE_ENABLE=207			# Mission command to enable the geofence |enable? (0=disable, 1=enable)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_PARACHUTE=208			# Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_MOTOR_TEST=209			# motor test command |Instance (1, ...)| throttle type| throttle| timeout [s]| Motor count | Test order| Empty|
uint16 VEHICLE_CMD_DO_INVERTED_FLIGHT=210		# Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_GRIPPER = 211			# Command to operate a gripper
uint16 VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL=214	# Mission command to set TRIG_INTERVAL for this flight |Camera trigger distance (meters)| Shutter integration time (ms)| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT=220		# Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_GUIDED_MASTER=221			# set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_GUIDED_LIMITS=222			# set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|
uint16 VEHICLE_CMD_DO_LAST = 240			# NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|
uint16 VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241		# Trigger calibration. This command will be only accepted if in pre-flight mode. See mavlink spec MAV_CMD_PREFLIGHT_CALIBRATION
uint16 PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION = 3# param value for VEHICLE_CMD_PREFLIGHT_CALIBRATION to start temperature calibration
uint16 VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242	# Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|
uint16 VEHICLE_CMD_PREFLIGHT_UAVCAN = 243		# UAVCAN configuration. If param 1 == 1 actuator mapping and direction assignment should be started
uint16 VEHICLE_CMD_PREFLIGHT_STORAGE = 245		# Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|
uint16 VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246	# Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|
uint16 VEHICLE_CMD_OBLIQUE_SURVEY=260			# Mission command to set a Camera Auto Mount Pivoting Oblique Survey for this flight|Camera trigger distance (meters)| Shutter integration time (ms)| Camera minimum trigger interval| Number of positions| Roll| Pitch| Empty|
uint16 VEHICLE_CMD_DO_SET_STANDARD_MODE=262		# Enable the specified standard MAVLink mode |MAV_STANDARD_MODE|
uint16 VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION = 283	# Command to ask information about a low level gimbal

uint16 VEHICLE_CMD_MISSION_START = 300			# start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|
uint16 VEHICLE_CMD_ACTUATOR_TEST = 310			# Actuator testing command|value [-1,1]|timeout [s]|Empty|Empty|output function|
uint16 VEHICLE_CMD_CONFIGURE_ACTUATOR = 311		# Actuator configuration command|configuration|Empty|Empty|Empty|output function|
uint16 VEHICLE_CMD_COMPONENT_ARM_DISARM = 400		# Arms / Disarms a component |1 to arm, 0 to disarm
uint16 VEHICLE_CMD_RUN_PREARM_CHECKS = 401      	# Instructs a target system to run pre-arm checks.
uint16 VEHICLE_CMD_INJECT_FAILURE = 420			# Inject artificial failure for testing purposes
uint16 VEHICLE_CMD_START_RX_PAIR = 500			# Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|
uint16 VEHICLE_CMD_REQUEST_MESSAGE = 512    		# Request to send a single instance of the specified message
uint16 VEHICLE_CMD_REQUEST_CAMERA_INFORMATION = 521     # Request camera information (CAMERA_INFORMATION). |0: No action 1: Request camera capabilities| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|
uint16 VEHICLE_CMD_SET_CAMERA_MODE = 530		# Set camera capture mode (photo, video, etc.)
uint16 VEHICLE_CMD_SET_CAMERA_ZOOM = 531		# Set camera zoom
uint16 VEHICLE_CMD_SET_CAMERA_FOCUS = 532
uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000 	# Setpoint to be sent to a gimbal manager to set a gimbal pitch and yaw
uint16 VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001 	# Gimbal configuration to set which sysid/compid is in primary and secondary control
uint16 VEHICLE_CMD_IMAGE_START_CAPTURE = 2000   	# Start image capture sequence.
uint16 VEHICLE_CMD_DO_TRIGGER_CONTROL = 2003		# Enable or disable on-board camera triggering system
uint16 VEHICLE_CMD_VIDEO_START_CAPTURE = 2500   	# Start a video capture.
uint16 VEHICLE_CMD_VIDEO_STOP_CAPTURE = 2501    	# Stop the current video capture.
uint16 VEHICLE_CMD_LOGGING_START = 2510			# start streaming ULog data
uint16 VEHICLE_CMD_LOGGING_STOP = 2511			# stop streaming ULog data
uint16 VEHICLE_CMD_CONTROL_HIGH_LATENCY = 2600		# control starting/stopping transmitting data over the high latency link
uint16 VEHICLE_CMD_DO_VTOL_TRANSITION = 3000    	# Command VTOL transition
uint16 VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST = 3001	# Request arm authorization
uint16 VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY = 30001	# Prepare a payload deployment in the flight plan
uint16 VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY = 30002	# Control a pre-programmed payload deployment
uint16 VEHICLE_CMD_FIXED_MAG_CAL_YAW = 42006            # Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
uint16 VEHICLE_CMD_DO_WINCH = 42600			# Command to operate winch.

uint16 VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE = 43003 # external reset of estimator global position when deadreckoning
uint16 VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE = 43004

# PX4 vehicle commands (beyond 16 bit mavlink commands)
uint32 VEHICLE_CMD_PX4_INTERNAL_START    = 65537        # start of PX4 internal only vehicle commands (> UINT16_MAX)
uint32 VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN = 100000       # Sets the GPS coordinates of the vehicle local origin (0,0,0) position. |Empty|Empty|Empty|Empty|Latitude|Longitude|Altitude|
uint32 VEHICLE_CMD_SET_NAV_STATE = 100001               # Change mode by specifying nav_state directly. |nav_state|Empty|Empty|Empty|Empty|Empty|Empty|

uint8 VEHICLE_MOUNT_MODE_RETRACT = 0			# Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization |
uint8 VEHICLE_MOUNT_MODE_NEUTRAL = 1			# Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. |
uint8 VEHICLE_MOUNT_MODE_MAVLINK_TARGETING = 2		# Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization |
uint8 VEHICLE_MOUNT_MODE_RC_TARGETING = 3		# Load neutral position and start RC Roll,Pitch,Yaw control with stabilization |
uint8 VEHICLE_MOUNT_MODE_GPS_POINT = 4			# Load neutral position and start to point to Lat,Lon,Alt |
uint8 VEHICLE_MOUNT_MODE_ENUM_END = 5			#

uint8 VEHICLE_ROI_NONE = 0                         	# No region of interest |
uint8 VEHICLE_ROI_WPNEXT = 1                       	# Point toward next MISSION |
uint8 VEHICLE_ROI_WPINDEX = 2                     	# Point toward given MISSION |
uint8 VEHICLE_ROI_LOCATION = 3                    	# Point toward fixed location |
uint8 VEHICLE_ROI_TARGET = 4                       	# Point toward target
uint8 VEHICLE_ROI_ENUM_END = 5

uint8 PARACHUTE_ACTION_DISABLE = 0
uint8 PARACHUTE_ACTION_ENABLE = 1
uint8 PARACHUTE_ACTION_RELEASE = 2

uint8 FAILURE_UNIT_SENSOR_GYRO = 0
uint8 FAILURE_UNIT_SENSOR_ACCEL = 1
uint8 FAILURE_UNIT_SENSOR_MAG = 2
uint8 FAILURE_UNIT_SENSOR_BARO = 3
uint8 FAILURE_UNIT_SENSOR_GPS = 4
uint8 FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5
uint8 FAILURE_UNIT_SENSOR_VIO = 6
uint8 FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7
uint8 FAILURE_UNIT_SENSOR_AIRSPEED = 8
uint8 FAILURE_UNIT_SYSTEM_BATTERY = 100
uint8 FAILURE_UNIT_SYSTEM_MOTOR = 101
uint8 FAILURE_UNIT_SYSTEM_SERVO = 102
uint8 FAILURE_UNIT_SYSTEM_AVOIDANCE = 103
uint8 FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104
uint8 FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105

uint8 FAILURE_TYPE_OK = 0
uint8 FAILURE_TYPE_OFF = 1
uint8 FAILURE_TYPE_STUCK = 2
uint8 FAILURE_TYPE_GARBAGE = 3
uint8 FAILURE_TYPE_WRONG = 4
uint8 FAILURE_TYPE_SLOW = 5
uint8 FAILURE_TYPE_DELAYED = 6
uint8 FAILURE_TYPE_INTERMITTENT = 7

# used as param1 in DO_CHANGE_SPEED command
uint8 SPEED_TYPE_AIRSPEED = 0
uint8 SPEED_TYPE_GROUNDSPEED = 1
uint8 SPEED_TYPE_CLIMB_SPEED = 2
uint8 SPEED_TYPE_DESCEND_SPEED = 3

# used as param3 in CMD_DO_ORBIT
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4
uint8 ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5

# used as param1 in ARM_DISARM command
int8 ARMING_ACTION_DISARM = 0
int8 ARMING_ACTION_ARM = 1

# param2 in VEHICLE_CMD_DO_GRIPPER
uint8 GRIPPER_ACTION_RELEASE = 0
uint8 GRIPPER_ACTION_GRAB = 1

uint8 ORB_QUEUE_LENGTH = 8

float32 param1			# Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param2			# Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param3			# Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param4			# Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param5			# Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
float64 param6			# Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
float32 param7			# Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
uint32 command			# Command ID
uint8 target_system		# System which should execute the command
uint8 target_component		# Component which should execute the command, 0 for all components
uint8 source_system		# System sending the command
uint16 source_component	# Component / mode executor sending the command
uint8 confirmation		# 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
bool from_external

uint16 COMPONENT_MODE_EXECUTOR_START = 1000

# TOPICS vehicle_command gimbal_v1_command vehicle_command_mode_executor
```

## px4_msgs/msg/VehicleStatus

```text
# Encodes the system state of the vehicle published by commander

uint32 MESSAGE_VERSION = 1

uint64 timestamp # time since system start (microseconds)

uint64 armed_time # Arming timestamp (microseconds)
uint64 takeoff_time # Takeoff timestamp (microseconds)

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED    = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
uint8 ARM_DISARM_REASON_STICK_GESTURE = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
uint8 ARM_DISARM_REASON_LOCKDOWN = 10
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
uint8 ARM_DISARM_REASON_SHUTDOWN = 12
uint8 ARM_DISARM_REASON_UNIT_TEST = 13

uint64 nav_state_timestamp # time when current nav_state activated

uint8 nav_state_user_intention                  # Mode that the user selected (might be different from nav_state in a failsafe situation)

uint8 nav_state                                 # Currently active mode
uint8 NAVIGATION_STATE_MANUAL = 0               # Manual mode
uint8 NAVIGATION_STATE_ALTCTL = 1               # Altitude control mode
uint8 NAVIGATION_STATE_POSCTL = 2               # Position control mode
uint8 NAVIGATION_STATE_AUTO_MISSION = 3         # Auto mission mode
uint8 NAVIGATION_STATE_AUTO_LOITER = 4          # Auto loiter mode
uint8 NAVIGATION_STATE_AUTO_RTL = 5             # Auto return to launch mode
uint8 NAVIGATION_STATE_POSITION_SLOW = 6
uint8 NAVIGATION_STATE_FREE5 = 7
uint8 NAVIGATION_STATE_FREE4 = 8
uint8 NAVIGATION_STATE_FREE3 = 9
uint8 NAVIGATION_STATE_ACRO = 10                # Acro mode
uint8 NAVIGATION_STATE_FREE2 = 11
uint8 NAVIGATION_STATE_DESCEND = 12             # Descend mode (no position control)
uint8 NAVIGATION_STATE_TERMINATION = 13         # Termination mode
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15                # Stabilized mode
uint8 NAVIGATION_STATE_FREE1 = 16
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17        # Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18           # Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19  # Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20       # Precision land with landing target
uint8 NAVIGATION_STATE_ORBIT = 21               # Orbit in a circle
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22   # Takeoff, transition, establish loiter
uint8 NAVIGATION_STATE_EXTERNAL1 = 23
uint8 NAVIGATION_STATE_EXTERNAL2 = 24
uint8 NAVIGATION_STATE_EXTERNAL3 = 25
uint8 NAVIGATION_STATE_EXTERNAL4 = 26
uint8 NAVIGATION_STATE_EXTERNAL5 = 27
uint8 NAVIGATION_STATE_EXTERNAL6 = 28
uint8 NAVIGATION_STATE_EXTERNAL7 = 29
uint8 NAVIGATION_STATE_EXTERNAL8 = 30
uint8 NAVIGATION_STATE_MAX = 31

uint8 executor_in_charge                        # Current mode executor in charge (0=Autopilot)

uint32 valid_nav_states_mask                    # Bitmask for all valid nav_state values
uint32 can_set_nav_states_mask                  # Bitmask for all modes that a user can select

# Bitmask of detected failures
uint16 failure_detector_status
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1              # (1 << 0)
uint16 FAILURE_PITCH = 2             # (1 << 1)
uint16 FAILURE_ALT = 4               # (1 << 2)
uint16 FAILURE_EXT = 8               # (1 << 3)
uint16 FAILURE_ARM_ESC = 16          # (1 << 4)
uint16 FAILURE_BATTERY = 32          # (1 << 5)
uint16 FAILURE_IMBALANCED_PROP = 64  # (1 << 6)
uint16 FAILURE_MOTOR = 128           # (1 << 7)

uint8 hil_state
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# Current vehicle locomotion method. A vehicle can have different methods (e.g. VTOL transitions from RW to FW method)
uint8 vehicle_type
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2 # Failsafes deferred, but would trigger a failsafe

bool failsafe # true if system is in failsafe state (e.g.:RTL, Hover, Terminate, ...)
bool failsafe_and_user_took_over # true if system is in failsafe state but the user took over control
uint8 failsafe_defer_state # one of FAILSAFE_DEFER_STATE_*

# Link loss
bool gcs_connection_lost              # datalink to GCS lost
uint8 gcs_connection_lost_counter     # counts unique GCS connection lost events
bool high_latency_data_link_lost # Set to true if the high latency data link (eg. RockBlock Iridium 9603 telemetry module) is lost

# VTOL flags
bool is_vtol             # True if the system is VTOL capable
bool is_vtol_tailsitter  # True if the system performs a 90° pitch down rotation during transition from MC to FW
bool in_transition_mode  # True if VTOL is doing a transition
bool in_transition_to_fw # True if VTOL is doing a transition from MC to FW

# MAVLink identification
uint8 system_type  # system type, contains mavlink MAV_TYPE
uint8 system_id	   # system id, contains MAVLink's system ID field
uint8 component_id # subsystem / component id, contains MAVLink's component ID field

bool safety_button_available # Set to true if a safety button is connected
bool safety_off # Set to true if safety is off

bool power_input_valid                            # set if input power is valid
bool usb_connected                                # set to true (never cleared) once telemetry received from usb link

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool rc_calibration_in_progress
bool calibration_enabled

bool pre_flight_checks_pass		# true if all checks necessary to arm pass
```

## px4_msgs/msg/VehicleLocalPosition

```text
# Fused local position in NED.
# The coordinate system origin is the vehicle position at the time when the EKF2-module was started.

uint32 MESSAGE_VERSION = 0

uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

bool xy_valid				# true if x and y are valid
bool z_valid				# true if z is valid
bool v_xy_valid				# true if vx and vy are valid
bool v_z_valid				# true if vz is valid

# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

# Position reset delta
float32[2] delta_xy			# Amount of lateral shift of position estimate in latest reset (in x and y) [m]
uint8 xy_reset_counter			# Index of latest lateral position estimate reset
float32 delta_z				# Amount of vertical shift of position estimate in latest reset [m]
uint8 z_reset_counter			# Index of latest vertical position estimate reset

# Velocity in NED frame
float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)
float32 z_deriv				# Down position time derivative in NED earth-fixed frame, (metres/sec)

# Velocity reset delta
float32[2] delta_vxy			# Amount of lateral shift of velocity estimate in latest reset (in x and y) [m/s]
uint8 vxy_reset_counter			# Index of latest vertical velocity estimate reset
float32 delta_vz			# Amount of vertical shift of velocity estimate in latest reset [m/s]
uint8 vz_reset_counter			# Index of latest vertical velocity estimate reset

# Acceleration in NED frame
float32 ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)
float32 az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)

float32 heading				# Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
float32 heading_var
float32 unaided_heading                 # Same as heading but generated by integrating corrected gyro data only
float32 delta_heading			# Heading delta caused by latest heading reset [rad]
uint8 heading_reset_counter		# Index of latest heading reset
bool heading_good_for_control

float32 tilt_var

# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
bool xy_global				# true if position (x, y) has a valid global reference (ref_lat, ref_lon)
bool z_global				# true if z has a valid global reference (ref_alt)
uint64 ref_timestamp			# Time when reference position was set since system start, (microseconds)
float64 ref_lat				# Reference point latitude, (degrees)
float64 ref_lon				# Reference point longitude, (degrees)
float32 ref_alt				# Reference altitude AMSL, (metres)

# Distance to surface
bool dist_bottom_valid			# true if distance to bottom surface is valid
float32 dist_bottom			# Distance from from bottom surface to ground, (metres)
float32 dist_bottom_var                 # terrain estimate variance (m^2)

float32 delta_dist_bottom               # Amount of vertical shift of dist bottom estimate in latest reset [m]
uint8 dist_bottom_reset_counter         # Index of latest dist bottom estimate reset

uint8 dist_bottom_sensor_bitfield	# bitfield indicating what type of sensor is used to estimate dist_bottom
uint8 DIST_BOTTOM_SENSOR_NONE = 0
uint8 DIST_BOTTOM_SENSOR_RANGE = 1	# (1 << 0) a range sensor is used to estimate dist_bottom field
uint8 DIST_BOTTOM_SENSOR_FLOW = 2	# (1 << 1) a flow sensor is used to estimate dist_bottom field (mostly fixed-wing use case)

float32 eph				# Standard deviation of horizontal position error, (metres)
float32 epv				# Standard deviation of vertical position error, (metres)
float32 evh				# Standard deviation of horizontal velocity error, (metres/sec)
float32 evv				# Standard deviation of vertical velocity error, (metres/sec)

bool dead_reckoning                     # True if this position is estimated through dead-reckoning

# estimator specified vehicle limits
# set to INFINITY when limiting not required
float32 vxy_max				# maximum horizontal speed (meters/sec)
float32 vz_max				# maximum vertical speed (meters/sec)
float32 hagl_min			# minimum height above ground level (meters)
float32 hagl_max_z			# maximum height above ground level for z-control (meters)
float32 hagl_max_xy			# maximum height above ground level for xy-control (meters)

# TOPICS vehicle_local_position vehicle_local_position_groundtruth external_ins_local_position
# TOPICS estimator_local_position
```

## px4_msgs/msg/VehicleAttitude

```text
# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
# The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)

uint32 MESSAGE_VERSION = 0

uint64 timestamp                # time since system start (microseconds)

uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[4] q                    # Quaternion rotation from the FRD body frame to the NED earth frame
float32[4] delta_q_reset        # Amount by which quaternion has changed during last reset
uint8 quat_reset_counter        # Quaternion reset counter

# TOPICS vehicle_attitude vehicle_attitude_groundtruth external_ins_attitude
# TOPICS estimator_attitude
```

## px4_msgs/msg/VehicleCommandAck

```text
# Vehicle Command Ackonwledgement uORB message.
# Used for acknowledging the vehicle command being received.
# Follows the MAVLink COMMAND_ACK message definition

uint32 MESSAGE_VERSION = 0

uint64 timestamp		# time since system start (microseconds)

# Result cases. This follows the MAVLink MAV_RESULT enum definition
uint8 VEHICLE_CMD_RESULT_ACCEPTED = 0			# Command ACCEPTED and EXECUTED |
uint8 VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1	# Command TEMPORARY REJECTED/DENIED |
uint8 VEHICLE_CMD_RESULT_DENIED = 2			# Command PERMANENTLY DENIED |
uint8 VEHICLE_CMD_RESULT_UNSUPPORTED = 3		# Command UNKNOWN/UNSUPPORTED |
uint8 VEHICLE_CMD_RESULT_FAILED = 4			# Command executed, but failed |
uint8 VEHICLE_CMD_RESULT_IN_PROGRESS = 5		# Command being executed |
uint8 VEHICLE_CMD_RESULT_CANCELLED = 6			# Command Canceled

# Arming denied specific cases
uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint8 ORB_QUEUE_LENGTH = 4

uint32 command						# Command that is being acknowledged
uint8 result						# Command result
uint8 result_param1					# Also used as progress[%], it can be set with the reason why the command was denied, or the progress percentage when result is MAV_RESULT_IN_PROGRESS
int32 result_param2					# Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied.
uint8 target_system
uint16 target_component 				# Target component / mode executor

bool from_external					# Indicates if the command came from an external source
```
