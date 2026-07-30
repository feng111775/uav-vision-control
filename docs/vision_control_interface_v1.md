# Vision–control interface V1

`vision_interface_node` is the formal boundary. It publishes only `/vision/target/tracked`, `/vision/landing_error`, and `/vision/health`; it never publishes `/fmu/in/*`. `mission_offboard_controller` remains the sole PX4 control publisher and task decision maker.

Target and landing messages use best-effort, depth 5, volatile QoS. Health uses reliable depth 5 volatile. `/uav/mission/state` uses reliable, transient-local depth 1. Repeated 20 Hz ROS publication does not advance `frame_sequence`, timestamp, or confirmation count. At 300 ms of no new observation it continuously publishes an invalid, NaN-valued measurement.

`metric_valid=false`, `physical_release_enabled=false`, and `ready_for_closed_loop=false` are default safety states. No control package is changed.
