# Competition V2 runtime contract

The unified entry point is `d_task_bringup competition_first_task.launch.py`.
It accepts exactly three profiles:

- `readonly_bench`: no control, no payload release, dry-run servo, unverified camera.
- `no_prop_control`: control path may be exercised without auto-arm or physical servo output.
- `real_competition`: explicit real UDP/servo configuration, still blocked until camera orientation and coordinate acceptance markers are verified.

The entry point starts `readiness_gate`, the OpenMV V2 `h7_bridge_node` and `vision_interface_node`, `car_start_gateway`, `mission_controller_node`, and `servo_node`. It does not start the Pi-camera/selector/filter/predictor/legacy landing-error chain. The readiness gate publishes `/uav/readiness/ready`; the mission controller subscribes to that same topic. A false readiness value blocks control and arm requests.

The competition profile uses `car_udp_real.yaml` (allowed subnet `192.168.4.0/24`, non-loopback configuration), `servo_real.yaml` only for the explicit real profile, and `openmv_v2_competition.yaml` for `/dev/dtask_openmv` at 115200 baud. The local UDP profile remains available only for bench/testing.

Car progress is mapped to the finite `CarProgress` values: B=2, C=3, D=4, return=A=5. The drop profile rejects unset/negative B or D values. Mission deadlines are bounded by the existing 90-second deadline, B/D deadlines, payload latest-command deadline, and return reserve; timeout paths return or land and cannot report `COMPLETE` without the landed/disarmed conditions.

PX4 serial DDS deployment is documented separately in `scripts/pi/px4_dds_921600.env` and `start_px4_dds_921600.sh` for Pixhawk 6C Mini/PX4 v1.16.0 with Micro XRCE-DDS Agent 2.4.3. The device is explicitly separate from the OpenMV serial alias and the script rejects duplicate agents. Hardware identity and readiness still require physical validation.

Current status: `READY_FOR_JOINT_NO_PROP_ACCEPTANCE`. This branch is not flight-ready. Raspberry Pi real-time vision, no-propeller orientation/sign checks, real ESP32 UDP, real GPIO18 servo, and low-altitude closed-loop acceptance remain required.
