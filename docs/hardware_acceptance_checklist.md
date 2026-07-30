# Hardware acceptance

SITL validation is not physical visual closed-loop acceptance. Capture protocol logs for SEARCH, FOLLOW, DROP_ALIGN and calculate separately: serial input, new frame, valid observation and ROS publish rate; only V2 clock-synchronized data can report end-to-end latency. Verify four aircraft directions, 1.5 m geometry, lighting, rotation, occlusion, blur and slow moving platform before enabling any later control adaptation.
