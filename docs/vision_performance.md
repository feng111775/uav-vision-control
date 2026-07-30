# Performance truthfulness

The historical approximately 3.7 Hz valid observation measurement was caused chiefly by full-frame Hough circle pair detection (historically about 289 ms), not ROS publish rate. `fast_v2` uses blob-first candidate selection, ROI tracking, loss expansion and periodic strong validation; `legacy` remains selectable.

Run `scripts/benchmark/benchmark_openmv_serial.py` only with real hardware and use `analyze_vision_log.py` for P50/P90/P95. No current repository result proves 20 Hz valid-target observation or <=100 ms end-to-end latency; performance gate and closed-loop permission remain false.
