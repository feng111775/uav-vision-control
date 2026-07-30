# Calibration

Templates in `config/` intentionally contain no fabricated intrinsics, homography, mounting transform or drop offset. Until all are measured and validated, image errors are pixel-normalized only and metric fields remain NaN with `metric_valid=false`.
