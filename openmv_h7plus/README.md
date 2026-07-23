# OpenMV Cam H7 Plus 红色色块检测

本目录是 OpenMV Cam H7 Plus 端的 MicroPython 程序，要求使用 **OpenMV 固件
5.0 / MicroPython 1.28**。OV5640 摄像头通过固件 5.0 的 `csi` 接口访问，默认按
竖直向下安装配置为 RGB565、QVGA（320×240），通过 USB CDC 虚拟串口以约
20 Hz 输出检测结果。

## 上传到 OpenMV

1. 用 USB 连接 H7 Plus，打开 OpenMV IDE 并连接开发板。
2. 将 `camera_config.py`、`detector.py`、`protocol.py`、`thresholds.py` 复制到
   板载文件系统根目录。
3. 在 IDE 中打开本目录的 `main.py`，选择 **Tools > Save open script to
   OpenMV Cam (as main.py)**。板载启动文件必须命名为 `main.py`。
4. 复位开发板或在 IDE 中运行脚本。脱离 IDE 自动启动前，确认所有五个 `.py`
   文件都位于板载文件系统根目录。

## 调整 LAB 阈值

阈值集中在 `thresholds.py` 的 `RED_THRESHOLDS`。在 OpenMV IDE 的
**Tools > Machine Vision > Threshold Editor** 中打开当前帧，框选红色目标并取得
LAB 六元组，然后替换文件中的起始阈值。不同光照、曝光和目标材料通常需要重新
标定。`detector.py` 中的 `PIXELS_THRESHOLD` 与 `AREA_THRESHOLD` 分别过滤像素数
过少和外接矩形面积过小的色块。

## 输出与帧率

协议每行均以换行符结束：

```text
TARGET,valid,cx,cy,width,height,area,confidence
TARGET,1,160,120,50,48,2400,90
TARGET,0,0,0,0,0,0,0
```

`area` 是通过颜色阈值的实际像素数 `blob.pixels`。固件 5.0 的 Blob 检测结果是
属性元组，因此代码使用 `blob.cx`、`blob.cy`、`blob.w`、`blob.h` 和
`blob.pixels`，不使用旧式的括号调用。`confidence` 是依据色块填充率
和画面占比计算的 **0～100 工程评分**，用于排序和状态观察，不是神经网络概率。
没有目标或单帧处理异常时仍会定期发送标准无目标行。

在 IDE 运行脚本时可查看右下角 FPS；若需在代码中临时观察，可在主循环中加入
`print(clock.fps())`。联调串口前应移除这类调试输出，避免与协议文本混杂。
协议发送周期为 50 ms（约 20 Hz），实际图像处理帧率可能更高或更低。

## 与 ROS 2 联调

OpenMV IDE 和 ROS 2 桥接节点不能同时占用同一个 USB CDC 串口。先停止脚本、
断开 IDE 与开发板的连接并关闭 IDE 的串口连接，再确认 Linux 出现
`/dev/ttyACM0`。如系统分配了其他设备名，通过 ROS 参数覆盖。

在工作空间根目录执行（需要先完成该工作空间自己的构建和环境加载）：

```bash
source install/setup.bash
ros2 run uav_vision h7_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 -p baudrate:=115200
```

另一个终端可查看桥接后的数据：

```bash
source install/setup.bash
ros2 topic echo /vision/h7/detection
```

USB CDC 本身不依赖传统 UART 波特率，但桥接节点保留 `115200` 参数以符合现有
接口。程序采用 OpenMV IDE 5.0 官方示例仍在使用的 `pyb.USB_VCP()`，并通过
`send()` 发送 ASCII 字节。只有关闭 OpenMV IDE 对串口的占用后，Linux 桥接节点
才能稳定逐行读取。

## 硬件验证清单

桌面 Python 只能检查语法，无法导入或运行 OpenMV 固件提供的 `csi`、`pyb`
模块。连接 H7 Plus 后还需验证摄像头方向、现场 LAB 阈值、曝光/白平衡稳定性、
实际 FPS、USB 断连恢复以及 `/dev/ttyACM0` 的持续逐行输出。
