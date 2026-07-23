"""OpenMV 固件 5.0 的 CSI 摄像头初始化。"""

import csi


FRAME_WIDTH = 320
FRAME_HEIGHT = 240
WARMUP_TIME_MS = 1500


def configure_camera():
    """创建并返回配置好的 CSI 对象。"""
    camera = csi.CSI()
    camera.reset()
    camera.pixformat(csi.RGB565)
    camera.framesize(csi.QVGA)

    # 默认假设镜头竖直向下；若板子安装方向导致图像倒置，可改为 True。
    camera.vflip(False)
    camera.hmirror(False)

    # 固件 5.0 官方示例用带 time 参数的 snapshot 等待设置生效。
    camera.snapshot(time=WARMUP_TIME_MS)

    # 自动白平衡保留开启，便于先完成联调。固定光照下调好阈值后，
    # 可在现场测试关闭自动白平衡以减少色彩漂移。
    return camera
