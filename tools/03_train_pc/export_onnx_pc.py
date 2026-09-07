# -*- coding: utf-8 -*-
"""在 Windows 电脑上把 best_fixed.pt 导出为 ONNX（在 D:\\yolov5 环境下运行）"""
import torch

# 若官方 export.py 可用，优先用命令行：
#   python export.py --weights runs/train/raicom_bolt_gear3/weights/best_fixed.pt --include onnx --img 640
#
# 本脚本作为备用：依赖本地 yolov5 的 export

import sys
import os

YOLOV5 = r"D:\yolov5"
WEIGHTS = r"D:\yolov5\runs\train\raicom_bolt_gear3\weights\best_fixed.pt"

os.chdir(YOLOV5)
sys.path.insert(0, YOLOV5)

from export import run

run(
    weights=WEIGHTS,
    imgsz=(640, 640),
    include=("onnx",),
    device="cpu",
    simplify=True,
)
print("done. 查看同目录下 best_fixed.onnx")
