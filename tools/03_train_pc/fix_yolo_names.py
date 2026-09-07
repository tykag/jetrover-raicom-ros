# -*- coding: utf-8 -*-
"""Swap bolt/gear class names inside a YOLOv5 best.pt (no retrain)."""
import torch

src = r"D:\yolov5\runs\train\raicom_bolt_gear3\weights\best.pt"
dst = r"D:\yolov5\runs\train\raicom_bolt_gear3\weights\best_fixed.pt"

ckpt = torch.load(src, map_location="cpu", weights_only=False)
model = ckpt["model"]
names = model.names
print("before:", names)

if isinstance(names, dict):
    model.names = {0: names[1], 1: names[0]}
else:
    model.names = [names[1], names[0]]

ckpt["model"] = model
ckpt["names"] = model.names
torch.save(ckpt, dst)

print("after: ", model.names)
print("saved: ", dst)
