#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Streaming camera frames to OpenVLA @ 20 FPS, blocking CLI prompt.
"""

import cv2, requests, json_numpy, numpy as np, time, logging, sys, signal
from threading import Event

# —— 配  置 ——————————————————————————————————————
SERVER_URL  = "http://<SERVER_IP>:8000/act"
DEVICE_ID   = 0          # 摄像头索引
FRAME_RATE  = 20         # FPS
LOG_FILE    = "actions.log"
TIMEOUT_S   = 10         # HTTP 超时
json_numpy.patch()
# —————————————————————————————————————————————

logging.basicConfig(
    filename=LOG_FILE,
    level=logging.INFO,
    format="%(asctime)s | %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)

stop_event = Event()

def signal_handler(sig, frame):
    """Ctrl‑C：停止当前指令的循环，但不中断程序 """
    print("\n已请求停止本轮执行…")
    stop_event.set()

signal.signal(signal.SIGINT, signal_handler)

def should_stop():
    """当需要结束当前指令时返回 True。
       这里默认监听 Ctrl‑C；你可以改成检测动作幅度 < 阈值等。"""
    return stop_event.is_set()

def run_instruction(cap: cv2.VideoCapture, instruction: str):
    """20 FPS 把相机帧送给 /act，打印并记录返回动作。"""
    inter_frame  = 1.0 / FRAME_RATE
    next_deadline = time.perf_counter()

    while True:
        # 时间控制：保持稳定帧率
        now = time.perf_counter()
        sleep_t = next_deadline - now
        if sleep_t > 0:
            time.sleep(sleep_t)
        next_deadline += inter_frame

        ret, frame_bgr = cap.read()
        if not ret:
            print("读取相机失败，跳过本帧")
            continue

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        payload = {
            "image": frame_rgb,
            "instruction": instruction,
        }

        try:
            resp = requests.post(SERVER_URL, json=payload, timeout=TIMEOUT_S)
            resp.raise_for_status()
            action = np.array(resp.json())  # -> (7,) or (8,) 连续动作向量
            # 打印 + 记录
            print("🡪", action)
            logging.info("%s | %s", instruction, action.tolist())
        except Exception as e:
            print(f"请求失败：{e}")

        if should_stop():
            stop_event.clear()          # 为下一指令复位
            print("本指令结束\n")
            break

def main():
    cap = cv2.VideoCapture(DEVICE_ID)
    if not cap.isOpened():
        sys.exit("camera can\'t open")

    print("输入指令；按 Ctrl‑C 结束当前指令动作流；Ctrl‑D / Ctrl‑Z 退出程序。\n")

    try:
        while True:
            try:
                instruction = input("👉 指令> ").strip()
            except (EOFError, KeyboardInterrupt):  # Ctrl‑D / Ctrl‑Z / Ctrl‑C at prompt
                print("\n退出。")
                break
            if not instruction:
                continue
            print(f"执行指令: “{instruction}” —— 按 Ctrl‑C 结束\n")
            run_instruction(cap, instruction)
    finally:
        cap.release()

if __name__ == "__main__":
    main()
