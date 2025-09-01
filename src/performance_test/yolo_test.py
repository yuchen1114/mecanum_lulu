#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 推論效能量測腳本
- 平均延遲 (ms/frame)
- FPS
- 記憶體佔用 (MB)
- 可指定相機/影片來源、模型、推論幀數、影像尺寸
"""

import argparse
import os
import time
import csv
import psutil
import subprocess
from statistics import mean
import cv2
from ultralytics import YOLO


def gpu_mem_rpi() -> float | None:
    """在 Raspberry Pi 上讀取 GPU 記憶體（若指令存在）。回傳 MB 或 None。"""
    try:
        out = subprocess.check_output(["vcgencmd", "get_mem", "gpu"], text=True)
        # e.g. "gpu=76M\n"
        val = out.strip().split("=")[-1].upper().replace("M", "")
        return float(val)
    except Exception:
        return None


def parse_args():
    ap = argparse.ArgumentParser(description="YOLO 推論效能量測")
    ap.add_argument("--model", type=str, required=True,
                    help="模型權重路徑，如 yolov8n.pt 或 runs/detect/xxx/weights/best.pt")
    ap.add_argument("--source", type=str, default="0",
                    help="影像來源：攝影機索引 (如 0) 或影片路徑")
    ap.add_argument("--frames", type=int, default=200, help="量測幀數（實際成功推論幀數）")
    ap.add_argument("--imgsz", type=int, default=640, help="推論影像尺寸（方形最長邊）")
    ap.add_argument("--save_csv", type=str, default="",
                    help="若指定檔名，將結果寫入 CSV（例如 results_pi5.csv）")
    ap.add_argument("--verbose", action="store_true", help="顯示每幀時間")
    return ap.parse_args()


def open_source(src: str) -> cv2.VideoCapture:
    if src.isdigit():
        cap = cv2.VideoCapture(int(src))
    else:
        cap = cv2.VideoCapture(src)
    return cap


def main():
    args = parse_args()

    # 讀取模型
    model = YOLO(args.model)

    # 開源（相機或影片）
    cap = open_source(args.source)
    if not cap.isOpened():
        raise RuntimeError(f"無法開啟來源：{args.source}")

    # 程式本身進程，用於記憶體量測
    proc = psutil.Process(os.getpid())

    # 預熱（避免第一次載入偏慢影響統計）
    for _ in range(3):
        ok, frame = cap.read()
        if not ok:
            break
        model.predict(source=frame, imgsz=args.imgsz, verbose=False)

    times = []          # 單幀推論時間（秒）
    mem_samples = []    # RSS 記憶體（MB）
    gpu_mem_samples = []  # Raspberry Pi GPU mem（MB）
    processed = 0

    start_wall = time.perf_counter()
    while processed < args.frames:
        ok, frame = cap.read()
        if not ok:
            # 影片播完或相機讀取失敗，跳出
            break

        t1 = time.perf_counter()
        _ = model.predict(source=frame, imgsz=args.imgsz, verbose=False)  # 推論
        t2 = time.perf_counter()

        dt = t2 - t1
        times.append(dt)
        mem_mb = proc.memory_info().rss / (1024 * 1024)
        mem_samples.append(mem_mb)

        rpi_gpu_mb = gpu_mem_rpi()
        if rpi_gpu_mb is not None:
            gpu_mem_samples.append(rpi_gpu_mb)

        processed += 1
        if args.verbose:
            print(f"[{processed:04d}] {dt*1000:.2f} ms, RSS={mem_mb:.1f} MB"
                  + (f", GPU={rpi_gpu_mb:.0f} MB" if rpi_gpu_mb is not None else ""))

    end_wall = time.perf_counter()
    cap.release()

    if processed == 0:
        raise RuntimeError("沒有成功處理任何影格，請確認來源或 frames 設定。")

    # 指標計算
    avg_latency_ms = mean(times) * 1000.0
    fps = processed / (end_wall - start_wall)  # 牽涉解碼與推論的整體 FPS（較接近實務）
    pure_infer_fps = 1.0 / mean(times)         # 純推論平均 FPS（不含解碼排隊等）

    avg_mem_mb = mean(mem_samples)
    max_mem_mb = max(mem_samples)
    avg_gpu_mb = mean(gpu_mem_samples) if gpu_mem_samples else None
    max_gpu_mb = max(gpu_mem_samples) if gpu_mem_samples else None

    # 結果列印
    print("\n=== YOLO 推論效能量測結果 ===")
    print(f"模型：{args.model}")
    print(f"來源：{args.source}")
    print(f"影像尺寸：{args.imgsz}")
    print(f"幀數（成功推論）：{processed}")
    print(f"平均延遲：{avg_latency_ms:.2f} ms/frame")
    print(f"純推論 FPS（1/avg latency）：{pure_infer_fps:.2f}")
    print(f"整體 FPS（含取幀等）：{fps:.2f}")
    print(f"記憶體（RSS）平均 / 最高：{avg_mem_mb:.1f} / {max_mem_mb:.1f} MB")
    if avg_gpu_mb is not None:
        print(f"Raspberry Pi GPU 記憶體 平均 / 最高：{avg_gpu_mb:.0f} / {max_gpu_mb:.0f} MB")

    # 可選：寫 CSV
    if args.save_csv:
        header = [
            "model", "source", "imgsz", "frames",
            "avg_latency_ms", "pure_infer_fps", "fps",
            "avg_mem_mb", "max_mem_mb", "avg_gpu_mb", "max_gpu_mb"
        ]
        row = [
            args.model, args.source, args.imgsz, processed,
            f"{avg_latency_ms:.3f}", f"{pure_infer_fps:.3f}", f"{fps:.3f}",
            f"{avg_mem_mb:.1f}", f"{max_mem_mb:.1f}",
            (f"{avg_gpu_mb:.0f}" if avg_gpu_mb is not None else ""),
            (f"{max_gpu_mb:.0f}" if max_gpu_mb is not None else "")
        ]
        write_header = not os.path.exists(args.save_csv)
        with open(args.save_csv, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            if write_header:
                w.writerow(header)
            w.writerow(row)
        print(f"\n已寫入 CSV：{os.path.abspath(args.save_csv)}")


if __name__ == "__main__":
    main()
