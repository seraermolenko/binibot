import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from ultralytics import YOLO
import numpy as np

def test_yolo_bytrack_on_dummy():
    model = YOLO("yolov8n.pt")

    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    frame2 = np.zeros((480, 640, 3), dtype=np.uint8)

    results = model.track(source=[frame1, frame2], stream=False, persist=True)

    assert results and hasattr(results[0], "boxes")

    # Checking that each frame's boxes can be accessed without error
    for r in results:
        _ = r.boxes.xyxy if r.boxes is not None else None