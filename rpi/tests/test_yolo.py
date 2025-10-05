import numpy as np
from ultralytics import YOLO

def test_yolo_infers_on_dummy():
    model = YOLO("yolov8n.pt")  
    img = np.zeros((480, 640, 3), dtype=np.uint8)  
    results = model(img)
    assert results and len(results[0].boxes) >= 0 
