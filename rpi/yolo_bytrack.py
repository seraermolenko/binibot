# rpi/vision/yolo_bytrack.py
from ultralytics import YOLO
import cv2, time

class YoloByteTrack:
    def __init__(self, model_path="yolov8n.pt", imgsz=416, conf=0.35):
        self.model = YOLO(model_path)
        self.imgsz = imgsz
        self.conf = conf
        # Built-in tracker config; fix/ clone & tune later
        self.tracker_cfg = "bytetrack.yaml"

    def stream(self, src=0):
        cap = cv2.VideoCapture(src)
        assert cap.isOpened(), "Camera not found"
        # streaming mode
        gen = self.model.track(
            source=src,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[0],            # person only
            tracker=self.tracker_cfg,
            persist=True,
            stream=True,
            verbose=False
        )
        for res in gen:
            tracks = []
            if len(res) == 0: 
                yield tracks; continue
            r = res[0]  # one source
            boxes = getattr(r, "boxes", None)
            if boxes is None: 
                yield tracks; continue

            # boxes.id contains ByteTrack IDs when tracker is active
            ids = boxes.id
            for i in range(len(boxes)):
                if ids is None:  
                    track_id = -1
                else:
                    track_id = int(ids[i].item())
                x1,y1,x2,y2 = map(int, boxes.xyxy[i].tolist())
                conf = float(boxes.conf[i].item())
                cls  = int(boxes.cls[i].item())  # 0 = person
                tracks.append({
                    "id": track_id,
                    "xyxy": (x1,y1,x2,y2),
                    "conf": conf,
                    "cls": cls
                })
            yield tracks
