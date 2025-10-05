# NOTE: Choosing a target to follow based of nearest (largest box) or most confident

from vision.yolo_bytrack import YoloByteTrack

tracker = YoloByteTrack("yolov8n.pt", imgsz=416, conf=0.35)

for tracks in tracker.stream(src=0):

    target = None
    if tracks:
        people = [t for t in tracks if t["cls"] == 0]
        if people:
            # Example policy: pick the largest box (rough proxy for closeness)
            people.sort(key=lambda t: (t["xyxy"][3]-t["xyxy"][1])*(t["xyxy"][2]-t["xyxy"][0]), reverse=True)
            target = people[0]

    if target:
        (x1,y1,x2,y2) = target["xyxy"]
        cx = (x1 + x2) / 2
        # You probably cached width/height from capture settings
        W = 640  
        bearing = (cx - W/2) / (W/2)      # -1..+1
        box_h = (y2 - y1)
        dist_hint = max(0.6, min(3.0, 1.6 * (480 / (box_h+1e-3))))
        # -> pass bearing/dist_hint into your state machine and UDP sender
    else:
        # No target; your FSM will go EXPLORE/WAIT
        pass


