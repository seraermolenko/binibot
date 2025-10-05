def bbox_to_target(track, frame_w, frame_h):
    x1,y1,x2,y2 = track["xyxy"]
    cx = (x1+x2)/2.0
    w  = (x2-x1)
    bearing = (cx - frame_w/2) / (frame_w/2)  # -1..1
    dist_m  = max(0.15, 2.0 - (w/frame_w)*2.0)  # crude proxy, tune later
    return {"conf": float(track.get("conf", 0.0)),
            "bearing": float(bearing),
            "dist_m": float(dist_m)}