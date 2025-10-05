import cv2
import glob

print("Scanning available cameras...")
for p in sorted(glob.glob("/dev/video*")):
    cap = cv2.VideoCapture(p)
    ok = cap.isOpened()
    print(f"{p}: {'works' if ok else 'cannot open'}")
    cap.release()