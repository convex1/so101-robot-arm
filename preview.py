"""Live camera preview window for aiming. Usage: python preview.py [camera_index] [seconds]. Press q to close."""
import sys, time, cv2
cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 90
cap = cv2.VideoCapture(cam, cv2.CAP_AVFOUNDATION)
t_end = time.time() + secs
while time.time() < t_end:
    ok, f = cap.read()
    if not ok:
        continue
    left = int(t_end - time.time())
    cv2.putText(f, f"camera {cam} - aim at table + arm - {left}s (q to close)", (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
    cv2.imshow("webcam preview", cv2.resize(f, (960, 540)))
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
ok, f = cap.read()
if ok:
    cv2.imwrite(sys.argv[3] if len(sys.argv) > 3 else "last_view.jpg", f)
cap.release(); cv2.destroyAllWindows()
