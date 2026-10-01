"""Records the webcam to an MP4 while running a follower script (dance, gesture...).
Usage: python film.py <out.mp4> <script.py> [script args...]   e.g. film.py dance_cam.mp4 dance.py"""
import sys, time, threading, subprocess, os, cv2

out, script, args = sys.argv[1], sys.argv[2], sys.argv[3:]
HERE = os.path.dirname(os.path.abspath(__file__))
cap = cv2.VideoCapture(int(os.environ.get("CAM", "0")), cv2.CAP_AVFOUNDATION)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
for _ in range(20):
    cap.read()  # let exposure settle
ok, f = cap.read()
h, w = f.shape[:2]
writer = cv2.VideoWriter(out + ".raw.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))

stop = threading.Event()
def grab():
    t0 = time.time(); n = 0
    while not stop.is_set():
        ok, f = cap.read()
        if ok:
            writer.write(f); n += 1
    print(f"recorded {n} frames in {time.time()-t0:.1f}s")

th = threading.Thread(target=grab); th.start()
time.sleep(1.0)  # a beat of stillness before motion
subprocess.run([sys.executable, os.path.join(HERE, script), *args])
time.sleep(1.0)
stop.set(); th.join(); writer.release(); cap.release()
# re-encode to web-friendly H.264
subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", out + ".raw.mp4", "-c:v", "libx264", "-crf", "22",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", out])
os.remove(out + ".raw.mp4")
print("saved", out)
