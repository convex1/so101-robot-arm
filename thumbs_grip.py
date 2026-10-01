"""Camera watches for a thumbs-up; each one makes the follower grip 3 times.
Arm goes to a slightly extended pose and holds it. Runs for N seconds, then returns to rest, torque OFF.
Usage: python thumbs_grip.py [seconds] [camera_index]"""
import sys, time, math, json, os
import cv2, mediapipe as mp, scservo_sdk as scs
from mediapipe.tasks.python import vision, BaseOptions

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 90
CAM = int(sys.argv[2]) if len(sys.argv) > 2 else 0
HERE = os.path.dirname(os.path.abspath(__file__))

# --- arm ---------------------------------------------------------------------
PORT = "/dev/tty.usbmodem5B3E0892191"
IDS = [1, 2, 3, 4, 5, 6]
cal = json.load(open(os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json")))
LIMITS = [(v["range_min"] + 30, v["range_max"] - 30) for v in cal.values()]
MID = [(v["range_min"] + v["range_max"]) // 2 for v in cal.values()]
REST = [2037, 3052, 835, 1033, 1953, 1809]
OPEN, SHUT = 2600, REST[5]
READY = [REST[0], MID[1] + 150, MID[2] + 200, MID[3], REST[4], SHUT]  # a little forward of straight up

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0); ph.openPort(); ph.setBaudRate(1_000_000)
sw = scs.GroupSyncWrite(ph, pk, 42, 2)
clamp = lambda pos: [int(p) if i == 4 else max(lo, min(hi, int(p))) for i, (p, (lo, hi)) in enumerate(zip(pos, LIMITS))]
cur = [pk.read2ByteTxRx(ph, i, 56)[0] for i in IDS]

def send(pos):
    sw.clearParam()
    for i, p in zip(IDS, pos):
        sw.addParam(i, [p & 0xFF, p >> 8])
    sw.txPacket()

def move(pose, dur):
    global cur
    target = clamp(pose); start = cur; t0 = time.time()
    while (t := time.time() - t0) < dur:
        a = 0.5 - 0.5 * math.cos(math.pi * t / dur)
        send([round(s + (e - s) * a) for s, e in zip(start, target)])
        time.sleep(0.02)
    send(target); cur = target
    if max(pk.read2ByteTxRx(ph, i, 60)[0] & 0x3FF for i in (2, 3)) > 700:
        raise RuntimeError("strain detected on shoulder/elbow")

def grip3():
    for _ in range(3):
        move(READY[:5] + [OPEN], 0.4); move(READY[:5] + [SHUT], 0.4)

def finish():
    move(REST, 2.5); time.sleep(0.8)
    for i in IDS:
        pk.write1ByteTxRx(ph, i, 40, 0)
    ph.closePort()

# --- camera + gesture model ----------------------------------------------------
cap = cv2.VideoCapture(CAM, cv2.CAP_AVFOUNDATION)
if not cap.isOpened():
    sys.exit("camera not available — allow camera access for your terminal app")
rec = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path=os.path.join(HERE, "models/gesture_recognizer.task"), delegate=BaseOptions.Delegate.CPU),
    running_mode=vision.RunningMode.VIDEO, min_hand_detection_confidence=0.5))

for i, p in zip(IDS, cur):
    pk.write2ByteTxRx(ph, i, 42, p); pk.write2ByteTxRx(ph, i, 46, 0); pk.write1ByteTxRx(ph, i, 41, 0)
    pk.write1ByteTxRx(ph, i, 40, 1)

try:
    move(READY, 2.5)
    print(f"READY — show a thumbs-up to the camera ({SECONDS:.0f}s)", flush=True)
    t_end = time.time() + SECONDS
    streak, armed, count, last_seen = 0, True, 0, 0.0
    while time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            continue
        img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        res = rec.recognize_for_video(img, int(time.time() * 1000))
        thumb = any(g[0].category_name == "Thumb_Up" and g[0].score > 0.6 for g in res.gestures)
        if thumb:
            streak += 1; last_seen = time.time()
        else:
            streak = 0
            if time.time() - last_seen > 0.6:
                armed = True  # hand must drop the thumb before the next trigger
        if armed and streak >= 3:
            count += 1; armed = False
            print(f"👍 thumbs-up #{count} — gripping", flush=True)
            grip3()
            for _ in range(5):
                cap.read()  # drop frames that queued up while gripping
    print(f"time's up — {count} thumbs-up handled", flush=True)
finally:
    cap.release()
    finish()
    print("back at rest, torque OFF", flush=True)
