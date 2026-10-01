"""Teach-by-hand: turns follower torque OFF and records joint positions while you move it.
Usage: python record.py <name> [seconds]"""
import sys, time, json, os, scservo_sdk as scs

PORT = "/dev/tty.usbmodem5B3E0892191"
IDS = [1, 2, 3, 4, 5, 6]  # base, shoulder, elbow, wrist_flex, wrist_roll, gripper
HZ = 30

name = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 20

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)
for i in IDS:
    pk.write1ByteTxRx(ph, i, 40, 0)  # torque off so the arm can be moved by hand

frames = []
t0 = time.time(); next_print = 0
STOP = "STOP"  # create this file to end the recording early
if os.path.exists(STOP):
    os.remove(STOP)
while (t := time.time() - t0) < seconds and not os.path.exists(STOP):
    pos = []
    for i in IDS:
        p, c, _ = pk.read2ByteTxRx(ph, i, 56)
        pos.append(p if c == 0 else (frames[-1]["pos"][len(pos)] if frames else None))
    if None not in pos:
        frames.append({"t": round(t, 3), "pos": pos})
    if t >= next_print:
        print(f"{t:5.1f}s  {pos}", flush=True); next_print += 2
    time.sleep(max(0, 1 / HZ - (time.time() - t0 - t)))

json.dump({"ids": IDS, "hz": HZ, "frames": frames}, open(f"{name}.json", "w"))
print(f"saved {len(frames)} frames to {name}.json")
if os.path.exists(STOP):
    os.remove(STOP)
ph.closePort()
