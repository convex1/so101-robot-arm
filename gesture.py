"""Named gestures for the follower. Usage: python gesture.py <name>
Smooth, range-clamped moves; stops and holds if shoulder/elbow strain. Ends at rest, torque OFF."""
import sys, time, math, json, os, scservo_sdk as scs

PORT = "/dev/tty.usbmodem5B3E0892191"
IDS = [1, 2, 3, 4, 5, 6]  # base, shoulder, elbow, wrist_flex, wrist_roll, gripper
cal = json.load(open(os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json")))
LIMITS = [(v["range_min"] + 30, v["range_max"] - 30) for v in cal.values()]
MID = [(v["range_min"] + v["range_max"]) // 2 for v in cal.values()]

REST = [2037, 3052, 835, 1033, 1953, 1809]
OPEN, SHUT = 2600, REST[5]
UP = [REST[0], MID[1], MID[2], MID[3], REST[4], SHUT]

def grips(pose, n=3):
    steps = []
    for _ in range(n):
        steps += [(pose[:5] + [OPEN], 0.5), (pose[:5] + [SHUT], 0.5)]
    return steps

GESTURES = {
    "up_grip": [(UP, 2.5), *grips(UP, 3), (UP, 0.5), (REST, 2.5)],
}

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0); ph.openPort(); ph.setBaudRate(1_000_000)
sw = scs.GroupSyncWrite(ph, pk, 42, 2)
clamp = lambda pos: [int(p) if i == 4 else max(lo, min(hi, int(p))) for i, (p, (lo, hi)) in enumerate(zip(pos, LIMITS))]

def send(pos):
    sw.clearParam()
    for i, p in zip(IDS, pos):
        sw.addParam(i, [p & 0xFF, p >> 8])
    sw.txPacket()

cur = [pk.read2ByteTxRx(ph, i, 56)[0] for i in IDS]
for i, p in zip(IDS, cur):
    pk.write2ByteTxRx(ph, i, 42, p); pk.write2ByteTxRx(ph, i, 46, 0); pk.write1ByteTxRx(ph, i, 41, 0)
    pk.write1ByteTxRx(ph, i, 40, 1)

for pose, dur in GESTURES[sys.argv[1]]:
    target = clamp(pose); start = cur; t0 = time.time()
    while (t := time.time() - t0) < dur:
        a = 0.5 - 0.5 * math.cos(math.pi * t / dur)
        send([round(s + (e - s) * a) for s, e in zip(start, target)])
        time.sleep(0.02)
    send(target); cur = target
    loads = [pk.read2ByteTxRx(ph, i, 60)[0] & 0x3FF for i in (2, 3)]
    if max(loads) > 700:
        sys.exit(f"strain detected {loads} — stopped and holding (run release.py after supporting the arm)")
    if pose is UP:
        print("up pose:", [pk.read2ByteTxRx(ph, i, 56)[0] for i in IDS])

time.sleep(0.8)
for i in IDS:
    pk.write1ByteTxRx(ph, i, 40, 0)
print("gesture done — back at rest, torque OFF")
ph.closePort()
