"""Short scripted dance on the follower. Smoothly interpolates between poses, clamped to
calibrated ranges, aborts (holding pose) if a joint strains. Ends at rest with torque OFF."""
import time, math, json, os, scservo_sdk as scs

PORT = "/dev/tty.usbmodem5B3E0892191"
IDS = [1, 2, 3, 4, 5, 6]
cal = json.load(open(os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json")))
LIMITS = [(v["range_min"] + 30, v["range_max"] - 30) for v in cal.values()]

REST = [2037, 3052, 835, 1033, 1953, 1809]
MID = [(v["range_min"] + v["range_max"]) // 2 for v in cal.values()]
B, WR = REST[0], REST[4]                 # face where the arm rests; wrist-roll neutral
SH, EL, WF = MID[1], MID[2], MID[3]      # "arm up" = middle of each joint's range
UP = [B, SH, EL, WF, WR, REST[5]]
L, R = B - 300, B + 300                  # base sway
OPEN, SHUT = 2600, REST[5]               # gripper chomp
seq = [  # (pose, seconds to get there)
    (UP, 2.0),
    ([L, SH, EL, WF, WR - 400, OPEN], 0.8), ([R, SH, EL, WF, WR + 400, SHUT], 1.0),
    ([L, SH, EL, WF, WR - 400, OPEN], 1.0), ([R, SH, EL, WF, WR + 400, SHUT], 1.0),
    (UP, 0.8),
    ([B, SH, EL, WF - 300, WR, OPEN], 0.5), ([B, SH, EL, WF + 300, WR, SHUT], 0.5),
    ([B, SH, EL, WF - 300, WR, OPEN], 0.5), ([B, SH, EL, WF + 300, WR, SHUT], 0.5),
    ([B, SH - 100, EL - 100, WF, WR - 1000, SHUT], 1.2), ([B, SH - 100, EL - 100, WF, WR + 1000, SHUT], 1.6),
    (UP, 1.0),
    (REST, 2.5),
]

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0); ph.openPort(); ph.setBaudRate(1_000_000)
sw = scs.GroupSyncWrite(ph, pk, 42, 2)
clamp = lambda pos: [max(lo, min(hi, int(p))) if i != 4 else int(p) for i, (p, (lo, hi)) in enumerate(zip(pos, LIMITS))]

def send(pos):
    sw.clearParam()
    for i, p in zip(IDS, pos):
        sw.addParam(i, [p & 0xFF, p >> 8])
    sw.txPacket()

cur = [pk.read2ByteTxRx(ph, i, 56)[0] for i in IDS]
for i, p in zip(IDS, cur):
    pk.write2ByteTxRx(ph, i, 42, p); pk.write2ByteTxRx(ph, i, 46, 0); pk.write1ByteTxRx(ph, i, 41, 0)
    pk.write1ByteTxRx(ph, i, 40, 1)

aborted = False
for pose, dur in seq:
    target = clamp(pose); start = cur; t0 = time.time()
    while (t := time.time() - t0) < dur:
        a = 0.5 - 0.5 * math.cos(math.pi * t / dur)  # ease in/out
        send([round(s + (e - s) * a) for s, e in zip(start, target)])
        time.sleep(0.02)
    send(target); cur = target
    loads = [pk.read2ByteTxRx(ph, i, 60)[0] & 0x3FF for i in (2, 3)]
    if max(loads) > 700:
        print(f"strain detected {loads} — stopping and holding"); aborted = True; break

if not aborted:
    time.sleep(0.8)
    for i in IDS:
        pk.write1ByteTxRx(ph, i, 40, 0)
    print("dance done — back at rest, torque OFF")
ph.closePort()
