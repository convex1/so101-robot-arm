"""Replays a recording on the follower. Moves slowly to the start pose first.
Torque stays ON at the end (holding). Usage: python replay.py <name> [speed_factor]"""
import sys, time, json, scservo_sdk as scs

PORT = "/dev/tty.usbmodem5B3E0892191"
name = sys.argv[1]
speed = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 1.0
rec = json.load(open(f"{name}.json")); ids = rec["ids"]; frames = rec["frames"]

# skip the still period before the demonstrator starts moving
start = next((k for k, f in enumerate(frames)
              if max(abs(a - b) for a, b in zip(f["pos"], frames[0]["pos"])) > 20), 0)
start = max(0, start - 15)  # keep half a second of lead-in
t_shift = frames[start]["t"]
frames = [{"t": f["t"] - t_shift, "pos": f["pos"]} for f in frames[start:]]
# trim trailing stillness after the demonstrator finished
end = len(frames) - 1
while end > 0 and max(abs(a - b) for a, b in zip(frames[end]["pos"], frames[-1]["pos"])) <= 20:
    end -= 1
frames = frames[:min(len(frames), end + 16)]
REST = [2037, 3052, 835, 1033, 1953, 1809]
TO_REST = "--rest" in sys.argv
UNTIL = next((float(a.split("=")[1]) for a in sys.argv if a.startswith("--until=")), None)  # recording time
if UNTIL is not None:
    frames = [f for f in frames if f["t"] + t_shift <= UNTIL]
    TO_REST = False

ph = scs.PortHandler(PORT); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)
sw = scs.GroupSyncWrite(ph, pk, 42, 2)  # Goal_Position

def send(pos):
    sw.clearParam()
    for i, p in zip(ids, pos):
        sw.addParam(i, [p & 0xFF, p >> 8])
    sw.txPacket()

# 1) glide to the first frame slowly, starting from the current pose (no jump on torque-on)
for i in ids:
    cur = pk.read2ByteTxRx(ph, i, 56)[0]
    pk.write2ByteTxRx(ph, i, 42, cur)
    pk.write1ByteTxRx(ph, i, 41, 20)     # gentle acceleration
    pk.write2ByteTxRx(ph, i, 46, 400)    # slow speed for the approach
    pk.write1ByteTxRx(ph, i, 40, 1)      # torque on
send(frames[0]["pos"]); print("moving to start pose..."); time.sleep(3)

# 2) stream the recording at its original timing
for i in ids:
    pk.write2ByteTxRx(ph, i, 46, 0)      # 0 = no speed cap; timing comes from the stream
    pk.write1ByteTxRx(ph, i, 41, 50)
t0 = time.time(); max_err = [0] * len(ids)
for k, f in enumerate(frames):
    while time.time() - t0 < f["t"] / speed:
        time.sleep(0.002)
    send(f["pos"])
    if k % 10 == 5:  # sample tracking error against the previous frame's target
        prev = frames[k - 1]["pos"]
        for j, i in enumerate(ids):
            max_err[j] = max(max_err[j], abs(pk.read2ByteTxRx(ph, i, 56)[0] - prev[j]))
print("max lag behind recording (ticks):", dict(zip(["base","shoulder","elbow","wflex","wroll","grip"], max_err)))
time.sleep(1)
if TO_REST:
    for i in ids:
        pk.write2ByteTxRx(ph, i, 46, 400); pk.write1ByteTxRx(ph, i, 41, 20)
    send(REST); time.sleep(3)
    for i in ids:
        pk.write1ByteTxRx(ph, i, 40, 0)
    print("replay done; back at rest, torque OFF")
else:
    time.sleep(1.5)
    tgt = frames[-1]["pos"]
    act = [pk.read2ByteTxRx(ph, i, 56)[0] for i in ids]
    print("frozen. target:", tgt)
    print("        actual:", act)
    print("        error :", dict(zip(["base","shoulder","elbow","wflex","wroll","grip"], [a - t for a, t in zip(act, tgt)])))
    print("torque ON (holding)")
ph.closePort()
