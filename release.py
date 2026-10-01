"""Turns follower torque OFF (arm goes limp — hold it first)."""
import scservo_sdk as scs
ph = scs.PortHandler("/dev/tty.usbmodem5B3E0892191"); pk = scs.PacketHandler(0)
ph.openPort(); ph.setBaudRate(1_000_000)
for i in range(1, 7):
    pk.write1ByteTxRx(ph, i, 40, 0)
print("torque OFF on all follower motors")
ph.closePort()
