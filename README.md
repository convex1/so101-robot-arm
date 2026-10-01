# SO-101 robot arm experiments

Small Python scripts for a Hugging Face **SO-101** 6-DoF arm (Feetech STS3215 servos), built on
[LeRobot](https://github.com/huggingface/lerobot)'s calibration and the Feetech servo SDK.

What's here:

| Script | What it does |
| --- | --- |
| `record.py` | **Teach by hand.** Turns torque off and records joint positions at 30 Hz while you move the arm. |
| `replay.py` | Replays a recording: glides to the start pose, trims idle time, streams the motion at the original timing, reports how far each joint lagged. |
| `gesture.py` | Named, smoothly interpolated gestures (e.g. `up_grip`), clamped to calibrated joint ranges. |
| `dance.py` | A short scripted dance with ease-in/out motion. |
| `thumbs_grip.py` | **Vision control.** Watches a webcam with MediaPipe's gesture recognizer; each thumbs-up makes the arm grip three times. |
| `film.py` | Records the camera while running any of the scripts above, then re-encodes to H.264. |
| `preview.py` | Live camera preview for aiming the webcam. |
| `release.py` | Torque off on every motor (hold the arm first). |

`push6.json` is a working demo recording: the arm pushes a tomato into a hole in the desk
(`python replay.py push6 1.0 --rest`).

## Safety

Every scripted move is clamped to the joint ranges from LeRobot calibration, eased in and out, and
watched for strain: if the shoulder or elbow load goes over a threshold the script stops and holds.
Scripts end at the rest pose with torque off.

## Setup

1. Assemble and calibrate the follower arm with LeRobot (`lerobot-calibrate`). The scripts read
   `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json`.
2. Native arm64 Python 3.12 (on Apple silicon, use `uv` or Homebrew Python, not an x86 Anaconda):
   ```
   uv venv --python 3.12 && source .venv/bin/activate
   uv pip install -r requirements.txt
   ```
3. Set `PORT` at the top of each script to your servo board's serial port
   (`ls /dev/tty.usbmodem*`).
4. For `thumbs_grip.py`, download the gesture model into `models/`:
   ```
   mkdir -p models && curl -L -o models/gesture_recognizer.task \
     https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task
   ```
5. `REST` in the scripts is the rest pose of my arm, in raw servo ticks. Record your own with
   `record.py` and update it.

## Next

A Claude vision-language planner that looks at an overhead camera and calls pick / place skills
("code as policies"), then a SmolVLA policy trained on the same task for comparison.
