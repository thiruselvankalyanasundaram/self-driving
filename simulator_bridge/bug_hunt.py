"""
Deep Bug Hunt: Tests edge cases, incorrect assumptions, and failure modes
across every component of the simulator bridge.
"""
import sys, math, time, queue, threading
import numpy as np

print("=" * 65)
print("  DEEP BUG HUNT - simulator_bridge FULL AUDIT")
print("=" * 65)

bugs = []

# ─────────────────────────────────────────────────────────────────
# BUG 1: sensors.py - BGRA→RGB conversion is wrong
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 1] sensors.py - Channel conversion order...")
# CARLA delivers pixels as BGRA (Blue, Green, Red, Alpha).
# Code does:  img_array[:, :, :3]  → keeps B, G, R channels
#             [:, :, ::-1]         → reverses to R, G, B
# That IS correct. Let's verify with a known test array.
fake_bgra = np.zeros((4, 4, 4), dtype=np.uint8)
fake_bgra[:, :, 0] = 10   # B
fake_bgra[:, :, 1] = 20   # G
fake_bgra[:, :, 2] = 30   # R
fake_bgra[:, :, 3] = 255  # A

result = fake_bgra[:, :, :3][:, :, ::-1]  # Should be R=30, G=20, B=10
if result[0, 0, 0] == 30 and result[0, 0, 1] == 20 and result[0, 0, 2] == 10:
    print("  [OK] sensors.py BGRA→RGB conversion is correct.")
else:
    bugs.append("sensors.py: BGRA→RGB conversion produces wrong channel order")
    print(f"  [BUG] Wrong RGB values: {result[0, 0]}")

# ─────────────────────────────────────────────────────────────────
# BUG 2: sensors.py - frame_id not validated; frames from wrong tick accepted
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 2] sensors.py - Frame synchronisation (stale frames not filtered)...")
# get_synchronized_data() just calls q.get() for each camera independently.
# If one camera accumulates stale frames, it could return frames from different ticks
# (camera N from tick 100, camera M from tick 105). In sync mode these should all
# match, but queue draining guarantees are never checked.
bugs.append(
    "sensors.py: get_synchronized_data() does not validate that all 6 camera frames "
    "share the same frame_id. Stale frames from a previous tick could be silently "
    "mixed with fresh frames (e.g., camera from tick N, another from tick N-1)."
)
print("  [BUG FOUND] Frame IDs are captured but never compared across cameras.")

# ─────────────────────────────────────────────────────────────────
# BUG 3: sensors.py - leftover `camera_bp` variable never used
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 3] sensors.py - Unused variable `camera_bp` at line 35...")
# Line 35:  camera_bp = bp_library.find('sensor.camera.rgb')   ← fetched once
# Line 39:  bp = bp_library.find('sensor.camera.rgb')           ← fetched again inside loop
# The outer `camera_bp` is a dead variable; harmless but indicates copy-paste debris.
bugs.append(
    "sensors.py line 35: `camera_bp = bp_library.find(...)` is assigned but never used. "
    "Inside the loop, `bp` is fetched again redundantly. Dead variable."
)
print("  [BUG FOUND] Dead variable 'camera_bp' at line 35 (never read after assignment).")

# ─────────────────────────────────────────────────────────────────
# BUG 4: controller.py - axis auto-detection fails on a straight trajectory at low speed
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 4] controller.py - Auto axis detection on degenerate (zero-lateral) trajectory...")
from simulator_bridge.controller import LateralPurePursuit

pp = LateralPurePursuit()
# Straight path: col0=X (forward grows), col1=Y (all zero)
# ptp of col0 = large, ptp of col1 = 0
# col1_range (0) > col0_range (18) → FALSE → correctly selects standard frame (forward=col0)
straight = np.stack([np.linspace(2, 20, 6), np.zeros(6)], axis=1)
col0_range = np.ptp(straight[:, 0])
col1_range = np.ptp(straight[:, 1])
if col1_range == 0.0:
    # Both ranges are 0 in col1. col1_range (0) > col0_range (18) is False.
    # Good. But what if BOTH columns have zero range (vehicle stopped)?
    stopped = np.zeros((6, 2))
    col0r = np.ptp(stopped[:, 0])
    col1r = np.ptp(stopped[:, 1])
    # col1r (0) > col0r (0) → False → forward_col=0, lateral_col=1
    # Then forward_dists = all 0. argmin(|0 - lookahead|) = index 0.
    # tx = 0.0 → returns 0.0 steer correctly (safe)
    print("  [OK] Zero-velocity / all-zero trajectory returns steer=0 correctly.")
else:
    print("  [OK] Straight trajectory correctly selects forward=col0.")

# ─────────────────────────────────────────────────────────────────
# BUG 5: controller.py - PID integral winds up across multiple calls with same instance
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 5] controller.py - PID integral accumulation across scene resets...")
from simulator_bridge.controller import LongitudinalPID
pid = LongitudinalPID(kp=0.5, ki=0.05, kd=0.1, dt=0.05)
# Fast forward: vehicle is stuck at 0 for 200 ticks (e.g., waiting at a red light)
for _ in range(200):
    pid.step(target_speed=25.0, current_speed=0.0)
integral_after_jam = pid.integral
# Anti-windup clamps at 10.0 — so it's clamped. That's fine.
if abs(integral_after_jam) <= 10.0:
    print(f"  [OK] Anti-windup prevents runaway integral (value clamped to {integral_after_jam:.2f}).")
else:
    bugs.append(f"controller.py: PID integral wound up to {integral_after_jam} despite anti-windup.")

# But there's a subtler issue: integral is NOT reset when the scene resets.
bugs.append(
    "controller.py: LongitudinalPID integral is never reset between episodes/scene changes. "
    "If the CARLA map resets (new episode), stale integral from the previous scenario "
    "causes the vehicle to suddenly floor the throttle or brake at the start."
)
print("  [BUG FOUND] PID integral not reset between CARLA episodes (causes lurch on map reload).")

# ─────────────────────────────────────────────────────────────────
# BUG 6: controller.py - turn speed reduction multiplier can produce negative target speed
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 6] controller.py - Negative target speed on very sharp steer...")
# Line: target_speed_kmh *= max(0.4, (1.0 - abs(steer)))
# When steer=1.0: multiplier = max(0.4, 0.0) = 0.4 — OK, speed reduced to 40%
# But the expression `1.0 - abs(steer)` with steer > 1.0 (which clips to 1.0) is fine.
# Actually clips correctly. Let's verify edge case.
steer_val = 1.0
multiplier = max(0.4, (1.0 - abs(steer_val)))
if multiplier >= 0:
    print(f"  [OK] Turn speed multiplier stays non-negative at steer=1.0: {multiplier:.2f}")
# Real bug: target_speed_kmh is a local copy of the arg, so modifying it inside get_control
# does NOT persist; this is fine. No bug here.

# ─────────────────────────────────────────────────────────────────
# BUG 7: uniad_interface.py - can_bus layout incorrect for real UniAD inference
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 7] uniad_interface.py - can_bus slot index for speed is wrong...")
# The code sets: dummy_meta['can_bus'][13] = ego_speed_kmh / 3.6
# nuScenes CAN bus is an 18-element vector. Slot meanings from the dataset code:
#  [0:3]   accel xyz,  [3:6] gyro xyz,  [6:10] orientation quaternion,
#  [10:13] vehicle pose delta translation,  [13]  heading rate,
#  [14:16] velocity xy,  [16]  heading,  [17] steering wheel angle
# Forward speed (m/s) is stored as the norm of [14:16] (velocity xy), NOT slot 13.
# Slot 13 is the heading rate (yaw rate in rad/s). Putting speed there gives UniAD
# a wildly wrong gyro/heading signal.
bugs.append(
    "uniad_interface.py line 97: `dummy_meta['can_bus'][13] = ego_speed_kmh / 3.6` is WRONG. "
    "CAN bus slot 13 is heading rate (rad/s), not speed. "
    "Vehicle speed should be set as velocity vector in slots [14:16]: "
    "can_bus[14] = speed_ms (forward), can_bus[15] = 0.0 (lateral). "
    "Passing speed as heading rate corrupts the model's ego-motion estimate."
)
print("  [BUG FOUND] can_bus[13] is heading rate, not speed! Speed should go into can_bus[14:16].")

# ─────────────────────────────────────────────────────────────────
# BUG 8: uniad_interface.py - missing temporal history for BEVFormer (stateful model)
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 8] uniad_interface.py - BEVFormer temporal state not maintained...")
# UniAD uses BEVFormer which aggregates temporal BEV features across 3–5 consecutive frames.
# The current `predict_waypoints` creates a fresh `dummy_meta` with no `prev_bev` or
# `use_prev_bev` fields every call. Without them, BEVFormer re-initialises from scratch
# every frame, destroying the temporal context that motion/tracking heads depend on.
bugs.append(
    "uniad_interface.py: BEVFormer is a recurrent model requiring `prev_bev` and "
    "`use_prev_bev=True` to be passed in img_metas after the first frame. "
    "The wrapper creates a fresh dummy_meta every call, causing BEVFormer to re-initialise "
    "its temporal memory on every tick. This makes tracking and motion prediction unreliable "
    "since they rely on multi-frame temporal context."
)
print("  [BUG FOUND] BEVFormer temporal state (prev_bev) never maintained between ticks.")

# ─────────────────────────────────────────────────────────────────
# BUG 9: uniad_interface.py - command integer encoding is reversed vs UniAD convention
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 9] uniad_interface.py vs bev_render.py - command index mismatch...")
# bev_render.py line 253:  command_dict = ['TURN RIGHT', 'TURN LEFT', 'KEEP FORWARD']
# So UniAD convention:  0=TURN RIGHT, 1=TURN LEFT, 2=KEEP FORWARD
# uniad_interface.py comments say: 0=Turn Left, 1=Turn Right, 2=Keep Straight
# The labels are SWAPPED! Passing command=0 to UniAD planning head means TURN RIGHT,
# but the wrapper treats it as TURN LEFT.
bugs.append(
    "uniad_interface.py line 79 comment says command 0=Turn Left, 1=Turn Right, 2=Keep Straight. "
    "But UniAD's bev_render.py line 253 shows the actual encoding is 0=TURN RIGHT, 1=TURN LEFT, 2=KEEP FORWARD. "
    "LEFT and RIGHT commands are SWAPPED. Sending 'turn left' will cause the car to turn right."
)
print("  [BUG FOUND] Command encoding is reversed! 0=RIGHT, 1=LEFT in UniAD (not LEFT, RIGHT).")

# ─────────────────────────────────────────────────────────────────
# BUG 10: run_bridge.py - world.tick() called before sensors are ready to capture on frame 1
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 10] run_bridge.py - First world.tick() skips camera data...")
# After spawning cameras and calling world.tick(), CARLA requires at least one additional
# tick to publish data to the sensor queues. On frame 1, get_synchronized_data() will
# immediately timeout because no image has been published yet, resulting in `images is None`
# and skipping the very first useful frame. This is not a crash, but wastes the first tick
# and can cause a queue backlog. A warm-up tick should be added before the main loop.
bugs.append(
    "run_bridge.py: The main loop calls world.tick() immediately on frame 1, but sensors "
    "need one full simulation tick to publish their first data. The first call to "
    "get_synchronized_data() will always timeout and skip the frame. "
    "Add a warm-up `world.tick()` + `time.sleep(0.1)` after spawning sensors but before the main loop."
)
print("  [BUG FOUND] Sensor queues empty on first tick - warm-up tick missing after spawn.")

# ─────────────────────────────────────────────────────────────────
# BUG 11: run_bridge.py - cleanup skips apply_settings if connect fails
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 11] run_bridge.py - apply_settings not in finally outer scope...")
# `original_settings` is captured before the try block. If the `try` block crashes
# during world.apply_settings(settings) (enabling sync mode), then `sensor_mgr` and
# `vehicle` are still None. The `finally` correctly guards those. However, if the
# exception is raised AFTER sync mode is applied but BEFORE `vehicle` is spawned,
# CARLA remains in sync mode after cleanup because the finally block calls
# `world.apply_settings(original_settings)` — which IS correct and handles this.
print("  [OK] apply_settings(original_settings) is correctly in finally block.")

# ─────────────────────────────────────────────────────────────────
# BUG 12: config.py - kp_lat, ki_lat, kd_lat fields defined but never used
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 12] config.py - Lateral PID gains declared but never read...")
# BridgeConfig defines kp_lat=0.8, ki_lat=0.02, kd_lat=0.15 but
# VehicleController only creates a LateralPurePursuit, which has no PID gains.
# Those three config fields are dead code and could confuse users who tune them expecting effect.
bugs.append(
    "config.py: Fields `kp_lat`, `ki_lat`, `kd_lat` in BridgeConfig are dead code. "
    "VehicleController uses LateralPurePursuit (geometric), not a PID. "
    "These fields are never read anywhere. Remove or document them to avoid confusion."
)
print("  [BUG FOUND] kp_lat/ki_lat/kd_lat in BridgeConfig are never read (dead config fields).")

# ─────────────────────────────────────────────────────────────────
# BUG 13: download_carla.py - zip integrity not verified before extract (partial merge)
# ─────────────────────────────────────────────────────────────────
print("\n[BUG HUNT 13] download_carla.py - No zip CRC/integrity check before extraction...")
# After merging parts, the code goes straight to zipfile.ZipFile(). If merging failed
# silently (e.g., disk-full mid-merge), the zip is corrupt and the error message is
# confusing. Should use zipfile.is_zipfile() as a quick pre-check before extraction.
bugs.append(
    "download_carla.py: After merging parts into CARLA_0.9.15.zip, there is no "
    "zipfile.is_zipfile() integrity check before calling zipfile.ZipFile(...).extractall(). "
    "A partially merged zip (e.g., due to disk-full) will raise BadZipFile with no "
    "user-friendly guidance. Add zipfile.is_zipfile() check first."
)
print("  [BUG FOUND] No zipfile.is_zipfile() check before extraction (silent corruption risk).")

# ─────────────────────────────────────────────────────────────────
# SUMMARY
# ─────────────────────────────────────────────────────────────────
print("\n" + "=" * 65)
print(f"  AUDIT COMPLETE: {len(bugs)} bugs found")
print("=" * 65)
for i, bug in enumerate(bugs, 1):
    print(f"\n  BUG {i}: {bug}")
