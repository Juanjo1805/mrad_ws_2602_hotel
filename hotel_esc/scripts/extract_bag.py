#!/usr/bin/env python3
"""
extract_bag.py  --  Read a rosbag and export synchronized data to CSV.

Usage:
    python3 extract_bag.py <path_to_bag_folder>
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

BAG_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
TYPESTORE = get_typestore(Stores.ROS2_JAZZY)


def read_topic(reader, typestore, topic_name, fields_fn):
    """Extract (timestamp_s, *fields) from a topic using fields_fn(msg) -> tuple."""
    rows = []
    connections = [c for c in reader.connections if c.topic == topic_name]
    if not connections:
        print(f"  WARNING: topic '{topic_name}' not found in bag")
        return pd.DataFrame()
    for conn, timestamp, rawdata in reader.messages(connections=connections):
        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
        t = timestamp * 1e-9  # nanoseconds → seconds
        rows.append((t,) + fields_fn(msg))
    return pd.DataFrame(rows)


def extract_pose(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    # Convert quaternion to yaw (2D assumption)
    yaw = np.arctan2(
        2.0 * (o.w * o.z + o.x * o.y),
        1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    )
    return (p.x, p.y, p.z, yaw)


def extract_esc_command(msg):
    return (float(msg.data),)


def extract_joy(msg):
    # axes[1] = left stick vertical (throttle), axes[3] = right stick horizontal (steering)
    throttle = float(msg.axes[1]) if len(msg.axes) > 1 else 0.0
    steering  = float(msg.axes[3]) if len(msg.axes) > 3 else 0.0
    return (throttle, steering)


with Reader(BAG_PATH) as reader:
    print(f"Reading bag:{BAG_PATH}")

    df_pose = read_topic(reader, TYPESTORE, "/vicon/zulu/pose", extract_pose)
    df_cmd  = read_topic(reader, TYPESTORE, "/esc/command",     extract_esc_command)
    df_joy  = read_topic(reader, TYPESTORE, "/joy",             extract_joy)

df_pose.columns = ["t", "x", "y", "z", "yaw"]
df_cmd.columns  = ["t", "u"]
df_joy.columns  = ["t", "joy_throttle", "joy_steering"]

# Sort by time
for df in [df_pose, df_cmd, df_joy]:
    df.sort_values("t", inplace=True)
    df.reset_index(drop=True, inplace=True)

# Save raw
df_pose.to_csv("pose_raw.csv", index=False)
df_cmd.to_csv("esc_command_raw.csv", index=False)
df_joy.to_csv("joy_raw.csv", index=False)

# Align time to start at 0
t0 = min(df_pose["t"].iloc[0], df_cmd["t"].iloc[0])
for df in [df_pose, df_cmd, df_joy]:
    df["t"] -= t0

print(f"  VICON  :{len(df_pose)} samples, dt={df_pose['t'].diff().median()*1000:.1f} ms")
print(f"  ESC cmd:{len(df_cmd)}  samples, dt={df_cmd['t'].diff().median()*1000:.1f} ms")
print(f"  Joy    :{len(df_joy)}  samples, dt={df_joy['t'].diff().median()*1000:.1f} ms")
print("Saved: pose_raw.csv, esc_command_raw.csv, joy_raw.csv")