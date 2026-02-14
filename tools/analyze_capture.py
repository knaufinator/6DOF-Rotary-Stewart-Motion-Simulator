#!/usr/bin/env python3
"""Analyze a .stwr recording file for jitter/jumps."""
import struct, sys, os, statistics

path = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\Chris\Documents\GitHub\6DOF-Rotary-Stewart-Motion-Simulator\app\recordings\002.stwr"

if not os.path.exists(path):
    print(f"File not found: {path}")
    sys.exit(1)

with open(path, "rb") as f:
    data = f.read()

HDR_FMT = "<II dd ii d 64s 32s 128s"
HDR_SIZE = struct.calcsize(HDR_FMT)
hdr = struct.unpack(HDR_FMT, data[:HDR_SIZE])
magic, version, rate_hz, duration, sample_count, channels, created, name_b, source_b, _ = hdr
name = name_b.split(b'\x00')[0].decode('utf-8', errors='replace')
source = source_b.split(b'\x00')[0].decode('utf-8', errors='replace')

print(f"File: {path}")
print(f"Name: {name}, Source: {source}")
print(f"Version: {version}, Rate: {rate_hz} Hz, Duration: {duration:.2f}s, Samples: {sample_count}, Channels: {channels}")
print()

SAMPLE_FMT = "<d 6f"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
offset = HDR_SIZE

samples = []
for i in range(sample_count):
    s = struct.unpack(SAMPLE_FMT, data[offset:offset+SAMPLE_SIZE])
    samples.append(s)
    offset += SAMPLE_SIZE

print(f"Parsed {len(samples)} samples")

if len(samples) < 2:
    print("Not enough samples")
    sys.exit(0)

axis_names = ["Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"]
print(f"\n{'Axis':>8} {'Min':>8} {'Max':>8} {'Mean':>8} {'StdDev':>8} {'Range':>8} {'MaxDelta':>10} {'Jumps>5%':>10}")
print("-" * 85)

all_jumps = []
for ax in range(6):
    vals = [s[1+ax] for s in samples]
    deltas = [abs(vals[i] - vals[i-1]) for i in range(1, len(vals))]
    mn, mx = min(vals), max(vals)
    mean = statistics.mean(vals)
    sd = statistics.stdev(vals) if len(vals) > 1 else 0
    max_d = max(deltas) if deltas else 0
    jumps_5pct = sum(1 for d in deltas if d > 5.0)

    print(f"{axis_names[ax]:>8} {mn:8.2f} {mx:8.2f} {mean:8.2f} {sd:8.3f} {mx-mn:8.2f} {max_d:10.2f} {jumps_5pct:10d}")

    for i in range(1, len(vals)):
        delta = abs(vals[i] - vals[i-1])
        if delta > 1.0:
            all_jumps.append((delta, ax, samples[i][0], vals[i-1], vals[i], i))

all_jumps.sort(reverse=True)
print(f"\nTop 20 largest frame-to-frame jumps (>1%):")
print(f"{'Delta':>8} {'Axis':>8} {'Time(s)':>10} {'Sample#':>8} {'From':>10} {'To':>10}")
print("-" * 60)
for delta, ax, t, old, new, idx in all_jumps[:20]:
    print(f"{delta:8.2f} {axis_names[ax]:>8} {t:10.4f} {idx:8d} {old:10.2f} {new:10.2f}")
