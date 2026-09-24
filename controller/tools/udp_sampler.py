#!/usr/bin/env python3
"""
UDP Packet Sampler — captures raw SimTools/AC packets and analyzes for jitter.
Usage: python udp_sampler.py [port] [seconds]
  Default: port=4123, seconds=5
"""
import socket, struct, sys, time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 4123
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('0.0.0.0', port))
sock.settimeout(0.1)

print(f"Listening on UDP port {port} for {duration}s...")
print(f"{'time_ms':>10} {'len':>4} {'v0':>6} {'v1':>6} {'v2':>6} {'v3':>6} {'v4':>6} {'v5':>6}  {'format':<8} {'jumps'}")
print("-" * 100)

samples = []
start = time.perf_counter()
prev_values = None

while time.perf_counter() - start < duration:
    try:
        data, addr = sock.recvfrom(512)
    except socket.timeout:
        continue

    t_ms = (time.perf_counter() - start) * 1000.0
    n = len(data)
    values = None
    fmt = "?"

    # Try 12-byte binary (6 x uint16 LE)
    if n == 12:
        values = list(struct.unpack('<6H', data))
        fmt = "bin12"
    # Try 15-byte framed binary (0xAA 0x55 + 6 x uint16 LE + XOR)
    elif n == 15 and data[0] == 0xAA and data[1] == 0x55:
        values = list(struct.unpack('<6H', data[2:14]))
        fmt = "bin15"
    # Try 6-byte (8-bit mode)
    elif n == 6:
        values = list(data)
        fmt = "bin6"
    else:
        # Try CSV text
        try:
            txt = data.decode('ascii', errors='ignore').strip().rstrip('X').rstrip('\r\n')
            parts = txt.split(',')
            if len(parts) >= 6:
                values = [float(x) for x in parts[:6]]
                fmt = f"csv{len(parts)}"
        except:
            fmt = f"raw{n}"

    if values is None:
        print(f"{t_ms:10.1f} {n:4d} {'[unparsed]':>40}  {fmt:<8} raw={data[:20].hex()}")
        continue

    # Detect jumps (>5% of full range change between consecutive packets)
    jumps = ""
    if prev_values is not None:
        jump_axes = []
        for i in range(min(len(values), len(prev_values))):
            delta = abs(values[i] - prev_values[i])
            # For 12-bit: range is 4095, 5% = ~205
            # For CSV %: range is 200, 5% = 10
            threshold = 205 if fmt.startswith("bin") else 10
            if delta > threshold:
                jump_axes.append(f"ax{i}:{prev_values[i]:.0f}->{values[i]:.0f}(d={delta:.0f})")
        if jump_axes:
            jumps = " JUMP! " + ", ".join(jump_axes)

    vals_str = " ".join(f"{v:6.0f}" for v in values[:6])
    print(f"{t_ms:10.1f} {n:4d} {vals_str}  {fmt:<8}{jumps}")

    samples.append((t_ms, values[:6], fmt))
    prev_values = values

sock.close()

# Summary
print(f"\n{'='*60}")
print(f"Captured {len(samples)} packets in {duration}s ({len(samples)/duration:.1f} pkt/s)")

if len(samples) < 2:
    print("Not enough samples for analysis.")
    sys.exit(0)

# Per-axis statistics
import statistics
print(f"\n{'Axis':>6} {'Min':>8} {'Max':>8} {'Mean':>8} {'StdDev':>8} {'Range':>8} {'MaxDelta':>8}")
print("-" * 60)
for ax in range(6):
    vals = [s[1][ax] for s in samples]
    deltas = [abs(vals[i] - vals[i-1]) for i in range(1, len(vals))]
    mn, mx = min(vals), max(vals)
    mean = statistics.mean(vals)
    sd = statistics.stdev(vals) if len(vals) > 1 else 0
    max_d = max(deltas) if deltas else 0
    print(f"  ax{ax}: {mn:8.1f} {mx:8.1f} {mean:8.1f} {sd:8.2f} {mx-mn:8.1f} {max_d:8.1f}")

# Timing analysis
times = [s[0] for s in samples]
intervals = [times[i] - times[i-1] for i in range(1, len(times))]
if intervals:
    print(f"\nPacket intervals (ms): min={min(intervals):.1f} max={max(intervals):.1f} "
          f"mean={statistics.mean(intervals):.1f} stdev={statistics.stdev(intervals):.2f}" if len(intervals) > 1 else "")

# Show largest jumps
print(f"\nLargest frame-to-frame jumps (top 10):")
all_jumps = []
for i in range(1, len(samples)):
    for ax in range(6):
        delta = abs(samples[i][1][ax] - samples[i-1][1][ax])
        all_jumps.append((delta, ax, samples[i][0], samples[i-1][1][ax], samples[i][1][ax]))
all_jumps.sort(reverse=True)
for delta, ax, t, old, new in all_jumps[:10]:
    print(f"  t={t:8.1f}ms ax{ax}: {old:.0f} -> {new:.0f} (delta={delta:.0f})")
