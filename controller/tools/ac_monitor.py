"""Monitor raw Assetto Corsa UDP telemetry and diagnose struct alignment.

Dumps every 4-byte field from the packet as both float and int32 so we can
map the correct struct layout.
"""

import socket
import struct
import time
import math
import sys
import os

# Force UTF-8 output on Windows
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

AC_PORT = 9996
HANDSHAKE_FMT = '<iii'

# Expected AC RTCarInfo fields starting at offset 8 (after int32 id + int32 size)
# Based on AC documentation + community implementations + 5 trailing fields (328 bytes total)
AC_FIELDS = [
    # idx  name
    (0,  'speed_Kmh'),
    (1,  'gas'),
    (2,  'brake'),
    (3,  'fuel'),
    (4,  'gear (int)'),
    (5,  'rpms (int)'),
    (6,  'steerAngle'),
    (7,  'slipAngle'),
    (8,  'tc'),
    (9,  'heading'),
    (10, 'pitch'),
    (11, 'roll'),
    (12, 'cgHeight'),
    (13, 'wheelSlip[0]'),    (14, 'wheelSlip[1]'),    (15, 'wheelSlip[2]'),    (16, 'wheelSlip[3]'),
    (17, 'wheelLoad[0]'),    (18, 'wheelLoad[1]'),    (19, 'wheelLoad[2]'),    (20, 'wheelLoad[3]'),
    (21, 'wheelsPressure[0]'), (22, 'wheelsPressure[1]'), (23, 'wheelsPressure[2]'), (24, 'wheelsPressure[3]'),
    (25, 'wheelAngSpeed[0]'), (26, 'wheelAngSpeed[1]'), (27, 'wheelAngSpeed[2]'), (28, 'wheelAngSpeed[3]'),
    (29, 'tyreDirty[0]'),    (30, 'tyreDirty[1]'),    (31, 'tyreDirty[2]'),    (32, 'tyreDirty[3]'),
    (33, 'tyreCoreTI[0]'),   (34, 'tyreCoreTI[1]'),   (35, 'tyreCoreTI[2]'),   (36, 'tyreCoreTI[3]'),
    (37, 'tyreCoreTM[0]'),   (38, 'tyreCoreTM[1]'),   (39, 'tyreCoreTM[2]'),   (40, 'tyreCoreTM[3]'),
    (41, 'tyreCoreTempO[0]'),(42, 'tyreCoreTempO[1]'),(43, 'tyreCoreTempO[2]'),(44, 'tyreCoreTempO[3]'),
    (45, 'suspTravel[0]'),   (46, 'suspTravel[1]'),   (47, 'suspTravel[2]'),   (48, 'suspTravel[3]'),
    (49, 'tyreTempI[0]'),    (50, 'tyreTempI[1]'),    (51, 'tyreTempI[2]'),    (52, 'tyreTempI[3]'),
    (53, 'tyreTempM[0]'),    (54, 'tyreTempM[1]'),    (55, 'tyreTempM[2]'),    (56, 'tyreTempM[3]'),
    (57, 'tyreTempO[0]'),    (58, 'tyreTempO[1]'),    (59, 'tyreTempO[2]'),    (60, 'tyreTempO[3]'),
    (61, 'lastFF'),
    (62, 'performanceMeter'),
    (63, 'abs'),
    (64, 'camberRAD[0]'),    (65, 'camberRAD[1]'),    (66, 'camberRAD[2]'),    (67, 'camberRAD[3]'),
    (68, 'brakeTemp[0]'),    (69, 'brakeTemp[1]'),    (70, 'brakeTemp[2]'),    (71, 'brakeTemp[3]'),
    (72, 'accG_vertical'),
    (73, 'accG_horizontal'),
    (74, 'accG_frontal'),
    (75, 'extra_0'),
    (76, 'extra_1'),
    (77, 'extra_2'),
    (78, 'extra_3'),
    (79, 'extra_4'),
]

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else AC_PORT
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    ac_addr = ('127.0.0.1', port)

    # Handshake
    hs = struct.pack(HANDSHAKE_FMT, 1, 1, 0)
    sock.sendto(hs, ac_addr)
    print(f"Sent handshake to 127.0.0.1:{port}")

    try:
        data, addr = sock.recvfrom(4096)
        print(f"Handshake response: {len(data)} bytes")
    except socket.timeout:
        print("ERROR: No handshake response — is AC running and in a session?")
        sock.close()
        return

    # Subscribe
    hs = struct.pack(HANDSHAKE_FMT, 1, 1, 1)
    sock.sendto(hs, ac_addr)
    print("Subscribed to realtime updates\n")

    count = 0
    last_heading = None
    rate_start = time.time()
    rate_count = 0
    hz = 0.0
    AXIS_MAX = [2.0, 2.5, 1.5, 0.5, 0.3, 2.0]
    AXIS_NAMES = ["Surge(G)", "Sway(G)", "Heave(G)", "Roll(rad)", "Pitch(rad)", "Yaw(r/s)"]

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue

            count += 1
            rate_count += 1
            now = time.time()
            elapsed = now - rate_start
            if elapsed >= 1.0:
                hz = rate_count / elapsed
                rate_start = now
                rate_count = 0

            # First 3 packets: full field dump
            if count <= 3:
                print(f"\n{'='*80}")
                print(f"Packet #{count}: {len(data)} bytes")
                
                # Header
                if len(data) >= 8:
                    id_raw = struct.unpack_from('<I', data, 0)[0]
                    sz_raw = struct.unpack_from('<i', data, 4)[0]
                    print(f"  Header: id=0x{id_raw:08X} size={sz_raw}")
                    print(f"  Header bytes: {data[:8].hex()}")
                
                # Dump all 4-byte fields from offset 8
                n_fields = (len(data) - 8) // 4
                print(f"  Payload: {n_fields} fields ({len(data)-8} bytes)")
                print(f"  {'idx':>3s}  {'off':>3s}  {'hex':>10s}  {'float':>12s}  {'int32':>12s}  expected_name")
                print(f"  {'-'*3}  {'-'*3}  {'-'*10}  {'-'*12}  {'-'*12}  {'-'*20}")
                for i in range(n_fields):
                    off = 8 + i * 4
                    raw = data[off:off+4]
                    f_val = struct.unpack_from('<f', data, off)[0]
                    i_val = struct.unpack_from('<i', data, off)[0]
                    
                    # Find expected field name
                    fname = '???'
                    for fi, fn in AC_FIELDS:
                        if fi == i:
                            fname = fn
                            break
                    
                    # Mark suspicious values
                    flag = ''
                    if '(int)' in fname:
                        flag = f'  <-- {i_val}'
                    elif abs(f_val) > 1e10 or (abs(f_val) < 1e-10 and f_val != 0):
                        flag = '  <-- SUSPECT'
                    elif 'accG' in fname:
                        flag = f'  <-- ***'
                    elif fname in ('speed_Kmh', 'heading', 'pitch', 'roll'):
                        flag = f'  <-- KEY'
                    
                    print(f"  {i:3d}  {off:3d}  {raw.hex():>10s}  {f_val:12.4f}  {i_val:12d}  {fname}{flag}")
                print()

            # Exit after initial dump so we can see the field mapping
            elif count == 4:
                break

            # After dump, show motion data using the expected layout
            elif count % 50 == 0:
                if len(data) >= 328:
                    # Parse with corrected layout: offset 8 + field_idx * 4
                    def fld(idx): return struct.unpack_from('<f', data, 8 + idx*4)[0]
                    def ild(idx): return struct.unpack_from('<i', data, 8 + idx*4)[0]
                    
                    speed = fld(0)
                    gear = ild(4)
                    rpms = ild(5)
                    heading = fld(9)
                    pitch = fld(10)
                    roll = fld(11)
                    accG_v = fld(72)
                    accG_h = fld(73)
                    accG_f = fld(74)
                    
                    surge = accG_f
                    sway = accG_h
                    heave = accG_v - 1.0
                    yaw_rate = 0.0
                    if last_heading is not None:
                        dh = heading - last_heading
                        if dh > math.pi: dh -= 2*math.pi
                        if dh < -math.pi: dh += 2*math.pi
                        dt = 1.0/max(hz, 10)
                        yaw_rate = dh / dt
                    last_heading = heading
                    
                    raw6 = [surge, sway, heave, roll, pitch, yaw_rate]
                    pct6 = [max(-100, min(100, (r/m)*100)) for r, m in zip(raw6, AXIS_MAX)]
                    print(f"  #{count} | {hz:.0f}Hz | spd={speed:.0f} rpm={rpms} g={gear} | accG: v={accG_v:.3f} h={accG_h:.3f} f={accG_f:.3f}")
                    for i in range(6):
                        bar = '#' * int(abs(pct6[i])/5)
                        s = '+' if pct6[i] >= 0 else '-'
                        print(f"    {AXIS_NAMES[i]:>12s}: {raw6[i]:+8.4f}  {pct6[i]:+6.1f}% {s}{bar}")
                else:
                    last_heading = None

    except KeyboardInterrupt:
        print(f"\nTotal: {count} packets")

    hs = struct.pack(HANDSHAKE_FMT, 1, 1, 2)
    sock.sendto(hs, ac_addr)
    sock.close()
    print("Disconnected.")

if __name__ == '__main__':
    main()
