"""
SmartGuard TinyML Data Collector (pure Python, no pyserial needed)
Uses Windows COM port file I/O via ctypes.

Usage:
  python collect.py
  python collect.py --port COM3
  python collect.py --max 500

Keys:
  1 = mark next as normal (label=0)
  2 = mark next as abnormal (label=1)
  m = toggle manual/auto mode
  s = print statistics
  q = quit
"""

import os
import sys
import time
import argparse
import msvcrt
import ctypes
import ctypes.wintypes as wintypes

kernel32 = ctypes.windll.kernel32

GENERIC_READ  = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value

class DCB(ctypes.Structure):
    _fields_ = [
        ("DCBlength", wintypes.DWORD),
        ("BaudRate", wintypes.DWORD),
        ("fFlags", wintypes.DWORD),
        ("wReserved", ctypes.c_ushort),
        ("XonLim", ctypes.c_ushort),
        ("XoffLim", ctypes.c_ushort),
        ("ByteSize", ctypes.c_byte),
        ("Parity", ctypes.c_byte),
        ("StopBits", ctypes.c_byte),
        ("XonChar", ctypes.c_char),
        ("XoffChar", ctypes.c_char),
        ("ErrorChar", ctypes.c_char),
        ("EofChar", ctypes.c_char),
        ("EvtChar", ctypes.c_char),
        ("wReserved1", ctypes.c_ushort),
    ]

class COM_PORT:
    def __init__(self, port_name, baudrate=115200):
        self.port_name = port_name
        self.baudrate = baudrate
        self.handle = None

    def open(self):
        port_path = f"\\\\.\\{self.port_name}"
        # SYNCHRONOUS I/O (no OVERLAPPED)
        self.handle = kernel32.CreateFileW(
            port_path,
            GENERIC_READ | GENERIC_WRITE,
            0, None, OPEN_EXISTING, 0, None,
        )
        if self.handle == INVALID_HANDLE_VALUE:
            err = kernel32.GetLastError()
            raise RuntimeError(f"Cannot open {port_path}, err={err}. Close Serial Assistant!")

        # 1. Build DCB
        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        dcb.BaudRate = self.baudrate
        dcb.fFlags = 0x0801  # fBinary=1, fRtsControl=0 (DISABLED), no flow control
        dcb.ByteSize = 8
        dcb.Parity = 0       # NOPARITY
        dcb.StopBits = 0     # ONESTOPBIT

        if not kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
            err = kernel32.GetLastError()
            kernel32.CloseHandle(self.handle)
            raise RuntimeError(f"SetCommState failed, err={err}")

        # 2. Set timeouts (read blocks up to 100ms between chars)
        timeouts = (wintypes.DWORD * 5)()
        timeouts[0] = 1000   # ReadIntervalTimeout
        timeouts[1] = 0      # ReadTotalTimeoutMultiplier
        timeouts[2] = 100    # ReadTotalTimeoutConstant
        timeouts[3] = 0
        timeouts[4] = 0
        kernel32.SetCommTimeouts(self.handle, timeouts)

        # 3. Purge old data
        kernel32.PurgeComm(self.handle, 0x0F)
        return True

    def read(self, bufsize=4096):
        if not self.handle:
            return b""
        buf = ctypes.create_string_buffer(bufsize)
        read = wintypes.DWORD()
        kernel32.ReadFile(self.handle, buf, bufsize, ctypes.byref(read), None)
        return buf.raw[:read.value] if read.value > 0 else b""

    def close(self):
        if self.handle and self.handle != INVALID_HANDLE_VALUE:
            kernel32.CloseHandle(self.handle)
            self.handle = None

    def __del__(self):
        self.close()


# ============================================================
# Main collector
# ============================================================

CSV_HEADER = "label,light,dist,temp,humi,mq2,light_dev,dist_delta,temp_dev,humi_dev,mq2_dev"

total_count = 0
normal_count = 0
abnormal_count = 0
manual_label = -1
stop_flag = False


def banner():
    print()
    print("=" * 46)
    print("  SmartGuard TinyML Data Collector")
    print("=" * 46)
    print("  Keys: 1=mark0 2=mark1 m=toggle s=status q=quit")
    print("=" * 46)
    print()


def ensure_header(output_file):
    if not os.path.exists(output_file) or os.path.getsize(output_file) == 0:
        with open(output_file, "w", encoding="utf-8") as f:
            f.write(CSV_HEADER + "\n")
        print(f"[OK] Created {output_file} with header")
    else:
        with open(output_file, "r", encoding="utf-8") as f:
            lines = f.readlines()
        data_rows = max(0, len(lines) - 1)
        print(f"[OK] Appending to {output_file} ({data_rows} data rows exist)")


def process_line(line, output_file):
    global total_count, normal_count, abnormal_count, manual_label

    if not line.startswith("ML,"):
        return
    csv = line[3:].strip()
    if not csv:
        return

    fields = csv.split(",")
    if len(fields) < 11:
        return

    # Skip header row
    try:
        label = int(fields[0])
    except ValueError:
        return

    if manual_label != -1:
        fields[0] = str(manual_label)
        print(f"  [MANUAL] override label -> {manual_label}")
        manual_label = -1

    out_line = ",".join(fields)

    with open(output_file, "a", encoding="utf-8") as f:
        f.write(out_line + "\n")

    total_count += 1
    if label == 0:
        normal_count += 1
    else:
        abnormal_count += 1

    if total_count % 5 == 0:
        mode = "auto" if manual_label == -1 else f"manual->{manual_label}"
        pct = lambda a: round(a * 100.0 / total_count, 1) if total_count else 0
        print(f"  [{mode}] total={total_count} normal={normal_count}({pct(normal_count)}%) abnormal={abnormal_count}({pct(abnormal_count)}%)")


def main():
    global manual_label, stop_flag

    parser = argparse.ArgumentParser(description="SmartGuard Data Collector")
    parser.add_argument("--port", default="COM8")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--file", default="sensor_data.csv")
    parser.add_argument("--max", type=int, default=0)
    args = parser.parse_args()

    banner()
    ensure_header(args.file)

    port = None
    try:
        port = COM_PORT(args.port, args.baud)
        port.open()
        print(f"[OK] Serial {args.port} @ {args.baud} opened")
    except Exception as e:
        print(f"[ERROR] {e}")
        print("  -> Close Serial Assistant, check port number, replug USB")
        return

    print()
    print("Waiting for ML data... press q to quit, 1/2 to override")
    print()

    buffer = b""
    alive_tick = 0

    try:
        while not stop_flag:
            try:
                data = port.read(4096)
                if data:
                    alive_tick = 0
                    buffer += data
                    # DEBUG: print raw bytes count
                    sys.stdout.write(f"\r  [RX] got {len(data)} bytes  ")
                    sys.stdout.flush()
            except Exception as e:
                sys.stdout.write(f"\r  [ERR] read exception: {e}  ")
                sys.stdout.flush()

            alive_tick += 1
            if alive_tick == 50:  # ~0.5s with 10ms sleep
                sys.stdout.write(f"\r  [WAIT] no data for 0.5s (STM32 sending?)  ")
                sys.stdout.flush()
                alive_tick = 0

            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
                if line:
                    process_line(line, args.file)

            if msvcrt.kbhit():
                ch = msvcrt.getch().decode("utf-8", errors="replace").upper()
                if ch == "1":
                    manual_label = 0
                    print("  [KEY] NEXT -> label=0 (normal)")
                elif ch == "2":
                    manual_label = 1
                    print("  [KEY] NEXT -> label=1 (abnormal)")
                elif ch == "M":
                    if manual_label == -1:
                        manual_label = 0
                        print("  [KEY] mode -> manual (press 1/2 to mark)")
                    else:
                        manual_label = -1
                        print("  [KEY] mode -> auto (follow STM32)")
                elif ch == "S":
                    pct = lambda a: round(a * 100.0 / total_count, 1) if total_count else 0
                    print(f"  [STAT] total={total_count} normal={normal_count} abnormal={abnormal_count}")
                elif ch == "Q":
                    print("Quitting...")
                    stop_flag = True

            if args.max > 0 and total_count >= args.max:
                print(f"Reached target {args.max}, auto-quit")
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print()
        print("Ctrl+C received, saving...")

    finally:
        port.close()
        print()
        print("=" * 46)
        print("  Collection Complete!")
        print("=" * 46)
        print(f"  File:    {args.file}")
        print(f"  Total:   {total_count}")
        print(f"  Normal:  {normal_count}")
        print(f"  Abnormal:{abnormal_count}")
        if total_count:
            apct = round(abnormal_count * 100.0 / total_count, 1)
            print(f"  Abnormal %: {apct}%")
            if abnormal_count < 10:
                print("  !! WARNING: too few abnormal samples! Wave hand near HCSR04")
        print()
        print("Next step:")
        print(f"  python train_sklearn.py --data {args.file}")
        print()


if __name__ == "__main__":
    main()
