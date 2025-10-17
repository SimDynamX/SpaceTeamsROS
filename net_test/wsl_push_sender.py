#!/usr/bin/env python3
import argparse
import json
import os
import time
from datetime import datetime, timezone
import zmq
import socket

def detect_windows_host_fallback() -> str:
    # 1) Try host.docker.internal (often works on recent WSL)
    try:
        return socket.gethostbyname("host.docker.internal")
    except Exception:
        pass
    # 2) Fallback to first nameserver in /etc/resolv.conf (Hyper-V gateway)
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if line.startswith("nameserver"):
                    return line.split()[1].strip()
    except Exception:
        pass
    return ""

def wait_for_connected(push_sock, timeout_s=5.0):
    """Wait for EVENT_CONNECTED so we don't drop early messages before TCP is up."""
    mon_addr = "inproc://push_mon"
    push_sock.monitor(mon_addr, zmq.EVENT_CONNECTED)
    mon = push_sock.get_monitor_socket()
    mon.RCVTIMEO = int(timeout_s * 1000)
    ok = False
    try:
        while True:
            try:
                evt = mon.recv_multipart()
            except zmq.Again:
                break
            if not evt:
                continue
            event_id = int.from_bytes(evt[0][:2], "little")
            if event_id == zmq.EVENT_CONNECTED:
                ok = True
                break
    finally:
        mon.close(0)
        push_sock.disable_monitor()
    return ok

def main():
    ap = argparse.ArgumentParser(description="WSL PUSH sender (connects to Windows)")
    ap.add_argument("--host", default="", help="Windows host/IP (auto-detect if empty)")
    ap.add_argument("--port", type=int, default=55555, help="Windows PULL port (default: 55555)")
    ap.add_argument("--count", type=int, default=1000, help="Messages to send")
    ap.add_argument("--rate", type=float, default=15.0, help="Msgs/sec (0 = max)")
    ap.add_argument("--payload", type=int, default=0, help="Payload bytes per message")
    ap.add_argument("--hwm", type=int, default=1000, help="SNDHWM queue depth")
    ap.add_argument("--connect-timeout", type=float, default=5.0, help="Seconds to wait for TCP connect")
    args = ap.parse_args()

    host = args.host or detect_windows_host_fallback()
    if not host:
        print("[WSL PUSH] Could not auto-detect Windows host; pass --host <WIN_IP>")
        return

    endpoint = f"tcp://{host}:{args.port}"

    ctx = zmq.Context.instance()
    push = ctx.socket(zmq.PUSH)
    push.setsockopt(zmq.SNDHWM, args.hwm)
    push.setsockopt(zmq.LINGER, 0)

    print(f"[WSL PUSH] Connecting to {endpoint} ...")
    push.connect(endpoint)

    if not wait_for_connected(push, args.connect_timeout):
        print(f"[WSL PUSH] WARNING: no EVENT_CONNECTED within {args.connect_timeout}s; proceeding anyway.")
    else:
        print("[WSL PUSH] Connected.")

    payload = os.urandom(args.payload) if args.payload > 0 else b""
    interval = 1.0 / args.rate if args.rate > 0 else 0.0
    next_send = time.perf_counter()
    t0 = time.perf_counter()

    for i in range(args.count):
        header = {
            "seq": i,
            "ts": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
            "payload_bytes": len(payload),
        }
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        msg = header_bytes + b"\n" + payload
        push.send(msg, copy=False)

        if interval > 0:
            next_send += interval
            delay = next_send - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

        if (i + 1) % 100 == 0:
            sent = i + 1
            rate = sent / (time.perf_counter() - t0)
            print(f"[WSL PUSH] sent {sent}/{args.count} ~{rate:,.0f} msg/s")

    push.close(0)
    ctx.term()
    print("[WSL PUSH] Done.")

if __name__ == "__main__":
    main()
