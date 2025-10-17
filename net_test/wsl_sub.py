#!/usr/bin/env python3
import argparse
import json
import os
import time
import zmq

def detect_windows_host_fallback() -> str:
    """
    Try a few common ways to reach the Windows host from WSL2.
    1) host.docker.internal (often works on recent WSL)
    2) nameserver in /etc/resolv.conf (Hyper-V gateway)
    """
    # Try host.docker.internal first
    try:
        import socket
        return socket.gethostbyname("host.docker.internal")
    except Exception:
        pass

    # Fallback to the first nameserver in resolv.conf
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if line.startswith("nameserver"):
                    return line.split()[1].strip()
    except Exception:
        pass

    # Last resort: user must supply --host
    return ""

def main():
    ap = argparse.ArgumentParser(description="WSL ZMQ SUB (connects to Windows host)")
    ap.add_argument("--host", default="", help="Windows host/IP (auto-detect if empty)")
    ap.add_argument("--port", type=int, default=55556, help="Publisher port (default: 55556)")
    ap.add_argument("--topic", default="demo", help="Subscribe topic (default: demo)")
    ap.add_argument("--hwm", type=int, default=1000, help="RCVHWM queue depth (default: 1000)")
    args = ap.parse_args()

    host = args.host or detect_windows_host_fallback()
    if not host:
        print("[SUB] Could not auto-detect Windows host; pass --host <WIN_IP>")
        return

    endpoint = f"tcp://{host}:{args.port}"

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, args.hwm)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.topic)

    print(f"[SUB] Connecting to {endpoint} and subscribing to topic '{args.topic}'")
    sub.connect(endpoint)

    count = 0
    t0 = time.perf_counter()
    try:
        while True:
            frames = sub.recv_multipart()
            if len(frames) != 2:
                continue
            topic, msg = frames
            count += 1

            try:
                header_raw, payload = msg.split(b"\n", 1)
            except ValueError:
                header_raw, payload = msg, b""

            try:
                header = json.loads(header_raw.decode("utf-8"))
            except Exception:
                header = {"seq": None, "ts": None, "note": "header_parse_error"}

            if count % 100 == 0:
                dt = time.perf_counter() - t0
                rate = count / dt if dt > 0 else 0.0
                print(f"[SUB] got {count} ~{rate:,.0f} msg/s | last seq={header.get('seq')} | payload={len(payload)} B")
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0)
        ctx.term()
        dt = time.perf_counter() - t0
        rate = count / dt if dt > 0 else 0.0
        print(f"[SUB] total {count} messages in {dt:.2f}s ({rate:,.1f} msg/s)")

if __name__ == "__main__":
    main()
