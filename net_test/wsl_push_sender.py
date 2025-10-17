#!/usr/bin/env python3
import argparse
import json
import os
import time
from datetime import datetime, timezone
import socket
import zmq

import subprocess
import socket

def detect_windows_host() -> str:
    """
    WSL2: The Windows host is the default gateway.
    Example: `ip route` → default via 172.27.240.1 dev eth0
    """
    try:
        out = subprocess.check_output(["/sbin/ip", "route"], text=True)
        for line in out.splitlines():
            if line.startswith("default "):
                parts = line.split()
                gw = parts[2]  # default via <gw>
                # quick sanity: looks like an IPv4
                socket.inet_aton(gw)
                return gw
    except Exception:
        pass

    # Fallbacks (less reliable)
    try:
        return socket.gethostbyname("host.docker.internal")
    except Exception:
        pass

    # As a last resort: nameserver from resolv.conf
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if line.startswith("nameserver"):
                    candidate = line.split()[1].strip()
                    socket.inet_aton(candidate)
                    return candidate
    except Exception:
        pass

    return ""

def tcp_probe(host: str, port: int, timeout: float = 2.0) -> bool:
    """Plain TCP test so we fail fast if the route/firewall is wrong."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

def main():
    ap = argparse.ArgumentParser(description="WSL PUSH sender (connects to Windows)")
    ap.add_argument("--host", default="", help="Windows host/IP (auto-detect if empty)")
    ap.add_argument("--port", type=int, default=55555, help="Windows PULL port (default: 55555)")
    ap.add_argument("--count", type=int, default=1000, help="Messages to send")
    ap.add_argument("--rate", type=float, default=15.0, help="Msgs/sec (0=max)")
    ap.add_argument("--payload", type=int, default=0, help="Payload bytes per message")
    ap.add_argument("--hwm", type=int, default=1000, help="SNDHWM")
    ap.add_argument("--wait-connect", type=float, default=0.0,
                    help="Optional seconds to sleep after connect (default: 0; avoids 'stuck on connecting')")
    ap.add_argument("--tcp-check", action="store_true",
                help="Do a plain TCP probe before sending")
    args = ap.parse_args()
    
    host = args.host or detect_windows_host()
    if not host:
        print("[WSL PUSH] Could not auto-detect Windows host; pass --host <WIN_IP>")
        return

    endpoint = f"tcp://{host}:{args.port}"
    print(f"[WSL PUSH] Using Windows host: {host}")

    if args.tcp_check:
        ok = tcp_probe(host, args.port, timeout=2.0)
        print(f"[WSL PUSH] TCP probe to {host}:{args.port} -> {'OK' if ok else 'FAILED'}")
        if not ok:
            print("[WSL PUSH] Check Windows bind IP/port or firewall rule.")
            return

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.SNDHWM, args.hwm)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.TCP_KEEPALIVE, 1)

    print(f"[WSL PUSH] Connecting to {endpoint} ...")
    sock.connect(endpoint)

    # Do NOT block on monitor; just optionally give TCP a moment
    if args.wait_connect > 0:
        time.sleep(args.wait_connect)
    print("[WSL PUSH] Starting send loop.")

    payload = os.urandom(args.payload) if args.payload > 0 else b""
    interval = 1.0 / args.rate if args.rate > 0 else 0.0
    next_send = time.perf_counter()
    t0 = time.perf_counter()

    try:
        for i in range(args.count):
            header = {
                "seq": i,
                "ts": datetime.now(timezone.utc).isoformat(timespec="microseconds"),
                "payload_bytes": len(payload),
            }
            header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
            sock.send(header_bytes + b"\n" + payload, copy=False)

            if interval > 0:
                next_send += interval
                delay = next_send - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

            if (i + 1) % 100 == 0:
                sent = i + 1
                rate = sent / (time.perf_counter() - t0)
                print(f"[WSL PUSH] sent {sent}/{args.count} ~{rate:,.0f} msg/s")
    except KeyboardInterrupt:
        print("\n[WSL PUSH] Ctrl+C received; stopping...")
    finally:
        sock.close(0)
        ctx.term()
        print("[WSL PUSH] Done.")

if __name__ == "__main__":
    main()
