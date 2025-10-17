#!/usr/bin/env python3
import argparse
import json
import socket
import subprocess
import time
import zmq

def detect_windows_host() -> str:
    """
    Best signal in WSL2: the Windows host is the default gateway.
    Parse `ip route` → default via <GW> dev eth0
    """
    try:
        out = subprocess.check_output(["/sbin/ip", "route"], text=True)
        for line in out.splitlines():
            if line.startswith("default "):
                parts = line.split()
                gw = parts[2]
                socket.inet_aton(gw)  # validate IPv4
                return gw
    except Exception:
        pass
    # Fallbacks (less reliable)
    try:
        return socket.gethostbyname("host.docker.internal")
    except Exception:
        pass
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if line.startswith("nameserver"):
                    cand = line.split()[1].strip()
                    socket.inet_aton(cand)
                    return cand
    except Exception:
        pass
    return ""

def tcp_probe(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

def main():
    ap = argparse.ArgumentParser(description="WSL ZMQ SUB (connects to Windows)")
    ap.add_argument("--host", default="", help="Windows host/IP (auto-detect if empty)")
    ap.add_argument("--port", type=int, default=55556, help="Publisher port (default: 55556)")
    ap.add_argument("--topic", default="demo", help="Subscribe topic (default: demo)")
    ap.add_argument("--hwm", type=int, default=1000, help="RCVHWM")
    ap.add_argument("--tcp-check", action="store_true", help="Plain TCP probe before connecting")
    ap.add_argument("--poll-ms", type=int, default=200, help="Poll timeout ms for Ctrl+C responsiveness")
    args = ap.parse_args()

    host = args.host or detect_windows_host()
    if not host:
        print("[SUB] Could not auto-detect Windows host; pass --host <WIN_IP>")
        return
    print(f"[SUB] Using Windows host: {host}")
    if args.tcp_check:
        ok = tcp_probe(host, args.port, 2.0)
        print(f"[SUB] TCP probe to {host}:{args.port} -> {'OK' if ok else 'FAILED'}")
        if not ok:
            return

    endpoint = f"tcp://{host}:{args.port}"

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVHWM, args.hwm)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.TCP_KEEPALIVE, 1)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.topic)

    print(f"[SUB] Connecting to {endpoint} and subscribing to '{args.topic}'")
    sub.connect(endpoint)

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    count = 0
    t0 = time.perf_counter()
    try:
        while True:
            events = dict(poller.poll(args.poll_ms))
            if events.get(sub) == zmq.POLLIN:
                topic, msg = sub.recv_multipart(zmq.NOBLOCK)
                try:
                    header_raw, payload = msg.split(b"\n", 1)
                except ValueError:
                    header_raw, payload = msg, b""
                try:
                    header = json.loads(header_raw.decode("utf-8"))
                except Exception:
                    header = {"seq": None, "ts": None, "note": "header_parse_error"}
                count += 1
                if count % 100 == 0:
                    dt = time.perf_counter() - t0
                    rate = count / dt if dt > 0 else 0.0
                    print(f"[SUB] got {count} ~{rate:,.0f} msg/s | last seq={header.get('seq')} | payload={len(payload)} B")
            # else: timeout; loop again (keeps Ctrl+C responsive)
    except KeyboardInterrupt:
        print("\n[SUB] Ctrl+C received; shutting down...")
    finally:
        sub.close(0)
        ctx.term()
        dt = time.perf_counter() - t0
        rate = count / dt if dt > 0 else 0.0
        print(f"[SUB] total {count} in {dt:.2f}s ({rate:,.1f} msg/s)")

if __name__ == "__main__":
    main()
