#!/usr/bin/env python3
import argparse
import json
import time
import zmq

def main():
    ap = argparse.ArgumentParser(description="WSL PULL receiver (WSL2 NAT-friendly)")
    ap.add_argument("--port", type=int, default=55555, help="TCP port to bind (default: 55555)")
    ap.add_argument("--public", action="store_true",
                    help="Bind on all interfaces (tcp://*:PORT). Default binds to 127.0.0.1 for safety.")
    ap.add_argument("--hwm", type=int, default=1000, help="ZMQ High Water Mark (queue depth)")
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.RCVHWM, args.hwm)
    sock.setsockopt(zmq.LINGER, 0)

    bind_ip = "*" if args.public else "127.0.0.1"
    endpoint = f"tcp://{bind_ip}:{args.port}"
    sock.bind(endpoint)
    if args.public:
        print(f"[WSL Receiver] Bound (PUBLIC) at {endpoint}")
    else:
        print(f"[WSL Receiver] Bound (LOCAL) at {endpoint} — Windows can connect via 127.0.0.1:{args.port}")

    count = 0
    t0 = time.perf_counter()
    try:
        while True:
            msg = sock.recv()
            count += 1

            # Expect "<json>\n" + optional payload
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
                print(f"[WSL Receiver] got {count} | ~{rate:,.0f} msg/s | last seq={header.get('seq')} | payload={len(payload)} B")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(0)
        ctx.term()
        dt = time.perf_counter() - t0
        rate = count / dt if dt > 0 else 0.0
        print(f"[WSL Receiver] total {count} messages in {dt:.2f}s ({rate:,.1f} msg/s)")

if __name__ == "__main__":
    main()
