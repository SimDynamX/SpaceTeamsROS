# # Simple subscriber to test WSL2 networking with a ZMQ publisher on Windows
# import zmq
# import subprocess
# ctx = zmq.Context()
# s = ctx.socket(zmq.SUB)
# s.setsockopt(zmq.SUBSCRIBE, b"")
# windows_ip = ""
# try:
#     result = subprocess.run(['cat', '/etc/resolv.conf'], capture_output=True, text=True)
#     nameserver_line = [line for line in result.stdout.split('\n') if 'nameserver' in line][0]
#     windows_ip = nameserver_line.split()[1]
# except Exception as e:
#     print(f"Failed to get Windows IP: {str(e)}")
#     windows_ip = "172.17.0.1"  # Common WSL2 default

# connect_address = f"tcp://{windows_ip}:47653"
# s.connect(connect_address)  # or use the IP from /etc/resolv.conf
# print(f"{connect_address} connected; waiting…")
# while True:
#     print(len(s.recv()))


# REQ client on WSL
import zmq, subprocess, time

PORT = 56565

def windows_ip():
    out = subprocess.run(['cat', '/etc/resolv.conf'], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if line.startswith('nameserver'):
            return line.split()[1]
    return "172.17.0.1"

ip = windows_ip()
addr = f"tcp://{ip}:{PORT}"
print(f"[REQ] Connecting to {addr}")

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.connect(addr)

try:
    i = 0
    while True:
        payload = f"ping {i}".encode()
        sock.send(payload)
        rep = sock.recv()  # blocks until server replies
        print(f"[REQ] sent={payload!r}  recv={rep!r}")
        i += 1
        time.sleep(1.0)
except KeyboardInterrupt:
    pass
finally:
    sock.close()
    ctx.term()
