import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 8080))

while True:
    data, addr = sock.recvfrom(8)
    if len(data) == 8:
        # '>d' means Big-Endian 64-bit float
        (val,) = struct.unpack(">d", data)
        print(f"Received float: {val}")