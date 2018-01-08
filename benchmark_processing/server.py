import pdb
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
if len(sys.argv) <= 1:
    server_address = ("localhost", 10000)
else:
    server_address = ("localhost", sys.argv[1])

sock.bind(server_address)
print("Server started")
while True:
    data, address = sock.recvfrom(4096)
    #pdb.set_trace()
    if data:

        for i in range(1):
            ct = 0
            for j in range(1):
                ct += 1
        sent = sock.sendto(data, address)
