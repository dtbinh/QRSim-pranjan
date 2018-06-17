import pdb
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
if len(sys.argv) <= 1:
    server_address = ("0.0.0.0", 10000)
else:
    server_address = ("0.0.0.0", sys.argv[1])

sock.bind(server_address)
print("Server started")
while True:
    data, address = sock.recvfrom(4096)
    #pdb.set_trace()
    if data:
        ct = 0
        for i in range(10):
            for j in range(10):
                ct += 1
                print("{0} ".format(ct), end=" ")
        sent = sock.sendto(data, address)
