import pdb
import pickle
import socket
import sys
import uuid
import time
import matplotlib.pyplot as plt


data = dict()
data["id"] = uuid.uuid1()
data["timestamp"] = time.time()
data["src"] = 1
data["dest"] = 1
data["origin_coord"] = [10, 20, 30]
data["type"] = 1
data["data"] = "Plume Detected"

result_array = list()


def start_client(server_address):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _ in range(1000):
            data["timestamp"] = time.time()
            b = pickle.dumps(data)
            sock.sendto(b, server_address)
            #pdb.set_trace()
            d, server = sock.recvfrom(4096)
            result_array.append((d, time.time()))
            #d = pickle.loads(d)
            #pdb.set_trace()
        ts = [(x[1], pickle.loads(x[0])['timestamp']) for x in result_array]
        dffs = [x[0]-x[1] for x in ts]
        plt.plot(dffs, linewidth=0.1)
        plt.show(dffs)
    except IndexError as e:
        print(e)
        #pdb.set_trace()


if __name__ == "__main__":
    if len(sys.argv) <= 1:
        server_address = ("localhost", 10000)
    else:
        server_address = (sys.argv[1], int(sys.argv[2]))
    start_client(server_address)

