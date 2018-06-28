from uuid import uuid1 
import numpy
import pickle, socket, sys, time
import matplotlib.pyplot as plt
import pdb
import matplotlib
font = {'size'   : 11}

matplotlib.rc('font', **font)

class Message(object):
    def __init__(self, src=1, dst=99, origin_coord=[10,20,30], type=1, data="nothing"):
        self.id = uuid1()
        self.src = src
        self.dst = dst
        self.origin_coord = origin_coord
        self.type = type
        self.data = data
        self.timestamp = time.time()
        

def start_client(server_address):
    fontsize = 14;
    mydict = dict()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for _ in range(1000):
        msg = Message()
        mydict[msg.id] = [msg.timestamp]
        b = pickle.dumps(msg)
        #pdb.set_trace()
        sock.sendto(b, server_address)
        d, server = sock.recvfrom(4096)
        d = pickle.loads(d)
        #pdb.set_trace()
        mydict[d.id].append(time.time())
    dffs = [t2-t1 for t1, t2 in mydict.values()]
    mean = sum(dffs) / len(dffs)
    std = numpy.std(dffs);
    print("mean = {0}, std= {1}".format(mean, std))
    plt.ylim(0, 0.02)
    plt.xlabel("Iteration Number", size=fontsize)
    plt.ylabel("Round Trip Time (Seconds)", size=fontsize)
    plt.title("Round Trip Time", size=fontsize)
    plt.text(200, 0.0175, "mean = {0:.5f}, std= {1:.5f}".format(mean, std), size=fontsize)
    plt.plot(dffs, 'r', linewidth=0.4)
    plt.show(dffs)
#    pdb.set_trace()
    
if __name__ == "__main__":
    if len(sys.argv) <= 1:
        server_address = ("localhost", 10000)
    else:
        server_address = (sys.argv[1], int(sys.argv[2]))
    start_client(server_address)

    