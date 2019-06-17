#!/usr/bin/python3
import gzip
import re
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(precision=3, suppress=True)

filename='sigrok-long-tim1-tim8.vcd.gz'


def fileToEdgeDeltas(filename):
    # Rising edges of two signals:
    #
    # ----A-------B--------------A-------B--------------A-------B
    #      <--7--> <-----13-----> <--7-->|<-----13-----> <--7-->|
    #                                    |                      |
    #                                    |                      |
    #                                Output 7                Output 7
    lastChannelA = None
    lastChannelB = None
    pattern = re.compile("^#(\d+) ([01])([\"#])$")
    result = []

    with gzip.open('sigrok-long-tim1-tim8.vcd.gz','rt') as f:
        for line in f:
            match = pattern.match(line)
            if match:
                thisTime = int(match.group(1))
                if match.group(2) is '1':
                    if match.group(3) is '#': # Found A
                        lastChannelA = thisTime
                    elif match.group(3) is '"': # Found B
                        if lastChannelA is not None and lastChannelB is not None:
                            timeBtoA = lastChannelA-lastChannelB;
                            timeAtoB = thisTime-lastChannelB;
                            if (timeBtoA > timeAtoB):
                                result.append([lastChannelB, timeAtoB])
                            else:
                                result.append([lastChannelB, -timeBtoA])
                        lastChannelB = thisTime
    return result

edgeDeltas = fileToEdgeDeltas(filename);
#for x in edgeDeltas:
#  print(x[1], int(x[1]))

#timestamp = np.array(map(lambda x : x[0], edgeDeltas))
delta = np.array(edgeDeltas)

plt.plot(delta[:,0],delta[:,1]); plt.show()
