#!/usr/bin/python3
import gzip
import re
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
np.set_printoptions(precision=3, suppress=True)
from datetime import timedelta, datetime

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

pfit, stats = np.polynomial.Polynomial.fit(delta[:,0],delta[:,1], 1, full=True)
print(pfit,stats)

prettyTimes = [datetime.fromtimestamp(i/1000000000) for i in delta[:,0]]

#plt.plot(delta[:,0], delta[:,1] );
#plt.plot(delta[:,0], pfit(delta[:,0]) );

#plt.plot(delta[:,0], delta[:,1]-pfit(delta[:,0]) );


fig, ax = plt.subplots()
ax.plot_date(prettyTimes, delta[:,1]-pfit(delta[:,0]), '-' );
ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
plt.show()
