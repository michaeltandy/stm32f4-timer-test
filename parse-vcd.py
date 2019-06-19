#!/usr/bin/python3
import gzip
import re
import sys
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import numpy as np
#np.set_printoptions(precision=3, suppress=True)
from datetime import timedelta, datetime, timezone
from scipy import optimize

# Parses a file recorded with e.g. /home/mtandy/apps/sigrok-cli-0.7.1-x86_64.AppImage --driver=fx2lafw --config samplerate=4M --time 18000s --output-format vcd --output-file sigrok-recording-5hour.vcd --channels D0,D1,D2;

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

    with gzip.open(filename,'rt') as f:
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


filename = sys.argv[1]
edgeDeltas = fileToEdgeDeltas(filename);
#for x in edgeDeltas:
#  print(x[1], int(x[1]))

delta = np.array(edgeDeltas)
prettyTimes = [datetime.fromtimestamp(i/1000000000, tz=timezone.utc) for i in delta[:,0]]

#pfit, stats = np.polynomial.Polynomial.fit(delta[:,0],delta[:,1], 1, full=True)
#print(pfit,stats)

def linear_model(x, m, c):
    return m * x + c
p, e = optimize.curve_fit(linear_model, delta[:,0], delta[:,1], p0=[1, 0]);
print("Fitted parameters:", p)
print("i.e. skew rate of", p[0]*1000000000, "parts per billion")

#plt.plot(delta[:,0], delta[:,1] );
#plt.plot(delta[:,0], linear_model(delta[:,0], *p) );
#plt.show()

fig, ax = plt.subplots()
ax.plot_date(prettyTimes, delta[:,1]-linear_model(delta[:,0], *p), '-' );
ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
plt.show()
