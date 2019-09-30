# Visualizer for LIDAR output from robot.
import threading
from networktables import NetworkTables
import numpy as np 
import matplotlib.pyplot as plt
import time
import matplotlib
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import matplotlib.animation as animation

# Configure the ip address below and then run. Make sure you
# have matplotlib, numpy, and networktables installed.
# This program requires that enableDashboardOutput() is set
# to true in the lidar class in robot code.
roborio_ip = '169.254.250.49'




cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server=roborio_ip)
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Insert your processing code here
print("Connected!")
table = NetworkTables.getTable('SmartDashboard')
fig = plt.figure()

ax1 = fig.add_subplot(1,1,1)
t = None
def animate(j):
    global t
    angles = table.getNumberArray("Angles", [])
    distances = table.getNumberArray("Distances", [])
    objx = table.getNumberArray("objx", [])
    objy = table.getNumberArray("objy", [])
    polarpts = []
    points = (int) (table.getNumber("Points", 0))
    for i in range(0,min(points,len(angles),len(distances))):
        polarpts.append([angles[i],distances[i]])
    polarpts.sort()
    pts = []
    for i in range(0,len(polarpts)):
        x = polarpts[i][1] * np.cos(polarpts[i][0] * 0.0174533)
        y = polarpts[i][1] * np.sin(polarpts[i][0] * 0.0174533)
        pts.append([x,y])
    pt = np.array(pts)
    if t is not None:
        t.remove()
    t = Polygon(pt,color='cyan', zorder=1)
    ax1.clear()
    ax1.add_patch(t)
    ax1.scatter([c[0] for c in pts],[c[1] for c in pts], c='red', zorder=10, alpha=0.7)
    ax1.scatter(objx,objy, c='yellow', zorder=10, alpha=0.5,s=700)
    ax1.scatter([0],[0], c='green', zorder=11)
    ax1.text(-250, 250, points, fontsize=12)
    ax1.set_xlim([-300,300])
    ax1.set_ylim([-300,300])

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()