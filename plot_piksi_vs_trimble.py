#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import pyproj
import rospy
import rostopic

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

proj = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')

class Plotter():
    def __init__(self):
        
        self.fig, self.ax = plt.subplots()
        self.lines_trimble, = self.ax.plot([],[],'b',label='trimble')
        self.lines_piksi, = self.ax.plot([],[],'r',label='piksi')
        self.ax.set_autoscaley_on(True)
        self.x_trimble = []
        self.y_trimble = []
        self.x_trimble_initial = 0
        self.y_trimble_initial = 0
        self.x_piksi = []
        self.y_piksi = []
        self.x_piksi_initial = 0
        self.y_piksi_initial = 0

        self.fig.suptitle("GPS positions", fontsize=16)
        self.ax.set_ylabel("GPS northing [m]")
        self.ax.set_xlabel("GPS easting [m]")
        self.l1 = self.ax.legend(loc="best")

        self.sub_trimble = rospy.Subscriber("/sensor/gps/trimble/fix", NavSatFix, self.callback_trimble)
        self.sub_piksi = rospy.Subscriber("/sensor/gps/piksi_1/gps/rtkfix", Odometry, self.callback_leica)

        print "created subscribers "
        rospy.Timer(rospy.Duration(0.1), self.update_plot)

    def update_plot(self, event):
        if not rospy.is_shutdown():
            self.lines_trimble.set_xdata(self.x_trimble)
            self.lines_trimble.set_ydata(self.y_trimble)
            self.lines_piksi.set_xdata(self.x_piksi)
            self.lines_piksi.set_ydata(self.y_piksi)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
#             print "updated plot"
        else:
            plt.close(self.fig)

    def callback_trimble(self, msg):
        e, n = proj(msg.longitude, msg.latitude)
        if (len(self.x_trimble) == 0):
            self.x_trimble_initial = e
            self.y_trimble_initial = n
            self.x_trimble.append(0)
            self.y_trimble.append(0)
        else:
            x_diff = e - self.x_trimble_initial
            y_diff = n - self.y_trimble_initial
            self.x_trimble.append(x_diff)
            self.y_trimble.append(y_diff)


    def callback_leica(self, msg):
        #e, n = proj(msg.longitude, msg.latitude)
        if (len(self.x_piksi) == 0):
            self.x_piksi_initial = msg.pose.pose.position.x
            self.y_piksi_initial = msg.pose.pose.position.y
            self.x_piksi.append(0)
            self.y_piksi.append(0)
        else:
            x_diff = msg.pose.pose.position.x - self.x_piksi_initial
            y_diff = msg.pose.pose.position.y - self.y_piksi_initial
            self.x_piksi.append(x_diff)
            self.y_piksi.append(y_diff)
        #self.x_piksi.append(msg.longitude)
        #self.y_piksi.append(msg.latitude)


rospy.init_node('plot_gps_Trimble_vs_piksi')
p = Plotter()
plt.show()
