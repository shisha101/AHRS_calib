#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import pyproj
import rospy
import rostopic
import numpy as np
import math

from sensor_msgs.msg import NavSatFix
from nmea_navsat_driver.msg import NavSatOrientation
from nav_msgs.msg import Odometry
from numpy import arctan2
from docutils.writers.odf_odt import ToString

proj = pyproj.Proj(proj='utm', zone=32, ellps='WGS84')

class Plotter():
    def __init__(self):
        
        self.invert_axis = -1 # 1 or -ve 1 to invert axis
        
        self.fig, self.ax = plt.subplots()
        self.lines_trimble, = self.ax.plot([],[],'b',label='trimble')
        self.lines_piksi, = self.ax.plot([],[],'r',label='piksi')
        self.ax.set_autoscaley_on(True)
        self.time_initial = -1
        self.time_trimble = []
        self.angle_trimble = []
        self.angle_trimble_initial = 0
        
        self.angle_piksi = []
        self.x_piksi = []
        self.y_piksi = []
        self.time_piksi = []
        self.x_piksi_initial = 0
        self.y_piksi_initial = 0
        self.angle_piksi_initial = 0
        
        self.error = []
        self.fig_rms_text = self.fig.text(0.5, 0.5, "RMS Error = ")

        self.fig.suptitle("GPS orientation", fontsize=16)
        self.ax.set_ylabel("GPS angle [deg]")
        self.ax.set_xlabel("GPS step []")
        self.l1 = self.ax.legend(loc="best")

        self.sub_trimble = rospy.Subscriber("/sensor/gps/trimble/orientation", NavSatOrientation, self.callback_trimble)
        self.sub_piksi = rospy.Subscriber("/sensor/gps/piksi_1/gps/rtkfix", Odometry, self.callback_leica)

        print "created subscribers "
        rospy.Timer(rospy.Duration(0.1), self.update_plot)

    def update_plot(self, event):
        if not rospy.is_shutdown():
#             print "updated plot **************\n"
            self.lines_trimble.set_xdata(self.time_trimble)
            self.lines_trimble.set_ydata(self.angle_trimble)
            self.lines_piksi.set_xdata(self.time_piksi)
            self.lines_piksi.set_ydata(self.angle_piksi)
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            iteration_start = len(self.error)
            if len(self.angle_piksi)> len(self.angle_trimble):
                iteration_end = len(self.angle_trimble)
            else:
                iteration_end = len(self.angle_piksi)
                print iteration_end
                print iteration_start
            if iteration_end > 1:
                for i in xrange(iteration_start,iteration_end-1):
                    self.error.append((self.angle_piksi[i]-self.angle_trimble[i])**2)
                rms_error = math.sqrt(sum(self.error) /len(self.error))
                rms_error_str = "RMS Error = "+str(rms_error)
                print rms_error_str
                self.fig_rms_text.set_text(rms_error_str)
            
        else:
            plt.close(self.fig)

    def callback_trimble(self, msg):
        if (len(self.time_trimble) == 0):
            self.angle_trimble_initial = msg.yaw
            self.time_trimble.append(0)
            self.angle_trimble.append(0)
            if self.time_initial == -1:# time has not been set yet
                self.time_initial = msg.header.stamp#.to_sec()
        else:
            y_diff = msg.yaw - self.angle_trimble_initial
#             self.time_trimble.append(len(self.time_trimble)-1)
            time_elapsed_since_start = (msg.header.stamp - self.time_initial).to_sec()
            if time_elapsed_since_start < 0:
                self.time_initial = msg.header.stamp
            self.time_trimble.append(time_elapsed_since_start)
            self.angle_trimble.append(y_diff*180/np.pi)


    def callback_leica(self, msg):
        #e, n = proj(msg.longitude, msg.latitude)
        if (len(self.x_piksi) == 0):
            self.angle_piksi_initial = self.invert_axis * np.arctan2(msg.pose.pose.position.y, msg.pose.pose.position.x)
            if self.time_initial == -1:# time has not been set yet
                self.time_initial = msg.header.stamp#.to_sec()
        else:
            current_angle = self.invert_axis * np.arctan2(msg.pose.pose.position.y, msg.pose.pose.position.x)
            angle_diff = current_angle-self.angle_piksi_initial
#             print angle_diff*180/np.pi
            if angle_diff <-180:
                angle_diff += 2*np.pi
#             print angle_diff*180/np.pi
            self.angle_piksi.append(angle_diff*180/np.pi)
            time_elapsed_since_start = (msg.header.stamp - self.time_initial).to_sec()
            if time_elapsed_since_start < 0:
                self.time_initial = msg.header.stamp
#             print time_elapsed_since_start
            self.time_piksi.append(time_elapsed_since_start)
        self.x_piksi.append(msg.pose.pose.position.x)
        self.y_piksi.append(msg.pose.pose.position.y)
        #self.x_piksi.append(msg.longitude)
        #self.y_piksi.append(msg.latitude)


rospy.init_node('plot_gps_plot_gps_Trimble_vs_piksi_orientation')
p = Plotter()
plt.show()
