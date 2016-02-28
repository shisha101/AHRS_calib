#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Imu, MagneticField
from tf import transformations
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from itertools import islice




class imu_data_capture(object):

    def __init__(self, imu_tn, mag_tn):
        self.acc_data = [[], [], []]  # x, y, z
        self.gyro_data = [[], [], []]  # x, y, z
        self.mag_data = [[], [], []]  # x, y, z
        self.orient_rpy = [[], [], []]  # roll, pitch, yaw
        self.orient_quat = [[], [], [], []]  # x, y, z, w
        self.time_mag = []
        self.time_imu = []
        self.data_dic = {"acc": self.acc_data, "gyro": self.gyro_data, "mag": self.mag_data,
                         "orient_rpy": self.orient_rpy, "orient_quat": self.orient_quat,
                         "time_mag": self.time_mag, "time_imu": self.time_imu}
        self.sub_imu = rospy.Subscriber(imu_tn, Imu, self.imu_msg_callback)
        self.sub_mag = rospy.Subscriber(mag_tn, MagneticField, self.mag_msg_callback)

    def imu_msg_callback(self, imu_msg):
        msg_data_acc = imu_msg.linear_acceleration
        msg_data_gyro = imu_msg.angular_velocity
        msg_data_orient = imu_msg.orientation

        self.data_dic["acc"][0].append(msg_data_acc.x)
        self.data_dic["acc"][1].append(msg_data_acc.y)
        self.data_dic["acc"][2].append(msg_data_acc.z)

        self.data_dic["gyro"][0].append(msg_data_gyro.x)
        self.data_dic["gyro"][1].append(msg_data_gyro.y)
        self.data_dic["gyro"][2].append(msg_data_gyro.z)

        self.data_dic["orient_quat"][0].append(msg_data_orient.x)
        self.data_dic["orient_quat"][1].append(msg_data_orient.y)
        self.data_dic["orient_quat"][2].append(msg_data_orient.z)
        self.data_dic["orient_quat"][2].append(msg_data_orient.w)

        self.data_dic["time_imu"].append(imu_msg.header.stamp)

        # convert quat to rpy for plotting and vis
        rpy = transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y,
                                                     imu_msg.orientation.z, imu_msg.orientation.w])
        self.data_dic["orient_rpy"][0].append(rpy[0])
        self.data_dic["orient_rpy"][1].append(rpy[1])
        self.data_dic["orient_rpy"][2].append(rpy[2])

    def mag_msg_callback(self, mag_msg):
        self.data_dic["mag"][0].append(mag_msg.magnetic_field.x)
        self.data_dic["mag"][1].append(mag_msg.magnetic_field.y)
        self.data_dic["mag"][2].append(mag_msg.magnetic_field.z)

        self.data_dic["time_mag"].append(mag_msg.header.stamp)

class plot_imu_data(object):

    def __init__(self, dict_of_values, absolute_time=False):
        self.data = dict_of_values
        # self.data = imu_data_capture("a", "b")
        if not absolute_time:
            # convert time to duration
            self.data["time_mag"] = [(x - self.data["time_mag"][0]).to_sec() for x in self.data["time_mag"]]
            self.data["time_imu"] = [(x - self.data["time_imu"][0]).to_sec() for x in self.data["time_imu"]]
        else:
            self.data["time_mag"] = [x.to_sec() for x in self.data["time_mag"]]
            self.data["time_imu"] = [x.to_sec() for x in self.data["time_imu"]]
        self.mag_min_max_dict = self.calc_min_max("mag")

    def calc_min_max(self, data_dict_entry):
        min_max_dict = {"min_x": min(self.data[data_dict_entry][0]), "max_x": max(self.data[data_dict_entry][0]),
                        "min_y": min(self.data[data_dict_entry][1]), "max_y": max(self.data[data_dict_entry][1]),
                        "min_z": min(self.data[data_dict_entry][2]), "max_z": max(self.data[data_dict_entry][2])}
        return min_max_dict

    def plot_imu(self, time_list, values_list, axis_names_list):
        if len(values_list) != len(axis_names_list) and len(values_list) != len(time_list):
            print "error cannot plot axis length not equal to name "
            return False
        number_of_sub_plot = 11 + 100*len(values_list)
        fig = plt.figure()

        for i in xrange(len(values_list)):
            plot_1 = fig.add_subplot(number_of_sub_plot + i)
            plot_1.plot(time_list, values_list[i])
            plot_1.set_xlabel("time (s)")
            plot_1.set_ylabel(axis_names_list[i])

        fig.tight_layout()
        fig.show()
        return fig  # return figure handle

    def plot_rpy(self):
        axis_names = ["roll", "pitch", "yaw"]
        self.plot_imu(self.data["time_imu"], self.data["orient_rpy"], axis_names)

    def plot_acc(self):
        axis_names = ["acc_x", "acc_y", "acc_z"]
        self.plot_imu(self.data["time_imu"], self.data["acc"], axis_names)

    def plot_gyro(self):
        axis_names = ["gyro_x", "gyro_y", "gyro_z"]
        self.plot_imu(self.data["time_imu"], self.data["gyro"], axis_names)

    def plot_mag(self):
        axis_names = ["mag_x", "mag_y", "mag_z"]
        self.plot_imu(self.data["time_mag"], self.data["mag"], axis_names)

    def plot_mag_circles(self):
        axis_names = ["normal to X", "normal to Y", "normal to Z"]
        fig = plt.figure()
        plot_1 = fig.add_subplot(311)
        plot_1.plot(self.data["mag"][1], self.data["mag"][2])
        plot_1.set_xlabel(axis_names[0])
        plot_1.axis("equal")

        plot_2 = fig.add_subplot(312)
        plot_2.plot(self.data["mag"][0], self.data["mag"][2])
        plot_2.set_xlabel(axis_names[1])
        plot_2.axis("equal")

        plot_3 = fig.add_subplot(313)
        plot_3.plot(self.data["mag"][0], self.data["mag"][1])
        plot_3.set_xlabel(axis_names[2])
        plot_3.axis("equal")

        fig.tight_layout()
        fig.show()

    def plot_mag_scatter_raw(self, downsampling_step=10):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        x_downsampled = self.downsample(self.data["mag"][0], downsampling_step)
        y_downsampled = self.downsample(self.data["mag"][1], downsampling_step)
        z_downsampled = self.downsample(self.data["mag"][2], downsampling_step)
        # using complete data set for min max
        ranges = [self.mag_min_max_dict["max_x"]-self.mag_min_max_dict["min_x"],
                  self.mag_min_max_dict["max_y"]-self.mag_min_max_dict["min_y"],
                  self.mag_min_max_dict["max_z"]-self.mag_min_max_dict["min_z"]]
        ax.scatter(x_downsampled, y_downsampled, z_downsampled, c="b", marker="o")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        axis_range_max = max([self.mag_min_max_dict["max_x"],
                              self.mag_min_max_dict["max_y"],
                              self.mag_min_max_dict["max_z"]])

        axis_rang_min = min([self.mag_min_max_dict["min_x"],
                             self.mag_min_max_dict["min_y"],
                             self.mag_min_max_dict["min_z"]])

        ax.set_xlim3d(axis_rang_min, axis_range_max)
        ax.set_ylim3d(axis_rang_min, axis_range_max)
        ax.set_zlim3d(axis_rang_min, axis_range_max)
        textstr = "range x: " + str(ranges[0]) + "\n range y: " + str(ranges[1]) + " \n range z: " + str(ranges[2])
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        fig.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14, verticalalignment='top', bbox=props)
        fig.show()

    def plot_norm_mag_vs_sphere(self, down_sampling_step=10, plot_sphere=True, sphere_radius=1):

        # This normalization considers that the vectors start at the origin

        # TODO: fix normalization which is assumes all vectors start from the origin
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # normalize vectors
        np_array = np.array([self.data["mag"][0], self.data["mag"][1], self.data["mag"][2]])
        norm = np.sqrt((np_array*np_array).sum(axis=0))  # np*np element wise mul sum(axis=0) sum all squared components
        norm_coordinates = np_array/norm

        # down sample the normalized coordinates
        x_down_sampled = self.downsample(norm_coordinates[0], down_sampling_step)
        y_down_sampled = self.downsample(norm_coordinates[1], down_sampling_step)
        z_down_sampled = self.downsample(norm_coordinates[2], down_sampling_step)

        ax.scatter(x_down_sampled, y_down_sampled, z_down_sampled, c="b", marker="o")
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        if plot_sphere:
            u = np.linspace(0, 2 * np.pi, 100)
            v = np.linspace(0, np.pi, 100)

            x = sphere_radius * np.outer(np.cos(u), np.sin(v))
            y = sphere_radius * np.outer(np.sin(u), np.sin(v))
            z = sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x, y, z, rstride=4, cstride=4, color='r', alpha=0.2)

        # using complete data set for min max
        ranges = [max(norm_coordinates[0])-min(norm_coordinates[0]),
                  max(norm_coordinates[1])-min(norm_coordinates[1]),
                  max(norm_coordinates[2])-min(norm_coordinates[2])]

        textstr = "range x: " + str(ranges[0]) + "\n range y: " + str(ranges[1]) + " \n range z: " + str(ranges[2])
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        fig.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14, verticalalignment='top', bbox=props)
        axis_range_all = 1.1
        # axis_range = [-axis_range_all, axis_range_all, -axis_range_all, axis_range_all, -axis_range_all, axis_range_all]
        # axes = Axes3D(fig)
        ax.set_xlim3d(-axis_range_all, axis_range_all)
        ax.set_ylim3d(-axis_range_all, axis_range_all)
        ax.set_zlim3d(-axis_range_all, axis_range_all)
        fig.show()

    def plot_all_imu_data(self):
        self.plot_rpy()
        self.plot_acc()
        self.plot_gyro()
        self.plot_mag()
        self.plot_mag_circles()
        # self.plot_mag_surface()
        # self.plot_mag_scatter()

    def downsample(self, input_array, frequency):
        return_list = list(islice(input_array, 0, len(input_array), frequency))
        return_list.append(input_array[-1])  # make sure the last entry is inside the list
        return return_list

if __name__ == '__main__':

    file_name_of_data_default = "um7_data.npy"
    path_of_file_save_default = ""
    imu_subs_topic_name_default = '/sensor/imu/um7/data'  #'/sensor/imu/xIMU/data' # /sensor/imu/razor_imu/data # /sensor/imu/xsens_mti/data
    mag_subs_topic_name_default = '/sensor/imu/um7/magfield_msg'  # '/sensor/imu/xIMU/magfield_msg' # /sensor/imu/razor_imu/mag_calib # /sensor/imu/xsens_mti/mag_calib
    rospy.init_node("capture_imu_data")

    # read parameters
    imu_subs_topic_name = rospy.get_param('imu_data_topic_name', imu_subs_topic_name_default)
    mag_subs_topic_name = rospy.get_param('magnetic_field_data_topic_name', mag_subs_topic_name_default)
    file_name_of_data = rospy.get_param('file_save_name', file_name_of_data_default)
    path_of_file_save = rospy.get_param('file_save_path', path_of_file_save_default)

    # start data capture
    imu_data_cap_obj = imu_data_capture(imu_subs_topic_name, mag_subs_topic_name)
    rospy.spin()
    # save data to HDD
    np.save(path_of_file_save+file_name_of_data,imu_data_cap_obj.data_dic)
