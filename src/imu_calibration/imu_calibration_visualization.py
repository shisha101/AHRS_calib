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

    def __init__(self, dict_of_values, absolute_time=False, bias_simple=None, sensitivity=None, bias_w_sensitivity=None):
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
        np_array = np.array([self.data["mag"][0], self.data["mag"][1], self.data["mag"][2]])
        # np*np element wise mul sum(axis=0) sum all squared components
        self.data["mag_magnitude"] = np.sqrt((np_array*np_array).sum(axis=0))

        if bias_simple is not None:
            # do correction
            self.bias_cor_val, self.bias_cor_magnitude = self.calculate_calibrated_magnetometer_values(bias_simple)
        else:
            self.bias_cor_val = None
            self.bias_cor_magnitude = None
        if bias_w_sensitivity is not None and sensitivity is not None:
            # do correction
            self.sens_cor_val, self.sens_cor_magnitude = self.calculate_calibrated_magnetometer_values(bias_simple,
                                                                                                       sensitivity)
        else:
            self.sens_cor_val = None
            self.sens_cor_magnitude = None

    def calc_min_max(self, data_dict_entry):
        min_max_dict = {"min_x": min(self.data[data_dict_entry][0]), "max_x": max(self.data[data_dict_entry][0]),
                        "min_y": min(self.data[data_dict_entry][1]), "max_y": max(self.data[data_dict_entry][1]),
                        "min_z": min(self.data[data_dict_entry][2]), "max_z": max(self.data[data_dict_entry][2])}
        return min_max_dict

    def calculate_calibrated_magnetometer_values(self, bias, sensitivity=[1.0, 1.0, 1.0]):
        measurement_matrix = np.matrix(self.data["mag"]).T   # create nx3 matrix from mag measurements
        b = np.array(bias)
        sens_array = np.array(sensitivity)
        bias_corrected_measurements = measurement_matrix - b  # element wise subtraction (bias correction)
        # sensitivity correction element wise multiplication
        corrected_values = np.multiply(bias_corrected_measurements, sens_array)
        magnitude_sq = np.multiply(corrected_values, corrected_values).sum(axis=1)
        magnitude = np.sqrt(magnitude_sq)  # magnitude of mag readings after correction

        magnitude = magnitude.tolist()
        corrected_values = np.asarray(corrected_values).T

        return corrected_values, magnitude


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
        if self.check_data_availability("orient_rpy"):
            axis_names = ["roll", "pitch", "yaw"]
            self.plot_imu(self.data["time_imu"], self.data["orient_rpy"], axis_names)

    def plot_acc(self):
        if self.check_data_availability("acc"):
            axis_names = ["acc_x", "acc_y", "acc_z"]
            self.plot_imu(self.data["time_imu"], self.data["acc"], axis_names)

    def plot_gyro(self):
        if self.check_data_availability("gyro"):
            axis_names = ["gyro_x", "gyro_y", "gyro_z"]
            self.plot_imu(self.data["time_imu"], self.data["gyro"], axis_names)

    def plot_mag(self):
       if self.check_data_availability("mag"):
            axis_names = ["mag_x", "mag_y", "mag_z"]
            self.plot_imu(self.data["time_mag"], self.data["mag"], axis_names)

    def plot_mag_circles_all(self):
        axis_names = ["normal to X", "normal to Y", "normal to Z", "magnitude_mag_all_axis"]
        if self.check_data_availability("mag"):
            self.plot_mag_circles(self.data["mag"], self.data["mag_magnitude"], axis_names, mag_normalizer=max(self.data["mag_magnitude"]))
        if self.sens_cor_val is not None:
            self.plot_mag_circles(self.sens_cor_val, self.sens_cor_magnitude, axis_names, mag_normalizer=max(self.sens_cor_magnitude))
        if self.bias_cor_val is not None:
            self.plot_mag_circles(self.bias_cor_val, self.bias_cor_magnitude, axis_names, mag_normalizer=max(self.bias_cor_magnitude))
            # print self.bias_cor_magnitude

    def plot_mag_circles(self, data_entry_matrix, data_magnitude_vector, axis_names, mag_normalizer=1.0):
        fig = plt.figure()
        avg_mag = np.average(data_magnitude_vector)
        plot_1 = fig.add_subplot(411)
        self.draw_mag_2d_circle(plot_1, data_entry_matrix[1], data_entry_matrix[2], avg_mag)

        plot_2 = fig.add_subplot(412)
        self.draw_mag_2d_circle(plot_2, data_entry_matrix[0], data_entry_matrix[2], avg_mag)

        plot_3 = fig.add_subplot(413)
        self.draw_mag_2d_circle(plot_3, data_entry_matrix[0], data_entry_matrix[1], avg_mag)

        plot_4 = fig.add_subplot(414)
        plot_4.plot(self.data["time_mag"], np.array(data_magnitude_vector)/mag_normalizer)
        # axes operations
        plot_1.set_xlabel(axis_names[0])
        plot_1.axis("equal")
        plot_2.set_xlabel(axis_names[1])
        plot_2.axis("equal")
        plot_3.set_xlabel(axis_names[2])
        plot_3.axis("equal")
        plot_4.set_xlabel(axis_names[3])

        fig.tight_layout()
        fig.show()


    def draw_mag_2d_circle(self, sub_plot_handle, x_axis_data, y_axis_data, radius , line_width=5):
        sub_plot_handle.add_artist(plt.Circle((0, 0), radius, color='r', fill=False, linewidth=line_width, alpha=0.5))
        sub_plot_handle.plot(x_axis_data, y_axis_data)

    def plot_mag_scatter_raw(self, downsampling_step=10):
        if self.check_data_availability("mag"):
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
        if self.check_data_availability("mag"):
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            # normalize vectors
            np_array = np.array([self.data["mag"][0], self.data["mag"][1], self.data["mag"][2]])
            norm_coordinates = np_array/self.data["mag_magnitude"]

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
        self.plot_mag_circles_all()
        # self.plot_mag_surface()
        # self.plot_mag_scatter()

    def downsample(self, input_array, frequency):
        return_list = list(islice(input_array, 0, len(input_array), frequency))
        return_list.append(input_array[-1])  # make sure the last entry is inside the list
        return return_list

    def check_data_availability(self, dict_entry):
        if len(self.data[dict_entry][0]) and len(self.data[dict_entry][1]) and len(self.data[dict_entry][2]):
            return True
        print ("no %s data to plot, skipping" % dict_entry)
        return False
