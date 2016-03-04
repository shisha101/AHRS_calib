#!/usr/bin/env python
import sys, getopt
import os
import yaml
import numpy as np
# sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
sys.path.append(parent_dir)
from src.imu_calibration.imu_calibration_visualization import plot_imu_data

if __name__ == '__main__':
    file_name_of_data_default = "um7_data_onboard_office.npy"
    path_of_file_load_default = "../data/imu_recordings/"
    imu_name = "um7"
    tut_string = "plot_imu_data.py -i <input_imu_file_name(.npy)> -n <imu_name> -p <path_where_to_find_input_file>"
    opt_string = "hp:i:n:"
    file_name_of_data = file_name_of_data_default
    path_of_file_load = path_of_file_load_default

    try:
        opts, arg = getopt.getopt(sys.argv[1:], opt_string)
    except getopt.GetoptError as err:
        print err
        print tut_string
    for opt, arg in opts:
        if opt == "-h":
            print tut_string
            sys.exit()
        elif opt == "-i":
            file_name_of_data = arg
        elif opt == "-p":
            path_of_file_load = arg
        elif "-n":
            imu_name = arg
        else:
            print tut_string
            assert False, "unhandled option"

    print "input data file = " + file_name_of_data
    print "input path of data file = " + path_of_file_load

    mag_config_file_path = "../configs/"+imu_name+"_imu_mag_config.yaml"
    if os.path.isfile(path_of_file_load + file_name_of_data):
        dic_from_drive = np.load(path_of_file_load + file_name_of_data).item()
        if os.path.isfile(mag_config_file_path):
            print "config file for magnetometer has been found"
            print "config file path = " + mag_config_file_path
            mag_calib_params = yaml.load(file("../configs/"+imu_name+"_imu_mag_config.yaml"))
            plot_obj = plot_imu_data(dic_from_drive,
                                   bias_simple=mag_calib_params["bias_simple"],
                                   sensitivity=mag_calib_params["sensitivities"],
                                   bias_w_sensitivity=mag_calib_params["bias_w_sensitivity"])
        else:
            print "NO config file for magnetometer has been found"
            print "Could NOT find " + mag_config_file_path
            plot_obj = plot_imu_data(dic_from_drive)

        plot_obj.plot_all_imu_data()
        plot_obj.plot_mag_scatter_raw()
        plot_obj.plot_norm_mag_vs_sphere(sphere_radius=1.0, plot_sphere=True)  # should be used post calibration
        print "Not that the raw sensor data acc, gyro, and mag are present only on the original sensor topics and not" \
              " onthe filtered madgwick data topic. Since they are exactly the same"
        raw_input("press to exit")
    else:
        print "No the given file has not been found"
        print path_of_file_load + file_name_of_data
