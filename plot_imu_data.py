from capture_imu_data import plot_imu_data
import numpy as np
import sys

if __name__ == '__main__':
    file_name_of_data_default = "Xsens_data_onboard.npy"
    path_of_file_load_default = "../data/imu_recordings/"
    if len(sys.argv) < 2:
        print "no file specified. Using default value"
        file_name_of_data = file_name_of_data_default
        path_of_file_load = path_of_file_load_default
    else:
        file_name_of_data = sys.argv[1]
        if len(sys.argv) == 3:
            path_of_file_load = sys.argv[2]
        else:
            path_of_file_load = path_of_file_load_default

    dic_from_drive = np.load(path_of_file_load + file_name_of_data).item()
    plot_obj = plot_imu_data(dic_from_drive)
    plot_obj.plot_all_imu_data()
    plot_obj.plot_mag_scatter_raw()
    plot_obj.plot_norm_mag_vs_sphere(sphere_radius=1.0, plot_sphere=True) # should be used post calibration
    print "Not that the raw sensor data acc, gyro, and mag are present only on the original sensor topics and not on" \
          "the filtered madgwick data topic. Since they are exactly the same"
    raw_input("press to exit")
