import sys
from imu_calibration.imu_mag_calibration import imu_mag_calibration
import numpy as np


if __name__ == '__main__':
    file_name_of_data_default = "Xsens_data_onboard_office.npy"
    path_of_file_load_default = "/home/abs8rng/catkin_ws/src/my_packages/low_cost_navigation/evaluation_code/data/imu_recordings/"
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
    imu_mag_calib_obj = imu_mag_calibration(dic_from_drive)
    imu_mag_calib_obj.calculate_calib_parameters_hard_iron()
    imu_mag_calib_obj.calculate_calib_parameters_x_sqr()
    imu_mag_calib_obj.dump_to_file()
    # imu_mag_calib_obj.calculate_calib_parameters_norm()
