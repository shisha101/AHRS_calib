import sys, getopt
from imu_calibration.imu_mag_calibration import imu_mag_calibration
import numpy as np


if __name__ == '__main__':
    file_name_of_data_default = "Xsens_data_onboard_office.npy"
    path_of_file_load_default = "/home/abs8rng/catkin_ws/src/my_packages/low_cost_navigation/evaluation_code/data/imu_recordings/"
    file_name_of_output_default = "some"
    opt_sting = "phi:o:"
    tutorial_string = "magnetometer_calibration.py -i <input_file_name(.npy)> -o <output_file_name(.yaml)> -p <path_of_input_file>"

    file_name_of_data = file_name_of_data_default
    path_of_file_load = path_of_file_load_default
    file_name_of_output = file_name_of_output_default
    try:
        opts, args = getopt.getopt(sys.argv[1:], opt_sting)

    except getopt.GetoptError as err:
        print tutorial_string
        print err
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-h":
            print tutorial_string
            sys.exit()
        elif opt == "-i":
            file_name_of_data = arg
        elif opt == "-o":
            file_name_of_output = arg
        elif opt == "-p":
            path_of_file_load = arg
        else:
            print "incorrect option received"
            print tutorial_string

    print "input data file = " + file_name_of_data
    print "input path of data file = " + path_of_file_load_default
    print "calibration output file = " + file_name_of_output

    dic_from_drive = np.load(path_of_file_load + file_name_of_data).item()
    imu_mag_calib_obj = imu_mag_calibration(dic_from_drive)
    imu_mag_calib_obj.calculate_calib_parameters_hard_iron()
    imu_mag_calib_obj.calculate_calib_parameters_x_sqr()
    imu_mag_calib_obj.dump_to_file(file_name_of_output)
    # imu_mag_calib_obj.calculate_calib_parameters_norm()
