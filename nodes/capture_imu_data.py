#!/usr/bin/env python
import rospy
# import evaluation_code.src.imu_calib_modu.imu_calib_modu
import imu_calibration.imu_calibration_visualization as imu_calib
# import evaluation_code.src.imu_calib_modu
import numpy as np

if __name__ == '__main__':

    file_name_of_data_default = "some_imu_data_sdk_calib_tt.npy"
    path_of_file_save_default = ""
    imu_subs_topic_name_default = '/sensor/imu/xsens_mti/data'  #'/sensor/imu/xIMU/data' # /sensor/imu/razor_imu/data # /sensor/imu/xsens_mti/data # /sensor/imu/um7/data
    mag_subs_topic_name_default = '/sensor/imu/xsens_mti/mag_calib'  # '/sensor/imu/xIMU/magfield_msg' # /sensor/imu/razor_imu/mag_calib # /sensor/imu/xsens_mti/mag_calib # /sensor/imu/um7/magfield_msg
    rospy.init_node("capture_imu_data")

    # read parameters
    file_name_of_data = rospy.get_param("~file_name_of_data", file_name_of_data_default)
    path_of_file_save = rospy.get_param("~path_of_file_save", path_of_file_save_default)
    imu_subs_topic_name = rospy.get_param("~imu_subscription_topic_name", imu_subs_topic_name_default)
    mag_subs_topic_name = rospy.get_param("~mag_subscription_topic_name", mag_subs_topic_name_default)

    # start data capture
    imu_data_cap_obj = imu_calib.imu_data_capture(imu_subs_topic_name, mag_subs_topic_name)
    rospy.spin()
    # save data to HDD
    np.save(path_of_file_save + file_name_of_data, imu_data_cap_obj.data_dic)
    print "file saved to: \n " + path_of_file_save+file_name_of_data
