from capture_imu_data import plot_imu_data
import numpy as np

if __name__ == '__main__':
    file_name_of_data = "um7_data.npy"
    path_of_file_save = ""
    dic_from_drive = np.load(path_of_file_save+file_name_of_data).item()
    plot_obj = plot_imu_data(dic_from_drive)
    plot_obj.plot_all_imu_data()
    plot_obj.plot_mag_scatter_raw()
    plot_obj.plot_norm_mag_vs_sphere(sphere_radius=1.0, plot_sphere=True) # should be used post calibration
    raw_input("press to exit")
