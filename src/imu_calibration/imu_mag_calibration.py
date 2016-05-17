import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations


class ImuMagCalibration(object):
    def __init__(self, data_dict):
        """
        :param data_dict: a dictionary containing the imu data, must contain mag data see capture_imu_data.py
        """
        self.dict_entry = "mag"
        self.data_dict = data_dict
        self.calib_values = {"sensitivities": None, "bias_w_sensitivity": None, "bias_simple": None,
                             "elipsoid_transformation": None, "elipsoid_center": None}
        self.number_of_points = len(self.data_dict[self.dict_entry][0])
        self.ones_vector = np.ones(self.number_of_points)

    def create_reading_matrix_x_sqr(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        # np_mag_values = self.normalize_readings()
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, -1.0 * squared_values[1:3], self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        # print self.data_dict[self.dict_entry][0][0:5]
        # print self.data_dict[self.dict_entry][1][0:5]
        # print self.data_dict[self.dict_entry][2][0:5]
        # print squared_values[0][0:5]
        # print squared_values[1][0:5]
        # print squared_values[2][0:5]
        # print measurement_matrix[0:7]
        # print measurement_matrix.shape
        # print np.linalg.cond(measurement_matrix.T * measurement_matrix)
        # print np.linalg.cond(measurement_matrix)
        # print np.linalg.matrix_rank(measurement_matrix)
        print "********"
        return measurement_matrix, squared_values

    def calculate_calib_parameters_x_sqr(self):
        measurement_matrix, sq_vals_np = self.create_reading_matrix_x_sqr()
        print "*********** sensitivity and bias calibration (3D only) *******"
        # pinv = np.linalg.pinv(measurement_matrix)
        print "the number of points used is: %i" % self.number_of_points
        print "the condition number of the measurement matrix is %f " % np.linalg.cond(measurement_matrix)
        # print "Hi"
        # print pinv.shape
        # print np.matrix(self.ones_vector).shape
        # print np.linalg.cond(pinv)
        # parameters = pinv * np.matrix(sq_vals_np[0]).T
        parameters, residual, p, pp = np.linalg.lstsq(measurement_matrix,  np.matrix(sq_vals_np[0]).T)
        parameters = np.squeeze(np.asarray(parameters))  # convert from matrix to array
        parameters = parameters.tolist()
        # print parameters

        # print "the parameters"
        # print parameters
        # print parameters_1
        # self.calib_values["sensitivities"] = np.sqrt(parameters[3:6])
        # print type(self.calib_values["sensitivities"])
        # print self.calib_values["sensitivities"].shape
        # print self.calib_values["sensitivities"]
        # print np.sqrt(parameters[3:5])
        bias = []
        bias.append(parameters[0]/2)
        bias.append(parameters[1]/(2*parameters[3]))
        bias.append(parameters[2]/(2*parameters[4]))
        self.calib_values["bias_w_sensitivity"] = bias
        sensitivity = []
        A = parameters[5] + bias[0]**2 + parameters[3] * bias[1]**2 + parameters[4] * bias[2]**2
        B = A / parameters[3]
        C = A / parameters[4]
        print "before SQRT", A, B, C
        sensitivity.append(np.sqrt(1/A).tolist())
        sensitivity.append(np.sqrt(1/B).tolist())
        sensitivity.append(np.sqrt(1/C).tolist())
        self.calib_values["sensitivities"] = sensitivity
        print "the scaling factors for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["sensitivities"][0],
                                                                               self.calib_values["sensitivities"][1],
                                                                               self.calib_values["sensitivities"][2])
        print "the biases for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["bias_w_sensitivity"][0],
                                                                      self.calib_values["bias_w_sensitivity"][1],
                                                                      self.calib_values["bias_w_sensitivity"][2])
        print "the quality without calibration is %s positive and negative percent off > |4| is bad" % (max(abs(np.array(bias))) * max(sensitivity))
        print "the residual is %s" % (residual[0, 0]/(self.number_of_points))
        # self.debug_magnitude(measurement_matrix, bias, sensitivity)

    def calculate_calib_parameters_pos_scale(self):
        # real data
        mag_readings_np = np.array(self.data_dict[self.dict_entry])
        x = mag_readings_np[0]
        y = mag_readings_np[1]
        z = mag_readings_np[2]

        # number_of_points  = 256
        # phi = np.linspace(0,2*np.pi, number_of_points).reshape(number_of_points, 1) # the angle of the projection in the xy-plane
        # theta = np.linspace(0, np.pi, number_of_points).reshape(-1, number_of_points) # the angle from the polar axis, ie the polar angle
        # ra_x = 0.5
        # ra_y = 0.7
        # ra_z = 0.6
        # x = ra_x*np.sin(theta)*np.cos(phi)
        # y = ra_y*np.sin(theta)*np.sin(phi)
        # z = ra_z*np.cos(theta) #+ 0.1*ra_z*np.random.rand(len(x))
        # print "shapes of data x y z theta phi", x.shape, y.shape, z.shape, theta.shape, phi.shape
        # print "types of data x y z theta phi", type(x), type(y), type(z), type(theta), type(phi)
        #
        # fig = plt.figure()  # Square figure
        # ax = fig.add_subplot(111, projection='3d')
        # ax.plot_surface(x, y, z, color='b')
        # fig.show()
        #
        #
        # x = ra_x*np.sin(np.linspace(0, np.pi, number_of_points))*np.cos(phi)
        # y = ra_x*np.sin(np.linspace(0, np.pi, number_of_points))*np.sin(phi)
        # mag_readings_np = np.vstack((x,y,z))
        # self.ones_vector = np.zeros(x.shape)
        # print self.ones_vector.shape

        # dummy data
        # ra_x = 1.0
        # ra_y = 1.0
        # ra_z = 0.0
        # number_of_points = 256
        # theta = np.linspace(0,2*np.pi, number_of_points)
        # phi = np.linspace(0,2*np.pi, number_of_points)
        # x = (ra_x*np.sin(theta)*np.cos(phi)).T
        # y = (ra_y*np.sin(theta)*np.sin(phi)+ 0.01*ra_z*np.random.rand(len(x))).T
        # z = (ra_z*np.cos(theta) + 0.01*ra_z*np.random.rand(len(x))).T
        # print "shapes of data x y z theta phi", x.shape, y.shape, z.shape
        # mag_readings_np = np.vstack((x,y,z))
        # self.ones_vector = np.zeros(mag_readings_np.shape[1])


        # fig = plt.figure("hi")
        # ax = fig.add_subplot(111, projection='3d')
        # ax.scatter(x, y, z, c="b", marker="o")
        # fig.show()
        # raw_input()
        # plt.close()


        yz_2 = 2*mag_readings_np[1]*mag_readings_np[2]
        xz_2 = 2*mag_readings_np[0]*mag_readings_np[2]
        xy_2 = 2*mag_readings_np[0]*mag_readings_np[1]
        # D_mat = np.asmatrix(np.vstack((mag_readings_np**2, yz_2, xz_2, xy_2,2*mag_readings_np, self.ones_vector))).T
        # m_mat = np.asmatrix(np.vstack((mag_readings_np**2, yz_2, xz_2, xy_2,2*mag_readings_np))).T
        # print "m_mat", m_mat.shape
        # parameters, residual, p, pp = np.linalg.lstsq(m_mat,  self.ones_vector)
        # print "params", parameters, "residual", residual

        print "CONSTRAINED PAPER 3D Variable K"
        D_mat = np.asmatrix(np.vstack((mag_readings_np**2, yz_2, xz_2, xy_2, 2*mag_readings_np, self.ones_vector))).T
        print "D_mat", D_mat.shape
        k = 4.0
        k_2_1 = k/2 - 1.0
        D_tD = D_mat.T * D_mat
        c = np.zeros((6,6))
        c11 = np.matrix([[-1.0, k_2_1, k_2_1 ],[k_2_1, -1, k_2_1],[k_2_1, k_2_1, -1]])
        c22 = np.diag([-k, -k, -k])
        c[0:3, 0:3] = c11
        c[3:6, 3:6] = c22
        C = np.zeros((10, 10))
        C[0:6, 0:6] = c
        print "c_1 \n", c
        # print "C matrix \n", C
        # print np.linalg.eig(C)
        # e_vals, e_vect = np.linalg.eig(np.linalg.inv(D_tD)*C)
        # print 'the cond num is ', np.linalg.cond(np.linalg.inv(D_tD)*C)
        # print "evals", e_vals
        # # print "evect", e_vect
        # print "evect \n", e_vect[:,np.argmax(e_vals)]

        print "new method"
        s11 = D_tD[0:6, 0:6]
        s12 = D_tD[0:6, 6:10]
        # s21 = s12.T
        s22 = D_tD[6:10,6:10]
        if np.linalg.cond(s22) > 10000:
            inv_s22 = np.linalg.pinv(s22)
            print "using puesodo inverse"
        else:
            inv_s22 = np.linalg.inv(s22)
        v2 = inv_s22 * s12.T
        M = np.linalg.inv(c) * (s11 - s12*v2)
        eig_v, eig_vec = np.linalg.eig(M)
        print eig_v
        print eig_vec
        index_of_max_eig_v = np.argmax(eig_v)
        u1 = eig_vec[:, index_of_max_eig_v]
        u2 = -v2 * u1
        param_ = np.vstack((u1,u2))
        # param_ = param_/param_[-1]
        print index_of_max_eig_v
        if param_[0]<0 and param_[1]<0 and param_[2]<0:
            param_ *= -1
            print "reveresed direction of vector"
        # print u1
        print param_
        print param_.shape
        print "a.T * C * a", param_.T * C * param_
        # param_ = param_ * 1/np.sqrt(param_.T * C * param_)
        # print "a.T * C * a", param_.T * C * param_
        print "the cond num is", np.linalg.cond(s22)

        j = param_[0]*param_[1] + param_[1]*param_[2] + param_[0]*param_[2] - (param_[4]**2 + param_[5]**2 + param_[6]**2)
        i = param_[0] +param_[0] + param_[0]
        p = (4.0*j - i**2)/(param_[0]**2 + param_[1]**2 + param_[2]**2)
        print "p is ", p

        param_array = np.squeeze(np.asarray(param_))
        A_mat = np.matrix([[param_array[0], param_array[5], param_array[4]],
                           [param_array[5], param_array[1], param_array[3]],
                           [param_array[4],param_array[3], param_array[2]]])
        rx = np.sqrt(1/abs(param_[0,0]))
        ry = np.sqrt(1/abs(param_[1,0]))
        rz = np.sqrt(1/abs(param_[2,0]))
        center = -0.5*np.linalg.inv(A_mat)*np.matrix([[2*param_array[6]], [2*param_array[7]], [2*param_array[8]]])
        print "the center is:", center.T
        print "xAxis_r: %f, yAxis_r: %f, zAxis_r: %f" % (rx, ry, rz)
        print "magnitude: %f" % param_array[-1]
        e_v, e_vec = np.linalg.eig(A_mat)
        print "the A matrix \n", A_mat
        print "A eig values \n", e_v
        print "A eig vectors \n", e_vec
        print "proof of transformation \n", e_vec*np.diag(e_v) - A_mat*e_vec
        # print "proof of transformation", np.linalg.inv(e_vec)*A_mat*e_vec

        transformation_mat = e_vec*(np.sqrt(np.diag(e_v)) * e_vec.T)  # Best, note however that the proof for this is missig
        # transformation_mat = np.sqrt(np.diag(e_v)) * e_vec.T#np.linalg.inv(e_vec)
        e_vec = np.linalg.inv(e_vec)
        print "scaling \n", np.sqrt(np.diag(e_v))
        print "rotation \n", e_vec
        r,p,y = tf.transformations.euler_from_matrix(np.linalg.inv(e_vec))
        print "euler angels", r * 180.0/np.pi, p * 180.0/np.pi, y * 180.0/np.pi
        print "rank of eigen vectors: ", np.linalg.matrix_rank(e_vec)

        print "test\n",

        print "transformation mat"
        print transformation_mat

        # tests
        # no_bais =


        print "END OF PAPER"
        transformation_to_list = transformation_mat.reshape(1,9).tolist()[0]
        print "transformation to list", transformation_to_list
        self.calib_values["elipsoid_transformation"] = transformation_mat.reshape(1,9).tolist()[0]
        self.calib_values["elipsoid_center"] = center.T.tolist()[0]
        print self.calib_values["elipsoid_transformation"]
        print self.calib_values["elipsoid_center"]
        print(len(self.calib_values["elipsoid_center"]))



        # D_tD = D_mat.T * D_mat
        # print D_mat.shape
        # print D_tD.shape
        # print mag_readings_np.shape
        # e_vals, e_vec = np.linalg.eig(D_tD)
        # print e_vals, np.argmax(e_vals)
        # print e_vec[np.argmax(e_vals)]
        # parameter_vector = np.asarray(e_vec[np.argmax(e_vals)]).squeeze().T
        # print parameter_vector.shape
        # print type(parameter_vector)
        # print parameter_vector[0]
        # print type(parameter_vector[0])
        # A_mat = np.matrix([[parameter_vector[0], parameter_vector[5], parameter_vector[4]],
        #                    [parameter_vector[5], parameter_vector[1], parameter_vector[3]],
        #                    [parameter_vector[4],parameter_vector[3], parameter_vector[2]]])
        # print "The A matrix",A_mat
        # A_eig_val, A_eig_vec = np.linalg.eig(A_mat)
        # print "A eig vals and eig vects", A_eig_val, A_eig_vec
        # print "rank of A", np.linalg.matrix_rank(A_mat)
        # print "**"
        # print np.linalg.inv(A_eig_vec)*A_mat*A_eig_vec
        # trans_matrix = np.diag(np.sqrt(A_eig_val))*A_eig_vec
        # print trans_matrix
        # # print np.linalg.matrix_rank(A_mat)
        # self.calib_values["sensitivities_2step"] = [1, 1, 1]
        pass
        # print "hi"
        # # alternative apporach
        # np_mag_values = np.array(self.data_dict[self.dict_entry])
        # magnitude_vector = np.sum(np_mag_values ** 2, axis=0)
        # # print magnitude_vector.size
        # # print magnitude_vector[0:5]
        # # print np_mag_values[:,0:1]
        # # print np_mag_values[:,0:1] ** 2
        # # print np.sum(np_mag_values[:,0:1] ** 2, axis=0)
        # average_mag = np.mean(np.sqrt(magnitude_vector))
        # print average_mag
        # np_mag_values_sq = np_mag_values ** 2
        # print np_mag_values_sq.T.shape
        # print np.matrix(self.ones_vector).T.shape
        # parameters, residual, p, pp = np.linalg.lstsq(np_mag_values_sq.T,  np.matrix(self.ones_vector).T)
        # print "parameters: %s radius:%s residual %s" % (np.sqrt(parameters), average_mag, residual)
        # print "END"


        # call position calibration
        # magnitude = self.calculate_calib_parameters_hard_iron()
        # np_mag_values = np.array(self.data_dict[self.dict_entry])
        # bias_corrected_measurements = np_mag_values.T - np.array(self.calib_values["bias_simple"])
        #
        # # print np_mag_values[0:5]
        # # print bias_corrected_measurements[0:5], bias_corrected_measurements.shape
        # mag_values_sq = bias_corrected_measurements ** 2
        # average_z = np.mean(bias_corrected_measurements[:, 2])
        # # print average_z
        # radius_to_scale_to = magnitude**2 - average_z**2
        # y_vector = self.ones_vector * radius_to_scale_to
        #
        # parameters, residual, p, pp = np.linalg.lstsq(mag_values_sq[:, 0:2],  np.matrix(y_vector).T)
        # parameters = np.sqrt(parameters).tolist()
        # x_scaling = parameters[0][0]
        # y_scaling = parameters[1][0]
        # z_scaling = 1.0
        # print "*********** Two stage calibration (2D only)*******"
        # print x_scaling, y_scaling, z_scaling
        # self.calib_values["sensitivities_2step"] = [x_scaling, y_scaling, z_scaling]
        # print self.calib_values["sensitivities_2step"]
        # # print bias_corrected_measurements[:, 0:2].shape, np.matrix(y_vector).T.shape
        # print "parameters: %s average z: %s radius:%s residual %s" % (np.sqrt(parameters), average_z, radius_to_scale_to, residual)
        # # b = np.array(self.calib_values["bias_simple"])
        # sens_array = np.array(sensitivity)
        # bias_corrected_measurements = measurement_matrix - b  # element wise subtraction (bias correction)

    def debug_magnitude(self, measurement_matrix, bias, sensitivity):
        b = np.array(bias)
        sens_array = np.array(sensitivity)
        bias_corrected_measurements = measurement_matrix[:, 0:3] - b
        corrected_values = np.multiply(bias_corrected_measurements, sens_array)
        magnitude_sq = np.multiply(corrected_values, corrected_values).sum(axis=1)
        magnitude = np.sqrt(magnitude_sq)
        print magnitude
        print "the average magnitude after projection is: %f " % np.average(magnitude)
        print "debug"
        print "bias"
        print b
        print "sensitivity"
        print sens_array
        print "raw data"
        print measurement_matrix[0:3, 0:3]
        print "bias corrected"
        print bias_corrected_measurements[0:3, :]
        print "corrected"
        print corrected_values[0:3, :]
        print "mag_sq"
        print magnitude_sq[0:3, :]
        print "magnitude"
        print magnitude[0:3]

    def create_reading_matrix_hard_iron(self):
        np_mag_values = np.array(self.data_dict[self.dict_entry])
        squared_values = np_mag_values * np_mag_values  # element wise multiplication correct
        measurement_matrix = np.vstack((np_mag_values, self.ones_vector))
        measurement_matrix = np.matrix(measurement_matrix.T)
        return measurement_matrix, squared_values

    def calculate_calib_parameters_hard_iron(self):
        measurement_matrix, sq_vals_np = self.create_reading_matrix_hard_iron()
        print "*********** Bias only calibration *******"
        print "the number of points used is: %i" % self.number_of_points
        print "the condition number of the measurement matrix is %f " % np.linalg.cond(measurement_matrix)
        # pinv = np.linalg.pinv(measurement_matrix)
        # print np.matrix(sq_vals_np.sum(axis=0)).T.shape
        # parameters = pinv * np.matrix(sq_vals_np.sum(axis=0)).T
        parameters, residual, p, pp = np.linalg.lstsq(measurement_matrix,  np.matrix(sq_vals_np.sum(axis=0)).T)
        parameters = np.squeeze(np.asarray(parameters))  # convert from matrix to array
        bias = (parameters[0:3]/2.0).tolist()
        self.calib_values["bias_simple"] = bias
        print "the scaling factors for the magnetometer are x: %s, y: %s, z: %s" % (1.0, 1.0, 1.0)
        print "the biases for the magnetometer are x: %s, y: %s, z: %s" % (self.calib_values["bias_simple"][0],
                                                                      self.calib_values["bias_simple"][1],
                                                                      self.calib_values["bias_simple"][2])
        magnetometer_vector_magnitude = np.sqrt(np.square(bias).sum() + parameters[3])
        print "the magnitude vector of the magnetometer is :%s" % magnetometer_vector_magnitude
        print "the quality without calibration is %s percent off > 4 is bad" % ((max(abs(np.array(bias)))/magnetometer_vector_magnitude)*100)
        print "the residual is %s" % (residual[0, 0]/(self.number_of_points))
        return magnetometer_vector_magnitude

    def check(self):
        pass

    def dump_to_file(self, name):
        with open("../configs/"+name+"_imu_mag_config.yaml", 'w+') as f:
            f.write('# autogenerated: Do not edit #\n')
            f.write(yaml.dump(self.calib_values))

