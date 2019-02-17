#!/usr/bin/env python3

import rospy
import numpy as np
import random

random.seed()
g_noise_gain = 0.1

if __name__ == '__main__':
    rospy.init_node('example_eigen_plane_fit')
    rate = rospy.Rate(1.0)

    normal_vec = np.array([[1], [2], [3]])
    rospy.loginfo("creating example noisy, planar data...")
    print("normal: ")
    normal_vec = normal_vec.T
    print(normal_vec)

    normal_vec = normal_vec / (np.linalg.norm(normal_vec))

    print("unit length normal: ")
    normal_vec = normal_vec.T
    print(normal_vec)

    dist = 1.23
    print("plane distance from origin: ")
    print(dist)

    v1 = np.ones((3, 1))
    v2 = np.ones((3, 1))
    Rot_z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    print("Rot_z: ")
    print(Rot_z)

    # ROS_INFO_STREAM
    rospy.loginfo(Rot_z)

    v1 = Rot_z @ normal_vec

    # ROS_INFO_STREAM
    rospy.loginfo(v1.T)

    print("v1: ")
    v1 = v1.transpose()
    print(v1)

    dotprod = v1.dot(normal_vec)
    dotprod2 = v1 @ normal_vec

    print("v1 dot normal: ")
    print(dotprod)
    print("v1.transpose() * normal_vec: ")
    print(dotprod2)
    print("(should be identical)")

    v2 = np.cross(v1, normal_vec.T)
    v2 = v2 / (np.linalg.norm(v2))

    print("v2: ")
    print(v2.T)

    dotprod = v2.dot(normal_vec)

    print("v2 dot normal_vec = ")
    print(dotprod)
    print(" (should be zero)")

    v1 = np.cross(v2, normal_vec.T)

    print("v1 = ")
    print(v1.transpose())
    print(" v1 dot v2 = ")
    print(v1.dot(v2.T))
    print("v1 dot normal_vec = ")
    print(v1.dot(normal_vec))
    print("(these should also be zero)")

    npts = 10
    points_mat = np.ones((3, npts))
    point = np.zeros((3, 1))
    rand_vec = np.ones((2, 1))

    for ipt in range(npts):
        rand_vec = np.random.random((2, 1))
        point = rand_vec[0] * v1.T + rand_vec[1] * v2.T + dist * normal_vec
        #     print(points_mat[:, ipt])
        points_mat[:, ipt] = point.T

    print("random points on plane (in columns): ")
    print(points_mat)

    Noise = np.random.random((3, npts))
    print("noise_gain = " + str(g_noise_gain) + "; edit this as desired")

    points_mat = points_mat + Noise * g_noise_gain
    print("random points on plane (in columns) w/ noise: ")
    print(points_mat)

    rospy.loginfo("starting identification of plane from data: ")

    centroid = np.zeros((3, 1))
    centroid = centroid.T

    mpts, npts = points_mat.shape

    print("matrix has ncols = ")
    print(npts)

    for ipt in range(npts):
        #     print(points_mat[:, ipt])
        centroid = centroid + points_mat[:, ipt]

    centroid = centroid / npts
    print("centroid: ")
    print(centroid)

    points_offset_mat = points_mat
    for ipt in range(npts):
        points_offset_mat[:, ipt] = points_offset_mat[:, ipt] - centroid

    CoVar = np.ones((3, npts))
    CoVar = points_offset_mat @ (points_offset_mat.transpose())
    print("covariance: ")
    print(CoVar)

    es3d = CoVar
    eigenvalues, eigenvectors = np.linalg.eig(es3d)
    evals = np.ones((3, 3))

    print("The eigenvalues of CoVar are:")
    print(eigenvalues.transpose())
    print(" (these should be real numbers, and one of them should be zero) ")
    print("The matrix of eigenvectors, V, is:")
    print(eigenvectors)
    print(" (these should be real-valued vectors) ")

    evals = eigenvalues.real
    print("real parts of evals: ")
    print(evals.transpose())

    min_lambda = evals[0]
    complex_vec = np.ones((3, 1))
    est_plane_normal = np.ones((3, 1))
    complex_vec = eigenvectors[:, 0].T
    est_plane_normal = complex_vec.real
    i_normal = 0

    for ivec in range(3):
        lambda_test = evals[ivec]
        if lambda_test < min_lambda:
            min_lambda = lambda_test
            i_normal = ivec
            est_plane_normal = eigenvectors[:, ivec].T.real

    print("min eval is " + str(min_lambda) + ", corresponding to component " + str(i_normal))

    print("corresponding evec (est plane normal): ")
    print(est_plane_normal.transpose())

    print("correct answer is: ")
    print(normal_vec.transpose())

    est_dist = est_plane_normal.dot(centroid.T)

    print("est plane distance from origin = ")
    print(est_dist)
    print("correct answer is: ")
    print(dist)

    rospy.loginfo("2ND APPROACH b = A*x SOLN")

    ones_vec = np.ones((npts, 1))
    A = points_mat.transpose()
    # print(A)
    # print(ones_vec)
    x_soln = np.linalg.lstsq(A, ones_vec, rcond=-1)[0]
    # x_soln = np.linalg.solve(A, ones_vec)
    print(x_soln)
    dist_est2 = 1.0 / (np.linalg.norm(x_soln))
    x_soln = x_soln * dist_est2
    dist_est2 = 1.0 / (np.linalg.norm(x_soln))

    print("normal vec, 2nd approach: ")
    print(x_soln.transpose())
    print("plane distance = ")
    print(dist_est2)













