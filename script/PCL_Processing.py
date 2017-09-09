#!/usr/bin/env python
import rospy
from roslib import message
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import sys

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from numpy.random import randn
from scipy import array, newaxis
from nav_msgs.msg import Odometry
import numpy as np
import math
import scipy.linalg

robot_odometry_z_coordinate = 0.0

def subscribe_to_point_cloud(point_cloud_data) :
    #self.lock.acquire()
    assert isinstance(point_cloud_data, PointCloud2)
    frame_height=point_cloud_data.height
    frame_width=point_cloud_data.width
    point_cloud_iterator = point_cloud2.read_points(point_cloud_data, field_names = ("x", "y", "z"),skip_nans=True)
    count=0
    null_count = 0
    # get_current_odometry_data()
    cloud_point_x = []
    cloud_point_y = []
    cloud_point_z = []
    for point in point_cloud_iterator:
        x_coordinate = point[0]
        y_coordinate = point[1]
        z_coordinate = point[2]
    
        cloud_point_x.append(point[0])
        cloud_point_y.append(point[1])
        cloud_point_z.append(point[2])
        count=count+1
        # if (count > 100000):
        #     break

    # surface_identifier(cloud_point_x,cloud_point_y,cloud_point_z)
    print count,frame_height*frame_width
    

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # ax.scatter(cloud_point_x, cloud_point_y, cloud_point_z, c='r', marker='o')

    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.set_zlabel('Z Label')

    # plt.show()

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # cset = ax.contour(cloud_point_x, cloud_point_y, cloud_point_z, cmap=cm.coolwarm)
    # ax.clabel(cset, fontsize=9, inline=1)

    # plt.show()

def get_current_odometry_data():
    # print 'odom_data.pose.pose'
    rospy.Subscriber('/odom',Odometry,subscribe_to_odometry)

def subscribe_to_odometry(odom_data):
    robot_odometry_z_coordinate = odom_data.pose.pose.position.z
    print odom_data.pose.pose
    # return pose

def surface_identifier(cloud_point_x,cloud_point_y,cloud_point_z):
    data = np.c_[cloud_point_x,cloud_point_y,cloud_point_z]
    mn = np.min(data, axis=0)
    mx = np.max(data, axis=0)
    X,Y = np.meshgrid(np.linspace(mn[0], mx[0], 20), np.linspace(mn[1], mx[1], 20))

    XX = X.flatten()
    YY = Y.flatten()

    order = 1   # 1: linear, 2: quadratic
    if order == 1:
        # best-fit linear plane
        A = np.c_[data[:,0], data[:,1], np.ones(data.shape[0])]
        C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])    # coefficients
        
        # evaluate it on grid
        Z = C[0]*X + C[1]*Y + C[2]
        
        
        # or expressed using matrix/vector product
        # Z = np.dot(np.c_[XX, YY, np.ones(XX.shape)], C).reshape(X.shape)
        print C[0]
        print C[1]
        print C[2]

    elif order == 2:
        # best-fit quadratic curve
        A = np.c_[np.ones(data.shape[0]), data[:,:2], np.prod(data[:,:2], axis=1), data[:,:2]**2]
        C,_,_,_ = scipy.linalg.lstsq(A, data[:,2])
        
        # evaluate it on a grid
        Z = np.dot(np.c_[np.ones(XX.shape), XX, YY, XX*YY, XX**2, YY**2], C).reshape(X.shape)

    # plot points and fitted surface
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
    ax.scatter(data[:,0], data[:,1], data[:,2], c='r', s=50)
    plt.xlabel('X')
    plt.ylabel('Y')
    ax.set_zlabel('Z')
    ax.axis('equal')
    ax.axis('tight')
    plt.show()



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, subscribe_to_point_cloud)
    rospy.spin()

if __name__ == '__main__':
    listener()


