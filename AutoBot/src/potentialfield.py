#!/usr/bin/env python3
from cmath import exp, inf
from math import atan2
from math import tan
from re import A
import rospy
import sys
import numpy as np
from rosgraph import rosenv
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

vel=Twist()

'''
def obs_counter(lsr_data):
    obs_count=0
    for i in range(len(lsr_data)):
        if lsr_data[i]!=inf and lsr_data[i-1]== inf:
            obs_count+=1
    return obs_count '''

def Rep_field(A_k,sigma_k,ang_data):
    theta_k=2*sigma_k #central angle of obstacle
    #print(ang_data)
    field=[-A_k*exp(-((theta_k-ang_data[0][i])**2)/(2*(sigma_k**2))) for i in range(len(ang_data))]

    return field

def Att_field(ang_data):
    gamma=5
    theta_goal=0
    field=[gamma*abs(theta_goal-ang_data[i]) for i in range(len(ang_data))]
    return field

def Adjusted_obs(dist_data,ang_data): # ang_data is 2D array
    
    d_k=sum(dist_data[0])/len(dist_data[0]) #avg dist to obstacle
    fi_k= abs(ang_data[0][0]-ang_data[0][-1]) #angle occupied by obstacle ##
    d_max=max(dist_data[0]) #maximum dist to obstacle
    w= 0.8 #width of robot in m
    print(d_k)
    sigma_k= atan2((d_k*tan(fi_k/2))+w/2,d_k)
    A_k=(d_max-(d_max*d_k))*exp(1/2)

    return A_k,sigma_k

def Heading_ang(lsr_pts):
    filt_dpts,filt_apts,k,obs_index= data_filt(lsr_pts)
    total_field=[]
    
    for i in range(len(obs_index)-1):
        arr_dist=np.array([filt_dpts[obs_index[i]:obs_index[i+1]]])
        arr_ang=np.array([filt_apts[obs_index[i]:obs_index[i+1]]])
        A_k,sigma_k=Adjusted_obs(arr_dist,arr_ang)      ##should be arr_ang
        rep_field=Rep_field(A_k,sigma_k,arr_ang)
        att_field=Att_field(arr_ang)
        sum_field=[rep_field[i]+att_field[i] for i in range(len(rep_field))]
        total_field.append(sum_field)
    #for last obstacle 
    
    last_darr=np.array(filt_dpts[obs_index[-1]:])
    last_angarr=np.array(filt_apts[obs_index[-1]:])
    A_k,sigma_k=Adjusted_obs(last_darr,last_angarr)
    lrep_field=Rep_field(A_k,sigma_k,last_angarr)
    latt_field=Att_field(last_angarr)
    lsum_field=[lrep_field[i]+latt_field[i] for i in range(len(lrep_field))]
    total_field.append(lsum_field) 

    l=[]
    for i in range(len(total_field)):
        for j in range(len(total_field[i])):
            l.append(float(total_field[i][j])) 
    theta_ind=l.index(sorted(l)[0]) #index of smallest theta
    theta=filt_apts[theta_ind]
    #plt.plot(filt_apts,l)
    #plt.show()
    #print(theta) 
 
    return theta

def data_filt(unfilt_pts):
    filt_dist_data=[]
    filt_ang_data=[]
    max_angle=0.787945 #in radians
    ang_increment=0.004389665555208921 #in radians
    unfilt_ang_data=[i for i in np.arange(max_angle,-max_angle,-ang_increment)]
    inf_count=0
    for i in range(len(unfilt_pts)):
        if unfilt_pts[i]==inf:      ##
            inf_count+=1
            #filt_ang_data.append(unfilt_ang_data[i])
            if inf_count==360:
                filt_dist_data=np.ones(360)*999999
                
                break
        if unfilt_pts[i]!=inf:
            filt_dist_data.append(unfilt_pts[i])
            filt_ang_data.append(unfilt_ang_data[i])

    
    #obstacle counter:
    obs_count=1
    obs_ind=[0]
    for i in range(len(filt_ang_data)):
        if  abs(filt_ang_data[i]-filt_ang_data[i-1])>ang_increment and i-1>0:
            obs_count+=1
            obs_ind.append(i)
    #print(obs_count)
    #plt.plot(filt_ang_data,filt_dist_data)
    #plt.show()
    #print(len(filt_ang_data))
    return filt_dist_data ,filt_ang_data,obs_count,obs_ind 


def callbacksub(lsr_data):
    scann=lsr_data
    vel_pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)
    lsr_pts=scann.ranges
    #print((lsr_pts))    
    theta= Heading_ang(lsr_pts)
    #vel.linear.x=0.3
    #vel.angular.z=theta
    vel_pub.publish(vel)

    return 0

def main(args):
    rospy.init_node('potential_field')
    scan_pts=rospy.Subscriber('/scan',LaserScan,callbacksub)


    try:
        rospy.Rate(10)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv[1:])