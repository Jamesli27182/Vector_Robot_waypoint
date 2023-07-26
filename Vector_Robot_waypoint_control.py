#!/usr/bin/env python3
## Need the connection with Vector Robot
## https://www.digitaldreamlabs.com/products/vector-robot
# Task 3
## waypoint control PD(friction)

## result shown at https://youtube.com/shorts/_96aOsehFdA?feature=share

import time
import anki_vector
from anki_vector.util import degrees, distance_mm, speed_mmps,Pose
import matplotlib.pyplot as plt
import math as m
def find_nearly_points(current_x,current_y):
    d_n = [0.0]*N
    for i in range(N):
        dx = way_x[i]-current_x
        dy = way_y[i]-current_y
        d_n[i] = (dx**2 + dy**2)**0.5
    c = min(d_n)
    for j in range(N):
        if abs(c-d_n[j])<0.1:
            return j,d_n[j]

def find_next_goal(k):
    if k<N-4:
        return way_x[k+3],way_y[k+3]
    else:
        return way_x[N-1],way_y[N-1]

accx = []
accl_x = open('acc_x.txt','w')
#acce y
accy = []
accl_y = open('acc_y.txt','w')
#positionx
current_x = []
c_x = open('c_x.txt','w')
#positiony
current_y=[]
c_y = open('c_y.txt','w')
#angle z
z_angle = []
c_z = open('angle_z.txt','w')
#v2
left_v = []
left_vel = open('left_vel.txt','w')
#v1
right_v=[]
right_vel = open('right_vel.txt','w')
#############################

N=15
way_x = [0.0]*N
way_y = [0.0]*N
way_angle = [0.0]*N
for i in range(len(way_x)):
    way_x[i]=i*(800/N)
    way_y[i]=200*(m.sin((way_x[i])*2*m.pi/800))
    way_angle[i]=m.pi/2*m.cos(way_x[i]*m.pi/800)*180/m.pi
x_length = len(way_x)
y_length = len(way_y)
goal_x = way_x[1]
goal_y = way_y[1]
p_distance = (goal_x**2 + goal_y**2 )**0.5

args = anki_vector.util.parse_command_args()
k_rou = 4
k_aph = 4
r= 12#mm radius of wheel
L= 48#mm distance of wheel
d_distance = p_distance
print('d',d_distance)
z_angle = []
with anki_vector.Robot(args.serial) as robot:
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees
    print('////////////')
    print(o_x)
    print(o_y)
    print(o_z)
    print('//////////')
    time.sleep(2)
    pose = Pose(x=0, y=0, z=0, angle_z=anki_vector.util.Angle(degrees=0))
    robot.behavior.go_to_pose(pose)
    o_x=robot.pose.position.x
    o_y=robot.pose.position.y
    o_z = robot.pose.rotation.angle_z.degrees
    print(o_x)
    print(o_y)
    print(o_z)
    time.sleep(2)
    t = 0

    count_i=0
    print('start')
    while not(t>100):
        count_i+= 1
        if (robot.pose.position.x-o_x)**2+(robot.pose.position.y-o_y)**2>=(800**2-1):
            break
        i_points = find_nearly_points(robot.pose.position.x-o_x,robot.pose.position.y-o_y)
        [goal_x,goal_y] = find_next_goal(i_points[0])
        time.sleep(0.02)
        t = t+0.02
        d_x = goal_x-(robot.pose.position.x-o_x)
        d_y = goal_y-(robot.pose.position.y - o_y)
        d_distance = m.sqrt(d_x**2 + d_y**2)
        vel = k_rou*d_distance
        theta = robot.pose.rotation.angle_z.degrees - o_z
        z_angle.append(theta)
        beta = (m.atan(d_y/d_x))*180/m.pi
        alpha = beta - theta
   
        if vel >60:
            vel = 60
        omiga = k_aph*alpha
        omiga = omiga*m.pi/180
        if omiga>=0:
            v1 = (L*omiga+2*vel)/2
            v2 = 2*vel-v1
        else:
            o_omiga = abs(omiga)
            v2 = (L*o_omiga+2*vel)/2
            v1 = 2*(vel)-v2

        robot.motors.set_wheel_motors(v2,v1)
        accx.append(robot.accel.x)
        accy.append(robot.accel.y)
        left_v.append(v2)
        right_v.append(v1)
        current_x.append(robot.pose.position.x-o_x)
        current_y.append(robot.pose.position.y-o_y)
        z_angle.append(robot.pose.rotation.angle_z.degrees-o_z)
    print('end')
    print(t)
    robot.motors.set_wheel_motors(0,0)
    time.sleep(10)
    print(count_i)
    for i in range(len(current_x)):
        accl_x.write(str(accx[i]) + '\n')
#acce y
        accl_y.write(str(accy[i])+ '\n') 
#positionx
        c_x.write(str(current_x[i])+ '\n') 
#positiony
        c_y.write(str(current_y[i])+ '\n') 
#angle z
        c_z.write(str(z_angle[i])+ '\n') 
#v2
        left_vel.write(str(left_v[i])+ '\n') 
#v1
        right_vel.write(str(right_v[i])+ '\n')
    time.sleep(10)  