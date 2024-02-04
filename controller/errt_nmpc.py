 #!/usr/bin/env python
 # license removed for brevity
import rospkg
import rospy
import opengen as og
import numpy 
from std_msgs.msg import String
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
#from euler_to_quaternion import euler_to_quaternion
import statistics
from mav_msgs.msg import RollPitchYawrateThrust
from std_msgs.msg import Float64
#from traj_msg.msg import OptimizationResult
import time
import sys
import math
mng = og.tcp.OptimizerTcpManager('MAV/errt_nmpc')
mng.start()
xpos = 0
ypos = 0
zpos = 0
qx = 0
qy = 0
qz = 0
qw = 0
vx = 0
vy = 0
vz = 0
roll = 0
pitch = 0
yaw = 0
roll_v = 0
pitch_v = 0
yaw_v = 0
yawrate = 0
t0 = 6
C = 9.81 / t0
#obsdata = [0]*(3)
N = 15
ustar = [9.81,0.0,0.0] * (N)


nu = 3
dt = 1.0/20
x0 = [0,0,0.0,0.0,0.0,0.0,0.0,0.0]
global uold
uold = [9.81,0.0,0.0]
uref = [9.81,0.0,0.0]
xref = [0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
v_z = [0.0, 0.0, 0.0]
p_f = [0,0,0]
f_nmhe = [0,0,0]
land_flag = 0
start_flag = 0
safety_counter = 0




def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]




def callback_pot(data):
    global p_f
    p_f = [0,0,0]
    p_f[0] = data.point.x
    p_f[1] = data.point.y
    p_f[2] = data.point.z



# def callback_vicon(data):
#     global xpos, ypos,zpos,vx, vy, vz,yaw_v
#     xpos = data.pose.pose.position.x
#     ypos = data.pose.pose.position.y
#     zpos = data.pose.pose.position.z
#     qx = data.pose.pose.orientation.x
#     qy = data.pose.pose.orientation.y
#     qz = data.pose.pose.orientation.z
#     qw = data.pose.pose.orientation.w
#     vx = data.twist.twist.linear.x
#     vy = data.twist.twist.linear.y
#     vz = data.twist.twist.linear.z
#     [roll_v, pitch_v, yaw_v] = quaternion_to_euler(qx,qy,qz,qw)
    #print([qx, qy, qz, qw])
    #print(yaw_v)

def callback_lio(data):
    global xpos, ypos,zpos,vx, vy, vz,yaw_v
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    [roll_v, pitch_v, yaw_v] = quaternion_to_euler(qx,qy,qz,qw)
    #print([qx, qy, qz, qw])
    #print(yaw_v)

def callback_imu(imu_data):
    global roll,pitch,yaw, yawrate
    qx = imu_data.orientation.x
    qy = imu_data.orientation.y
    qz = imu_data.orientation.z
    qw = imu_data.orientation.w
    [roll, pitch, yaw] = quaternion_to_euler(qx,qy,qz,qw)
    pitch = pitch
    #print(yaw)
    yawrate = imu_data.angular_velocity.z


def callback_safety(data):
    global land_flag
    land_flag = 1

def callback_start(data):
    global xref, start_flag, yaw_ref, yaw_v
    if start_flag == 0:
        xref[0] = xpos
        xref[1] = ypos
        xref[2] = zpos + 1.3
        yaw_ref = yaw_v
    start_flag = 1



    #print(zpos)
def callback_ref(data):
    global xref, yaw_ref
    xref[0] = data.pose.pose.position.x
    xref[1] = data.pose.pose.position.y
    xref[2] = data.pose.pose.position.z
    #xref[3] = data.twist.twist.linear.x
    #xref[4] = data.twist.twist.linear.y
    #xref[5] = data.twist.twist.linear.z

    yaw_ref = data.pose.pose.orientation.z





def PANOC():
    rospy.init_node('PANOC', anonymous=True)
    pub = rospy.Publisher('/hummingbird/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
    #pub_traj = rospy.Publisher('traj_1', OptimizationResult, queue_size=1)


    #sub_sonar = rospy.Subscriber('/mavros/distance_sensor/lidarlite_pub', Range, callback_sonar)
    #sub = rospy.Subscriber('/odometry/imu', Odometry, callback_lio)
    # pub_ref = rospy.Publisher('pixyy/reference', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('/hummingbird/ground_truth/odometry', Odometry, callback_lio)
    sub_safety = rospy.Subscriber('hummingbird/safety_land', String, callback_safety)
    sub_start = rospy.Subscriber('hummingbird/set_start', String, callback_start)
    sub_imu = rospy.Subscriber('/hummingbird/ground_truth/imu', Imu, callback_imu)
    sub_pot = rospy.Subscriber('/potential_delta_p_hummingbird', PointStamped, callback_pot)
    sub_ref = rospy.Subscriber('/hummingbird/reference', Odometry, callback_ref)


    #sub_traj = rospy.Subscriber('traj_2', OptimizationResult, callback_traj)
    #sub_traj_2 = rospy.Subscriber('traj_3', OptimizationResult, callback_traj_2)
 
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]

    ustar = [9.81,0.0,0.0] * (N)
    i = 0
    t = 0
    safety_counter = 0
    global integrator
    integrator = 0
    global xref, yaw_ref
    xref = [0.0,0.0,1.3,0.0,0.0,0.0,0.0,0.0]
    yaw_ref = 0


    ##ADAPT WEIGHT PARAMS####
    Qx = [10,10,20,1,1,1,5,5]





    xpos_ref = 0
    ypos_ref = 0
    zpos_ref = 1.0


    while not rospy.is_shutdown():
        global p_f, land_flag, start_flag

        start = time.time()



        ######BODY ROTATIONS####
        zpos_angle = zpos * (math.cos(roll) * math.cos(pitch))
        x0_body = [math.cos(yaw_v)*xpos + math.sin(yaw_v)*ypos, -math.sin(yaw_v)*xpos + math.cos(yaw_v)*ypos, zpos, vx, vy, vz, roll, pitch]
        if (t < 100) | (land_flag == 1):
            p_f = [0,0,0]

        f_nmhe = [0.0, 0, 0]


        xref_body = [(math.cos(yaw_v)*xref[0] + math.sin(yaw_v)*xref[1]) + p_f[0], (-math.sin(yaw_v)*xref[0] + math.cos(yaw_v)*xref[1]) + p_f[1], xref[2] + p_f[2], (math.cos(yaw_v)*xref[3] + math.sin(yaw_v)*xref[4]), (-math.sin(yaw_v)*xref[3] + math.cos(yaw_v)*xref[4]), xref[5], xref[6], xref[7]]

        p_ref_dist = math.sqrt((xref_body[0] - x0_body[0])**2 + (xref_body[1] - x0_body[1])**2 + (xref_body[2] - x0_body[2])**2)
        if p_ref_dist > 1:
            p_ref_norm = [(xref_body[0] - x0_body[0]) / p_ref_dist, (xref_body[1] - x0_body[1]) / p_ref_dist, (xref_body[2] - x0_body[2]) / p_ref_dist]
            xref_body[0:3] = [x0_body[0] + p_ref_norm[0], x0_body[1] + p_ref_norm[1], x0_body[2] + p_ref_norm[2]]

       


        z0 = x0_body + xref_body + uref + uold + f_nmhe + Qx
        solution = mng.call(z0, initial_guess=[9.81,0,0]*(N),buffer_len = 4*4096)
        ustar = solution['solution']
        uold = ustar[0:3]

        print("odom:", x0_body);
        print("p_ref:", xref_body[0:3]);

        u_r = ustar[1]
        u_p = ustar[2]

        rpyt = RollPitchYawrateThrust()

        if land_flag == 0:
            integrator = integrator + 0.005*(xref_body[2] - zpos)

        t0_ = t0 + integrator


        C = 9.81 / t0_
        u_t = ustar[0] / C


        if (t < 40) & (start_flag == 1):
            u_t = 0.2
            u_r = 0
            u_p = 0
            integrator = 0

        if start_flag == 0:
            u_t = 0
            t = 0
            u_r = 0
            u_p = 0
            integrator = 0

        if land_flag == 1:
            u_t = (t0_ - 0.5) - safety_counter*0.001
            safety_counter+=1
            if (u_t < 0) | (zpos < 0.2):
                u_t = 0
                mng.kill()

        rpyt.roll = u_r
        rpyt.pitch = u_p

        rpyt.thrust.x = 0
        rpyt.thrust.y = 0

        ang_diff = yaw_ref - yaw_v



        rpyt.thrust.z = u_t

        ang_diff = numpy.mod(ang_diff + math.pi, 2*math.pi) - math.pi


        
        u_y = 0.5 * (ang_diff)  #+ yaw_integrator
        # u_y = 0.7*(ang_diff) - 0.1*yawrate
        if u_y > 1:
            u_y = 1

        if u_y < -1:
            u_y = -1
        
        rpyt.yaw_rate = u_y

        rpyt.header = std_msgs.msg.Header()
        rpyt.header.stamp = rospy.Time.now()
        rpyt.header.frame_id = 'world'
        
        pub.publish(rpyt)






        end = time.time()
        #print(end-start)
        rate.sleep()
        #end = time.time()

        t = t + 1

if __name__ == '__main__':
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass
