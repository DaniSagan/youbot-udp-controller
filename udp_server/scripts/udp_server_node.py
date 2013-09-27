#!/usr/bin/env python
import roslib; roslib.load_manifest('udp_server')
import rospy
from std_msgs.msg import Float32
import socket
import math

def talker():
    linear_pub = rospy.Publisher('/youbot_client/platform_vel_cmd/linear', Float32)
    angular_pub = rospy.Publisher('/youbot_client/platform_vel_cmd/angular', Float32)
    rospy.init_node('udp_server')
    UDP_IP = '0.0.0.0'
    UDP_PORT = 5555

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))
    print('Connected to address %s:%d' % (UDP_IP, UDP_PORT))
    while not rospy.is_shutdown():
        data_str, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        data = data_str.split(',')
        accx = float(data[2])
        accy = float(data[3])
        accz = float(data[4])        
        ang_turn = math.atan2(accy, accx)
        ang_acc = math.atan2(accz, accx)
        
        if (ang_acc < 0.4 * math.pi and ang_acc > -0.4 * math.pi and
           ang_turn < 0.4 * math.pi and ang_turn > -0.4 * math.pi):
            if ang_turn >= 0.1:
                ang_vel = (ang_turn - 0.1) * 2.0
            elif ang_turn <= -0.1:
                ang_vel = (ang_turn + 0.1) * 2.0
            else:
                ang_vel = 0.0
            
            if ang_acc >= 0.1:
                lin_vel = (ang_acc - 0.1) * 1.0
            elif ang_acc <= -0.1:
                lin_vel = (ang_acc + 0.1) * 1.0
                ang_vel = ang_vel * (-1.0);
            else:
                lin_vel = 0.0
            
            
        else:
            ang_vel = 0.0
            lin_vel = 0.0
            
        rospy.loginfo('angs: %f %f' % (lin_vel, ang_vel))
        linear_pub.publish(Float32(lin_vel))
        angular_pub.publish(Float32(ang_vel))
        #rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
