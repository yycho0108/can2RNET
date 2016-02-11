"""

ROS Binding for the RNet Wheelchair.
Currently supports teleoperation and battery readings; hoping to get odometry via wheel encoders.

To figure out 'non-trivial' rnet messages:
candump can0 -L | grep -Ev '02001100#|02000200#|00E#|03C30F0F#|0C140300#|0C140000#|1C0C0000#|14300000#|1C300004#'
"""
import rospy
import socket, sys, os, array, threading
from fcntl import ioctl
#import can2RNET as cr
#from can2RNET import *
import can
import time
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

import threading

def dec2hex(dec,hexlen):  #convert dec to hex with leading 0s and no '0x'
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0'*hexlen+h)[l:l+hexlen]

def aid_str(msg):
    if msg.id_type:
        return '{0:08x}'.format(msg.arbitration_id)
    else:
        return '{0:03x}'.format(msg.arbitration_id)

class RNETInterface(object):
    def __init__(self):
        # joystick
        self._can  = can.interface.Bus(channel='can0', bustype='socketcan_ctypes'
                ) # TODO : configure can_filters to filter joy input

        ## battery
        self._can2 = can.interface.Bus(channel='can0', bustype='socketcan_ctypes',
                can_filters=[{"can_id":0x1C0C0000, "can_mask":0x1FFFF0FF, "extended":True}])
        
        self._battery = None

        t = threading.Thread(target=self.recv_battery)
        t.daemon = True
        t.start()

    def recv_battery(self):
        while True:
            msg = self._can2.recv()
            self._battery = msg.data[0]

    def set_speed(self, v):
        if 0<=v<=0x64:
            return self.send('0a040100#'+dec2hex(v,2))
        else:
            return False

    def beep(self):
        self.send("181c0100#0260000000000000")

    def sing(self):
        self.send("181C0100#2056080010560858")
        self.send("181C0100#105a205b00000000")

    def get_joy_frame(self):
        msg = self.recvfrom()#16)
        return aid_str(msg)
        #m_id = ''
        #if msg.id_type:
        #    m_id = '{0:08x}'.format(msg.arbitration_id)
        #else:
        #    m_id = '{0:03x}'.format(msg.arbitration_id)
        #print m_id
        #return m_id
        #s = m_id + '#' + ''.join('%02X' % x for x in msg.data)
        #if msg.is_remote_frame:
        #    s += 'R'
        #return s
        # format : id#data[R]
        #candump_frame = dissect_frame(cf)
        #return (can_idtxt + '#'+''.join(["%02X" % x for x in data[:can_dlc]]) + 'R'*rtr)

        #frame_id = cf.split('#')[0]
        #return frame_id

    def disable_joy(self):
        for i in range(0,3):
            self.send('0c000000#')

    def send(self, msg_str, *args, **kwargs):
        msg_l = msg_str.split('#')
        rtr   = ('#R' in msg_str)
        data  = None
        if rtr:
            data = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
        else:
            data = bytearray.fromhex(msg_l[1])
        msg = can.Message(
                timestamp       = time.time(),
                is_remote_frame = rtr,
                extended_id     = len(msg_l[0])>4,
                arbitration_id  = int(msg_l[0], 16),
                dlc             = 0 if rtr else len(data), # TODO : why 0?
                data            = data
                )
        self._can.send(msg, *args, **kwargs)
        return True

    def recvfrom(self, *args, **kwargs):
        #return self._can.recvfrom(*args, **kwargs)
        return self._can.recv(*args, **kwargs)

class RNETTeleopNode(object):

    def __init__(self):
        self._disable_chair_joy=rospy.get_param('~disable_chair_joy', default=False)
        self._joy_frame = rospy.get_param('~joy_frame', default=None)#'02001100')
        if not self._joy_frame is None:
            self._joy_frame = '{0:08x}'.format(self._joy_frame)
        self._speed=rospy.get_param('~speed', default=50) # speed, percentage 0-100
        self._min_v=rospy.get_param('~min_v', default=0.0)
        self._min_w=rospy.get_param('~min_w', default=0.0)
        self._cmd_timeout=rospy.get_param('~cmd_timeout', default=0.1) # stops after timeout
        self._cmd_vel_sub=rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        self._bat_pub=rospy.Publisher('battery', BatteryState, queue_size=10)
        self._rnet = RNETInterface()

        self._cmd_vel = Twist()
        self._last_cmd = rospy.Time.now()

    def wait_rnet_joystick_frame(self, dur=0.2):
        frame_id = ''
        start = rospy.Time.now()
        while frame_id[0:3] != '020': # look for joystick frame ID
            frame_id = self._rnet.get_joy_frame()
            if (rospy.Time.now() - start).to_sec() > dur:
                rospy.loginfo('... Joy frame wait timed out')
                return False, None
        return True, frame_id

    def cmd_vel_cb(self, msg):
        self._cmd_vel = msg
        self._rnet._cmd_vel=self._cmd_vel
        self._last_cmd = rospy.Time.now()

    def step(self):
        if self._rnet._battery is not None:
            bs_msg = BatteryState()
            bs_msg.header.stamp = rospy.Time.now()
            bs_msg.percentage = 1.0 * self._rnet._battery
            self._bat_pub.publish(bs_msg)

        if (rospy.Time.now() - self._last_cmd).to_sec() > self._cmd_timeout:
            # zero-out velocity commands
            self._cmd_vel.linear.x = 0
            self._cmd_vel.angular.z = 0
            self._rnet._cmd_vel=self._cmd_vel

        if self._disable_chair_joy:
            cf = self._joy_frame
        else:
            cf = self._rnet.recvfrom()#16)
            cf = aid_str(cf)

        # TODO : calibrate to m/s and scale accordingly
        # currently, v / w are expressed in fractions where 1 = max fw, -1 = max bw
        v = np.clip(self._cmd_vel.linear.x, -1.0, 1.0)
        w = np.clip(self._cmd_vel.angular.z, -1.0, 1.0)

        if cf == self._joy_frame:
            # for joy : y=fw, x=turn; 0-256
            cmd_y = 0x100 + int(v * 0x3FFF) >> 8 & 0xFF
            cmd_x = 0x100 + int(-w * 0x3FFF) >> 8 & 0xFF

            if np.abs(v) > self._min_v or np.abs(w) > self._min_w:
                self._rnet.send(self._joy_frame + '#' + dec2hex(cmd_x, 2) + dec2hex(cmd_y, 2))
            else:
                # below thresh, stop
                self._rnet.send(self._joy_frame + '#' + dec2hex(0, 2) + dec2hex(0, 2))

    def spin(self):
        rate = rospy.Rate(50)
        rospy.on_shutdown(self.save)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

    def save(self):
        pass

    def run(self):
        # 1 - check R-NET Joystick
        rospy.loginfo('Waiting for R-NET Joystick Frame ... ')
        suc, joy_frame = self.wait_rnet_joystick_frame(0.2)
        if suc:
            rospy.loginfo('Found R-NET Joystick frame: {}'.format(joy_frame))
            self._joy_frame = joy_frame
        else:
            if self._joy_frame is not None:
                rospy.logwarn('No R-NET Joystick frame seen within minimum time')
                rospy.logwarn('Using Supplied Joy Frame : {}'.format(self._joy_frame))
            else:
                rospy.logerr('No R-NET Joystick frame seen within minimum time')
                return
        # set chair's speed to the lowest setting.
        suc = self._rnet.set_speed(self._speed)
        if not suc:
            rospy.logwarn('RNET Set SpeedRange Failed @ v={}' .format(self._speed))
            return
        rospy.loginfo('RNET Set SpeedRange Success @ v={}' .format(self._speed))

        if self._disable_chair_joy:
            self._rnet.disable_joy()
            rospy.loginfo("You chose to disable the R-Net Joystick temporary. Restart the chair to fix.")
        else:
            rospy.loginfo("You chose to allow the R-Net Joystick - There may be some control issues.")

        self.spin()

if __name__ == "__main__":
    rospy.init_node('rnet_teleop_node')
    app = RNETTeleopNode()
    app.run()
