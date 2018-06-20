import rospy
import socket, sys, os, array, threading
from fcntl import ioctl
import can2RNET as cr
from can2RNET import *

from geometry_msgs.msg import Twist

class RNETInterface(object):
    def __init__(self):
        self._can = cr.opencansocket(0)

    def set_speed(self, v):
        if 0<=v<=0x64:
            self.send('0a040100#'+dec2hex(speed_range,2))
        else:
            print('Invalid RNET SpeedRange: ' + str(speed_range))

    def beep(self):
        self.send("181c0100#0260000000000000")

    def sing(self):
        self.send("181C0100#2056080010560858")
        self.send("181C0100#105a205b00000000")

    def get_joy_frame(self):
        cf, addr = self.recvfrom(16)
        candump_frame = dissect_frame(cf)
        frame_id = candump_frame.split('#')[0]
        return frame_id

    def send(self, *args, **kwargs):
        return cansend(self._can, *args, **kwargs)

    def recvfrom(self, *args, **kwargs):
        return self._can.recvfrom(*args, **kwargs)

class RNETTeleopNode(object):

    def __init__(self):
        self._disable_chair_joy=rospy.get_param('~disable_chair_joy', default=False)
        self._speed=rospy.get_param('~speed', default=50) # speed, percentage 0-100
        self._min_v=rospy.get_param('~min_v', default=0.0)
        self._min_w=rospy.get_param('~min_w', default=0.0)
        self._cmd_timeout=rospy.get_param('~cmd_timeout', default=0.1) # stops after timeout
        self._cmd_vel_sub=rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        self._rnet = RNETInterface()
        self._joy_frame = None

    def wait_rnet_joystick_frame(self, dur=0.2):
        frame_id = ''
        start = rospy.Time.now()
        while frame_id[0:3] != '020': # look for joystick frame ID
            frame_id = self._rnet.get_joy_frame()
            if (rospy.Time.now() - start.to_sec()) > dur:
                rospy.loginfo('... Joy frame wait timed out')
                return False, None
        return True, frame_id

    def cmd_vel_cb(self, msg):
        self._cmd_vel = msg
        self._last_cmd = rospy.Time.now()

    def move(self):
        if (rospy.Time.now() - self._last_cmd).to_sec() > self._cmd_timeout:
            # zero-out velocity commands
            self._cmd_vel.linear.x = 0
            self._cmd_vel.angular.z = 0

        #prebuild the frame we are waiting on
	rnet_joystick_frame_raw = build_frame(self._joy_frame+ "#0000")
        cf, addr = self._rnet.recvfrom(16)
        v = self._cmd_vel.linear.x
        w = self._cmd_vel.angular.z

        if cf == rnet_joystick_frame_raw:
            # for joy : y=fw, x=turn; 0-200
            cmd_y = int(v * 100.)
            cmd_x = -int(w * 100.)

            if np.abs(v) > self._min_v or np.abs(w) > self._min_w:
                self._rnet.send(rnet_joystick_id + '#' + dec2hex(cmd_x, 2) + dec2hex(cmd_y, 2))
            else:
                self._rnet.send(rnet_joystick_id + '#' + dec2hex(cmd_x, 2) + dec2hex(cmd_y, 2))

    def run(self):
        # 1 - check R-NET Joystick
        rospy.loginfo('Waiting for R-NET Joystick Frame ... ')
        suc, joy_frame = self.wait_rnet_joystick_frame(0.2)
        self._joy_frame = joy_frame
        if not suc:
            rospy.logerr('No R-NET Joystick frame seen within minimum time')
            return
        rospy.loginfo('Found R-NET Joystick frame: {}'.format(joy_frame))

        # set chair's speed to the lowest setting.
        self._rnet.set_speed(self._speed)

        if self._disable_chair_joy:
            rospy.loginfo('Disabling Joystick is currently not supported. restart with _disable_chair_joy:=False')
            return
            #rospy.loginfo("\n You chose to disable the R-Net Joystick temporary. Restart the chair to fix. ")
            #joy_frame = RNET_JSMerror_exploit(can_socket) # ?????

            #sendjoyframethread = threading.Thread(
            #    target=send_joystick_canframe,
            #    args=(can_socket,joy_frame,),
            #    daemon=True)
            #sendjoyframethread.start()
        else:
            rate = rospy.Rate(50)
            while not rospy.is_shutdown():
                self.move()
                rate.sleep()
            #rospy.loginfo("\n You chose to allow the R-Net Joystick.")
            #rospy.loginfo('Waiting for RNET-Joystick frame ...')

            #suc, joy_frame = self.wait_rnet_joystick_frame(0.2)
            #if not suc:
            #    rospy.logerr('No RNET-Joystick frame seen within minimum time')
            #    return

            #rospy.loginfo('Found RNET-Joystick frame: {}'.format(joy_frame))

            #inject_rnet_joystick_frame_thread = threading.Thread(
            #    target=inject_rnet_joystick_frame,
            #    args=(can_socket, rnet_joystick_id,),
            #    daemon=True)
            #inject_rnet_joystick_frame_thread.start()

if __name__ == "__main__":
    rospy.init_node('rnet_teleop_node')
    app = RNETTeleopNode()
    app.run()