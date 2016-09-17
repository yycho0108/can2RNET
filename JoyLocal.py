#JoyLocal.py - Translate USB joystick x/y axis to Rnet and inject onto canbus
#Requires: socketCan, can0 interface
#!/python3
# joystick based on: https://www.kernel.org/doc/Documentation/input/joystick-api.txt

import socket, sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *


debug = True


class X360:

    axis_map = []
    button_map = []
    xthreshold = 8 * 0x10000 / 128
    ythreshold = 8 * 0x10000 / 128

    joyx = 0
    joyy = 0


    # We'll store the states here.
    axis_states = {}
    button_states = {}

    # These constants were borrowed from linux/input.h
    axis_names = {
        0x00 : 'x',
        0x01 : 'y',
        0x02 : 'z',
        0x03 : 'rx',
        0x04 : 'ry',
        0x05 : 'rz',
        0x06 : 'trottle',
        0x07 : 'rudder',
        0x08 : 'wheel',
        0x09 : 'gas',
        0x0a : 'brake',
        0x10 : 'hat0x',
        0x11 : 'hat0y',
        0x12 : 'hat1x',
        0x13 : 'hat1y',
        0x14 : 'hat2x',
        0x15 : 'hat2y',
        0x16 : 'hat3x',
        0x17 : 'hat3y',
        0x18 : 'pressure',
        0x19 : 'distance',
        0x1a : 'tilt_x',
        0x1b : 'tilt_y',
        0x1c : 'tool_width',
        0x20 : 'volume',
        0x28 : 'misc',
    }

    button_names = {
        0x120 : 'trigger',
        0x121 : 'thumb',
        0x122 : 'thumb2',
        0x123 : 'top',
        0x124 : 'top2',
        0x125 : 'pinkie',
        0x126 : 'base',
        0x127 : 'base2',
        0x128 : 'base3',
        0x129 : 'base4',
        0x12a : 'base5',
        0x12b : 'base6',
        0x12f : 'dead',
        0x130 : 'a',
        0x131 : 'b',
        0x132 : 'c',
        0x133 : 'x',
        0x134 : 'y',
        0x135 : 'z',
        0x136 : 'tl',
        0x137 : 'tr',
        0x138 : 'tl2',
        0x139 : 'tr2',
        0x13a : 'select',
        0x13b : 'start',
        0x13c : 'mode',
        0x13d : 'thumbl',
        0x13e : 'thumbr',

        0x220 : 'dpad_up',
        0x221 : 'dpad_down',
        0x222 : 'dpad_left',
        0x223 : 'dpad_right',

        # XBox 360 controller uses these codes.
        0x2c0 : 'dpad_left',
        0x2c1 : 'dpad_right',
        0x2c2 : 'dpad_up',
        0x2c3 : 'dpad_down',
    }

    def init_joystick(self):

        if debug:
            # Iterate over the joystick devices.
            print('Available devices:')

            for fn in os.listdir('/dev/input'):
                if fn.startswith('js'):
                    print('  /dev/input/%s' % (fn))

        # Open the joystick device.
        try:
            fn = '/dev/input/js0'
            if debug:
                print('Opening %s...' % fn)
            jsdev = open(fn, 'rb')
        except IOError:
            print ('No joystick at ' + fn)
            return ('')

       
        #jsdev = os.open(fn, 'rb', os.O_RDONLY|os.O_NONBLOCK)

        # Get the device name.
        #buf = bytearray(63)
        buf = bytearray([0] * 64)
        ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf

        if debug:
            print('Device name: %s' % js_name)

        # Get number of axes and buttons.
        buf = array.array('B', [0] )
        ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]

        buf = array.array('B', [0] )
        ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]

        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0

        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0

        if debug:
            print ('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
            print ('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))
        return (jsdev)


    def joyread_thread(self, jsdev):
        global joyx
        global joyy
        global rnet_threads_running
        while rnet_threads_running:
            try:
                evbuf = jsdev.read(8)
                jtime, jvalue, jtype, jnumber = struct.unpack('IhBB', evbuf)
                if jtype & 0x02:
                    axis = self.axis_map[jnumber]
                    if (axis == 'x'):
                            if abs(jvalue) > self.xthreshold:
                                    joyx = 0x100 + int(jvalue * 100 / 128) >> 8 &0xFF
                            else:
                                    joyx = 0
                    elif (axis == 'y'):
                            if abs(jvalue) > self.ythreshold:
                                    joyy = 0x100 - int(jvalue * 100 / 128) >> 8 &0xFF
                            else:
                                    joyy = 0

            except:
                print("Error reading joystick")
                joyx = 0
                joyy = 0
                rnet_threads_running=False
                                                           
def dec2hex(dec,hexlen):  #convert dec to hex with leading 0s and no '0x' 
    h=hex(int(dec))[2:]
    l=len(h)
    if h[l-1]=="L":
        l-=1  #strip the 'L' that python int sticks on
    if h[l-2]=="x":
        h= '0'+hex(int(dec))[1:]
    return ('0'*hexlen+h)[l:l+hexlen]
                            
#THREAD: sends RnetJoyFrame every mintime seconds
#Not used
def send_joystick_canframe(s,joy_id):
        mintime = .01
        nexttime = time() + mintime
        priorjoyx=joyx
        priorjoyy=joyy
        while rnet_threads_running:
                joyframe = joy_id+'#'+dec2hex(joyx,2)+dec2hex(joyy,2)
                cansend(s,joyframe)
                nexttime += mintime
                t= time()
                if t < nexttime:
                    sleep(nexttime - t)
                else:
                    nexttime += mintime

#THREAD: Waits for joyframe and injects another spoofed frame ASAP
def inject_joy_frame(s,joy_id):
	joyframeraw = build_frame(joy_id+"#0000") #prebuild the frame we are waiting on
	while rnet_threads_running:
		cf, addr = cansocket.recvfrom(16)
		if cf == joyframeraw:
			cansend(s,joy_id+'#'+dec2hex(joyx,2)+dec2hex(joyy,2))
			



#Waits for any frame containing a Joystick position
#Returns: JoyFrame extendedID as text
def wait_joystick_frame(cansocket,t):
    frameid = ''
    while frameid[0:3] != '020':  #just look for joystick frame ID (no extended frame)
        cf, addr = cansocket.recvfrom(16) #this is a blocking read.... so if there is no canbus traffic it will sit forever (to fix!)
        candump_frame = dissect_frame(cf)
        frameid = candump_frame.split('#')[0]
        if time()>t:
             print("JoyFrame wait timed out ")
             return('Err!')
    return(frameid)

#Set speed_range: 0% - 100%
def RNETsetSpeedRange(cansocket,speed_range):
        if speed_range>=0 and speed_range<=0x64:
            cansend(cansocket,'0a040100#'+dec2hex(speed_range,2))
        else:
            print('Invalid RNET SpeedRange: ' + str(speed_range))

def RNETshortBeep(cansocket):
        cansend(cansocket,"181c0100#0260000000000000")
        
#Play little song
def RNETplaysong(cansocket):
        cansend(cansocket,"181C0100#2056080010560858")
        sleep(.77)
        cansend(cansocket,"181C0100#105a205b00000000")

#do very little and output something as sign-of-life
def watch_and_wait():
        while threading.active_count() > 0:
            sleep(0.5)
            print('X: '+dec2hex(joyx,2)+'\tY: '+dec2hex(joyy,2)+ '\tThreads: '+str(threading.active_count()))
            
#does not use a thread queue.  Instead just sets a global flag.
def kill_rnet_threads():
    global rnet_threads_running
    rnet_threads_running = False        



if __name__ == "__main__":
        global rnet_threads_running
        rnet_threads_running = True        
        cansocket = opencansocket(0)
                   
        #init /dev joystick
        x360 = X360()
        jsdev = x360.init_joystick()
        global joyx
        global joyy
        joyx = 0
        joyy = 0
        if jsdev != '':
            print('using USB Joystick @ ' + str(jsdev).split("'")[1])
            joyreadthread = threading.Thread(target=x360.joyread_thread,args=(jsdev,),daemon=True)
            joyreadthread.start()
            t=time()+.20
            print('waiting for RNET-Joystick frame')
            joy_id = wait_joystick_frame(cansocket,t) #t=timeout time
            if joy_id == 'Err!':
                print('No RNET-Joystick frame seen within minimum time')
                sys.exit()
            print('Found RNET-Joystick frame: '+joy_id)
            #joy_id = RNET_JSMerror_exploit(cansocket)
            speed_range = 00
            RNETsetSpeedRange(cansocket,speed_range)
            #sendjoyframethread = threading.Thread(target=send_joystick_canframe,args=(cansocket,joy_id,),daemon=True)
            #sendjoyframethread.start()
            injectjoyframethread = threading.Thread(target=inject_joy_frame,args=(cansocket,joy_id,),daemon=True)
            injectjoyframethread.start()
            sleep(0.5)
            watch_and_wait()
            kill_rnet_threads()            
        else:
            print('No Joystick found.')
            kill_rnet_threads()

        
        print("Exiting")
        
        
                