# Python program to simulate field rover for command_and_control_sever
import socket
import select
import sys
import logging
import getopt
import numpy as np
from daimyo_utils import TCPcompose, cmdparse
from time import time, sleep


def ncos(ang):
    return np.cos(np.radians(ang))


def nsin(ang):
    return np.sin(np.radians(ang))


inbuffer = ''  # Use malloc and/or ringbuffer if on embedded system
inbufmax = 32
inpacket = ''
commandstate = 0  # State of command assembly
cflag = False  # Whether a full command has been asssembled
servercommand = ''

# Field rover state variables
name = 'Ronin%d' % np.random.randint(low=0, high=99)
version = '0'
IP_address = 'localhost'  # command server IP
Port = 8081  # command server port
Nangle = 90.0  # Nangle is between north and x-axis
xpos = 0  # 2*np.random.random()-1
ypos = 0  # 2*np.random.random()-1
angle = 0  # 360*np.random.random()
# angle is direction rover is facing (derivable from digital compass)
[Dxy, Dangle] = [0.01, 1]  # Precision (meters, degrees.)
maxvel = 0.5  # maximum speed (m/s)
defvel = 0.3  # default velocity
[vx, vy] = [0.0, 0.0]  # current x and y velocity components
angvel = 45  # angular speed (deg./s)
heartbeat = False  # heartbeat flag for periodic position reporting
heartperiod = 0.5  # heartbeat period (seconds)


def orient_compass(Nangle=0):
    '''Code here would go to convert compass reading to angle relative
    to global map x-axis as supplied by server.'''
    # Since this is a dummy rover, just return input
    # All dummy rovers start facing north
    return Nangle


class multiple_timers():
        heart = 0.0
        gong = 0.0
        ping = 0.0
        gotostep = 0
        # For dummy rover with no obstacles, GOTO has three steps:
        # turn (3), move (2), turn(1).
        # First turn to orient. Last turn if GOTO command had angle field
        goto_x = 0.0
        goto_y = 0.0
        goto_vel = 0.0
        goto_ang = 0.0
        goto_destang = 0.0


mytimers = multiple_timers()
state = 0  # {0: st_IDLE}  # External state (communicated with server)
verb_dict = dict([('HALT', 0),
                  ('FWD', 1),
                  ('BWD', 2),
                  ('CFWD', 3),
                  ('CBWD', 4),
                  ('TURN', 5),
                  ('ATURN', 6),
                  ('CTURN', 7),
                  ('GOTO', 8),
                  ('OBS', 9),
                  ('POBS', 10)])
moveflag = verb_dict['HALT']  # Internal state
oldmoveflag = moveflag
movefields = []

try:
    opts, args = getopt.getopt(sys.argv[1:], "daip",
                               ["debug", "info",  "address", "port"])
except getopt.GetoptError:
    print('usage: python ', sys.argv[0],
          '[-d|--debug|-i|--info] [-a|--address <IP_address>] '
          '[-p|--port <port_number>]')
    sys.exit(0)
numerical_level = logging.WARNING
logfmt = '%(name)s:%(levelname)s:\t%(message)s'
for opt, arg in opts:
    if opt in ["-i", "--info"]:
        numerical_level = logging.INFO
    elif opt in ["-d", "--debug"]:
        numerical_level = logging.DEBUG
        logfmt = '%(asctime)s:'+logfmt
    elif opt in ["-a", "--address"]:
        IP_address = str(arg)
    elif opt in ["-p", "--port"]:
        Port = int(arg)

roverlog = logging.getLogger(name+'_v'+version)
logging.basicConfig(  # filename=basename+'.log', filemode='w',
    level=numerical_level,
    format='%(name)s:%(levelname)s:\t%(message)s')

# Open communication with server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.connect((IP_address, Port))
running = True
sendmsg = '<MYID,%s,%s>' % (name, version)
sleep(1)
server.send(sendmsg.encode())
sendmsg = '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
sleep(1)
server.send(sendmsg.encode())


def respond_to_server_cmd():
    global servercommand, cflag, commandstate, server
    global xpos, ypos, angle, Nangle, Dxy, Dangle, maxvel
    global heartbeat, heartperiod, state, moveflag, movefields
    global verb_dict, running, oldmoveflag
    sendmsg = ''
    valid, type, fields = cmdparse(version=version, kind='cmd',
                                   instr=servercommand)
    if not valid:
        roverlog.warning("Received invalid command: %s" %
                         servercommand)
    else:
        roverlog.debug("Processing: %s" % servercommand)
        if type == 'ID':  # Server is asking for ID
            roverlog.debug("Sending ID.")
            sendmsg = '<MYID,%s,%s>' % (name, version)
        elif type == 'POS':  # Server is asking for position
            roverlog.debug("Sending position.")
            sendmsg = '<MYPOS,%.3f,%.3f,%.3f>' % (
                xpos, ypos, angle)
        elif type == 'SETPOS':  # Server is setting position
            roverlog.debug("Setting position.")
            [xpos, ypos] = [float(fields[0]), float(fields[1])]
            Nangle = float(fields[2]) if fields[2] else Nangle
            angle = orient_compass(Nangle)  # Compute rover orientation
            sendmsg = '<ACK,>'
            sendmsg = '<MYPOS,%.3f,%.3f,%.3f>' % (
                xpos, ypos, angle)
        elif type == 'PRES':  # Server is asking for precision
            roverlog.debug("Sending precision.")
            sendmsg = '<MYPRES,%.3f,%.3f>' % (Dxy, Dangle)
        elif type == 'SETPRES':  # Server is setting precision
            roverlog.debug("Setting precision.")
            Dxy = float(fields[0])
            Dangle = float(fields[1]) if fields[1] else Dangle
            sendmsg = '<ACK,>'
        elif type == 'MAXVEL':  # Server is asking for max speed
            roverlog.debug("Sending maxvel.")
            sendmsg = '<MYMAXV,%.3f>' % maxvel
        elif type == 'HEART':  # Server is asking for position periodically
            roverlog.debug("Setting heartbeat and period.")
            heartbeat = True
            heartperiod = int(fields[0])*1e-3 if fields[0] else heartperiod
            if heartperiod <= 0.01:  # Don't go below 10 ms
                heartperiod = 0.01
            sendmsg = '<ACK,>'
        elif type == 'SILENT':  # Server is asking to stop periodic updates
            roverlog.debug("Stopping heartbeat.")
            heartbeat = False
            sendmsg = '<ACK,>'
        elif type in ['HALT', 'FWD', 'BWD', 'CFWD', 'CBWD', 'TURN',
                      'ATURN', 'CTURN', 'GOTO', 'OBS', 'POBS']:
            roverlog.debug("Received verb command: %s" % servercommand)
            moveflag = verb_dict[type]
            oldmoveflag = -1  # Make this different from moveflag
            movefields = fields
        elif type == 'DIE':  # Server is closing connection
            roverlog.info("Server died.")
            server.close()
            running = False
            moveflag = verb_dict['HALT']
        if sendmsg:
            server.send(sendmsg.encode())
    servercommand = ''
    cflag = False
    commandstate = 0
    return


def state_machine_chug():
    global heartbeat, heartperiod, mytimers, xpos, ypos, angle
    global moveflag, oldmoveflag, verb_dict, movefields, state
    global vx, vy, angvel
    sendmsg = ''
    curtime = time()
    # Obey heartbeat directive
    if heartbeat:
        if curtime-mytimers.heart >= heartperiod:
            mytimers.heart = curtime
            sendmsg = '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
            roverlog.debug('heartbeat.')
    # Handle other verbs
    # ['HALT', 'FWD', 'BWD', 'CFWD', 'CBWD', 'TURN',
    #  'ATURN', 'CTURN', 'GOTO', 'OBS', 'POBS']
    if moveflag == verb_dict['GOTO']:
        # Just received command
        # if mytimers.gotostep == 0 and oldmoveflag != moveflag:
        if oldmoveflag != moveflag:
            state = 1  # Moving
            sendmsg = '<ACK,%d>' % state
            mytimers.goto_x = float(movefields[0])
            mytimers.goto_y = float(movefields[1])
            if movefields[2] == '':
                mytimers.goto_vel = defvel  # default speed
            elif abs(float(movefields[2])) > maxvel:
                mytimers.goto_vel = maxvel  # maximum speed
            else:
                mytimers.goto_vel = abs(float(movefields[2]))
            if abs(mytimers.goto_x - xpos) > 0.0:
                tanarg = (mytimers.goto_y-ypos)/(mytimers.goto_x-xpos)
                destang = np.degrees(np.arctan(tanarg))
                if mytimers.goto_y >= ypos and mytimers.goto_x < xpos:
                    destang += 180
                elif mytimers.goto_y <= ypos and mytimers.goto_x < xpos:
                    destang += 180
                destang = destang % 360
            else:
                if mytimers.goto_y > ypos:
                    destang = 90.0
                else:
                    destang = 270.0
            mytimers.goto_destang = destang
            if movefields[3] != '':
                mytimers.goto_ang = float(movefields[3]) % 360
            else:
                mytimers.goto_ang = destang
            diffang = (destang-angle)
            if abs(diffang) > 180:
                diffang = np.sign(diffang)*(abs(diffang) - 360)
            if abs(destang-angle) == 180:
                diffang = 180
            # Determine which way to turn
            angvel = (-1)*abs(angvel) if diffang < 0 else abs(angvel)
            mytimers.gong = abs(diffang)/abs(angvel) + curtime
            mytimers.gotostep = 1
            mytimers.ping = curtime
            oldmoveflag = moveflag
            roverlog.debug("GOTO beginning first turn.")
        elif mytimers.gotostep == 1:  # In first-turn mode
            if mytimers.gong > curtime:
                angle += angvel*(curtime-mytimers.ping)
                mytimers.ping = curtime
            else:
                angle = mytimers.goto_destang
                sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
                mytimers.gotostep = 2  # First turn finished. Start forward
                dist = np.sqrt(
                    (mytimers.goto_x-xpos)**2+(mytimers.goto_y-ypos)**2)
                mytimers.gong = abs(dist)/mytimers.goto_vel + curtime
                [vx, vy] = [mytimers.goto_vel*ncos(angle),
                            mytimers.goto_vel*nsin(angle)]
                mytimers.ping = curtime
                roverlog.debug("GOTO finished first turn. Now forward.")
        elif mytimers.gotostep == 2:  # In forward mode
            if mytimers.gong > curtime:
                xpos += vx*(curtime-mytimers.ping)
                ypos += vy*(curtime-mytimers.ping)
                mytimers.ping = curtime
            else:
                roverlog.debug('GOTO finished forward.')
                sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
                mytimers.gotostep = 3  # Begin second turn
                diffang = mytimers.goto_ang-mytimers.goto_destang
                if abs(diffang) > 180:
                    diffang = np.sign(diffang)*(abs(diffang) - 360)
                if abs(mytimers.goto_ang-mytimers.goto_destang) == 180:
                    diffang = 180
                # Determine which way to turn
                angvel = (-1)*abs(angvel) if diffang < 0 else abs(angvel)
                mytimers.gong = abs(diffang)/abs(angvel) + curtime
                mytimers.ping = curtime
        elif mytimers.gotostep == 3:
            if mytimers.gong > curtime:
                angle += angvel*(curtime-mytimers.ping)
                mytimers.ping = curtime
            else:
                state = 0  # st_IDLE
                angle = angle % 360
                [moveflag, oldmoveflag] = [verb_dict['HALT']]*2
                sendmsg = '<ACK,%d>' % state
                angle = mytimers.goto_ang
                sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
                mytimers.gotostep = 0  # Last turn finished.
                roverlog.debug('GOTO finished last turn.')
    if moveflag in [verb_dict['FWD'], verb_dict['BWD'], verb_dict['CFWD'],
                    verb_dict['CBWD']]:
        if oldmoveflag == moveflag:
            if mytimers.gong > curtime or moveflag in [
                    verb_dict['CFWD'], verb_dict['CBWD']]:
                xpos += vx*(curtime-mytimers.ping)
                ypos += vy*(curtime-mytimers.ping)
                mytimers.ping = curtime
            if mytimers.gong < curtime and moveflag in [
                    verb_dict['FWD'], verb_dict['BWD']] and state != 0:
                roverlog.debug('Stopping.')
                state = 0  # st_IDLE
                [moveflag, oldmoveflag] = [verb_dict['HALT']]*2
                sendmsg = '<ACK,%d>' % state
                sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
        else:
            roverlog.debug('Moving.')
            state = 1  # st_MOVE
            sendmsg = '<ACK,%d>' % state
            if moveflag in [verb_dict['FWD'], verb_dict['BWD']]:
                speed = float(movefields[1]) if movefields[1] != '' else defvel
                speed = maxvel if speed > maxvel else speed
                mytimers.gong = abs(float(movefields[0]))/speed + curtime
            else:
                speed = float(movefields[0]) if movefields[0] != '' else defvel
                speed = abs(speed)
                speed = maxvel if speed > maxvel else speed
            [vx, vy] = [speed*ncos(angle), speed*nsin(angle)]
            if moveflag in [verb_dict['BWD'], verb_dict['CBWD']]:
                [vx, vy] = [(-1)*vx, (-1)*vy]
            mytimers.ping = curtime
            oldmoveflag = moveflag
    elif moveflag in [verb_dict['TURN'], verb_dict['ATURN'],
                      verb_dict['CTURN']]:
        if oldmoveflag == moveflag:
            if mytimers.gong > curtime or moveflag == verb_dict['CTURN']:
                angle += angvel*(curtime-mytimers.ping)
                mytimers.ping = curtime
            if mytimers.gong < curtime and moveflag in [
                    verb_dict['TURN'], verb_dict['ATURN']] and state != 0:
                state = 0  # st_IDLE
                sendmsg = '<ACK,%d>' % state
                if moveflag == verb_dict['TURN']:
                    angle = float(movefields[0]) % 360.0
                else:
                    angle = (angle+np.sign(angvel)*float(movefields[0])) % 360
                sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
                [moveflag, oldmoveflag] = [verb_dict['HALT']]*2
                angle = angle % 360
        else:
            state = 1  # st_MOVE
            sendmsg = '<ACK,%d>' % state
            if moveflag == verb_dict['TURN']:
                destang = float(movefields[0]) % 360
                diffang = (destang-angle)
                if abs(diffang) > 180:
                    diffang = np.sign(diffang)*(abs(diffang) - 360)
                if abs(destang-angle) == 180:
                    diffang = 180
                # Determine which way to turn
                angvel = (-1)*abs(angvel) if diffang < 0 else abs(angvel)
                mytimers.gong = abs(diffang)/abs(angvel) + curtime
            elif moveflag == verb_dict['ATURN']:
                diffang = float(movefields[1]) % 360
                if int(movefields[0]) != 0:
                    angvel = (-1)*abs(angvel)
                else:
                    angvel = abs(angvel)
                mytimers.gong = abs(diffang)/abs(angvel) + curtime
            elif moveflag == verb_dict['CTURN']:
                if int(movefields[0]) != 0:
                    angvel = (-1)*abs(angvel)
                else:
                    angvel = abs(angvel)
            mytimers.ping = curtime
            oldmoveflag = moveflag
    elif moveflag == verb_dict['HALT'] and oldmoveflag != verb_dict['HALT']:
        roverlog.debug('HALTED.')
        oldmoveflag = moveflag
        mytimers.gong = curtime  # stop moving
        angle = angle % 360
        state = 0  # st_IDLE
        sendmsg = '<ACK,%d>' % state
        sendmsg += '<MYPOS,%.3f,%.3f,%.3f>' % (xpos, ypos, angle)
    if sendmsg:
        server.send(sendmsg.encode())
    return


while running:
    # 1. Handle command assembly from server
    # 2. Handle console/terminal user input
    # 3. Respond to assembled commands
    # 4. Handle timed execution of current state

    # Check for inputs from user (console/terminal) and server
    soclist = [sys.stdin, server]
    inlist, outlist, exlist = select.select(soclist, [], [], 0.1)

    # 1. Handle command assembly from server
    if server in inlist:
        inpacket = server.recv(16)
        if not inpacket:  # has no content. Server died on us
            roverlog.info("Server died.")
            server.close()
            running = False
        inbuffer += inpacket.decode()
        if len(inbuffer) > inbufmax:
            roverlog.warning("Buffer is becoming too big (%d)!\n" %
                             len(inbuffer))
        if commandstate != 2:
            servercommand, inbuffer, commandstate = TCPcompose(
                growstr=servercommand, newstr=inbuffer,
                messagestate=commandstate)
        if commandstate == 2:
            cflag = True  # A full command has been assembled

    # 2. Handle console/terminal user input
    if sys.stdin in inlist:
        message = sys.stdin.readline().split()
        if message:
            message = message[0]
        else:
            message = ' '
        if message in ['quit', 'bye', 'die', 'exit']:
            server.send('<BYE>'.encode())
            server.close()
            running = False
        else:
            valid, _, _ = cmdparse(version=version, kind='msg',
                                   instr=message)
            if valid:
                server.send(message.encode())
                roverlog.info("sent.")
            else:
                roverlog.warning("Invalid message.")

    # 3. Respond to assembled commands, modify appropriate state variables
    if cflag:
        respond_to_server_cmd()

    # 4. Handle timed execution of current state
    state_machine_chug()
try:
    server.close()
except socket.error:
    pass
