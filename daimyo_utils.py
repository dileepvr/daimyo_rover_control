import socket
import select
import copy
import threading
import logging
import os
import json
from time import time
import datetime


# List of valid commands/messages and number of fields
# First key is rover version name
vtypes = {}
vtypes['0'] = {}
vtypes['0']['cmd'] = [['ID', 0],
                      ['POS', 0],
                      ['SETPOS', 3],
                      ['PRES', 0],
                      ['SETPRES', 2],
                      ['MAXVEL', 0],
                      ['HEART', 1],
                      ['SILENT', 0],
                      ['HALT', 0],
                      ['FWD', 3],
                      ['BWD', 3],
                      ['CFWD', 2],
                      ['CBWD', 2],
                      ['TURN', 2],
                      ['ATURN', 3],
                      ['CTURN', 2],
                      ['GOTO', 5],
                      ['OBS', 2],
                      ['POBS', 3],
                      ['DIE', 0]]
vtypes['0']['msg'] = [['MYID', 2],
                      ['MYPOS', 3],
                      ['MYPRES', 2],
                      ['MYMAXV', 1],
                      ['ACK', 1],
                      ['COL', 2],
                      ['FAIL', 0],
                      ['TIMEOUT', 0],
                      ['DOBS', 1],
                      ['BYE', 0]]
vtypes['0_RFID'] = copy.deepcopy(vtypes['0'])
vtypes['0_RFID']['cmd'].extend([['SEARCH', 3]])
vtypes['0_RFID']['msg'].extend([['RFID', 3]])


def cmdparse(version='0', kind='cmd', instr='<x>'):

    if len(instr) < 3:
        return False, '', []

    strlist = instr[1:-1].split(',')
    cmdtype = strlist.pop(0)
    if [cmdtype, len(strlist)] in vtypes[version][kind]:
        return True, cmdtype, strlist
    return False, '', []


def TCPcompose(growstr='', newstr='', start='<', end='>',
               messagestate=0, maxnew=2048):
    '''

    TCP chops intended transmitted string into arbitrarily-sized packets.
    This function helps reassemble it. The arguments 'start' and 'end'
    define the beginning and ending substrings of string intended for
    capture. New TCP messages supplied in argument 'newstr' are scanned for
    first occurences of either substring (depending on value of the
    'messagestate' argument), and this function returns a growing string
    and the remainder of 'newstr', along with an updated state variable.
    Meaning of 'messagestate' values and return state values:
    0:\t Treat 'growstr' as empty. Look for 'start' substring to start growth
    1:\t Look for 'end' substring. If 'start' found first, regrow from there
    2:\t (return only) 'end' found. A full message has been reassembled
    3:\t (return only) 'newstr' is longer than 'maxnew'. Not processing
    4:\t (return only) Illegal input 'messagestate' value. Not processing
    5:\t (return only) 'start' and/or 'end' are empty or equal. Not processing

    '''
    threadname = threading.currentThread().getName()
    TCPlog = logging.getLogger('TCPcompose:%s' % threadname)

    # Handle basic errors
    if len(newstr) > maxnew:
        TCPlog.warning("Got 'newstr' of length %d" %
                       len(newstr))
        return growstr, newstr, 3
    if messagestate not in [0, 1]:
        TCPlog.warning("Got messagestate: %d" % messagestate)
        return growstr, newstr, 4
    if 0 in [len(start), len(end)] or start == end:
        TCPlog.warning("Got 'start'=%s and 'end'=%s" %
                       (start, end))
        return growstr, newstr, 5

    # Handle proper input arguments
    elif messagestate == 0:
        st_indx = newstr.find(start)
        if st_indx == -1:  # 'start' not found in 'newstr'
            TCPlog.debug("'newstr' didn't contain 'start'=%s" %
                         start)
            return '', newstr, 0
        en_indx = newstr.find(end)  # will return -1 if 'end' not found
        if st_indx < en_indx:  # 'start' preceeds 'end'
            TCPlog.debug("Full message captured: %s" %
                         newstr[st_indx:en_indx+1])
            return newstr[st_indx:en_indx+1], newstr[en_indx+1:], 2
        else:  # 'end' not found
            TCPlog.debug("Partial message started: %s" %
                         newstr[st_indx:])
            return newstr[st_indx:], '', 1
    elif messagestate == 1:
        en_indx = newstr.find(end)
        st_indx = newstr.find(start)
        if st_indx > -1:  # 'start' was found
            if st_indx < en_indx:  # both 'start' and 'end' exist in that order
                TCPlog.debug("Full message captured: %s" %
                             newstr[st_indx:en_indx+1])
                return newstr[st_indx:en_indx+1], newstr[en_indx+1:], 2
            elif en_indx > -1:  # 'end' was found and 'start' didn't preceed it
                TCPlog.debug("Message ending captured: %s" %
                             newstr[:en_indx+1])
                return growstr+newstr[:en_indx+1], newstr[st_indx:], 2
            else:  # 'end' was not found
                TCPlog.debug("New partial message started: %s" %
                             newstr[st_indx:])
                return newstr[st_indx:], '', 1
        else:  # 'start' was not found
            if en_indx > -1:  # 'end' was found
                TCPlog.debug("Message ending captured: %s" %
                             newstr[:en_indx+1])
                return growstr+newstr[:en_indx+1], '', 2
            else:  # 'end' was not found
                TCPlog.debug("Full packet captured: %s" %
                             newstr)
                return growstr+newstr, '', 1


# Rover states
num_states = 5
st_dict = dict([(0, 'st_IDLE'),
                (1, 'st_MOVE'),
                (2, 'st_WAIT'),
                (3, 'st_PAUSED'),
                (4, 'st_LISTEN'),
                (5, 'st_SEARCH')])


class Rover:

    def __init__(self, conn, addr, streamhandler, filehandler, loglevel):
        self.conn = conn
        self.addr = addr
        self.logsh = streamhandler  # Stream handler for logging
        self.logfh = filehandler  # File handler for logging (main log file)
        self.loglvl = loglevel  # Log level
        self.name = 'Rover-0'
        self.version = '0'
        self.x = 0.0  # Spawn point (meters)
        self.y = 0.0
        self.xold = 0.0
        self.yold = 0.0
        self.angle = 0.0  # Angle of forward with global (map) x-axis
        self.Dxy = 0.01  # XY-precision (meters)
        self.Dangle = 1  # Angle precision (degrees)
        self.maxvel = 0.01  # Maximum (latching) speed (m/s)
        self.alive = True
        self.state = 0  # st_IDLE
        self.superstate = -1  # For command sequences and patrols
        self.loopflag = False  # For command sequence wrap around
        self.seqfile = ''  # Command/patrol sequence file
        self.seqlist = []  # List of sequence commands
        self.numseq = 0  # Length of sequence
        self.gong = time()  # For time keeping when asked to wait
        self.heartbeat = False  # Whether or not in heartbeat mode
        self.lock = threading.RLock()
        self.cflag = False  # Flag for when server command is available
        self.command = ''
        self.mflag = False  # Flag for when message from rover is ready
        self.message = ''
        self.sflag = False  # Flag to pass message upstream to server thread
        self.smsg_buffer = []  # Buffer of messages meant for server thread
        self.wflag = False  # Flag to pass info to webUI server thread
        self.wlist = []  # List of params to send webUI server thread
        self.pause = False   # Whether to pause stepping through sequence
        self.pflag = False   # Used to track reply to HALT when paused
        self.pause_t_store = 0.0  # Store wait time when paused
        self.pause_s_store = 0  # Store state when paused
        self.listen_str = []  # Strings to listen for (sequence command)
        self.syn_str = []  # Strings broadcasted from main thread
        self.syn_limit = 10  # Maximum number of syn broadcasts to store
        self.ackflag = False  # Marking reception of ACK message
        self.thread = threading.Thread(target=self.run_loop)
        # self.thread.setName(self.name)
        _ddate = datetime.datetime.now()
        self.dataprefix = 'datalogs/'+_ddate.strftime(
            '%Y_%m_%d/')+_ddate.strftime('%H_%M_%S_')
        self.datafile = self.dataprefix+self.name+'.dat'
        self.datfid = open(self.datafile, 'a')
        _msgdat = '# ID, version, Thread: ({on}, v{ov}, {thread}) '.format(
            on=self.name, ov=self.version,
            thread=self.thread.getName())
        self.datfid.write(_msgdat+'\n')
        self.datfid.write('# Time (s)\tx (m)\ty (m)\n')
        self.datfid.flush()
        self.log = logging.getLogger(self.name+':'+self.thread.getName())
        self.log.setLevel(self.loglvl)
        self.log.addHandler(self.logfh)
        self.log.addHandler(self.logsh)
        self.thread.start()

    def state_machine_chug(self):
        self.lock.acquire()
        # prune syn_str list
        if len(self.syn_str) > self.syn_limit:
            self.syn_str.pop(0)  # Lose oldest broadcast
        # Patrols and command sequence stuff will go here
        if self.superstate == -2 and self.seqfile != '':  # load new seq file
            if os.path.exists(self.seqfile):
                self.log.info('Loading sequence: %s' % self.seqfile)
                _fp = open(self.seqfile, 'r')
                _line1 = _fp.readline()
                try:
                    _line1 = json.loads(_line1.rstrip())
                    [_line1['loopflag'], _line1['start']]
                except (TypeError, KeyError, json.decoder.JSONDecodeError):
                    self.log.warning('Invalid sequence file: %s' %
                                     self.seqfile)
                    _fp.close()
                    self.superstate = -1
                else:
                    self.seqlist = [
                        line.rstrip('\n').split('>', 1)[0] for line in _fp]
                    _fp.close()
                    # Remove empty strings
                    self.seqlist[:] = [x+'>' for x in self.seqlist if x != '']
                    try:
                        self.command = '<HALT>'
                        self.clean_send()
                    except OSError:
                        self.die()
                    self.numseq = len(self.seqlist)
                    self.loopflag = _line1['loopflag']
                    self.log.debug('Setting loopflag to %r' % self.loopflag)
                    self.superstate = _line1['start']
                    if self.superstate > self.numseq - 1:
                        self.superstate = self.numseq - 1
                    self.log.info('Sequence starting at %d of %d.' % (
                        self.superstate, self.numseq-1))
                    self.ackflag = False
                    self.state = 0
                    self.pause = False
            else:
                self.log.warning("Sequence file %s not found." % self.seqfile)
                self.superstate = -1
        if self.superstate > -1 and self.pause:  # Paused
            if not self.pflag and self.state in [1, 4]:  # st_MOVE, st_SEARCH
                self.pause_s_store = self.state  # store state when paused
                self.ackflag = False
                self.command = '<HALT>'  # Become IDLE first
                self.clean_send()
                self.pflag = True
            elif self.state in [0, 2]:  # st_IDLE, st_WAIT
                if self.pause_s_store == 0:
                    self.pause_s_store = self.state  # store state when paused
                self.log.info('Paused in middle of sequence (%d of %d).' %
                              (self.superstate, self.numseq-1))
                if self.state == 2:  # st_WAIT
                    self.pause_t_store = self.gong - time()  # pending wait
                self.pflag = False
                self.state = 3  # st_PAUSE
                self.wflag = True
                self.wlist = ['ACK', self.state]  # Let webUI know state change
        elif self.superstate > -1 and self.state == 3 and not self.pause:
            # Unpaused
            self.log.info('Unpaused. Continuing sequence (%d of %d)' %
                          (self.superstate, self.numseq-1))
            self.state = self.pause_s_store
            self.pause_s_store = 0  # Reset this
            self.wflag = True
            self.wlist = ['ACK', self.state]  # Let webUI know state change
            if self.state == 2:  # st_WAIT
                self.gong = time() + self.pause_t_store
            elif self.state == 1:  # st_MOVE
                self.state = 0  # st_IDLE
                self.superstate -= 1
                if self.superstate < 0:
                    self.superstate = self.numseq - 1
        elif self.superstate > -1 and self.state == 4 and any(
                string in self.syn_str for string in self.listen_str):
            # Found a listen string in the broadcast list
            self.state = 0  # IDLE
            self.log.info('Heard %r in broadcasts.' %
                          set(self.listen_str).intersection(
                              set(self.syn_str)))
            self.listen_str = []
            self.syn_str = []
        elif self.superstate > -1 and self.ackflag and self.state == 0:  # IDLE
            # ready for next sequence command
            if self.superstate >= self.numseq:
                if self.loopflag:  # loop around
                    self.superstate = 0
                else:
                    self.superstate = -1  # Sequence is finished
                    self.log.info('Sequence completed.')
            if self.superstate > -1 and not self.pause:  # yeah. check again
                if self.seqlist[self.superstate][0:6] == '<WAIT,':
                    try:
                        _waitfor = float(
                            self.seqlist[self.superstate].split('>')[
                                0].split(',', 1)[1])
                    except (ValueError, IndexError):
                        self.superstate = -1  # Wait command malformed
                        self.log.warning(
                            'Sequence interrupted. Bad wait time supplied')
                    else:
                        self.gong = time() + _waitfor
                        self.state = 2  # st_WAIT
                        self.wflag = True
                        self.wlist = ['ACK', self.state]  # Let webUI know
                        self.log.info('Started wait for %.2f seconds' %
                                      _waitfor)
                elif self.seqlist[self.superstate][0:5] == '<SYN,':
                    # Broadcast message found. Send to server main thread
                    self.sflag = True
                    self.smsg_buffer.append(self.seqlist[self.superstate])
                    self.superstate += 1
                elif self.seqlist[self.superstate][0:5] == '<LIS,':
                    # Set up to listen for one of the string broadcasts
                    self.listen_str = self.seqlist[
                        self.superstate][5:-1].split(',')
                    self.log.info('Listening for %r' % self.listen_str)
                    self.state = 4  # st_LISTEN
                    self.wflag = True
                    self.wlist = ['ACK', self.state]  # Let webUI know
                    self.superstate += 1
                else:
                    self.command = self.seqlist[self.superstate]  # payload
                    self.log.info('Continuing sequence at %d of %d.' % (
                        self.superstate, self.numseq-1))
                    self.clean_send()
                    self.ackflag = False
                    self.superstate += 1
        elif self.superstate > -1 and self.state == 2:  # st_WAIT
            if time() > self.gong:
                self.state = 0
                self.superstate += 1
        elif self.superstate == -2:
            self.log.warning('No sequence file supplied.')
            self.superstate = -1
        self.lock.release()

    def msg_parser(self):
        '''Parse messages received from field rover and take action'''
        _floatflag = True
        _valid, _type, _fields = cmdparse(
            version=self.version, kind='msg', instr=self.message)
        if not _valid:
            self.log.warning("Received invalid message: %s" % self.message)
        else:
            self.log.debug("Processing: %s" % self.message)
            if _type != 'ACK' and '' in _fields:  # Check for empty fields
                self.log.warning("Received empty field(s): %s"
                                 % self.message)
                _floatflag = False
            elif _type not in ['ACK', 'MYID', 'RFID']:
                # Check if float cast-able
                try:
                    [float(ii) for ii in _fields]
                except ValueError:
                    self.log.warning("Received bad field(s): %s"
                                     % self.message)
                    _floatflag = False

            if _floatflag:
                if _type == 'MYID':
                    if [self.name, self.version] != [str(_fields[0]),
                                                     str(_fields[1])]:
                        # Change data logging file name
                        self.datfid.close()
                        os.rename(
                            self.datafile, self.dataprefix+str(
                                _fields[0])+'.dat')
                        self.datafile = self.dataprefix+str(_fields[0])+'.dat'
                        self.datfid = open(self.datafile, 'a')
                        _msgdat = '# ID: ({on}, v{ov})'.format(
                            on=self.name, ov=self.version)
                        _msgdat += ' -> ({nn}, v{nv})'.format(
                            nn=_fields[0], nv=_fields[1])
                        self.datfid.write(_msgdat+'\n')
                        self.datfid.flush()
                    self.name = str(_fields[0])
                    self.version = str(_fields[1])
                    self.log.info("Changing (name, version) to (%s, %s)" %
                                  (self.name, self.version))
                    for _handler in self.log.handlers:
                        _handler.close()
                        self.log.removeFilter(_handler)
                    self.log = logging.getLogger(
                        self.name+':'+self.thread.getName())
                    self.log.setLevel(self.loglvl)
                    self.log.addHandler(self.logfh)
                    self.log.addHandler(self.logsh)
                    self.sflag = True  # Report ID change to main server
                    self.smsg_buffer.append(self.message)
                    self.wflag = True  # Report ID change to webUI
                    self.wlist = [_type, self.name, self.version]
                elif _type == 'MYPOS':
                    [self.x, self.y, self.angle] = [
                        float(_fields[0]), float(_fields[1]),
                        float(_fields[2])]
                    if [self.xold, self.yold] != [self.x, self.y]:
                        # _msgdat = '{t}\t{x}\t{y}'.format(
                        #     t=time(), x=self.x, y=self.y)
                        _msgdat = '%.1f\t%.3f\t%.3f\n' % (
                            time(), self.x, self.y)
                        self.datfid.write(_msgdat)
                        self.datfid.flush()
                        [self.xold, self.yold] = [self.x, self.y]
                    if self.ackflag and self.state == 0:  # st_IDLE
                        if self.superstate == -1:
                            self.ackflag = False
                        self.log.info(
                            "Updated POS to (%.3f, %.3f, %.3f)" %
                            (self.x, self.y, self.angle))
                    else:
                        self.log.debug(
                            "Updated POS to (%.3f, %.3f, %.3f)" %
                            (self.x, self.y, self.angle))
                    self.wflag = True  # Send upstream to webUI thread
                    self.wlist = [_type, self.x, self.y, self.angle]
                elif _type == 'MYPRES':
                    [self.Dxy, self.Dangle] = [float(_fields[0]),
                                               float(_fields[1])]
                    self.log.info("Updated PRES to (%.3f, %.3f)" %
                                  (self.Dxy, self.Dangle))
                elif _type == 'MYMAXV':
                    self.maxvel = float(_fields[0])
                    self.log.info("Updated maxvel to %.3f" % self.maxvel)
                elif _type == 'ACK':
                    self.ackflag = True
                    if _fields[0] == '':
                        self.log.debug("Received <ACK,>")
                    else:
                        try:
                            _tempflag = int(_fields[0]) < num_states
                        except ValueError:
                            self.log.warning("Received bad fields: %s" %
                                             self.message)
                            _tempflag = False
                        if _tempflag:
                            if int(_fields[0]) != self.state:
                                self.log.debug("State changed: %s->%s" %
                                               (st_dict[self.state],
                                                st_dict[int(_fields[0])]))
                            self.state = int(_fields[0])
                            self.wflag = True  # Send upstream to webUI thread
                            self.wlist = [_type, self.state]
                elif _type == 'COL':
                    self.wflag = True  # Report obstacle to webUI thread
                    self.wlist = [_type, float(_fields[0]), float(_fields[1])]
                    self.log.debug("Collision reported: %s" % self.message)
                elif _type in ['FAIL', 'TIMEOUT']:
                    self.sflag = True  # Report this to main server thread
                    self.smsg_buffer.append(self.message)
                    if _type == 'FAIL':
                        self.log.warning("Failure reported!")
                        try:
                            self.conn.send('<HALT>'.encode())
                        except OSError:
                            self.die()
                        self.superstate = -1
                    else:
                        self.log.debug("Timeout reproted!")
                        self.paused = True  # timeout => sequence paused
                elif _type == 'DOBS':
                    self.wflag = True  # Report dist to webUI thread
                    self.wlist = [_type, float(_fields[0])]
                    self.log.debug("Distance from obstacle: %s" %
                                   self.message)
                elif _type == 'BYE':  # Field rover is closing connection
                    self.log.info("Rover is closing connection.")
                    self.die()
                elif _type == 'RFID':  # Found an RFID, report upstream
                    self.log.info("Found: %s" % self.message)
                    self.sflag = True  # Report RFID tag to main server
                    self.smsg_buffer.append(self.message)
        self.mflag = False
        return True

    def clean_send(self):
        '''Check if valid command, then send to field rover'''
        _sendflag = True
        _valid, _type, _fields = cmdparse(
            version=self.version, kind='cmd', instr=self.command)
        if not _valid:
            _sendflag = False
        else:
            # Handle optional fields and decide if to check for floats
            # List of _types where last field is optional
            if _type in ['SETPOS', 'SETPRES', 'HEART', 'CFWD', 'CBWD',
                         'FWD', 'BWD',
                         'TURN', 'ATURN', 'CTURN', 'GOTO', 'OBS', 'POBS']:
                # This next line won't be sent.
                _fields[-1] = '0' if _fields[-1] == '' else _fields[-1]
            # List of _types where second-last field is optional
            if _type in ['FWD', 'BWD', 'CFWD', 'CBWD', 'GOTO', 'OBS', 'POBS']:
                # This lext line won't be sent.
                _fields[-2] = '0' if _fields[-2] == '' else _fields[-2]
            if _type == 'GOTO':  # This one has third-last field optional
                _fields[-3] = '0' if _fields[-3] == '' else _fields[-3]
            if _type in ['SEARCH']:
                _fields[0] = '0' if _fields[0] == '' else _fields[0]
                _fields[1] = '0'

            try:  # Check if float cast-able. Works with empty list
                [float(ii) for ii in _fields]
            except ValueError:
                _sendflag = False

        if _sendflag:
            if _type == 'HEART':
                self.heartbeat = True
                self.wflag = True
                self.wlist = ['HEART']
            elif _type == 'SILENT':
                self.heartbeat = False
                self.wflag = False
                self.wlist = ['SILENT']
            try:
                self.conn.send(self.command.encode())
                self.log.info("Sent: %s" % self.command)
            except OSError:
                self.die()
        else:
            self.log.warning('Bad command string: %s' % self.command)
            _sendflag = False
        self.command = ''
        self.cflag = False
        return _sendflag

    def die(self):
        for _handler in self.log.handlers:
            _handler.close()
            self.log.removeFilter(_handler)
        self.datfid.close()
        self.conn.close()
        self.lock.acquire()
        self.alive = False
        self.lock.release()
        self.log.info('Died.')

    def run_loop(self):

        _inbuffer = ''
        _msgstate = 0
        _rogue_stream_limit = 10240
        while self.alive:
            # 1. Handle message assembly from field rover
            # 2. Handle assembled message from field rover
            # 3. Handle new commands from server to change state
            # 4. Handle timed execution of current command and state

            # 1. Message assembly, append received packets to _inbuffer
            inl, outl, exl = select.select(
                [self.conn], [], [], 0.1)  # timeout necessary
            if self.conn in inl:
                _inpacket = self.conn.recv(2048).decode()
                if not _inpacket:
                    # _message has no content. Die.
                    self.die()
                    break
                _inbuffer += _inpacket
                if len(_inbuffer) > 2048:
                    self.log.warning('_inbuffer length: %d' %
                                     len(_inbuffer))
                    if len(_inbuffer) > _rogue_stream_limit:
                        self.die()
                        break
            # Eat through _inbuffer for message assembly
            if _msgstate != 2 and _inbuffer != '':
                self.message, _inbuffer, _msgstate = TCPcompose(
                    growstr=self.message, newstr=_inbuffer,
                    messagestate=_msgstate)
            if _msgstate == 2:  # valid message has been assembled
                self.mflag = True

            # 2. Process message from field rover
            if self.mflag:
                self.lock.acquire()
                self.msg_parser()
                _msgstate = 0
                self.lock.release()

            # 3. Process commands from server
            if self.cflag:
                self.lock.acquire()
                if self.superstate != -1 and self.command[0:4] not in [
                        '<HEA', '<SIL']:
                    # If in middle of command sequence
                    try:
                        self.conn.send('<HALT>'.encode())
                    except OSError:
                        self.die()
                    self.superstate = -1
                self.clean_send()
                self.lock.release()

            # 4. Perform command sequence executions
            self.state_machine_chug()
