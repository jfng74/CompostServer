#!/usr/bin/python

from socket import *
import subprocess
import pickle
import serial
import time
import thread
import csv
import rrdtool
import struct
import os

from compost_data import *
from fan_compost_config import *

# Global Variables

# Feather Packet
#[HEADER][MESSAGE_TYPE][NODE_ID][DATA_TYPE][DATA_1][DATA_2][DATA_3][DATA_4][END]
# Feather message header
FEATHER_MSG_HEADER = 0xaa
# Feather ending message
FEATHER_MSG_END = 0x55

# Feather message type
FEATHER_MSG_QUERY_DATA = 0x01
FEATHER_MSG_RESPONSE_DATA = 0x02
FEATHER_MSG_SET_DATA = 0x03
FEATHER_MSG_GET_DATA = 0x04
FEATHER_MSG_GET_ALL_DATA = 0x05
FEATHER_MSG_RESPONSE_ALL_DATA = 0x06
FEATHER_MSG_READY_FOR_COMMANDS = 0x07
FEATHER_MSG_REQUEST_ALL_NODE_SLEEP = 0x08
FEATHER_MSG_SEND_ALL_TEMP = 0x09
FEATHER_MSG_ALL_DATA_READY = 0x10

# Feather message data type
TEMP_1 = 0x00  # Temperature de surface
TEMP_2 = 0x01  # Temperature de profondeur
TEMP_3 = 0x02  # Temperature ambiante
TEMP_4 = 0x03
HUMIDITY_1 = 0x04  # Humidite ambiante
HUMIDITY_2 = 0x05
OXYGEN_1 = 0x06
CO2_1 = 0x07
TURN_ON_RELAY = 0x08
TURN_OFF_RELAY = 0x09
RELAY_STATE = 0x10
MODE_AUTO = 0x11
READ_BATTERY_VOLTAGE = 0x12
READ_ALL_DATA = 0x13
RELAY_THRESHOLD = 0x14
DELAY_BETWEEN_READS = 0x15
LAST_RSSI = 0x16
SEND_ALL_TEMP = 0x17

TCP_STOP_DATA = 0
TCP_GET_NODE_DATA = 1
TCP_GET_NODE_0_DATA = 2
TCP_GET_NODE_1_DATA = 3
TCP_GET_NODE_2_DATA = 4
TCP_GET_NODE_3_DATA = 5
TCP_PUT_RELAIS_CONSIGNE = 6
TCP_PUT_RELAIS_ETAT = 7
TCP_PUT_RELAIS_DELAIS = 8
TCP_GET_RRDGRAPH = 14

os.chdir(fan_compost_dir)


class CompostFAN:
    def __init__(self, no):
        self.no = no
        self.dtt = int(time.time())  # tt - time in seconds
        self.rrd_graph = CompostRRDGRAPH()
        self.compost_fan_data = CompostFanData()
        self.compost_fan_config = CompostFanConfig()
        self.list_node_data = []
        for x in range(4):
            self.list_node_data.append(NodeData())
        self.ndr = NodeData()
        self.ndr_00 = NodeData()
        self.ndr_01 = NodeData()
        self.ndr_02 = NodeData()
        self.ndr_03 = NodeData()
        self.all_node = AllNode()
#        self.list_node_data[0] = [CompostFanData()]
        print self.list_node_data[0]
        print self.list_node_data[1]
        self.relais_etat = 0
        self.relais_t_avg = 0.0
        self.ser = 0
        self.node_id = 0
        self.msg_type = 0
        self.data_type = 0
        self.sock = socket(AF_INET, SOCK_STREAM)
        self.socket_data = 0
        self.cnt_serial_data_num = 1
        self.node_relais_ready_for_command = 0

    def init_serial(self):
        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.parity = serial.PARITY_NONE
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.port = serial_feather_port
        self.ser.timeout = 1
        self.ser.open()  # Opens SerialPort
        # print port open or closed

        if self.ser.isOpen():
            print 'Open: ' + self.ser.portstr

    def init_server(self):
        self.sock.bind(('', port))
        self.sock.listen(5)

    def read_float(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_float = struct.unpack('>f', b)
        return value_float[0]

    def read_int8(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_int8 = struct.unpack('>b', b)
        return value_int8[0]

    def read_uint8(self, b_bytes):
        b_array_data = bytearray(b_bytes)
        b = ''.join(chr(i) for i in b_array_data)
        import struct
        value_uint8 = struct.unpack('>B', b)
        return value_uint8[0]


    def handleClient(self, connection):
        print 'handleClient'

        state = 1
        cnt_client_data_num = 0

        while True:
            self.socket_data = connection.recv(1024)
            if not self.socket_data:
                print('no data')
                print
                break

            if self.socket_data == 'STOP_DATA':
                print ('STOP_DATA')
                state = TCP_STOP_DATA
            elif self.socket_data == 'GET_NODE_DATA':
#                print ('GET_NODE_DATA')
                reply_string = pickle.dumps(self.compost_fan_data)
                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_DATA
            elif self.socket_data == 'GET_NODE_0_DATA':
#                print ('GET_NODE_0_DATA')
#                print("Node ID : " + str(self.list_node_data[0].node_id))
#                print("T. 1 : " + str(self.list_node_data[0].t_1))
#                print("T. 2 : " + str(self.list_node_data[0].t_2))
#                print("T. 3 : " + str(self.list_node_data[0].t_3))
#                print("Humidity : " + str(self.list_node_data[0].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[0].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[0])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_0_DATA
            elif self.socket_data == 'GET_NODE_1_DATA':
#                print ('GET_NODE_1_DATA')
#                print("Node ID : " + str(self.list_node_data[1].node_id))
#                print("T. 1 : " + str(self.list_node_data[1].t_1))
#                print("T. 2 : " + str(self.list_node_data[1].t_2))
#                print("T. 3 : " + str(self.list_node_data[1].t_3))
#                print("Humidity : " + str(self.list_node_data[1].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[1].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[1])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_1_DATA
            elif self.socket_data == 'GET_NODE_2_DATA':
#                print ('GET_NODE_2_DATA')
#                print("Node ID : " + str(self.list_node_data[2].node_id))
#                print("T. 1 : " + str(self.list_node_data[2].t_1))
#                print("T. 2 : " + str(self.list_node_data[2].t_2))
#                print("T. 3 : " + str(self.list_node_data[2].t_3))
#                print("Humidity : " + str(self.list_node_data[2].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[2].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[2])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_2_DATA
            elif self.socket_data == 'GET_NODE_3_DATA':
#                print ('GET_NODE_3_DATA')
#                print("Node ID : " + str(self.list_node_data[3].node_id))
#                print("T. 1 : " + str(self.list_node_data[3].t_1))
#                print("T. 2 : " + str(self.list_node_data[3].t_2))
#                print("T. 3 : " + str(self.list_node_data[3].t_3))
#                print("Humidity : " + str(self.list_node_data[3].h_1))
#                print("Battery Voltage: " + str(self.list_node_data[3].bat_voltage))

                reply_string = pickle.dumps(self.list_node_data[3])
#                print reply_string
                connection.send(reply_string)
                state = TCP_GET_NODE_3_DATA
            elif self.socket_data == 'GET_RELAY_STATE':
                print ('GET_RELAY_STATE')
                reply_string = pickle.dumps(self.compost_fan_config)
                connection.send(reply_string)
            elif self.socket_data == 'PUT_RELAY_CONSIGNE':
                print ('PUT_RELAY_CONSIGNE')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_CONSIGNE
            elif self.socket_data == 'PUT_RELAIS_ETAT':
                print ('PUT_RELAIS_ETAT')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_ETAT
            elif self.socket_data == 'PUT_RELAIS_DELAIS':
                print ('PUT_RELAIS_DELAIS')
                reply_string = 'Put ready'
                connection.send(reply_string)
                state = TCP_PUT_RELAIS_DELAIS
            elif self.socket_data == 'GET_RRDGRAPH':
                reply_string = 'Get ready'
                connection.send(reply_string)
                state = TCP_GET_RRDGRAPH

            else:
                if state == 1:
                    reply = 'Echo=>%s at %s' % (self.socket_data, self.now())
                    print('Server reply : %s' % reply)
                    connection.send(reply.encode())
                elif state == TCP_PUT_RELAIS_CONSIGNE:
                    print ('Receive new consign')
                    self.compost_fan_config = pickle.loads(self.socket_data)
                    print(self.compost_fan_config.relais_consigne_temperature_fan)
                    ba = bytearray(struct.pack('f', self.compost_fan_config.relais_consigne_temperature_fan))
                    print ba[3]
                    print ba[2]
                    print ba[1]
                    print ba[0]
                    b_array_set_message_data = bytearray(9)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254
                    b_array_set_message_data[3] = RELAY_THRESHOLD
                    b_array_set_message_data[4] = ba[3]
                    b_array_set_message_data[5] = ba[2]
                    b_array_set_message_data[6] = ba[1]
                    b_array_set_message_data[7] = ba[0]
                    b_array_set_message_data[8] = FEATHER_MSG_END
                    print('Envoie la consigne...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Consigne envoye...')
                elif state == TCP_PUT_RELAIS_ETAT:
                    print ('Receive relais etat')
                    self.compost_fan_config = pickle.loads(self.socket_data)

                    b_array_set_message_data = bytearray(9)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254

                    if self.compost_fan_config.relais_etat == 1:
                        b_array_set_message_data[3] = TURN_ON_RELAY
                        print ('relais_on')
                    elif self.compost_fan_config.relais_etat == 0:
                        b_array_set_message_data[3] = TURN_OFF_RELAY
                        print ('relais_off')
                    elif self.compost_fan_config.relais_etat == 2:
                        b_array_set_message_data[3] = MODE_AUTO
                        print ('relais_auto')
                    b_array_set_message_data[4] = FEATHER_MSG_END
                    print('Envoie l etat du relais...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Etat du relais envoye...')
                elif state == TCP_PUT_RELAIS_DELAIS:
                    print ('Receive new delais')
                    self.compost_fan_config = pickle.loads(self.socket_data)
                    print(self.compost_fan_config.relais_delais)
                    ba = bytearray(struct.pack('B', self.compost_fan_config.relais_delais))
                    print ba[0]
                    b_array_set_message_data = bytearray(6)
                    b_array_set_message_data[0] = FEATHER_MSG_HEADER
                    b_array_set_message_data[1] = FEATHER_MSG_SET_DATA
                    b_array_set_message_data[2] = 254
                    b_array_set_message_data[3] = DELAY_BETWEEN_READS
                    b_array_set_message_data[4] = ba[0]
                    b_array_set_message_data[5] = FEATHER_MSG_END
                    print('Envoie du delais...')
                    self.writeserialdata(b_array_set_message_data)
                    state = TCP_STOP_DATA
                    print('Delais envoye...')
                elif state == TCP_GET_RRDGRAPH:
                    print ('Receive rrdgraph info')
                    self.rrd_graph = pickle.loads(self.socket_data)
                    print('graph id : ') + str(self.rrd_graph.graph_id)
                    print('graph_start : ') + self.rrd_graph.graph_start
                    print('graph_end : ') + self.rrd_graph.graph_end

                    open_file_string = ('node_' + str(self.rrd_graph.graph_id) + '.png', 'rb')
                    b = os.path.getsize('node_1.png')
                    print (open_file_string)
                    f_rrdgraph = open('node_1.png', 'rb')
                    reply_string = str(b)
                    connection.send(reply_string)
                    self.socket_data = connection.recv(1024)
#                    if self.socket_data == "OK":
#                        print ("OK")
                    reply_string = f_rrdgraph.read(1024)
                    ii = 1
                    while reply_string:
#                       print("Sending : " + str(ii))
                        connection.send(reply_string)
                        self.socket_data = connection.recv(1024)
#                        if self.socket_data == "OK":
#                            print ("OK")
                        reply_string = f_rrdgraph.read(1024)
                        ii += 1
                    print("Sending done")
#                    self.socket_data = connection.recv(1024)
#                    if self.socket_data == "OK":
#                        print ("OK")
                    f_rrdgraph.close()
                    state = TCP_STOP_DATA



        print('handleClient finish...')

    def now(self):
        return time.ctime(time.time())

    def writeserialdata(self, serial_data):
        print('writeserialdata')
        self.ser.write(serial_data)         #Writes to the SerialPort

    def read_test(self):
        while 1:
            timestamp = time.strftime('%X')
            line = timestamp
            bytesToRead = self.ser.inWaiting()
            if bytesToRead:
                print ('nombre de byte a lire : ' + str(bytesToRead) )
                b_bytes = self.ser.read(bytesToRead)
                line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                print line
                print

    def readserialdata(self):
        print('Start Reading Serial Data...')
        while 1:
            # Lecture du premier byte
            nb_bytes = self.ser.inWaiting()
            if nb_bytes:
                b_bytes = self.ser.read(1)  # On lit le premier byte
                b_array_start_header = bytearray(b_bytes)
                # verification si le byte correspond a un debut de message
                if b_array_start_header[0] == FEATHER_MSG_HEADER:
#                    print ('Got header')
#                    print ('Receive feather msg header data...')
                    # recuperation du numero de node
                    b_bytes = self.ser.read(3)   # Lecture du Message Type, node id et du Data Type
                    b_array_header = bytearray(b_bytes)
                    timestamp = time.strftime('%d-%m-%Y %X')
                    tt = int(time.time())
                    line = timestamp
                    line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
#                    print line
#                    print ('===== Node number :' + str(b_array_header[1]) + (' ====='))
                    self.msg_type = b_array_header[0]
                    self.node_id = b_array_header[1]

                    if b_array_header[0] == FEATHER_MSG_RESPONSE_ALL_DATA:
                         if b_array_header[2] == READ_ALL_DATA:
                             print('FEATHER_MSG_RESPONSE_ALL_DATA')
                             self.ndr.node_id = self.node_id

                             self.ndr.timestamp = tt

                             # Lecture des 4 prochains byte qui correspondent a t_1
                             b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                             b_array_data = bytearray(b_bytes)

                             b = ''.join(chr(i) for i in b_array_data)
                             import struct
                             t1_float = struct.unpack('>f', b)
                             t1_float = t1_float[0]
                             self.ndr.t_1 = t1_float

                             # Lecture des 4 prochains byte qui correspondent a t_2
                             b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                             b_array_data = bytearray(b_bytes)

                             b = ''.join(chr(i) for i in b_array_data)
                             import struct
                             t2_float = struct.unpack('>f', b)
                             t2_float = t2_float[0]
                             self.ndr.t_2 = t2_float

                             # Lecture des 4 prochains byte qui correspondent a t_3
                             b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                             b_array_data = bytearray(b_bytes)

                             if self.node_id == 0x00:
                                 print('pouet pouet')
                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 t3_float = struct.unpack('>f', b)
                                 t3_float = t3_float[0]
                                 self.ndr.t_3 = t3_float
                             else:
                                 self.ndr.t_3 = -40

                             # Lecture des 4 prochains byte qui correspondent a h_1
                             b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                             b_array_data = bytearray(b_bytes)

                             if self.node_id == 0x00:
                                 b = ''.join(chr(i) for i in b_array_data)
                                 import struct
                                 h1_float = struct.unpack('>f', b)
                                 h1_float = h1_float[0]
                                 self.ndr.h_1 = h1_float
                             else:
                                 self.ndr.h_1 = 0.0

                             # Lecture des 4 prochains byte qui correspondent a t_2
                             b_bytes = self.ser.read(4)   # Lecture du Message Type, node id et du Data Type
                             b_array_data = bytearray(b_bytes)

                             b = ''.join(chr(i) for i in b_array_data)
                             import struct
                             batt_float = struct.unpack('>f', b)
                             batt_float = batt_float[0]
                             self.ndr.bat_voltage = batt_float

#                            print("Node ID : " + str(self.ndr.node_id))
#                            print("T. 1 : " + str(self.ndr.t_1))
#                            print("T. 2 : " + str(self.ndr.t_2))
#                            print("T. 3 : " + str(self.ndr.t_3))
#                            print("Humidity : " + str(self.ndr.h_1))
#                            print("Battery Voltage: " + str(self.ndr.bat_voltage))

#                            with open(compost_csv_file, 'ab') as csvfile:
#                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
#                                compost_data_writer.writerow([timestamp,
#                                                              'node_id', '{:02d}'.format(self.ndr.node_id),
#                                                              't_1', "{0:.2f}".format(self.ndr.t_1),
#                                                              't_2', "{0:.2f}".format(self.ndr.t_2),
#                                                              't_3', "{0:.2f}".format(self.ndr.t_3),
#                                                              'h_1', "{0:.2f}".format(self.ndr.h_1),
#                                                              'bat', "{0:.2f}".format(self.ndr.bat_voltage)])

                             if self.ndr.node_id == 0:
                                 # self.list_node[0] = self.ndr
                                 self.all_node.node_0 = self.ndr
                                 self.ndr_00 = self.ndr
                                 print('NODE 0')
                                 print("Node ID : " + str(self.all_node.node_0.node_id))
                                 print("T. 1 : " + str(self.all_node.node_0.t_1))
                                 print("T. 2 : " + str(self.all_node.node_0.t_2))
                                 print("T. 3 : " + str(self.all_node.node_0.t_3))
                                 print("Humidity : " + str(self.all_node.node_0.h_1))
                                 print("Battery Voltage: " + str(self.all_node.node_0.bat_voltage))
                             elif self.ndr.node_id == 1:
                                 self.all_node.node_1 = self.ndr
                                 # self.list_node[1] = self.ndr
                                 self.ndr_01 = self.ndr
                                 print('NODE 1')
                                 print("Node ID : " + str(self.all_node.node_1.node_id))
                                 print("T. 1 : " + str(self.all_node.node_1.t_1))
                                 print("T. 2 : " + str(self.all_node.node_1.t_2))
                                 print("T. 3 : " + str(self.all_node.node_1.t_3))
                                 print("Humidity : " + str(self.all_node.node_1.h_1))
                                 print("Battery Voltage: " + str(self.all_node.node_1.bat_voltage))
                             elif self.ndr.node_id == 2:
                                 self.all_node.node_2 = self.ndr
                                 # self.list_node[2] = self.ndr
                                 self.ndr_02 = self.ndr
                                 print('NODE 2')
                                 print("Node ID : " + str(self.all_node.node_2.node_id))
                                 print("T. 1 : " + str(self.all_node.node_2.t_1))
                                 print("T. 2 : " + str(self.all_node.node_2.t_2))
                                 print("T. 3 : " + str(self.all_node.node_2.t_3))
                                 print("Humidity : " + str(self.all_node.node_2.h_1))
                                 print("Battery Voltage: " + str(self.all_node.node_2.bat_voltage))
                             elif self.ndr.node_id == 3:
                                 self.all_node.node_3 = self.ndr
                                 # self.list_node[3] = self.ndr
                                 self.ndr_03 = self.ndr
                                 print('NODE 3')
                                 print("Node ID : " + str(self.all_node.node_3.node_id))
                                 print("T. 1 : " + str(self.all_node.node_3.t_1))
                                 print("T. 2 : " + str(self.all_node.node_3.t_2))
                                 print("T. 3 : " + str(self.all_node.node_3.t_3))
                                 print("Humidity : " + str(self.all_node.node_3.h_1))
                                 print("Battery Voltage: " + str(self.all_node.node_3.bat_voltage))

                             #print self.cnt_serial_data_num
                             print tt
#                            cmd = []
#                            cmd.append('/usr/bin/rrdtool')
#                            cmd.append('update')
#                            cmd.append(rrd_dir + 'node_' + '{:02d}'.format(self.ndr.node_id) + '.rrd')
#                            if self.node_id == 0x00:
#                                cmd.extend(['-t', "t_surface:t_profondeur:t_air:humidity:batt_voltage"])
#                                cmd.append(str(tt) + ':' + str(self.ndr.t_1) +
#                                           ":" + str(self.ndr.t_2) +
#                                           ":" + str(self.ndr.t_3) + ":" + str(self.ndr.h_1) +
#                                           ":" + str(self.ndr.bat_voltage))
#                            else:
#                                cmd.extend(['-t', "t_surface:t_profondeur:batt_voltage"])
#                                cmd.append(str(tt) + ':' + str(self.ndr.t_1) +
#                                           ":" + str(self.ndr.t_2) +
#                                           ":" + str(self.ndr.bat_voltage))

                             # now execute the command
                             # this could really do with having some error-trapping
#                           subprocess.call(cmd)
                    if b_array_header[0] == FEATHER_MSG_READY_FOR_COMMANDS:
                        self.node_relais_ready_for_command = 1
                    if b_array_header[0] == FEATHER_MSG_ALL_DATA_READY:
                        print ('Feather relay id :' + str(self.node_id))
                        b_array_set_message_data = bytearray(5)
                        b_array_set_message_data[0] = FEATHER_MSG_HEADER
                        b_array_set_message_data[1] = FEATHER_MSG_SEND_ALL_TEMP
                        b_array_set_message_data[2] = self.node_id
                        b_array_set_message_data[3] = SEND_ALL_TEMP
                        b_array_set_message_data[4] = FEATHER_MSG_END
                        print('Envoie la commande SEND_ALL_TEMP')
                        self.writeserialdata(b_array_set_message_data)
                    elif b_array_header[0] == FEATHER_MSG_GET_ALL_DATA:
                        print ("FEATHER_MSG_GET_ALL_DATA : Node : " + str(self.node_id))
                    elif b_array_header[0] == FEATHER_MSG_RESPONSE_DATA:
                        print ('FEATHER_MSG_RESPONSE_DATA from node id : ' + str(self.node_id))
                        if b_array_header[2] == SEND_ALL_TEMP:
                            # Node 0
                            print('')
                            self.list_node_data[0].node_id = 0
                            self.list_node_data[0].timestamp = timestamp

                            # Lecture de T_1
                            b_bytes = self.ser.read(4)
                            self.list_node_data[0].t_1 = self.read_float(b_bytes)
                            print ('Node 0 : T1 : ' + str(self.list_node_data[0].t_1))
                            # Lecture de T_2
                            b_bytes = self.ser.read(4)
                            self.list_node_data[0].t_2 = self.read_float(b_bytes)
                            print ('Node 0 : T2 : ' + str(self.list_node_data[0].t_2))

                            # Lecture de T_3
                            b_bytes = self.ser.read(4)
                            self.list_node_data[0].t_3 = self.read_float(b_bytes)
                            print ('Node 0 : T3 : ' + str(self.list_node_data[0].t_3))

                            # Lecture de H_1
                            b_bytes = self.ser.read(4)
                            self.list_node_data[0].h_1 = self.read_float(b_bytes)
                            print ('Node 0 : H1 : ' + str(self.list_node_data[0].h_1))

                            # Lecture de batt_voltage
                            b_bytes = self.ser.read(4)
                            self.list_node_data[0].bat_voltage = self.read_float(b_bytes)
                            print ('Node 0 : Batt_voltage : ' + str(self.list_node_data[0].bat_voltage))

                            # Lecture de last_rssi
                            b_bytes = self.ser.read(1)
                            self.list_node_data[0].last_rssi = self.read_int8(b_bytes)
                            print ('Node 0 : Last_rssi : ' + str(self.list_node_data[0].last_rssi))

                            # Node 1
                            self.list_node_data[1].node_id = 1
                            self.list_node_data[1].timestamp = timestamp

                            # Lecture de T_1
                            b_bytes = self.ser.read(4)
                            self.list_node_data[1].t_1 = self.read_float(b_bytes)
                            print ('Node 1 : T1 : ' + str(self.list_node_data[1].t_1))

                            # Lecture de T_2
                            b_bytes = self.ser.read(4)
                            self.list_node_data[1].t_2 = self.read_float(b_bytes)
                            print ('Node 1 : T2 : ' + str(self.list_node_data[1].t_2))

                            # Lecture de batt_voltage
                            b_bytes = self.ser.read(4)
                            self.list_node_data[1].bat_voltage = self.read_float(b_bytes)
                            print ('Node 1 : batt_voltage : ' + str(self.list_node_data[1].bat_voltage))

                            # Lecture de last_rssi
                            b_bytes = self.ser.read(1)
                            self.list_node_data[1].last_rssi = self.read_int8(b_bytes)
                            print ('Node 1 : last_rssi : ' + str(self.list_node_data[1].last_rssi))

                            # Node 2

                            self.list_node_data[2].node_id = 2
                            self.list_node_data[2].timestamp = timestamp
                            # Lecture de T_1
                            b_bytes = self.ser.read(4)
                            self.list_node_data[2].t_1 = self.read_float(b_bytes)
                            print ('Node 2 : T1 : ' + str(self.list_node_data[2].t_1))

                            # Lecture de T_2
                            b_bytes = self.ser.read(4)
                            self.list_node_data[2].t_2 = self.read_float(b_bytes)
                            print ('Node 2 : T2 : ' + str(self.list_node_data[2].t_2))

                            # Lecture de batt_voltage
                            b_bytes = self.ser.read(4)
                            self.list_node_data[2].bat_voltage = self.read_float(b_bytes)
                            print ('Node 2 : batt_voltage : ' + str(self.list_node_data[2].bat_voltage))

                            # Lecture de last_rssi
                            b_bytes = self.ser.read(1)
                            self.list_node_data[2].last_rssi = self.read_int8(b_bytes)
                            print ('Node 2 : last_rssi : ' + str(self.list_node_data[2].last_rssi))

                            # Node 3
                            self.list_node_data[3].node_id = 3
                            self.list_node_data[3].timestamp = timestamp

                            # Lecture de T_1
                            b_bytes = self.ser.read(4)
                            self.list_node_data[3].t_1 = self.read_float(b_bytes)
                            print ('Node 3 : T_1 : ' + str(self.list_node_data[3].t_1))

                            # Lecture de T_2
                            b_bytes = self.ser.read(4)
                            self.list_node_data[3].t_2 = self.read_float(b_bytes)
                            print ('Node 3 : T_2 : ' + str(self.list_node_data[3].t_2))

                            # Lecture de batt_voltage
                            b_bytes = self.ser.read(4)
                            self.list_node_data[3].bat_voltage = self.read_float(b_bytes)
                            print ('Node 3 : batt_voltage : ' + str(self.list_node_data[3].bat_voltage))

                            # Lecture de last_rssi
                            b_bytes = self.ser.read(1)
                            self.list_node_data[3].last_rssi = self.read_int8(b_bytes)
                            print ('Node 3 : last_rssi : ' + str(self.list_node_data[3].last_rssi))

                            # Lecture de l'etat du relais
                            self.relais_etat = self.read_uint8(self.ser.read(1))
                            self.compost_fan_config.relais_etat = self.relais_etat
                            print ('Etat du relais : ' + str(self.relais_etat))

                            # Lecture de la temperature moyenne
                            b_bytes = self.ser.read(4)
                            self.relais_t_avg = self.read_float(b_bytes)
                            self.compost_fan_config.relais_t_moyenne = self.relais_t_avg
                            print ('Relais t_avg : ' + str(self.relais_t_avg))

                            # Lecture de la temperature de consigne
                            b_bytes = self.ser.read(4)
                            self.compost_fan_config.relais_consigne_temperature_fan = self.read_float(b_bytes)
                            print ('Relais t_consigne : ' + str(self.compost_fan_config.relais_consigne_temperature_fan))

                            # Delais des prises de temperatures
                            relais_delais = self.read_uint8(self.ser.read(1))
                            self.compost_fan_config.relais_delais = relais_delais
                            print ('Delais de lecture des temperature : ' + str(self.compost_fan_config.relais_delais))

                            # Mode du relais
                            relais_mode = self.read_uint8(self.ser.read(1))
                            self.compost_fan_config.relais_mode = relais_mode
                            print ('Mode du relais : ' + str(self.compost_fan_config.relais_mode))


                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=',',
#                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|',
                                                                 quoting=csv.QUOTE_MINIMAL)
#                                for x in range(4):
#                                    compost_data_writer.writerow([timestamp,
#                                                                  'node_id', '{:02d}'.format(self.list_node_data[x].node_id),
#                                                                  't_1', "{0:.2f}".format(self.list_node_data[x].t_1),
#                                                                  't_2', "{0:.2f}".format(self.list_node_data[x].t_2),
#                                                                  't_3', "{0:.2f}".format(self.list_node_data[x].t_3),
#                                                                  'h_1', "{0:.2f}".format(self.list_node_data[x].h_1),
#                                                                  'bat', "{0:.2f}".format(self.list_node_data[x].bat_voltage),
#                                                                  'last_rssi', "{0:.2f}".format(self.list_node_data[x].last_rssi)])
                                compost_data_writer.writerow([timestamp,
                                                              '{:02d}'.format(self.list_node_data[0].node_id),
                                                              "{0:.2f}".format(self.list_node_data[0].t_1),
                                                              "{0:.2f}".format(self.list_node_data[0].t_2),
                                                              "{0:.2f}".format(self.list_node_data[0].t_3),
                                                              "{0:.2f}".format(self.list_node_data[0].h_1),
                                                              "{0:.2f}".format(self.list_node_data[0].bat_voltage),
                                                              "{0:.2f}".format(self.list_node_data[0].last_rssi),
                                                              '{:02d}'.format(self.list_node_data[1].node_id),
                                                              "{0:.2f}".format(self.list_node_data[1].t_1),
                                                              "{0:.2f}".format(self.list_node_data[1].t_2),
                                                              "{0:.2f}".format(self.list_node_data[1].bat_voltage),
                                                              "{0:.2f}".format(self.list_node_data[1].last_rssi),
                                                              '{:02d}'.format(self.list_node_data[2].node_id),
                                                              "{0:.2f}".format(self.list_node_data[2].t_1),
                                                              "{0:.2f}".format(self.list_node_data[2].t_2),
                                                              "{0:.2f}".format(self.list_node_data[2].bat_voltage),
                                                              "{0:.2f}".format(self.list_node_data[2].last_rssi),
                                                              '{:02d}'.format(self.list_node_data[3].node_id),
                                                              "{0:.2f}".format(self.list_node_data[3].t_1),
                                                              "{0:.2f}".format(self.list_node_data[3].t_2),
                                                              "{0:.2f}".format(self.list_node_data[3].bat_voltage),
                                                              "{0:.2f}".format(self.list_node_data[3].last_rssi),
                                                              "{0:.2f}".format(self.compost_fan_config.relais_t_moyenne),
                                                              "{0:.2f}".format(self.compost_fan_config.relais_consigne_temperature_fan),
                                                              '{:02d}'.format(self.compost_fan_config.relais_etat),
                                                              '{:02d}'.format(self.compost_fan_config.relais_delais)])

                            for x in range(4):
                                cmd = []
                                cmd.append('/usr/bin/rrdtool')
                                cmd.append('update')
                                cmd.append(rrd_dir + 'node_' + '{:02d}'.format(x) + '.rrd')
                                if self.list_node_data[x].node_id == 0x00:
                                    cmd.extend(['-t', "t_surface:t_profondeur:t_air:humidity:batt_voltage:last_rssi"])
                                    cmd.append(str(tt) + ':' + str(self.list_node_data[x].t_1) +
                                               ":" + str(self.list_node_data[x].t_2) +
                                               ":" + str(self.list_node_data[x].t_3) + ":" + str(self.list_node_data[x].h_1) +
                                               ":" + str(self.list_node_data[x].bat_voltage) +
                                               ":" + str(self.list_node_data[x].last_rssi))
                                else:
                                    cmd.extend(['-t', "t_surface:t_profondeur:batt_voltage:last_rssi"])
                                    cmd.append(str(tt) + ':' + str(self.list_node_data[x].t_1) +
                                               ":" + str(self.list_node_data[x].t_2) +
                                               ":" + str(self.list_node_data[x].bat_voltage) +
                                               ":" + str(self.list_node_data[x].last_rssi))

                                # now execute the command
                                # this could really do with having some error-trapping
#                                print(cmd)
                                subprocess.call(cmd)

                            cmd = []
                            cmd.append('/usr/bin/rrdtool')
                            cmd.append('update')
                            cmd.append(rrd_dir + 'node_' + 'FE' + '.rrd')
                            cmd.extend(['-t', "setpoint_relais:etat_relais:t_avg"])
                            cmd.append(str(tt) +
                                       ':' + str(self.compost_fan_config.relais_consigne_temperature_fan) +
                                       ":" + str(self.compost_fan_config.relais_etat) +
                                       ":" + str(self.relais_t_avg))

                            subprocess.call(cmd)
                            self.graph_node_1()

                        elif b_array_header[2] == TEMP_1:
                            print ('READ_TEMP_1')
                            b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                            b_array_data = bytearray(b_bytes)
                            end_msg = self.ser.read(1)
                            b = ''.join(chr(i) for i in b_array_data)
                            import struct
                            t1_float = struct.unpack('>f', b)
                            f_value = t1_float[0]
                            print f_value
                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                compost_data_writer.writerow([timestamp, b_array_header[0], 't_1', f_value])
                            if self.node_id == 1:
                                self.compost_fan_data.node_1_temperature_surface = f_value
                            elif self.node_id == 2:
                                self.compost_fan_data.node_2_temperature_surface = f_value
                            elif self.node_id == 3:
                                self.compost_fan_data.node_3_temperature_surface = f_value
                            elif self.node_id == 4:
                                self.compost_fan_data.node_4_temperature_surface = f_value
                        elif b_array_header[2] == TEMP_2:
                            print ('READ_TEMP_2')
                            b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                            b_array_data = bytearray(b_bytes)
                            end_msg = self.ser.read(2)
                            b = ''.join(chr(i) for i in b_array_data)
                            import struct
                            t2_float = struct.unpack('>f', b)
                            f_value = (t2_float)[0]
                            print f_value
                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                compost_data_writer.writerow([timestamp, b_array_header[0], 't_2', f_value])
                            if self.node_id == 1:
                                self.compost_fan_data.node_1_temperature_profondeur = f_value
                            elif self.node_id == 2:
                                self.compost_fan_data.node_2_temperature_profondeur = f_value
                            elif self.node_id == 3:
                                self.compost_fan_data.node_3_temperature_profondeur = f_value
                            elif self.node_id == 4:
                                self.compost_fan_data.node_4_temperature_profondeur = f_value
                        elif b_array_header[2] == TEMP_3:
                            print ('READ_TEMP_3')
                            b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                            b_array_data = bytearray(b_bytes)
                            end_msg = self.ser.read(1)
                            b = ''.join(chr(i) for i in b_array_data)
                            import struct
                            t3_float = struct.unpack('>f', b)
                            f_value = (t3_float)[0]
                            print f_value
                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                compost_data_writer.writerow([timestamp, b_array_header[0], 't_3', f_value])
                            if self.node_id == 1:
                                self.compost_fan_data.node_1_temperature_ambiant = f_value
                        elif b_array_header[2] == HUMIDITY_1:
                            print ('READ_HUMIDITY_1')
                            b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                            b_array_data = bytearray(b_bytes)
                            end_msg = self.ser.read(2)
                            b = ''.join(chr(i) for i in b_array_data)
                            import struct
                            h1_float = struct.unpack('>f', b)
                            f_value = (h1_float)[0]
                            print f_value
                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                compost_data_writer.writerow([timestamp, b_array_header[0], 'h_1', f_value])
                            if self.node_id == 1:
                                self.compost_fan_data.node_1_humidity_ambiant = f_value
                        elif b_array_header[2] == READ_BATTERY_VOLTAGE:
                            print ('READ_BATTERY_VOLTAGE')
                            b_bytes = self.ser.read(4)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                            b_array_data = bytearray(b_bytes)
                            end_msg = self.ser.read(2)
                            b = ''.join(chr(i) for i in b_array_data)
                            import struct
                            bat_float = struct.unpack('>f', b)

                            f_value = (bat_float)[0]
                            print f_value
                            with open(compost_csv_file, 'ab') as csvfile:
                                compost_data_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                                compost_data_writer.writerow([timestamp, b_array_header[0], 'bat', f_value])
                            if self.node_id == 1:
                                self.compost_fan_data.node_1_battery_voltage = f_value
                            elif self.node_id == 2:
                                self.compost_fan_data.node_2_battery_voltage = f_value
                            elif self.node_id == 3:
                                self.compost_fan_data.node_3_battery_voltage = f_value
                            elif self.node_id == 4:
                                self.compost_fan_data.node_4_battery_voltage = f_value
                        else:
                            print('No Msg Data')
                            b_bytes = self.ser.read(6)   # Lecture de 4 bytes pour temperature
                            line = line + "  " + ':'.join('{:02x}'.format(ord(c)) for c in b_bytes)
                            print line
                    elif b_array_header[0] == FEATHER_MSG_QUERY_DATA:
                        print ('***** Receive query data *****')
                        if b_array_header[2] == TEMP_1:
                            print ('READ_TEMP_1')
                        elif b_array_header[2] == TEMP_2:
                            print ('READ_TEMP_2')
                        elif b_array_header[2] == TEMP_3:
                            print ('READ_TEMP_3')
                        else:
                            print ('Query data : ' + str(b_array_header[2]))
                        b_bytes = self.ser.read(1)
                    elif b_array_header[0] == FEATHER_MSG_GET_DATA:
                        print ('***** Receive get data *****')
                    elif b_array_header[0] == FEATHER_MSG_SET_DATA:
                        print ('***** Receive set data *****')
#                    print
#                    print
            time.sleep(0.5)
                    #time.sleep(0.5)
       #     self.d_tt = int(time.time()) - tt

    def dispatcher(self):
        print ('dispatcher')
        thread.start_new_thread(self.readserialdata, ())
        while True:
            print('Waiting for client connection...')
            connection, address = self.sock.accept()
            print ('Server connected by ', address)
            print ('at', self.now())
            thread.start_new_thread(self.handleClient, (connection,))

    def graph_node_1(self):
        print("Graph node 1")
#        info = rrdtool.info("node_00.rrd")
#        print info
        size_w = 640
        size_h = 480
        start_time = "end-12h"
        end_time = "now"
        rrdtool.graph("node_1.png",
                      "--imgformat", "PNG",
                      "--alt-autoscale",
                      "--title", "Node 0",
                      "-w", str(size_w),
                      "-h", str(size_h),
                      "-s", start_time,
                      "-e", end_time,
                      "--vertical-label", "Temperature (degre C)",
                      "--right-axis-label", "Temperature (degre C)",
                      "--right-axis-format", "%.1lf",
                      "--right-axis", "1:0",
                      "--no-gridfit",
                      "DEF:node_00_t_surface=node_00.rrd:t_surface:AVERAGE",
                      "DEF:node_00_t_profondeur=node_00.rrd:t_profondeur:AVERAGE",
                      "DEF:node_00_t_air=node_00.rrd:t_air:AVERAGE",
                      "DEF:node_00_humidity=node_00.rrd:humidity:AVERAGE",
                      "DEF:node_00_batt_voltage=node_00.rrd:batt_voltage:AVERAGE",
                      "LINE1:node_00_t_surface#FC0303:Node 00 - Temperature de surface",
                      "LINE1:node_00_t_profondeur#FC5A03:Node 00 - Temperature de profondeur",
                      "LINE1:node_00_t_air#FC9E03:Node 00 - Temperature air",
                      "LINE1:node_00_humidity#0000FF:Node 00 - Humidite air",
                      "LINE1:node_00_batt_voltage#000000:Node 00 - Battery voltage")

        def graph_node_2(self):
            print("Graph node 1")
            #        info = rrdtool.info("node_00.rrd")
            #        print info
            size_w = 640
            size_h = 480
            start_time = "end-12h"
            end_time = "now"
            rrdtool.graph("node_1.png",
                          "--imgformat", "PNG",
                          "--alt-autoscale",
                          "--title", "Node 1",
                          "-w", str(size_w),
                          "-h", str(size_h),
                          "-s", start_time,
                          "-e", end_time,
                          "--vertical-label", "Temperature (degre C)",
                          "--right-axis-label", "Temperature (degre C)",
                          "--right-axis-format", "%.1lf",
                          "--right-axis", "1:0",
                          "--no-gridfit",
                          "DEF:node_01_t_surface=node_01.rrd:t_surface:AVERAGE",
                          "DEF:node_01_t_profondeur=node_01.rrd:t_profondeur:AVERAGE",
                          "DEF:node_01_batt_voltage=node_01.rrd:batt_voltage:AVERAGE",
                          "LINE1:node_01_t_surface#FC0303:Node 01 - Temperature de surface",
                          "LINE1:node_01_t_profondeur#FC5A03:Node 01 - Temperature de profondeur",
                          "LINE1:node_01_batt_voltage#000000:Node 01 - Battery voltage")


#  Main

if __name__ == '__main__':
    compostFAN = CompostFAN(1)
    compostFAN.init_serial()
    compostFAN.init_server()
    compostFAN.dispatcher()

#    compostFAN.readserialdata()
#    compostFAN.read_test()