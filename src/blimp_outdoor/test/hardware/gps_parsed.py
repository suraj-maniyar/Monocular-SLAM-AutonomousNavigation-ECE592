import serial
import pynmea2
import socket
import sys
import datetime

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.0.4', 10000)
sock.connect(server_address)

port = serial.Serial("/dev/serial0", baudrate=9600, timeout=3.0)

while True:
    rcv= port.readline()
    # print("Received: " + repr(rcv))
    # port.write("\r\nYou sent:"+repr(rcv))
    msg = pynmea2.parse(rcv.decode("utf-8"))
    str_msg = str(msg)
    # print(str_msg)

    out_msg = ''
    delimiter = ' | '
    space = ' '

    if '$GPGSA' in str_msg:
        msg_gsa = msg
        out_msg += 'Type: GPGSA' + delimiter
        out_msg += 'Mode: ' + msg.mode + delimiter
        out_msg += '3D Fix: ' + msg.mode_fix_type + delimiter
        out_msg += 'Sattellites: ' + str((msg.sv_id01, msg.sv_id02, msg.sv_id03, 
                                          msg.sv_id04, msg.sv_id05, msg.sv_id06, 
                                          msg.sv_id07, msg.sv_id08, msg.sv_id09, 
                                          msg.sv_id10, msg.sv_id11, msg.sv_id12)) + delimiter
        out_msg += 'PDOP: ' + msg.pdop + delimiter
        out_msg += 'HDOP: ' + msg.hdop + delimiter
        out_msg += 'VDOP: ' + msg.vdop + delimiter
        
    if '$GPRMC' in str_msg:
        msg_rmc = msg
        out_msg += 'Type: GPRMC' + delimiter
        out_msg += 'Datetime: ' + (msg.datestamp).strftime('%Y/%m/%d') + space + (msg.timestamp).strftime('%H:%M:%S') + delimiter
        out_msg += 'Status: ' + msg.status + delimiter
        out_msg += 'Lat: ' + str(msg.lat) + space + msg.lat_dir + delimiter
        out_msg += 'Long: ' + str(msg.lon) + space + msg.lon_dir + delimiter
        out_msg += 'Speed: ' + str(msg.spd_over_grnd) + delimiter
        out_msg += 'Course: ' + str(msg.true_course) + delimiter
        print(out_msg)

        try:
            sock.sendall(out_msg.encode("utf-8"))
        except:
            pass
