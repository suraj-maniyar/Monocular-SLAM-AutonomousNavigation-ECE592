import serial
import pynmea2
import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.0.4', 10000)
sock.connect(server_address)

port = serial.Serial("/dev/serial0", baudrate=9600, timeout=3.0)

while True:
    rcv= port.readline()
    # print("Received: " + repr(rcv))
    # port.write("\r\nYou sent:"+repr(rcv))
    msg = str(pynmea2.parse(rcv.decode("utf-8")))
    print(msg)

    if msg:
        # try:
        sock.sendall(msg.encode("utf-8"))
        # except:
           #  pass


