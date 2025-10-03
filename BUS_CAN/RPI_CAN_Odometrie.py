import os
import can
import struct

# config CAN
os.system('sudo ip link set can0 type can bitrate 5000000')  # adapte le bitrate
os.system('sudo ifconfig can0 up')

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

while True:
    msg = bus.recv(1.0)  # attend max 1 seconde
    if msg is None:
        continue

    if msg.arbitration_id == 0x10:
        x = struct.unpack('f', bytes(msg.data))[0]
        print("X =", x)

    elif msg.arbitration_id == 0x11:
        y = struct.unpack('f', bytes(msg.data))[0]
        print("Y =", y)

    elif msg.arbitration_id == 0x12:
        teta = struct.unpack('f', bytes(msg.data))[0]
        print("Teta =", teta)

    else:
        print("Autre ID reçu :", hex(msg.arbitration_id), msg.data)
