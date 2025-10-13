import os
import can
import struct
import time 

# config CAN
os.system('sudo ip link set can0 type can bitrate 5000000')  # adapte le bitrate
os.system('sudo ifconfig can0 up')

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

x_depart = 145
y_depart = 120
angle_depart = -180

data_x = struct.pack('<f',x_depart)
data_y = struct.pack('<f',y_depart)
data_angle = struct.pack('<f',angle_depart)

msg = can.Message(arbitration_id=0x20, data=data_x, is_extended_id=False)
bus.send(msg)
print(f"Trame envoyée : {msg}")

msg = can.Message(arbitration_id=0x21, data=data_y, is_extended_id=False)
bus.send(msg)
print(f"Trame envoyée : {msg}")

msg = can.Message(arbitration_id=0x22, data=data_angle, is_extended_id=False)
bus.send(msg)
print(f"Trame envoyée : {msg}")
time.sleep(1)

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
