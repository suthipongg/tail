'''THE BLUEPRINT LABORATORY

Reach Control SDK
Master Arm V1 Relative Position Packet Parsing Example

***
14/Feb/2020
Shaun Barlow
***

The following program demonstrates handling of Relative Position packets
from the Master Arm V1 product.

***IMPORTANT***
The user must set the name of the COM_PORT variable below.
To find the correct comport name, use the MasterArmV1.usb_list() function.

INSTRUCTIONS
This example code can be run by executing the file: "python path_to_main/master_arm_example.py"

An example sequence can be seen in the last section of this document
and may be edited to test the various functions found in MasterArmV1 class.

'''

import time

from main import Reach5Mini
from RS1_hardware import PacketID

DEVICE_ID_TO_INDEX_MAP = {0xC1: 0, 0xC2: 1, 0xC3: 2, 0xC4: 3, 0xC5: 4}
INDEX_TO_DEVICE_ID_MAP = {0: 0xC1, 1: 0xC2, 2: 0xC3, 3: 0xC4, 4: 0xC5}

'''USER SETTINGS'''
COM_PORT = 'COM49'      # Name of the comport connected to MA (use COM port name if on Windows)
# COM_PORT = '/dev/ttyUSB0'     # Name of the comport connected to MA (use ports in /dev/ folder for name Linux)
BAUD = 115200   # Should not need changing from 115200


class MasterArmV1(Reach5Mini):

    def __init__(self):
        self.positions = [0, 0, 0, 0, 0]
        self.deviceIDs = [0xC1, 0xC2, 0xC3, 0xC4, 0xC5]
        self.global_serial.deviceIDs = self.deviceIDs

    def connect(self):
        self.global_serial.connect(COM_PORT, BAUD)

    def receive_relative_position(self, device_id, relative_position):
        index = DEVICE_ID_TO_INDEX_MAP[device_id]
        self.positions[index] += relative_position

    def get_packets(self):
        packets = self.global_serial.serial_device.readdata()
        return packets

    def parse_packets(self, packets):
        for p in packets:
            packet = self.global_serial.serial_device.parsepacket(p)
            device_id = packet[0]
            packet_id = packet[1]
            data = packet[2]

            if packet_id == PacketID.RELATIVE_POSITION:
                self.receive_relative_position(device_id, data[0])

    def get_position(self, device_id):
        index = DEVICE_ID_TO_INDEX_MAP[device_id]
        pos = self.positions[index]
        return pos


if __name__ == '__main__':
    ma = MasterArmV1()
    ma.usb_list()
    ma.connect()

    while True:
        packets = ma.get_packets()
        ma.parse_packets(packets)

        relative_positions = ma.positions  # Positions of Master Arm in radians.

        '''
         Positions are calculated as the integral of all RELATIVE_POSITION packets from Master Arm.
         E.G.
         position[0] = 0  (initial value)
        
         RELATIVE_POSITION value of 0.1 received
         position[0] = 0.1
        
         RELATIVE_POSITION value of 0.1 received
         position[0] = 0.2
        
         RELATIVE_POSITION value of -0.3 received
         position[0] = -0.1
        '''

        print("Master arm positions (relative):", [f'{pos:0.2f}' for pos in relative_positions])
        time.sleep(0.05)
