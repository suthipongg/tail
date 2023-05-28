'''THE BLUEPRINT LABORATORY

Reach Control SDK
Master Arm V2 Packet Parsing Example

***
24/Sept/2020
Kyle Mclean sourced from Shaun Barlow's master_arm_example
***

The following program demonstrates handling of packets
from the Master Arm V2 product.

***IMPORTANT***
The user must set the name of the COM_PORT variable below.
To find the correct comport name, use the MasterArmV2.usb_list() function.

INSTRUCTIONS
This example code can be run by executing the file: "python path_to_main/master_arm_V2_example.py"

An example sequence can be seen in the last section of this document.

'''
import time

from serial_device import SerialDevice
from RS1_hardware import PacketID, Mode, get_name_of_packet_id
from bplprotocol import BplSerial
import serial.tools.list_ports

'''USER SETTINGS'''
COM_PORT = 'COM9'      # Name of the comport connected to MA (use COM port name if on Windows)
# COM_PORT = '/dev/ttyUSB0'     # Name of the comport connected to MA (use ports in /dev/ folder for name Linux)
BAUD = 115200   # Should not need changing from 115200


class MasterArmV2:
    # Device IDs to allow for the axis config.
    deviceIDs = [0x01, 0x02, 0x03, 0x04, 0x05]

    # create an instance of SerialDevice()
    serial_dev = SerialDevice()

    # that instance is passed as argument to BplSerial
    global_serial = BplSerial(serial_dev, len(deviceIDs))

    def __init__(self):
        self.deviceIDs = [0xC1, 0xC2, 0xC3, 0xC4, 0xC5]
        self.global_serial.deviceIDs = self.deviceIDs

    def usb_list(self):
        '''
        Prints a list of available serial ports.
        This is a helper function and is non-essential.
        :return: None
        '''
        comports = serial.tools.list_ports.comports()
        print('***\nCOM PORT LIST:')
        for comport in comports:
            print(comport, comport.device)
        print('***\n')

    def connect(self):
        self.global_serial.connect(COM_PORT, BAUD)

    def get_packets(self):
        packets = self.global_serial.serial_device.readdata()
        return packets

    def parse_packets(self, packets):
        return [self.global_serial.serial_device.parsepacket(p)
                for p in packets]


if __name__ == '__main__':
    ma = MasterArmV2()
    ma.usb_list()
    ma.connect()

    while True:

        # Fetch raw packets from master arm
        packets = ma.get_packets()

        # Parse raw packets into list:
        #        decoded_packets -> List[packet_id, device_id, packet_data]
        decoded_packets = ma.parse_packets(packets)

        [print(
         str(get_name_of_packet_id(p[1])).ljust(20),
         'ID: ', str(hex(p[0])).ljust(5),
         'Value: ', str(p[2]).ljust(10))
         for p in decoded_packets]
        time.sleep(0.05)

        '''
        Notes: 
            - When the master arm is paused in zero velocity mode (blue flashing led's), 
              the master arm will be sending zero value velocity packets to each device.
            - Packets are sent on a 50ms interval
            - Devices are structured as: 
                    0xCE: MA Base                       -> Reach control 
                    0xC1: Joystick Y axis               -> Joint A
                    0xC2: Joystick X axis/MA Joint B    -> Joint B
                    0xC3: MA Joint C                    -> Joint C
                    0xC4: MA Joint D                    -> Joint D
                    0xC5: MA Joint E                    -> Joint E
                    0xC6: MA Joint F                    -> Joint F
                    0xC7: MA Joint G                    -> Joint G
        '''
