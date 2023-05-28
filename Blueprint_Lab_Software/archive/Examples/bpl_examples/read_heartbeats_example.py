"""read_heartbeats_example.py

Script to set the read heartbeats packets from the joints of the arm.
This script will read all incoming packets from the connection and print them out on the screen.

For Serial Communications, comment/uncomment 'read_heartbeats_serial()' at the bottom of this script.
For UDP Communications, comment/uncomment 'read_heartbeats_udp()' at the bottom of this script.
"""
import re
import socket
import struct
from typing import Union, Tuple, List, Optional
from cobs import cobs
import serial
import time
from crcmod import crcmod
import logging

logging.basicConfig()
logger = logging.getLogger(__name__)


class PacketID:
    """
    Mock class holding relevant packet Ids for this script.
    Look in 'from BPL_serial_protocol.RS1_SDK_v4.RS1_SDK.Examples.RS1_hardware' for a comprehensive list of packet ids.
    """

    MODE = 0x01
    DEVICE_ID = 0x64
    POSITION = 0x03
    VELOCITY = 0x02
    REQUEST_PACKET = 0x60

    HEARTBEAT_FREQUENCY_SET = 0x92
    HEARTBEAT_SET = 0x91


class BPLProtocol:
    """ Helper class used to encode and decode BPL packets."""
    CRC8_FUNC = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF)

    @staticmethod
    def packet_splitter(buff: bytes) -> Tuple[List[bytes], Optional[bytes]]:
        """
        Split packets coming in along bpl protocol, Packets are split via 0x00.
        """
        incomplete_packet = None
        packets = re.split(b'\x00', buff)
        if buff[-1] != b'0x00':
            incomplete_packet = packets.pop()
        return packets, incomplete_packet

    @staticmethod
    def parse_packet(packet_in: Union[bytes, bytearray]) -> Tuple[int, int, bytes]:
        """
        Parse the packet returning a tuple of [int, int, bytes].
         If unable to parse the packet, then return 0,0,b''.
        """

        packet_in = bytearray(packet_in)

        if packet_in and len(packet_in) > 3:
            try:
                decoded_packet: bytes = cobs.decode(packet_in)
            except cobs.DecodeError as e:
                logger.warning(f"parse_packet(): Cobs Decoding Error, {e}")
                return 0, 0, b''

            if decoded_packet[-2] != len(decoded_packet):
                logger.warning(f"parse_packet(): Incorrect length: length is {len(decoded_packet)} "
                               f"in {[hex(x) for x in list(decoded_packet)]}")
                return 0, 0, b''
            else:
                if BPLProtocol.CRC8_FUNC(decoded_packet[:-1]) == decoded_packet[-1]:
                    rx_data = decoded_packet[:-4]

                    device_id = decoded_packet[-3]
                    packet_id = decoded_packet[-4]
                    rx_data = rx_data
                    return device_id, packet_id, rx_data
                else:
                    logger.warning(f"parse_packet(): CRC error in {[hex(x) for x in list(decoded_packet)]} ")
                    return 0, 0, b''
        return 0, 0, b''

    @staticmethod
    def encode_packet(device_id: int, packet_id: int, data: Union[bytes, bytearray]):
        """ Encode the packet using the bpl protocol."""
        tx_packet = bytes(data)
        tx_packet += bytes([packet_id, device_id, len(tx_packet)+4])
        tx_packet += bytes([BPLProtocol.CRC8_FUNC(tx_packet)])
        packet: bytes = cobs.encode(tx_packet) + b'\x00'
        return packet

    @staticmethod
    def decode_floats(data: Union[bytes, bytearray]) -> List[float]:
        """ Decode a received byte list, into a float list as specified by the bpl protocol"""
        list_data = list(struct.unpack(str(int(len(data)/4)) + "f", data))
        return list_data


class PacketReader:
    """
    Packet Reader
    Helper class to read bytes, account for the possibility of incomplete packets on arrival.

    Usage:

    packet_reader = PacketReader()


    data == .... receive data here as bytes ...

    packets = packet_reader.receive_bytes()


    """
    incomplete_packets = b''

    def receive_bytes(self, data: bytes) -> List[Tuple[int, int, bytes]]:
        # Receive data, and return a decoded packet
        packet_list = []
        encoded_packets, self.incomplete_packets = BPLProtocol.packet_splitter(self.incomplete_packets + data)
        if encoded_packets:
            for encoded_packet in encoded_packets:
                if not encoded_packet:
                    continue
                decoded_packet = BPLProtocol.parse_packet(encoded_packet)
                packet_list.append(decoded_packet)
        return packet_list


def read_heartbeats_serial(port: str):

    packet_reader = PacketReader()
    # Create the Serial Connection
    serial_device = serial.Serial(port,
                                  baudrate=115200,
                                  stopbits=serial.STOPBITS_ONE,
                                  parity=serial.PARITY_NONE,
                                  bytesize=serial.EIGHTBITS)
    serial_device.timeout = 0.01


    while True:
        try:
            read_data = serial_device.read(4096)
        except:
            read_data = b''
        if read_data:
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet

                    if read_device_id == 0:
                        continue

                    # For float packets, they must be decoded.
                    if read_packet_id in [PacketID.POSITION, PacketID.VELOCITY]:

                        data = BPLProtocol.decode_floats(data_bytes)
                    else:
                        data = list(data_bytes)

                    print(f"Received packet: \tDevice_ID: {read_device_id}\tPacket_ID: {read_packet_id}\t data: {data}")


def read_heartbeats_udp(ip_address: str, port: int):
    packet_reader = PacketReader()

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)

    # Send empty message to tell arm to return on udp.
    udp.sendto(b'', (ip_address, port))

    while True:
        try:
            read_data = udp.recv(4096)
        except:
            read_data = b''
        if read_data:
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet

                    if read_device_id == 0:
                        continue

                    # For float packets, they must be decoded.
                    if read_packet_id in [PacketID.POSITION, PacketID.VELOCITY]:

                        data = BPLProtocol.decode_floats(data_bytes)
                    else:
                        data = list(data_bytes)

                    print(f"Received packet: \tDevice_ID: {read_device_id}\tPacket_ID: {read_packet_id}\t data: {data}")


def set_heartbeats_udp(ip_address: str, port: int, device_ids: List[int], packets: List[int], frequency: int):
    # Create the UDP socket connection
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)

    for device_id in device_ids:
        _packets = [0] * 10
        _packets[:(len(packets))] = packets

        print(f"Setting Heartbeat packets for device id: {device_id}")

        set_heartbeat_packet = BPLProtocol.encode_packet(device_id, PacketID.HEARTBEAT_SET, bytes(_packets))
        udp.sendto(set_heartbeat_packet, (ip_address, port))

        time.sleep(0.1)

        set_heartbeat_frequency_packet = BPLProtocol.encode_packet(device_id, PacketID.HEARTBEAT_FREQUENCY_SET,
                                                                   bytes([frequency]))

        udp.sendto(set_heartbeat_frequency_packet, (ip_address, port))
        time.sleep(0.1)


SERIAL_PORT = "COM12"  # Serial port in your computer.

UDP_IP_ADDRESS = "192.168.2.3"
UDP_PORT = 6789


if __name__ == '__main__':

    # read_heartbeats_serial(SERIAL_PORT)
    read_heartbeats_udp(UDP_IP_ADDRESS, UDP_PORT)






