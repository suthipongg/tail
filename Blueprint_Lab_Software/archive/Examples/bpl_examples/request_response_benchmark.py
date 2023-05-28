"""request_response_benchmark.py

Script to Produce statistics on time to request information from the joints on the arm.

For Serial Communications, comment/uncomment 'request_serial()' at the bottom of this script.
For UDP Communications, comment/uncomment 'request_udp()' at the bottom of this script.
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

    POSITION = 0x03
    REQUEST_PACKET = 0x60


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


def request_serial(device_ids: List[int], packet_id: int, request_period):
    # Create the packet Reader
    packet_reader = PacketReader()

    # Create the Serial Connection
    serial_device = serial.Serial(SERIAL_PORT,
                                  baudrate=BAUDRATE,
                                  stopbits=serial.STOPBITS_ONE,
                                  parity=serial.PARITY_NONE,
                                  bytesize=serial.EIGHTBITS)
    serial_device.timeout = 0.01

    # Create and encode the request packet
    request_packet = BPLProtocol.encode_packet(0xFF, PacketID.REQUEST_PACKET, bytes([packet_id]))

    # Variable to store response statistics
    response_times: list = [None] * len(device_ids)  # Time it took for the device to respond
    average_response_times = [0.0] * len(device_ids)  # Average time it took for the device to respond
    received_responses = [0] * len(device_ids)  # Count of amount of received responses
    missed_packets = [0] * len(device_ids)  # Count of amount of missed responses

    while True:
        start_time = time.perf_counter()  # Start time

        # Reset response times
        response_times: list = [None] * len(device_ids)

        # Send a request to the devices
        serial_device.write(request_packet)

        # Read data for (request_period) seconds
        while time.perf_counter() - start_time < request_period:
            time.sleep(0.01)
            try:
                read_data = serial_device.read(4096)
            except:
                read_data = b''
            if read_data != b'':
                packets = packet_reader.receive_bytes(read_data)
                if packets:
                    for packet in packets:
                        read_device_id, read_packet_id, data_bytes = packet
                        if read_device_id in device_ids:

                            # When a packet is received then record response times.
                            idx = device_ids.index(read_device_id)
                            response_times[idx] = time.perf_counter() - start_time

        # Check through response_times
        for idx, d in enumerate(device_ids):
            if response_times[idx] is None:
                missed_packets[idx] += 1
            else:
                average_response_times[idx] \
                    = (average_response_times[idx] * received_responses[idx] + response_times[idx]) \
                      / (received_responses[idx] + 1)
                received_responses[idx] += 1

        # Generate Printout
        dev_id_string = ""
        resp_time_string = ""
        avg_response_time_string = ""
        missing_string = ""
        for idx, d in enumerate(device_ids):
            dev_id_string += hex(d) + " \t"
            a = response_times[idx]
            if a is not None:
                resp_time_string += f"{a * 1000:4.1f}" + "\t"
            else:
                resp_time_string += "--  \t"
            avg_response_time_string += f"{average_response_times[idx]*1000:.1f}" + " \t"
            missing_string += f"{missed_packets[idx]}" + "   \t"

        strings = "Device_ID             \t" + dev_id_string + "\n" \
                  + "Response time (ms)    \t" + resp_time_string + "\n" \
                  + "Average Response (ms) \t" + avg_response_time_string + "\n" \
                  + "Missing Responses     \t" + missing_string + "\n"
        print(strings)


def request_udp(device_ids: List[int], packet_id: int, request_period):
    # Create the packet Reader
    packet_reader = PacketReader()

    # Create the UDP socket connection
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setblocking(False)

    # Create and encode the request packet
    request_packet = BPLProtocol.encode_packet(0xFF, PacketID.REQUEST_PACKET, bytes([packet_id]))

    # Variable to store response statistics
    response_times: list = [None] * len(device_ids)  # Time it took for the device to respond
    average_response_times = [0.0] * len(device_ids)  # Average time it took for the device to respond
    received_responses = [0] * len(device_ids)  # Count of amount of received responses
    missed_packets = [0] * len(device_ids)  # Count of amount of missed responses

    while True:
        start_time = time.perf_counter()  # Start time

        # Reset response times
        response_times: list = [None] * len(device_ids)

        # Send a request to the devices
        udp.sendto(request_packet, (UDP_IP_ADDRESS, UDP_PORT))

        # Read data for (request_period) seconds
        while time.perf_counter() - start_time < request_period:
            try:
                read_data = udp.recv(128)
            except:
                read_data = b''
            if read_data:
                packets = packet_reader.receive_bytes(read_data)
                if packets:
                    for packet in packets:
                        read_device_id, read_packet_id, data_bytes = packet
                        if read_device_id in device_ids:

                            # When a packet is received then record response times.
                            idx = device_ids.index(read_device_id)
                            response_times[idx] = time.perf_counter() - start_time

        # Check through response_times
        for idx, d in enumerate(device_ids):
            if response_times[idx] is None:
                missed_packets[idx] += 1
            else:
                average_response_times[idx] \
                    = (average_response_times[idx] * received_responses[idx] + response_times[idx]) \
                      / (received_responses[idx] + 1)
                received_responses[idx] += 1

        # Generate Printout
        dev_id_string = ""
        resp_time_string = ""
        avg_response_time_string = ""
        missing_string = ""
        for idx, d in enumerate(device_ids):
            dev_id_string += hex(d) + " \t"
            a = response_times[idx]
            if a is not None:
                resp_time_string += f"{a * 1000:4.1f}" + "\t"
            else:
                resp_time_string += "--  \t"
            avg_response_time_string += f"{average_response_times[idx]*1000:.1f}" + " \t"
            missing_string += f"{missed_packets[idx]}" + "   \t"

        strings = "Device_ID             \t" + dev_id_string + "\n" \
                  + "Response time (ms)    \t" + resp_time_string + "\n" \
                  + "Average Response (ms) \t" + avg_response_time_string + "\n" \
                  + "Missing Responses     \t" + missing_string + "\n"
        print(strings)


SERIAL_PORT = "COM12"  # Serial port in your computer.
BAUDRATE = 115200

UDP_IP_ADDRESS = "192.168.1.221"
UDP_PORT = 6789


if __name__ == '__main__':
    device_ids = [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7]

    request_period = 0.5  # seconds

    packet_id = PacketID.POSITION  # packet to request

    request_serial(device_ids, packet_id, request_period)
    # request_udp(device_ids, packet_id, request_period)





