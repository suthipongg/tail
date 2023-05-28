
import re
import struct
from typing import Union, Tuple, List, Optional
from cobs import cobs
from crcmod import crcmod
import logging

logging.basicConfig()
logger = logging.getLogger(__name__)



class PacketID:
    """
    Class holding relevant packet Ids for this script.
    Look in 'from BPL_serial_protocol.RS1_SDK_v4.RS1_SDK.Examples.RS1_hardware' for a comprehensive list of packet ids.
    """
    MODE = 0x01
    VELOCITY = 0x02
    POSITION = 0x03
    OPENLOOP = 0x04
    CURRENT = 0x05
    VELOCITY_CONSTRAINT = 0xB5
    REQUEST_PACKET = 0x60
    DEVICE_TYPE = 0x67
    SYSTEM_RESET = 0xFD


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
        tx_packet += bytes([packet_id, device_id, len(tx_packet) + 4])
        tx_packet += bytes([BPLProtocol.CRC8_FUNC(tx_packet)])
        packet: bytes = cobs.encode(tx_packet) + b'\x00'
        return packet

    @staticmethod
    def decode_floats(data: Union[bytes, bytearray]) -> List[float]:
        """ Decode a received byte list, into a float list as specified by the bpl protocol"""
        list_data = list(struct.unpack(str(int(len(data) / 4)) + "f", data))
        return list_data

    @staticmethod
    def encode_floats(float_list: List[float]) -> bytes:
        """ Decode a received byte list, into a float list as specified by the bpl protocol"""
        data = struct.pack('%sf' % len(float_list), *float_list)
        return data


if __name__ == '__main__':

    packet = BPLProtocol.encode_packet(0x01, PacketID.VELOCITY, BPLProtocol.encode_floats([6.0]))
    print(packet)




