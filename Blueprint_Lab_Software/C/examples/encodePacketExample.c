
/* 
C/examples/encodePacketExample.c

Author: John Sumskas
Email: j.sumskas@blueprintlab.com
Date: 21/06/2022

*/
#include <stdio.h>
#include "../bplprotocol.h"

void main(void){

    /* **************************** encodePacket() Example *************************************** */
    // create a buffer of bytes for your packet to be filled in with.
    uint8_t encodedPacket[MAX_PACKET_LENGTH];
    // Set data to zeros
    memset(encodedPacket, 0, MAX_PACKET_LENGTH);   

    // Encoding data with the following information
    uint8_t deviceID = 0x01;
    uint8_t packetID = MODE;

    uint8_t data[5] = {1, 2, 3, 4, 5};   // Int packet

    struct Packet packet;

    packet.deviceID = deviceID;
    packet.packetID = packetID;
    memcpy(packet.data, data, 5);
    packet.dataLength = 5;

    // encode the packet. 
    size_t packetLength = encodePacket(encodedPacket, &packet);

    // Print the encoded packet to stdio.
    printf("Encoded Packet: ");
    for (int i = 0; i<=packetLength; i++){
        printf(" %d", encodedPacket[i]);
    }
    printf("\n");

}