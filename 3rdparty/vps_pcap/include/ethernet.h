#ifndef VPSKIT_FUN_ETHERNET_H
#define VPSKIT_FUN_ETHERNET_H

namespace vpskit {
    // 小端转到大端，或者大端转到小端
    unsigned short transEnd(const unsigned short &data);
    unsigned int transEnd(const unsigned int &data);

    unsigned short udpChecksum(unsigned short *buf1, unsigned short *buf2, int nword);
    unsigned short udpChecksum(unsigned short *buf1, unsigned short *buf2, unsigned short *buf3, int nword);
}

#endif // VPSKIT_FUN_ETHERNET_H
