/*************************************************************************************************************************
 * Copyright 2024 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the “Software”), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *************************************************************************************************************************/
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <iostream>
#include <net/if.h>
#include <unistd.h>

#include "utils/ip_tools.h"
#include "utils/logger.h"

std::string getLocalIPAddress(const char *interfaceName)
{
    struct ifaddrs *ifAddrList;
    if (getifaddrs(&ifAddrList) == -1)
    {
        MLOG_ERROR("getifaddrs");
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    std::string localIP;

    for (struct ifaddrs *ifa = ifAddrList; ifa != nullptr; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == nullptr || ifa->ifa_addr->sa_family != AF_INET)
            continue;

        if (strcmp(ifa->ifa_name, interfaceName) == 0)
        {
            char ipBuffer[INET_ADDRSTRLEN];
            struct sockaddr_in *sa = reinterpret_cast<struct sockaddr_in *>(ifa->ifa_addr);
            inet_ntop(AF_INET, &(sa->sin_addr), ipBuffer, INET_ADDRSTRLEN);
            localIP = ipBuffer;
            break;
        }
    }

    freeifaddrs(ifAddrList);

    return localIP;
}

bool joinMulticastGroup(const std::string &interfaceName,const std::string &multicastIP)
{
    // 创建 socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        MLOG_ERROR("socket create error");
        perror("socket");
        return false;
    }
    // 设置网卡绑定
    if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, interfaceName.c_str(), strlen(interfaceName.c_str())) == -1)
    {
        MLOG_ERROR("setsockopt error");
        perror("setsockopt");
        close(sockfd);
        return false;
    }
    // 加入组播组
    struct ip_mreqn mreq;
    auto localIP = getLocalIPAddress(interfaceName.c_str());
    mreq.imr_multiaddr.s_addr = inet_addr(multicastIP.c_str());
    mreq.imr_address.s_addr = inet_addr(localIP.c_str());
    mreq.imr_ifindex = if_nametoindex(interfaceName.c_str());
    if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) == -1)
    {
        MLOG_ERROR("setsockopt error");
        perror("setsockopt");
        close(sockfd);
        return false;
    }
    return true;
}
