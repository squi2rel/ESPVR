#include "SocketTransport.hpp"
#include "driverlog.h"
#include "ringbuffer.h"
#include "settingsAPIKeys.h"
#include <ctime>

#define TIME time(NULL)

using namespace std;

const char *header = "HadesVR Sniffer <3";

int last_err;
char str_buffer[128];
sockaddr_in broadcastAddr;

SocketTransport::SocketTransport()
    : initialized(false),
    lastBroadcast(0),
    udpSocket(INVALID_SOCKET),
    port(-1),
    newContPacket(false),
    newHMDPacket(false),
    packet{ 0, 2 },
    hmd { 0, 3 }
{
}

int SocketTransport::Start()
{
    port = vr::VRSettings()->GetInt32(k_pch_Driver_Section, k_pch_Socket_Port);
    char address[16];
    vr::VRSettings()->GetString(k_pch_Driver_Section, k_pch_Socket_Address, address, 16);
    if (port <= 0 || port > 65535) {
        DriverLog("[SocketTransport] Invaild port! Should >0 and <65536");
        return 1;
    }
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        DriverLog("[SocketTransport] WSAStartup failed");
        return 1;
    }

    udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpSocket == INVALID_SOCKET) {
        DriverLog("[SocketTransport] Socket creation failed");
        WSACleanup();
        return 1;
    }

    BOOL broadcast = TRUE;
    if (setsockopt(udpSocket, SOL_SOCKET, SO_BROADCAST, (char*)&broadcast, sizeof(broadcast)) == SOCKET_ERROR) {
        DriverLog("[SocketTransport] Broadcast setting failed");
        closesocket(udpSocket);
        WSACleanup();
        return 1;
    }

    u_long mode = 1;
    if (ioctlsocket(udpSocket, FIONBIO, &mode) == SOCKET_ERROR) {
        DriverLog("[SocketTransport] Async setting failed.");
        closesocket(udpSocket);
        WSACleanup();
        return 1;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if ((last_err = bind(udpSocket, (sockaddr*)&addr, sizeof(addr))) == SOCKET_ERROR) {
        snprintf(str_buffer, sizeof(str_buffer), "[SocketTransport] Bind failed (%d)", last_err);
        DriverLog(str_buffer);
        closesocket(udpSocket);
        WSACleanup();
        return 1;
    }

    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(port);
    if (InetPton(AF_INET, address, &broadcastAddr.sin_addr) != 1) {
        snprintf(str_buffer, sizeof(str_buffer), "[SocketTransport] Invalid address! (%s)", address);
        DriverLog(str_buffer);
        closesocket(udpSocket);
        WSACleanup();
        return 1;
    }

    snprintf(str_buffer, sizeof(str_buffer), "[SocketTransport] Socket opened successfully on port %d", port);
    DriverLog(str_buffer);
    initialized = true;
    return 0;
}

void SocketTransport::Stop()
{
    initialized = false;
    closesocket(udpSocket);
    WSACleanup();
}

bool SocketTransport::IsConnected()
{
    return initialized;
}

int SocketTransport::ReadPacket(uint8_t* outBuf, size_t length)
{
    if (TIME - lastBroadcast > 5) {
        if ((last_err = sendto(udpSocket, header, (int)strlen(header), 0, (sockaddr*)&broadcastAddr, sizeof(broadcastAddr))) < 0) {
            snprintf(str_buffer, sizeof(str_buffer), "[SocketTransport] Broadcast failed (%d)", last_err);
            DriverLog(str_buffer);
        }
        lastBroadcast = TIME;
    }

    char buffer[SOCKET_BUFFER_SIZE];
    sockaddr_in clientAddr;
    int clientAddrSize = sizeof(clientAddr);

    int recvLen = recvfrom(udpSocket, buffer, SOCKET_BUFFER_SIZE, 0, nullptr, nullptr);
    if (recvLen == SOCKET_ERROR) {
        if ((last_err = WSAGetLastError()) != WSAEWOULDBLOCK) {
            snprintf(str_buffer, sizeof(str_buffer), "[SocketTransport] Socket read error (%d)", last_err);
            DriverLog(str_buffer);
        }
        return 0;
    }

    if (recvLen == SOCKET_BUFFER_SIZE) {
        DriverLog("[SocketTransport] Packet is too big!");
        return 0;
    }

    switch (buffer[0])
    {
        case RIGHT_CONTROLLER: {
            if (recvLen != 30) return 0;
            memcpy_s(&packet.Ctrl1_QuatW, 29, buffer + 1, 29);
            newContPacket = true;
            break;
        }
        case LEFT_CONTROLLER: {
            if (recvLen != 30) return 0;
            memcpy_s(&packet.Ctrl2_QuatW, 29, buffer + 1, 29);
            newContPacket = true;
            break;
        }
        case HMD_PKT: {
            if (recvLen != 19) return 0;
            memcpy_s(&hmd.AccX, 18, buffer + 1, 18);
            newHMDPacket = true;
            break;
        }
        case TRACKER: {
            break;
        }
        default: {
            return 0;
        }
    }

    if (newContPacket) {
        newContPacket = false;
        memcpy_s(outBuf, length, &packet, sizeof(packet));
        return recvLen;
    }

    if (newHMDPacket) {
        newHMDPacket = false;
        memcpy_s(outBuf, length, &hmd, sizeof(hmd));
        return recvLen;
    }

    return 0;
}
