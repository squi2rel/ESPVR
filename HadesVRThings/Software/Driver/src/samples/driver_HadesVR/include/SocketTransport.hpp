#pragma once

#ifndef _SOCKET_TRANSPORT_H_
#define _SOCKET_TRANSPORT_H_

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#include "DataTransport.hpp"
#include "dataHandler.h"

#define SOCKET_BUFFER_SIZE 512
#define PORT 6778

constexpr char RIGHT_CONTROLLER = 0xE1; // same as Pipe
constexpr char LEFT_CONTROLLER = 0xD2;
constexpr char TRACKER = 0xC3;

constexpr char HMD_PKT = 0xAA;

class SocketTransport : public DataTransport {
public:
	SocketTransport();

	// Inherited via DataTransport
	int Start() override;
	void Stop() override;
	bool IsConnected() override;
	int ReadPacket(uint8_t* buffer, size_t length) override;

private:
	bool initialized;
	SOCKET udpSocket;
	time_t lastBroadcast;
	int port;
	bool newContPacket;
	bool newHMDPacket;
	ControllerPacket packet;
	HMDRAWPacket hmd;
};

#endif
