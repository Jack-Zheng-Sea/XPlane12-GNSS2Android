// GpsUdpServer.h - X-Plane 12 GNSS-to-Android UDP server.
//
// This is the X-Plane port of the UDP listener found in the MSFS project:
//   MSFS-SimGPStoAndroid-main/CPP/MSFSSimConnect/main.cpp
// The wire protocol is byte-for-byte compatible so the existing Android app
// (package com.msfs.simconnect.client) works without any changes:
//
//   Phone -> PC :  "HELLO:<phoneUptimeMillis>"   (register / keep-alive)
//   PC    -> Phone: "PONG:<echoedTimestamp>"      (latency echo)
//   PC    -> Phone: "HEARTBEAT"                   (keep-alive, every 2 s)
//   PC    -> Phone: "SERVER_FULL"                 (client table is full)
//   PC    -> Phone: "<posFrame>\n"                (position data, see below)
//
// Position frame (one line, 8 comma separated fields):
//   lat,lon,altFt,heading,pitch,roll,groundSpeed_mps,airspeed_mps
//
// The server runs on its own thread; the X-Plane flight loop calls
// broadcast() on the main thread to push position frames.

#ifndef XP12_GPS_UDP_SERVER_H
#define XP12_GPS_UDP_SERVER_H

// Keep winsock2 before windows.h (XPLM headers include windows.h).
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIORW(IOC_VENDOR, 12)
#endif
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class GpsUdpServer
{
public:
    static const int  MAX_CLIENTS      = 3;    // mirrors the MSFS PC server
    static const int  HEARTBEAT_MS     = 2000;
    static const long CLIENT_TIMEOUT_MS = 15000;

    GpsUdpServer() = default;
    ~GpsUdpServer() = default;

    GpsUdpServer(const GpsUdpServer&) = delete;
    GpsUdpServer& operator=(const GpsUdpServer&) = delete;

    /// Opens a UDP socket on the given port and starts the listener thread.
    /// Safe to call again (old server is stopped first).
    void start(uint16_t port);

    /// Stops the listener thread and closes the socket. Joins the thread.
    /// Must be called before the plugin is unloaded.
    void stop();

    /// Number of currently registered phone clients (thread-safe).
    int clientCount() const;

    /// Sends one text line to every registered phone client.
    /// Called from the X-Plane flight loop (main thread).
    void broadcast(const std::string& line);

private:
    struct Client
    {
        sockaddr_storage addr      = {};
        int              addrLen   = 0;
        long long        lastHelloMs = 0;
        long long        lastPhoneTs  = 0;
    };

    void  serverLoop();
    void  pruneAndHeartbeatLocked();
    long long nowMs() const;

    mutable std::mutex   m_mutex;
    std::vector<Client>  m_clients;
    SOCKET               m_sock = INVALID_SOCKET;
    std::atomic<bool>    m_running{ false };
    std::thread          m_thread;
    long long            m_lastHeartbeatMs = 0;
};

#endif // XP12_GPS_UDP_SERVER_H
