// GpsUdpServer.cpp - X-Plane 12 GNSS-to-Android UDP server.
//
// Protocol mirror of MSFS-SimGPStoAndroid-main/CPP/MSFSSimConnect/main.cpp.
// Cross-platform: winsock on Windows, POSIX sockets elsewhere.

#include "GpsUdpServer.h"

#include <cstring>
#include <chrono>

namespace
{
    bool SameAddr(const sockaddr_storage& a, int aLen, const sockaddr_storage& b, int bLen)
    {
        return aLen == bLen && std::memcmp(&a, &b, static_cast<size_t>(aLen)) == 0;
    }
}

long long GpsUdpServer::nowMs() const
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

void GpsUdpServer::start(uint16_t port)
{
    stop(); // no-op if not running

#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        return;
#endif

    SOCKET sock = INVALID_SOCKET;
    // IPv6 dual-stack socket (accepts IPv4-mapped and IPv6, for LAN and public IPv6).
    sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sock != INVALID_SOCKET)
    {
        int v6only = 0;
        setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, (const char*)&v6only, sizeof(v6only));
        sockaddr_in6 addr6 = {};
        addr6.sin6_family = AF_INET6;
        addr6.sin6_port   = htons(port);
        addr6.sin6_addr   = in6addr_any;
        if (bind(sock, (sockaddr*)&addr6, sizeof(addr6)) == SOCKET_ERROR)
        {
#ifdef _WIN32
            closesocket(sock);
#else
            close(sock);
#endif
            sock = INVALID_SOCKET;
        }
    }

    // Fall back to IPv4.
    if (sock == INVALID_SOCKET)
    {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock != INVALID_SOCKET)
        {
            sockaddr_in addr4 = {};
            addr4.sin_family      = AF_INET;
            addr4.sin_port        = htons(port);
            addr4.sin_addr.s_addr = INADDR_ANY;
            if (bind(sock, (sockaddr*)&addr4, sizeof(addr4)) == SOCKET_ERROR)
            {
#ifdef _WIN32
                closesocket(sock);
#else
                close(sock);
#endif
                sock = INVALID_SOCKET;
            }
        }
    }
    if (sock == INVALID_SOCKET)
        return;

#ifdef _WIN32
    // Ignore UDP WSAECONNRESET from ICMP port-unreachable replies.
    DWORD bytesReturned = 0;
    BOOL  newBehavior   = FALSE;
    WSAIoctl(sock, SIO_UDP_CONNRESET, &newBehavior, sizeof(newBehavior),
             NULL, 0, &bytesReturned, NULL, NULL);
    // Periodic wake-up so stop() can join the thread promptly.
    DWORD timeout = 500;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
    timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 500000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#endif

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sock = sock;
        m_clients.clear();
        m_lastHeartbeatMs = nowMs();
    }
    m_running = true;
    m_thread  = std::thread(&GpsUdpServer::serverLoop, this);
}

void GpsUdpServer::stop()
{
    if (!m_running.load())
        return;
    m_running = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_sock != INVALID_SOCKET)
        {
#ifdef _WIN32
            closesocket(m_sock);
#else
            close(m_sock);
#endif
            m_sock = INVALID_SOCKET;
        }
        m_clients.clear();
    }
    if (m_thread.joinable())
        m_thread.join();
}

int GpsUdpServer::clientCount() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<int>(m_clients.size());
}

void GpsUdpServer::broadcast(const std::string& line)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_sock == INVALID_SOCKET || m_clients.empty())
        return;
    for (const Client& c : m_clients)
    {
        sendto(m_sock, line.data(), (int)line.size(), 0,
               (const sockaddr*)&c.addr, c.addrLen);
    }
}

void GpsUdpServer::pruneAndHeartbeatLocked()
{
    long long now = nowMs();
    // Drop clients that stopped sending HELLO; heartbeat the rest.
    for (size_t i = m_clients.size(); i-- > 0;)
    {
        if (now - m_clients[i].lastHelloMs > CLIENT_TIMEOUT_MS)
        {
            m_clients.erase(m_clients.begin() + i);
            continue;
        }
        static const char hb[] = "HEARTBEAT";
        sendto(m_sock, hb, (int)(sizeof(hb) - 1), 0,
               (const sockaddr*)&m_clients[i].addr, m_clients[i].addrLen);
    }
}

void GpsUdpServer::serverLoop()
{
    char buf[2048];
    while (m_running.load())
    {
        // Periodic timer: heartbeats + client timeout pruning run on a
        // schedule regardless of incoming HELLO traffic (500 ms recv timeout
        // guarantees we wake up regularly).
        {
            long long now = nowMs();
            if (now - m_lastHeartbeatMs >= HEARTBEAT_MS)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_lastHeartbeatMs = now;
                pruneAndHeartbeatLocked();
            }
        }

        sockaddr_storage from = {};
        int fromLen = (int)sizeof(from);
        int len = recvfrom(m_sock, buf, (int)sizeof(buf) - 1, 0,
                           (sockaddr*)&from, &fromLen);
        if (len == SOCKET_ERROR)
        {
            // Timeout (500 ms) or socket closed; just re-check the run flag.
            if (!m_running.load())
                break;
            continue;
        }
        if (len <= 0)
            continue;
        buf[len] = '\0';
        std::string payload(buf, static_cast<size_t>(len));

        // Phone registration / keep-alive: "HELLO:<timestamp>"
        if (payload.rfind("HELLO", 0) != 0)
            continue;

        long long phoneTs = 0;
        size_t colon = payload.find(':');
        if (colon != std::string::npos)
        {
            try { phoneTs = std::stoll(payload.substr(colon + 1)); } catch (...) {}
        }

        bool accepted = false;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_sock == INVALID_SOCKET)
                break;

            bool found = false;
            for (Client& c : m_clients)
            {
                if (SameAddr(c.addr, c.addrLen, from, fromLen))
                {
                    c.lastHelloMs = nowMs();
                    c.lastPhoneTs = phoneTs;
                    found = true;
                    accepted = true;
                    break;
                }
            }
            if (!found)
            {
                if (static_cast<int>(m_clients.size()) < MAX_CLIENTS)
                {
                    Client c;
                    c.addr = from;
                    c.addrLen = fromLen;
                    c.lastHelloMs = nowMs();
                    c.lastPhoneTs = phoneTs;
                    m_clients.push_back(c);
                    accepted = true;
                }
                else
                {
                    static const char full[] = "SERVER_FULL";
                    sendto(m_sock, full, (int)(sizeof(full) - 1), 0,
                           (const sockaddr*)&from, fromLen);
                }
            }
        }

        if (accepted)
        {
            // Reply PONG immediately with the echoed timestamp (latency probe).
            std::string pong = "PONG:" + std::to_string(phoneTs);
            sendto(m_sock, pong.data(), (int)pong.size(), 0,
                   (const sockaddr*)&from, fromLen);
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_sock != INVALID_SOCKET)
        {
#ifdef _WIN32
            closesocket(m_sock);
#else
            close(m_sock);
#endif
            m_sock = INVALID_SOCKET;
        }
    }
}
