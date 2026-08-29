#include "network/vtd_client.hpp"

#include <limits>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket INVALID_NATIVE_SOCKET = INVALID_SOCKET;

int getLastSocketError() {
    return WSAGetLastError();
}

void closeNativeSocket(NativeSocket socket) {
    closesocket(socket);
}
#else
using NativeSocket = int;
constexpr NativeSocket INVALID_NATIVE_SOCKET = -1;

int getLastSocketError() {
    return errno;
}

void closeNativeSocket(NativeSocket socket) {
    ::close(socket);
}
#endif

NativeSocket getNativeSocket(const VtdClientData& client) {
    return static_cast<NativeSocket>(client.socket);
}

std::runtime_error makeSocketError(
    const std::string& operation,
    int error) {
    return std::runtime_error(
        operation + " failed (socket error " + std::to_string(error) + ")");
}

void enableTcpNoDelay(NativeSocket socket) {
    const int enabled = 1;
#ifdef _WIN32
    setsockopt(
        socket,
        IPPROTO_TCP,
        TCP_NODELAY,
        reinterpret_cast<const char*>(&enabled),
        sizeof(enabled));
#else
    setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
#endif
}

VtdClientData makeVtdClient(const VtdNetworkConfig& config) {
    if (config.host_ip.empty()) {
        throw std::invalid_argument("VTD host must not be empty");
    }
    if (config.tcp_port == 0) {
        throw std::invalid_argument("VTD TCP port must be between 1 and 65535");
    }

    VtdClientData client;
    client.host = config.host_ip;
    client.port = config.tcp_port;
    return client;
}

bool isVtdClientConnected(const VtdClientData& client) noexcept {
    return client.socket != -1;
}

bool didVtdReceiveTimeOut(const VtdClientData& client) noexcept {
    return client.receiveTimedOut;
}

void setVtdReceiveTimeout(VtdClientData& client, double timeoutSeconds) {
    if (!isVtdClientConnected(client) || timeoutSeconds <= 0.0) return;
#ifdef _WIN32
    const DWORD milliseconds = static_cast<DWORD>(std::clamp(
        std::ceil(timeoutSeconds * 1000.0), 1.0,
        static_cast<double>(std::numeric_limits<DWORD>::max())));
    if (setsockopt(getNativeSocket(client), SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&milliseconds),
                   sizeof(milliseconds)) != 0) {
        throw makeSocketError("setsockopt(SO_RCVTIMEO)", getLastSocketError());
    }
#else
    timeval timeout{};
    timeout.tv_sec = static_cast<long>(timeoutSeconds);
    timeout.tv_usec = static_cast<long>(
        (timeoutSeconds - timeout.tv_sec) * 1000000.0);
    if (setsockopt(getNativeSocket(client), SOL_SOCKET, SO_RCVTIMEO,
                   &timeout, sizeof(timeout)) != 0) {
        throw makeSocketError("setsockopt(SO_RCVTIMEO)", getLastSocketError());
    }
#endif
}

void connectVtdClient(VtdClientData& client) {
    if (isVtdClientConnected(client)) {
        return;
    }

#ifdef _WIN32
    WSADATA data{};
    const int startupResult = WSAStartup(MAKEWORD(2, 2), &data);
    if (startupResult != 0) {
        throw makeSocketError("WSAStartup", startupResult);
    }
    client.winsockInitialized = true;
#endif

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* addresses = nullptr;
    const std::string portText = std::to_string(client.port);
    const int lookupResult =
        getaddrinfo(client.host.c_str(), portText.c_str(), &hints, &addresses);
    if (lookupResult != 0) {
#ifdef _WIN32
        const std::string detail = gai_strerrorA(lookupResult);
#else
        const std::string detail = gai_strerror(lookupResult);
#endif
        disconnectVtdClient(client);
        throw std::runtime_error(
            "cannot resolve VTD host '" + client.host + "': " + detail);
    }

    int connectionError = 0;
    for (addrinfo* address = addresses;
         address != nullptr;
         address = address->ai_next) {
        NativeSocket candidate =
            ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (candidate == INVALID_NATIVE_SOCKET) {
            connectionError = getLastSocketError();
            continue;
        }

        enableTcpNoDelay(candidate);
        if (::connect(
                candidate,
                address->ai_addr,
                static_cast<int>(address->ai_addrlen)) == 0) {
            client.socket = static_cast<std::intptr_t>(candidate);
            break;
        }

        connectionError = getLastSocketError();
        closeNativeSocket(candidate);
    }
    freeaddrinfo(addresses);

    if (!isVtdClientConnected(client)) {
        disconnectVtdClient(client);
        throw makeSocketError(
            "connection to " + client.host + ':' + portText,
            connectionError);
    }
}

void disconnectVtdClient(VtdClientData& client) noexcept {
    if (isVtdClientConnected(client)) {
        closeNativeSocket(getNativeSocket(client));
        client.socket = -1;
    }
#ifdef _WIN32
    if (client.winsockInitialized) {
        WSACleanup();
        client.winsockInitialized = false;
    }
#endif
}

void sendVtdData(
    VtdClientData& client,
    const void* data,
    std::size_t size) {
    if (!isVtdClientConnected(client)) {
        throw std::runtime_error("cannot send: VTD client is not connected");
    }
    if (size != 0 && data == nullptr) {
        throw std::invalid_argument("cannot send from a null buffer");
    }

    const auto* bytes = static_cast<const char*>(data);
    std::size_t sent = 0;
    while (sent < size) {
        const auto remaining = size - sent;
#ifdef _WIN32
        const int chunk = static_cast<int>(
            remaining > static_cast<std::size_t>(std::numeric_limits<int>::max())
                ? std::numeric_limits<int>::max()
                : remaining);
        const int result =
            ::send(getNativeSocket(client), bytes + sent, chunk, 0);
        if (result == SOCKET_ERROR) {
#else
        const auto result =
            ::send(getNativeSocket(client), bytes + sent, remaining, 0);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
#endif
            throw makeSocketError("send", getLastSocketError());
        }
        if (result == 0) {
            throw std::runtime_error("send failed: VTD closed the connection");
        }
        sent += static_cast<std::size_t>(result);
    }
}

std::size_t receiveVtdData(
    VtdClientData& client,
    void* buffer,
    std::size_t capacity) {
    if (!isVtdClientConnected(client)) {
        throw std::runtime_error("cannot receive: VTD client is not connected");
    }
    if (capacity == 0) {
        return 0;
    }
    if (buffer == nullptr) {
        throw std::invalid_argument("cannot receive into a null buffer");
    }

    client.receiveTimedOut = false;
#ifdef _WIN32
    const int chunk = static_cast<int>(
        capacity > static_cast<std::size_t>(std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : capacity);
    const int result =
        ::recv(getNativeSocket(client), static_cast<char*>(buffer), chunk, 0);
    if (result == SOCKET_ERROR) {
        const int error = getLastSocketError();
        if (error == WSAETIMEDOUT || error == WSAEWOULDBLOCK) {
            client.receiveTimedOut = true;
            return 0;
        }
#else
    std::ptrdiff_t result = 0;
    do {
        result = ::recv(getNativeSocket(client), buffer, capacity, 0);
    } while (result < 0 && errno == EINTR);
    if (result < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            client.receiveTimedOut = true;
            return 0;
        }
#endif
        throw makeSocketError("receive", getLastSocketError());
    }
    return static_cast<std::size_t>(result);
}
