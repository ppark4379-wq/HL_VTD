#include "network/vtd_client.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

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

namespace hlvtd::network {
namespace {

#ifdef _WIN32
using NativeSocket = SOCKET;
constexpr NativeSocket kInvalidSocket = INVALID_SOCKET;
int lastSocketError() { return WSAGetLastError(); }
void closeSocket(NativeSocket socket) { closesocket(socket); }
#else
using NativeSocket = int;
constexpr NativeSocket kInvalidSocket = -1;
int lastSocketError() { return errno; }
void closeSocket(NativeSocket socket) { ::close(socket); }
#endif

NativeSocket asNative(std::intptr_t socket) {
    return static_cast<NativeSocket>(socket);
}

std::runtime_error socketError(const std::string& operation, int error) {
    return std::runtime_error(operation + " failed (socket error " + std::to_string(error) + ")");
}

}  // namespace

VtdClient::VtdClient(std::string host, std::uint16_t port)
    : host_(std::move(host)), port_(port) {
    if (host_.empty()) {
        throw std::invalid_argument("VTD host must not be empty");
    }
    if (port_ == 0) {
        throw std::invalid_argument("VTD TCP port must be between 1 and 65535");
    }
}

VtdClient::~VtdClient() {
    disconnect();
}

void VtdClient::connect() {
    if (isConnected()) {
        return;
    }

#ifdef _WIN32
    WSADATA data{};
    const int startup_result = WSAStartup(MAKEWORD(2, 2), &data);
    if (startup_result != 0) {
        throw socketError("WSAStartup", startup_result);
    }
    winsock_initialized_ = true;
#endif

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* addresses = nullptr;
    const std::string port_text = std::to_string(port_);
    const int lookup_result = getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &addresses);
    if (lookup_result != 0) {
#ifdef _WIN32
        const std::string detail = gai_strerrorA(lookup_result);
#else
        const std::string detail = gai_strerror(lookup_result);
#endif
        disconnect();
        throw std::runtime_error("cannot resolve VTD host '" + host_ + "': " + detail);
    }

    int connection_error = 0;
    for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        NativeSocket candidate =
            ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (candidate == kInvalidSocket) {
            connection_error = lastSocketError();
            continue;
        }

        const int no_delay = 1;
#ifdef _WIN32
        setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&no_delay), sizeof(no_delay));
#else
        setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
#endif

        if (::connect(candidate, address->ai_addr, static_cast<int>(address->ai_addrlen)) == 0) {
            socket_ = static_cast<std::intptr_t>(candidate);
            break;
        }

        connection_error = lastSocketError();
        closeSocket(candidate);
    }
    freeaddrinfo(addresses);

    if (!isConnected()) {
        disconnect();
        throw socketError("connection to " + host_ + ':' + port_text, connection_error);
    }
}

void VtdClient::disconnect() noexcept {
    if (isConnected()) {
        closeSocket(asNative(socket_));
        socket_ = -1;
    }
#ifdef _WIN32
    if (winsock_initialized_) {
        WSACleanup();
        winsock_initialized_ = false;
    }
#endif
}

bool VtdClient::isConnected() const noexcept {
    return socket_ != -1;
}

void VtdClient::sendAll(const void* data, std::size_t size) {
    if (!isConnected()) {
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
        const int result = ::send(asNative(socket_), bytes + sent, chunk, 0);
        if (result == SOCKET_ERROR) {
#else
        const auto result = ::send(asNative(socket_), bytes + sent, remaining, 0);
        if (result < 0) {
            if (errno == EINTR) {
                continue;
            }
#endif
            throw socketError("send", lastSocketError());
        }
        if (result == 0) {
            throw std::runtime_error("send failed: VTD closed the connection");
        }
        sent += static_cast<std::size_t>(result);
    }
}

std::size_t VtdClient::receive(void* buffer, std::size_t capacity) {
    if (!isConnected()) {
        throw std::runtime_error("cannot receive: VTD client is not connected");
    }
    if (capacity == 0) {
        return 0;
    }
    if (buffer == nullptr) {
        throw std::invalid_argument("cannot receive into a null buffer");
    }

#ifdef _WIN32
    const int chunk = static_cast<int>(
        capacity > static_cast<std::size_t>(std::numeric_limits<int>::max())
            ? std::numeric_limits<int>::max()
            : capacity);
    const int result = ::recv(asNative(socket_), static_cast<char*>(buffer), chunk, 0);
    if (result == SOCKET_ERROR) {
#else
    std::ptrdiff_t result = 0;
    do {
        result = ::recv(asNative(socket_), buffer, capacity, 0);
    } while (result < 0 && errno == EINTR);
    if (result < 0) {
#endif
        throw socketError("receive", lastSocketError());
    }
    return static_cast<std::size_t>(result);
}

}  // namespace hlvtd::network
