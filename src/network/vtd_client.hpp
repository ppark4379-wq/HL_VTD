#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace hlvtd::network {

class VtdClient {
public:
    VtdClient(std::string host, std::uint16_t port);
    ~VtdClient();

    VtdClient(const VtdClient&) = delete;
    VtdClient& operator=(const VtdClient&) = delete;

    void connect();
    void disconnect() noexcept;
    [[nodiscard]] bool isConnected() const noexcept;

    void sendAll(const void* data, std::size_t size);
    // Returns 0 when the peer has closed the connection.
    std::size_t receive(void* buffer, std::size_t capacity);

private:
    std::string host_;
    std::uint16_t port_;
    std::intptr_t socket_{-1};
#ifdef _WIN32
    bool winsock_initialized_{false};
#endif
};

}  // namespace hlvtd::network
