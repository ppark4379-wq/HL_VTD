#include "network/receive/rdb_clock.hpp"

#include "network/vtd_client.hpp"

#include <cmath>
#include <cstring>
#include <iterator>
#include <stdexcept>

namespace {

constexpr std::uint16_t kRdbMagic = 35712;
constexpr std::size_t kRdbMessageHeaderSize = 24;
constexpr std::size_t kRdbEntryHeaderSize = 16;
constexpr std::uint16_t kRdbObjectStatePackage = 9;
constexpr std::size_t kObjectNameOffset = 8;
constexpr std::size_t kObjectPositionOffset = 64;
constexpr std::size_t kObjectStateBaseSize = 112;
constexpr std::size_t kMaximumRdbMessageSize = 256 * 1024 * 1024;
constexpr std::size_t kMaximumClockSamples = 256;

std::uint16_t readUint16LE(const std::uint8_t* data) {
    return static_cast<std::uint16_t>(data[0]) |
           static_cast<std::uint16_t>(data[1] << 8);
}

std::uint32_t readUint32LE(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

double readFloat64LE(const std::uint8_t* data) {
    const std::uint64_t bits = static_cast<std::uint64_t>(data[0]) |
        (static_cast<std::uint64_t>(data[1]) << 8) |
        (static_cast<std::uint64_t>(data[2]) << 16) |
        (static_cast<std::uint64_t>(data[3]) << 24) |
        (static_cast<std::uint64_t>(data[4]) << 32) |
        (static_cast<std::uint64_t>(data[5]) << 40) |
        (static_cast<std::uint64_t>(data[6]) << 48) |
        (static_cast<std::uint64_t>(data[7]) << 56);
    double value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool objectNameIsEgo(const std::uint8_t* object) {
    constexpr char kEgo[] = "Ego";
    return std::memcmp(object + kObjectNameOffset, kEgo, 3) == 0 &&
           object[kObjectNameOffset + 3] == 0;
}

void appendEgoSample(RdbClockData& clock,
                     const std::uint8_t* message,
                     std::size_t messageSize,
                     std::uint32_t frameNo,
                     double simulationTime,
                     std::size_t headerSize) {
    std::size_t entryOffset = headerSize;
    while (entryOffset + kRdbEntryHeaderSize <= messageSize) {
        const auto* entry = message + entryOffset;
        const std::size_t entryHeaderSize = readUint32LE(entry);
        const std::size_t dataSize = readUint32LE(entry + 4);
        const std::size_t elementSize = readUint32LE(entry + 8);
        const std::uint16_t packageId = readUint16LE(entry + 12);
        if (entryHeaderSize < kRdbEntryHeaderSize ||
            entryHeaderSize > messageSize - entryOffset ||
            dataSize > messageSize - entryOffset - entryHeaderSize) {
            return;
        }

        const auto* payload = entry + entryHeaderSize;
        if (packageId == kRdbObjectStatePackage &&
            elementSize >= kObjectStateBaseSize) {
            for (std::size_t offset = 0;
                 offset + kObjectStateBaseSize <= dataSize;
                 offset += elementSize) {
                const auto* object = payload + offset;
                if (!objectNameIsEgo(object)) continue;
                const double x = readFloat64LE(object + kObjectPositionOffset);
                const double y = readFloat64LE(object + kObjectPositionOffset + 8);
                if (std::isfinite(simulationTime) && std::isfinite(x) &&
                    std::isfinite(y)) {
                    clock.samples.push_back({frameNo, simulationTime, x, y});
                    while (clock.samples.size() > kMaximumClockSamples) {
                        clock.samples.pop_front();
                    }
                }
                return;
            }
        }
        entryOffset += entryHeaderSize + dataSize;
    }
}

}  // namespace

RdbClockData makeRdbClock(const VtdNetworkConfig& config) {
    VtdNetworkConfig rdbConfig = config;
    rdbConfig.tcp_port = config.rdb_port;
    RdbClockData clock;
    clock.client = makeVtdClient(rdbConfig);
    clock.streamBuffer.reserve(128 * 1024);
    return clock;
}

void connectRdbClock(RdbClockData& clock) {
    connectVtdClient(clock.client);
}

void disconnectRdbClock(RdbClockData& clock) noexcept {
    disconnectVtdClient(clock.client);
}

bool isRdbClockConnected(const RdbClockData& clock) noexcept {
    return isVtdClientConnected(clock.client);
}

void extractRdbClockSamples(RdbClockData& clock) {
    while (clock.streamBuffer.size() >= kRdbMessageHeaderSize) {
        const auto* header = clock.streamBuffer.data();
        if (readUint16LE(header) != kRdbMagic) {
            clock.streamBuffer.erase(clock.streamBuffer.begin());
            continue;
        }
        const std::size_t headerSize = readUint32LE(header + 4);
        const std::size_t dataSize = readUint32LE(header + 8);
        if (headerSize < kRdbMessageHeaderSize ||
            headerSize > kMaximumRdbMessageSize ||
            dataSize > kMaximumRdbMessageSize - headerSize) {
            clock.streamBuffer.erase(clock.streamBuffer.begin());
            continue;
        }
        const std::size_t messageSize = headerSize + dataSize;
        if (clock.streamBuffer.size() < messageSize) return;

        const std::uint32_t frameNo = readUint32LE(header + 12);
        const double simulationTime = readFloat64LE(header + 16);
        appendEgoSample(clock, header, messageSize, frameNo, simulationTime,
                        headerSize);
        clock.streamBuffer.erase(clock.streamBuffer.begin(),
                                 clock.streamBuffer.begin() + messageSize);
    }
}

void pollRdbClock(RdbClockData& clock) {
    if (!isRdbClockConnected(clock)) return;
    // 한 GT 주기 사이에 쌓인 메시지를 모두 비우되 무한 루프는 막는다.
    for (int attempt = 0; attempt < 64; ++attempt) {
        const std::size_t received = receiveVtdData(
            clock.client, clock.receiveBuffer.data(), clock.receiveBuffer.size());
        if (received == 0) {
            if (!didVtdReceiveTimeOut(clock.client)) {
                throw std::runtime_error("VTD closed the RDB connection");
            }
            return;
        }
        clock.streamBuffer.insert(clock.streamBuffer.end(),
                                  clock.receiveBuffer.begin(),
                                  clock.receiveBuffer.begin() + received);
        extractRdbClockSamples(clock);
        // TCP preserves bytes, not application-message boundaries. A short
        // recv therefore does not mean that VTD's socket queue is empty. Keep
        // draining until SO_RCVTIMEO reports no immediately available data;
        // otherwise large RDB scenes accumulate in VTD and eventually hit its
        // 50 MiB per-client output-buffer limit.
    }
}

std::optional<double> matchRdbSimulationTime(RdbClockData& clock,
                                             const EgoState& ego,
                                             double positionTolerance) {
    const double toleranceSquared = positionTolerance * positionTolerance;
    // 정차 중에는 여러 프레임의 좌표가 완전히 같다. 최신 샘플을 고르면
    // 중간 시각을 건너뛰므로, 도착 순서가 보존되는 가장 오래된 일치값을 쓴다.
    for (auto iterator = clock.samples.begin();
         iterator != clock.samples.end(); ++iterator) {
        const double dx = iterator->egoX - static_cast<double>(ego.x);
        const double dy = iterator->egoY - static_cast<double>(ego.y);
        if (dx * dx + dy * dy > toleranceSquared) continue;
        clock.lastMatchedSample = *iterator;
        clock.hasLastMatchedSample = true;
        const double result = clock.lastMatchedSample.simulationTime;
        clock.samples.erase(clock.samples.begin(), std::next(iterator));
        return result;
    }
    // 9910 플러그인이 같은 VTD snapshot을 여러 번 보낼 수 있다. 새 RDB
    // 메시지가 없고 Ego 좌표도 같으면 시각 역시 같은 프레임으로 유지한다.
    if (clock.hasLastMatchedSample) {
        const double dx = clock.lastMatchedSample.egoX -
                          static_cast<double>(ego.x);
        const double dy = clock.lastMatchedSample.egoY -
                          static_cast<double>(ego.y);
        if (dx * dx + dy * dy <= toleranceSquared) {
            return clock.lastMatchedSample.simulationTime;
        }
    }
    return std::nullopt;
}
