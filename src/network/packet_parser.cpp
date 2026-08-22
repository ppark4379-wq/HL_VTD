#include "network/packet_parser.hpp"

#include "network/network_structs.hpp"

#include <cstring>

namespace hlvtd::network {
namespace {

std::uint32_t readUint32LE(const std::uint8_t* data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8) |
           (static_cast<std::uint32_t>(data[2]) << 16) |
           (static_cast<std::uint32_t>(data[3]) << 24);
}

std::int32_t readInt32LE(const std::uint8_t* data) {
    const std::uint32_t bits = readUint32LE(data);
    std::int32_t value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

float readFloat32LE(const std::uint8_t* data) {
    const std::uint32_t bits = readUint32LE(data);
    float value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace

bool parseGtFrame(
    const std::uint8_t* data,
    std::size_t size,
    GtFrame& output) {
    if (data == nullptr || size != GT_FRAME_SIZE) {
        return false;
    }

    std::size_t offset = 0;
    output.ego.x = readFloat32LE(data + offset);
    offset += 4;
    output.ego.y = readFloat32LE(data + offset);
    offset += 4;
    output.ego.z = readFloat32LE(data + offset);
    offset += 4;
    output.ego.heading = readFloat32LE(data + offset);
    offset += 4;
    output.ego.pitch = readFloat32LE(data + offset);
    offset += 4;
    output.ego.roll = readFloat32LE(data + offset);
    offset += 4;

    for (auto& object : output.objects) {
        object.id = readUint32LE(data + offset);
        offset += 4;
        object.x = readFloat32LE(data + offset);
        offset += 4;
        object.y = readFloat32LE(data + offset);
        offset += 4;
        object.z = readFloat32LE(data + offset);
        offset += 4;
        object.heading = readFloat32LE(data + offset);
        offset += 4;
        object.speed = readFloat32LE(data + offset);
        offset += 4;
        object.length = readFloat32LE(data + offset);
        offset += 4;
        object.width = readFloat32LE(data + offset);
        offset += 4;
        object.height = readFloat32LE(data + offset);
        offset += 4;
    }

    output.trafficLight.id = readInt32LE(data + offset);
    offset += 4;
    output.trafficLight.state = data[offset];
    offset += 1;
    return offset == GT_FRAME_SIZE;
}

}  // namespace hlvtd::network
