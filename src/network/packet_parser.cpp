#include "packet_parser.hpp"

#include <cstring>

namespace
{

std::uint32_t readUint32LE(const std::uint8_t* data)
{
    return static_cast<std::uint32_t>(data[0])
         | (static_cast<std::uint32_t>(data[1]) << 8)
         | (static_cast<std::uint32_t>(data[2]) << 16)
         | (static_cast<std::uint32_t>(data[3]) << 24);
}


std::int32_t readInt32LE(const std::uint8_t* data)
{
    const std::uint32_t value = readUint32LE(data);

    std::int32_t result;
    std::memcpy(&result, &value, sizeof(result));

    return result;
}


float readFloat32LE(const std::uint8_t* data)
{
    const std::uint32_t bits = readUint32LE(data);

    float value;
    std::memcpy(&value, &bits, sizeof(value));

    return value;
}

} // namespace


bool PacketParser::parseGtFrame(
    const std::uint8_t* data,
    std::size_t size,
    GtFrame& output
)
{
    if (data == nullptr)
    {
        return false;
    }

    if (size != GT_FRAME_SIZE)
    {
        return false;
    }

    std::size_t offset = 0;


    // ========================================================
    // Ego
    // 6 x float32 = 24 Byte
    // ========================================================

    output.ego.x =
        readFloat32LE(data + offset);
    offset += 4;

    output.ego.y =
        readFloat32LE(data + offset);
    offset += 4;

    output.ego.z =
        readFloat32LE(data + offset);
    offset += 4;

    output.ego.heading =
        readFloat32LE(data + offset);
    offset += 4;

    output.ego.pitch =
        readFloat32LE(data + offset);
    offset += 4;

    output.ego.roll =
        readFloat32LE(data + offset);
    offset += 4;


    // ========================================================
    // Object[30]
    //
    // Object 1개:
    // uint32 id + float32 x 8
    // = 36 Byte
    // ========================================================

    for (auto& object : output.objects)
    {
        object.id =
            readUint32LE(data + offset);
        offset += 4;

        object.x =
            readFloat32LE(data + offset);
        offset += 4;

        object.y =
            readFloat32LE(data + offset);
        offset += 4;

        object.z =
            readFloat32LE(data + offset);
        offset += 4;

        object.heading =
            readFloat32LE(data + offset);
        offset += 4;

        object.speed =
            readFloat32LE(data + offset);
        offset += 4;

        object.length =
            readFloat32LE(data + offset);
        offset += 4;

        object.width =
            readFloat32LE(data + offset);
        offset += 4;

        object.height =
            readFloat32LE(data + offset);
        offset += 4;
    }


    // ========================================================
    // Traffic Light
    //
    // int32 id  = 4 Byte
    // uint8 state = 1 Byte
    // ========================================================

    output.trafficLight.id =
        readInt32LE(data + offset);
    offset += 4;

    output.trafficLight.state =
        data[offset];
    offset += 1;


    // 최종적으로 정확히 1109 Byte를 읽었는지 확인
    return offset == GT_FRAME_SIZE;
}