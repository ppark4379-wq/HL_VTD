#pragma once

#include "data/gt/gt_structs.hpp"
#include "network/network_structs.hpp"

#include <optional>

RdbClockData makeRdbClock(const VtdNetworkConfig& config);
void connectRdbClock(RdbClockData& clock);
void disconnectRdbClock(RdbClockData& clock) noexcept;
bool isRdbClockConnected(const RdbClockData& clock) noexcept;

// 현재 소켓에 도착해 있는 RDB 메시지를 읽어 frameNo/simTime과 Ego 위치를
// 보관한다. 기다리는 시간은 connect 뒤 설정한 SO_RCVTIMEO로 제한된다.
void pollRdbClock(RdbClockData& clock);

// 9910 GT의 Ego 위치와 같은 RDB 프레임만 골라 시뮬레이션 시각을 반환한다.
std::optional<double> matchRdbSimulationTime(RdbClockData& clock,
                                             const EgoState& ego,
                                             double positionTolerance = 0.05);

// 수신 버퍼에 완성된 RDB 메시지가 있으면 표준 헤더와 Ego 상태를 파싱한다.
// 네트워크 없이 프레이밍을 단위 테스트할 수 있도록 분리해 공개한다.
void extractRdbClockSamples(RdbClockData& clock);
