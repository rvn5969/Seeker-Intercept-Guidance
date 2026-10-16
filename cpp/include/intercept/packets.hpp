#pragma once
#include <cstdint>

namespace intercept {
#pragma pack(push,1)
struct SensorPacket {
  std::uint32_t magic{0x5345454B};
  double measurement_timestamp{}, current_timestamp{};
  double range{}, bearing{}, own_speed{}, own_heading{}, own_lateral_accel{};
  std::uint8_t valid{1};
};
struct CommandPacket {
  std::uint32_t magic{0x434D4421};
  double timestamp{}, lateral_accel{};
  std::uint8_t valid{1};
};
#pragma pack(pop)
}
