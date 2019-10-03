// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/sensor/data/IMUEvent.h"
#include "carla/sensor/s11n/IMUSerializer.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> IMUSerializer::Deserialize(RawData && data) {
    return SharedPtr<SensorData>(new data::IMUEvent(std::move(data)));
  }

} // namespace s11n
} // namespace sensor
} // namespace carla
