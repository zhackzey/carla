// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Memory.h"
#include "carla/sensor/RawData.h"

namespace carla {
namespace sensor {

  class SensorData;

namespace s11n {

  /// Serializes the current state of the whole episode.
  class IMUSerializer {
  public:

    struct Data {

      geom::Vector3D accelerometer;
      geom::Vector3D gyroscope;
      geom::Vector3D compass;

      MSGPACK_DEFINE_ARRAY(accelerometer, gyroscope, compass)
    };

    static Data DeserializeRawData(const RawData &message) {
      return MsgPack::UnPack<Data>(message.begin(), message.size());
    }

    template <typename SensorT>
    static Buffer Serialize(
    const SensorT &,
    geom::Vector3D accelerometer,
    geom::Vector3D gyroscope,
    geom::Vector3D compass) {
      return MsgPack::Pack(Data{accelerometer, gyroscope, compass});
    }

    static SharedPtr<SensorData> Deserialize(RawData && data);
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
