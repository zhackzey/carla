// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/Debug.h"
#include "carla/client/detail/ActorVariant.h"
#include "carla/geom/Vector3D.h"
#include "carla/sensor/SensorData.h"
#include "carla/sensor/s11n/IMUSerializer.h"

namespace carla {
namespace sensor {
namespace data {

  /// A registered collision.
  class IMUEvent : public SensorData  {
    using Super = SensorData;

  protected:

    using Serializer = s11n::IMUSerializer;

    friend Serializer;

    explicit IMUEvent(const RawData &data)
      : Super(data),
        _accelerometer(Serializer::DeserializeRawData(data).accelerometer),
        _gyroscope(Serializer::DeserializeRawData(data).gyroscope),
        _compass(Serializer::DeserializeRawData(data).compass) {}

  public:

    /// Accelerometer data.
    geom::Vector3D GetAccelerometer() const {
      return _accelerometer;
    }

    /// Gyroscope data.
    geom::Vector3D GetGyroscope() const {
      return _gyroscope;
    }

    /// Compass data.
    geom::Vector3D GetCompass() const {
      return _compass;
    }

  private:

    geom::Vector3D _accelerometer;

    geom::Vector3D _gyroscope;

    geom::Vector3D _compass;
  };

} // namespace data
} // namespace sensor
} // namespace carla
