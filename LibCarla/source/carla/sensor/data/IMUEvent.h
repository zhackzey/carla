// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/sensor/SensorData.h"

namespace carla {
namespace sensor {
namespace data {

  /// A change of IMU data.
  class IMUEvent : public SensorData {
  public:

    explicit IMUEvent(
    size_t frame,
    double timestamp,
    const rpc::Transform &sensor_transform,
    carla::geom::Vector3D accelerometer,
    carla::geom::Vector3D gyroscope,
    carla::geom::Vector3D compass
    ) : SensorData(frame, timestamp, sensor_transform),
        _accelerometer(accelerometer),
        _gyroscope(gyroscope),
        _compass(compass) {}

    carla::geom::Vector3D GetAccelerometer() const {
      return _accelerometer + _bias;
    }

    carla::geom::Vector3D GetGyroscope() const {
      return _gyroscope + _bias;
    }

    carla::geom::Vector3D GetCompass() {
      return _compass;
    }

  private:

    carla::geom::Vector3D _accelerometer;
    carla::geom::Vector3D _gyroscope;
    carla::geom::Vector3D _compass;
    carla::geom::Vector3D _bias;
  };

} // namespace data
} // namespace sensor
} // namespace carla
