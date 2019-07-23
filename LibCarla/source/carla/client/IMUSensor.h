// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/ClientSideSensor.h"

#include <atomic>

namespace carla {
namespace client {

  class IMUSensor final : public ClientSideSensor {
  public:

    using ClientSideSensor::ClientSideSensor;

    ~IMUSensor();

    /// Register a @a callback to be executed each time a new measurement is
    /// received.
    ///
    /// @warning Calling this function on a sensor that is already listening
    /// steals the data stream from the previously set callback. Note that
    /// several instances of Sensor (even in different processes) may point to
    /// the same sensor in the simulator.
    void Listen(CallbackFunctionType callback) override;

    /// Stop listening for new measurements.
    void Stop() override;

    /// Return whether this Sensor instance is currently listening to the
    /// associated sensor in the simulator.
    bool IsListening() const override {
      return _callback_id != 0u;
    }

    void SetNoise(std::function<void(geom::Vector3D)> in_noise) {
      noise = in_noise;
    }
    /// User defined parameters
    geom::Vector3D bias;
    std::function<void(geom::Vector3D)> noise;

  private:
    std::atomic_size_t _callback_id{0u};
  };

} // namespace client
} // namespace carla
