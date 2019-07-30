// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/client/ClientSideSensor.h"
#include "carla/client/RandomEngine.h"

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

    void SetDefaultNoise(float mean = 0.0f, float stddev = 1.0f) {
      RandomEngine random_engine;
      uint32_t seed = random_engine.GenerateRandomSeed();
      random_engine.Seed(seed);
      _noise_function = [=]() mutable {
        return random_engine.GetNormalDistribution(mean, stddev);
      };
    }

    void SetNoiseFunction(std::function<float(void)> noise_function) {
      _noise_function = noise_function;
    }

    /// User defined parameters
    geom::Vector3D bias;

    std::function<float(void)> _noise_function = nullptr;

  private:

    std::atomic_size_t _callback_id{0u};
  };

} // namespace client
} // namespace carla
