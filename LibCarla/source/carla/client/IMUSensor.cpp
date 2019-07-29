// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/client/IMUSensor.h"

#include "carla/Logging.h"
#include "carla/client/Map.h"
#include "carla/client/detail/Simulator.h"
#include "carla/geom/GeoLocation.h"
#include "carla/sensor/data/IMUEvent.h"

#include <exception>

namespace carla {
namespace client {

  // ===========================================================================
  // -- IMUCallback -----------------------------------------------------------
  // ===========================================================================

  class IMUCallback {
  public:

    IMUCallback(
    ActorId sensor_id,
    const Vehicle &vehicle,
    geom::Vector3D &bias,
    std::function<float(void)>&& noise_function,
    Sensor::CallbackFunctionType && user_callback)
      : _sensor_id(sensor_id),
        _vehicle_id(vehicle.GetId()),
        _bias(bias),
        _noise_function(std::move(noise_function)),
        _callback(std::move(user_callback)) {}

    void Tick(const WorldSnapshot &snapshot);

  private:

    ActorId _sensor_id;

    ActorId _vehicle_id;

    carla::geom::Vector3D _bias;

    std::function<float(void)> _noise_function;

    Sensor::CallbackFunctionType _callback;
  };

  void IMUCallback::Tick(const WorldSnapshot &snapshot) {
    // Take parent and child snapshots.
    auto vehicle_snapshot = snapshot.Find(_vehicle_id);
    if (!vehicle_snapshot) {
      return;
    }
    // Obtain rotation
    auto sensor_snapshot = snapshot.Find(_sensor_id);
    if (!sensor_snapshot) {
      return;
    }

    // Obtain sensor world acceleration
    geom::Vector3D location = static_cast<geom::Vector3D>(sensor_snapshot->transform.location);

    auto vehicle_acceleration = vehicle_snapshot->acceleration;
    sensor_snapshot->transform.TransformPoint(vehicle_acceleration);
    auto acceleration = sensor_snapshot->acceleration + vehicle_acceleration - location;

    // Obtain sensor world angular velocity
    auto vehicle_angular_velocity = vehicle_snapshot->angular_velocity;
    sensor_snapshot->transform.TransformPoint(vehicle_angular_velocity);
    auto angular_velocity = sensor_snapshot->angular_velocity + vehicle_angular_velocity - location;

    // Obtain sensor compass
    // TODO: Get the north from Waypoints API
    geom::Vector3D compass(0.0f, 0.0f, 1.0f);
    sensor_snapshot->transform.TransformPoint(compass);
    compass -= location;

    // TODO: Apply noise to all axis
    float noise = _noise_function();
    acceleration.x += noise;

    // Get angle from forward vector
    _callback(MakeShared<sensor::data::IMUEvent>(
        snapshot.GetTimestamp().frame,
        snapshot.GetTimestamp().elapsed_seconds,
        sensor_snapshot->transform,
        acceleration + _bias,
        angular_velocity + _bias,
        compass + _bias));

  }
  // ===========================================================================
  // -- IMUSensor -------------------------------------------------------------
  // ===========================================================================

  IMUSensor::~IMUSensor() {
    Stop();
  }

  void IMUSensor::Listen(CallbackFunctionType callback) {
    auto episode = GetEpisode().Lock();
    auto vehicle = boost::dynamic_pointer_cast<Vehicle>(GetParent());
    if (vehicle == nullptr) {
      log_error(GetDisplayId(), ": not attached to a vehicle");
      return;
    }

    auto cb = std::make_shared<IMUCallback>(
        GetId(),
        *vehicle,
        bias,
        std::move(_noise_function),
        std::move(callback));

    const size_t callback_id = episode->RegisterOnTickEvent([cb = std::move(cb)](const auto &snapshot) {
      try {
        cb->Tick(snapshot);
      } catch (const std::exception &e) {
        log_error("IMUSensor:", e.what());
      }
    });

    const size_t previous = _callback_id.exchange(callback_id);
    if (previous != 0u) {
      episode->RemoveOnTickEvent(previous);
    }
  }

  void IMUSensor::Stop() {
    const size_t previous = _callback_id.exchange(0u);
    auto episode = GetEpisode().TryLock();
    if ((previous != 0u) && (episode != nullptr)) {
      episode->RemoveOnTickEvent(previous);
    }
  }

} // namespace client
} // namespace carla
