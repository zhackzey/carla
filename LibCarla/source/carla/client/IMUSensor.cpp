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
    carla::geom::Vector3D &bias,
    Sensor::CallbackFunctionType && user_callback)
      : _sensor_id(sensor_id),
        _parent_id(vehicle.GetId()),
        _bias(bias),
        _callback(std::move(user_callback)) {}

    void Tick(const WorldSnapshot &snapshot);

  private:

    ActorId _sensor_id;

    ActorId _parent_id;

    carla::geom::Vector3D _bias;

    // Additional stored values
    carla::geom::Vector3D _prev_sensor_location;
    carla::geom::Vector3D _prev_sensor_velocity;
    carla::geom::Vector3D _prev_sensor_forward;

    Sensor::CallbackFunctionType _callback;
  };

  float getAngleFromVectors(geom::Vector3D &a, geom::Vector3D &b) {
    return atan2f(geom::Math::Cross(a, b).Length(), geom::Math::Dot(a, b));
  }

  void IMUCallback::Tick(const WorldSnapshot &snapshot) {
    // Take parent and child snapshots.
    auto parent_snapshot = snapshot.Find(_parent_id);
    if (!parent_snapshot) {
      return;
    }

    // TODO: Provide data atomicity
    // First frame it'll be null.
    // if ((prev == nullptr) && _bounds.compare_exchange(&prev, next)) {
    //   return;
    // }

    auto sensor_snapshot = snapshot.Find(_sensor_id);

    // Obtain rotation
    auto parent_transform = parent_snapshot->transform;

    // Obtain sensor world angular velocity
    auto sensor_location = static_cast<geom::Vector3D>(sensor_snapshot->transform.location);
    parent_transform.TransformPoint(sensor_location);

    auto sensor_location_forward = sensor_location + sensor_snapshot->transform.rotation.GetForwardVector();
    parent_transform.TransformPoint(sensor_location_forward);

    auto sensor_forward = (sensor_location_forward - sensor_location).MakeUnitVector();

    // Make it per axis
    auto sensor_forward_xy = geom::Vector3D(sensor_forward.x, sensor_forward.y, 0.0f);
    auto _prev_sensor_forward_xy = geom::Vector3D(_prev_sensor_forward.x, _prev_sensor_forward.y, 0.0f);
    auto sensor_diff_angle_xy = getAngleFromVectors(sensor_forward_xy, _prev_sensor_forward_xy);

    auto sensor_forward_xz = geom::Vector3D(sensor_forward.x, 0.0f, sensor_forward.z);
    auto _prev_sensor_forward_xz = geom::Vector3D(_prev_sensor_forward.x, 0.0f, _prev_sensor_forward.z);
    auto sensor_diff_angle_xz = getAngleFromVectors(sensor_forward_xz, _prev_sensor_forward_xz);

    auto sensor_forward_yz = geom::Vector3D(0.0f, sensor_forward.y, sensor_forward.z);
    auto _prev_sensor_forward_yz = geom::Vector3D(0.0f, _prev_sensor_forward.y, _prev_sensor_forward.z);
    auto sensor_diff_angle_zy = getAngleFromVectors(sensor_forward_yz, _prev_sensor_forward_yz);

    auto sensor_diff_angle = geom::Vector3D(sensor_diff_angle_xy, sensor_diff_angle_xz, sensor_diff_angle_zy);
    float dt = static_cast<float>(snapshot.GetTimestamp().delta_seconds);
    auto sensor_angular_velocity = sensor_diff_angle / dt;

    // Obtain sensor acceleration
    auto sensor_velocity = (sensor_location - _prev_sensor_location) / dt;
    auto sensor_acceleration = (sensor_velocity - _prev_sensor_velocity) / dt;

    // Obtain sensor compass
    // TODO: Get the north from Waypoints API
    auto compass_angle = geom::Vector3D(1.0f, 0.0f, 0.0f);

    // Get angle from forward vector
    _callback(MakeShared<sensor::data::IMUEvent>(
        snapshot.GetTimestamp().frame,
        snapshot.GetTimestamp().elapsed_seconds,
        sensor_snapshot->transform,
        sensor_acceleration + _bias,
        sensor_angular_velocity,
        compass_angle));

    // Update for previous frame values
    _prev_sensor_location = sensor_location;
    _prev_sensor_velocity = sensor_velocity;
    _prev_sensor_forward = sensor_forward;

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
        _bias,
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
