#include "Carla.h"
#include "Carla/Sensor/IMUSensor.h"

#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

AIMUSensor::AIMUSensor(const FObjectInitializer &ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

FActorDefinition AIMUSensor::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeGenericSensorDefinition(
      TEXT("other"),
      TEXT("imu"));
}

void AIMUSensor::Tick(float DeltaSeconds)
{
  Super::Tick(DeltaSeconds);

  auto Stream = GetDataStream(*this);

  auto Vehicle = Super::GetOwner();
  auto &Episode = GetEpisode();
  if (Vehicle != nullptr)
  {
    // Get the Actor Views
    FActorView SensorView = Episode.FindActor(this);
    FActorView VehicleView = Episode.FindActor(Vehicle);

    // Get Vehicle Transform and build its inverse rotation transform
    FTransform VehicleTransform = VehicleView.GetActor()->GetTransform();
    FVector VehicleLocation = VehicleTransform.GetLocation();

    cg::Location vehicle_location(VehicleLocation.X, VehicleLocation.Y, VehicleLocation.Z);
    cg::Rotation vehicle_rotation(VehicleTransform.Rotator());

    cg::Transform vehicle_transform(vehicle_location, vehicle_rotation);
    float inv_vehicle_pitch = -vehicle_transform.rotation.pitch;
    float inv_vehicle_yaw = -vehicle_transform.rotation.yaw;
    float inv_vehicle_roll = -vehicle_transform.rotation.roll;

    cg::Transform inv_vehicle_transform(cg::Location(), cg::Rotation(
        inv_vehicle_pitch,
        inv_vehicle_yaw,
        inv_vehicle_roll));

    // Get Sensor Transform and builf its inverse rotation transform
    cg::Transform sensor_transform = GetActorTransform();
    float inv_sensor_pitch = -sensor_transform.rotation.pitch;
    float inv_sensor_yaw = -sensor_transform.rotation.yaw;
    float inv_sensor_roll = -sensor_transform.rotation.roll;

    cg::Transform inv_sensor_transform(cg::Location(), cg::Rotation(
        inv_sensor_pitch,
        inv_sensor_yaw,
        inv_sensor_roll));

    // TODO: Check and test calculations
    // Note that we need to send the accelerometer, gyroscope and compass data
    // in Sensor coordinates

    // Accelerometer data: Sum of the sensor acceleration and vehicle
    // acceleration in sensor coordinates

    // Get Acceleration for vehicle and sensor
    FVector VehicleAcc = VehicleView.GetAcceleration();
    cg::Vector3D vehicle_acc(VehicleAcc.X, VehicleAcc.Y, VehicleAcc.Z);

    FVector SensorAcc = SensorView.GetAcceleration();
    cg::Vector3D sensor_acc(SensorAcc.X, SensorAcc.Y, SensorAcc.Z);

    // Convert to sensor coordinates
    inv_vehicle_transform.TransformPoint(vehicle_acc);
    vehicle_acc = inv_vehicle_transform.location - vehicle_transform.location;

    cg::Vector3D Acceleration = vehicle_acc;

    // Angular Velocity (Gyroscope): Sum of the sensor and vehicle angular
    // velocity in sensor coordinates

    // Get World Coordinates
    FVector SensorAngVel = SensorView.GetAngularVelocity();
    FVector VehicleAngVel = VehicleView.GetAngularVelocity();

    FVector AngularVelocity = SensorAngVel + VehicleAngVel;
    cg::Vector3D angular_velocity(AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z);

    // TODO: Convert gyroscope data from world coordinates to sensor coordinates

    // Compass
    // World coordinate of compass is the negative Y axis of map
    cg::Vector3D compass(0.0f, -10000000.0f, 0.0f);

    // Convert it to sensor coordinates
    inv_vehicle_transform.TransformPoint(compass);
    inv_sensor_transform.TransformPoint(compass);
    compass = compass.MakeUnitVector();

    // Sends the data to the stream each frame
    Stream.Send(*this,
        Acceleration,
        angular_velocity,
        compass);
  }

}
