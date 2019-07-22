// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include "IMUSensor.generated.h"

/// Sensor that produces "depth" images.
UCLASS()
class CARLA_API AIMUSensor : public ASensor
{
  GENERATED_BODY()

public:

  AIMUSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

};
