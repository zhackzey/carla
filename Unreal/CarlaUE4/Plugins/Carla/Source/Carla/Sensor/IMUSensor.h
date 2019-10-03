#pragma once

#include <Carla/Sensor/Sensor.h>

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include "IMUSensor.generated.h"

namespace cg = carla::geom;

UCLASS()
class CARLA_API AIMUSensor : public ASensor
{
  GENERATED_BODY()

public:

  AIMUSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Tick(float DeltaSeconds) override;

};
