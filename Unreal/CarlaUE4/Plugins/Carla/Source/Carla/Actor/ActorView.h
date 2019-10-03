// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla/Actor/ActorInfo.h"

class AActor;

/// A view over an actor and its properties.
class FActorView
{
public:

  using IdType = uint32;

  enum class ActorType : uint8
  {
    Other,
    Vehicle,
    Walker,
    TrafficLight,
    INVALID
  };

  FActorView() = default;
  FActorView(const FActorView &) = default;
  FActorView(FActorView &&) = default;

  bool IsValid() const
  {
    return (TheActor != nullptr) && !TheActor->IsPendingKill();
  }

  IdType GetActorId() const
  {
    return Id;
  }

  ActorType GetActorType() const
  {
    return Type;
  }

  AActor *GetActor()
  {
    return TheActor;
  }

  const AActor *GetActor() const
  {
    return TheActor;
  }

  const FActorInfo *GetActorInfo() const
  {
    return Info.Get();
  }

  FVector GetVelocity() const
  {
    constexpr float TO_METERS = 1e-2;
    return TheActor->GetVelocity() * TO_METERS;
  }

  FVector GetAngularVelocity() const
  {
    return Info->AngularVelocity;
  }

  FVector GetAcceleration() const
  {
    return Info->Acceleration;
  }

  void SetAngularVelocity(const FVector &AngularVelocity) const
  {
    Info->AngularVelocity = AngularVelocity;
  }

  void SetAcceleration(const FVector &Acceleration) const
  {
    Info->Acceleration = Acceleration;
  }

private:

  friend class FActorRegistry;

  FActorView(IdType ActorId, AActor &Actor, TSharedPtr<const FActorInfo> Info)
    : Id(ActorId),
      TheActor(&Actor),
      Info(std::move(Info))
  {}

  IdType Id = 0u;

  ActorType Type = ActorType::Other;

  AActor *TheActor = nullptr;

  TSharedPtr<const FActorInfo> Info = nullptr;
};
