// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <random>

class RandomEngine
{
public:

  uint32_t GenerateRandomSeed()
  {
    std::random_device random_device;
    std::uniform_int_distribution<uint32_t> dist(
        std::numeric_limits<uint32_t>::lowest(),
        std::numeric_limits<uint32_t>::max());
    return dist(random_device);
  }

  /// Seed the random engine.
  void Seed(uint32_t in_seed)
  {
    _engine.seed(in_seed);
  }

  // ===========================================================================
  /// @name Uniform distribution
  // ===========================================================================

  float GetUniformFloat()
  {
    return std::uniform_real_distribution<float>()(_engine);
  }

  float GetUniformFloatInRange(float minimum, float maximum)
  {
    return std::uniform_real_distribution<float>(minimum, maximum)(_engine);
  }

  int GetUniformIntInRange(int minimum, int maximum)
  {
    return std::uniform_int_distribution<int>(minimum, maximum)(_engine);
  }

  bool GetUniformBool()
  {
    return (GetUniformIntInRange(0, 1) == 1);
  }

  // ===========================================================================
  /// @name Other distributions
  // ===========================================================================

  bool GetBernoulliDistribution(float P)
  {
    return std::bernoulli_distribution(P)(_engine);
  }

  int GetBinomialDistribution(int T, float P)
  {
    return std::binomial_distribution<int>(T, P)(_engine);
  }

  int GetPoissonDistribution(float Mean)
  {
    return std::poisson_distribution<int>(Mean)(_engine);
  }

  float GetExponentialDistribution(float Lambda)
  {
    return std::exponential_distribution<float>(Lambda)(_engine);
  }

  float GetNormalDistribution(float mean, float standard_deviation)
  {
    return std::normal_distribution<float>(mean, standard_deviation)(_engine);
  }

  // ===========================================================================
  /// @name Sampling distributions
  // ===========================================================================

  bool GetBoolWithWeight(float Weight)
  {
    return (Weight >= GetUniformFloat());
  }

  // int GetIntWithWeight(const TArray<float> &Weights)
  // {
  //   return std::discrete_distribution<int>(
  //       Weights.GetData(),
  //       Weights.GetData() + Weights.Num())(_engine);
  // }

  // ===========================================================================
  /// @name Elements in TArray
  // ===========================================================================

  // template <typename T>
  // auto &PickOne(const TArray<T> &Array)
  // {
  //   check(Array.Num() > 0);
  //   return Array[GetUniformIntInRange(0, Array.Num() - 1)];
  // }

  // template <typename T>
  // void Shuffle(TArray<T> &Array)
  // {
  //   std::shuffle(Array.GetData(), Array.GetData() + Array.Num(), _engine);
  // }

private:

  std::minstd_rand _engine;
};
