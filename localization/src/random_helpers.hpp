#ifndef RANDOM_HELPERS_HPP_
#define RANDOM_HELPERS_HPP_

#include <random>

namespace localization
{

class UniformRandomGenerator
{
public:
  UniformRandomGenerator();

  double Sample();

private:
  std::default_random_engine engine_;
  std::uniform_real_distribution<double> distribution_;
};

class GaussianRandomGenerator
{
public:
  GaussianRandomGenerator();

  double Sample();

private:
  std::default_random_engine engine_;
  std::normal_distribution<double> distribution_;
};

}  // namespace localization

#endif  // RANDOM_HELPERS_HPP_