#include "random_helpers.hpp"

namespace localization
{

// TODO(barulicm) Do we need to use a fixed random seed?
constexpr auto RANDOM_SEED = 100;

UniformRandomGenerator::UniformRandomGenerator()
: engine_(RANDOM_SEED), distribution_(0.0, 1.0)
{
}

double UniformRandomGenerator::Sample()
{
  return distribution_(engine_);
}

GaussianRandomGenerator::GaussianRandomGenerator()
: engine_(RANDOM_SEED), distribution_(0.0, 1.0)
{
}

double GaussianRandomGenerator::Sample()
{
  return distribution_(engine_);
}

}  // namespace localization
