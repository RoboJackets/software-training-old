// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include <random>
#include <iostream>

namespace localization
{

struct Particle
{
  // global positions
  double x = 0;
  double y = 0;
  double yaw = 0;

  // body rates
  double vx = 0;
  double yaw_rate = 0;

  // normalized weight of the particle
  double weight = 1.0;
};

class ParticleNoise
{
public:
  ParticleNoise()
  {
    normal_dist_ = std::normal_distribution<double>(0, 1.0);
    uniform_dist_ = std::uniform_real_distribution<double>(0, 1);
    generator_ = std::default_random_engine(100);
  }

  double sampleUniform()
  {
    return uniform_dist_(generator_);
  }

  double sampleGaussian()
  {
    return normal_dist_(generator_);
  }

private:
  // creates N(0,1)
  std::normal_distribution<double> normal_dist_;

  // creates U(0,1)
  std::uniform_real_distribution<double> uniform_dist_;
  std::default_random_engine generator_;
};

}  // namespace localization

#endif  // PARTICLE_HPP_
