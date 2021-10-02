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

#include <iostream>

namespace localization
{

struct Particle
{
  // global positions
  double x = 0;
  double y = 0;
  double yaw = 0;

  // body velocities
  double x_vel = 0;
  double yaw_vel = 0;

  // normalized weight of the particle
  double weight = 1.0;

  Particle Weighted() const
  {
    Particle weighted = *this;
    weighted.x *= weight;
    weighted.y *= weight;
    weighted.yaw *= weight;
    weighted.x_vel *= weight;
    weighted.yaw_vel *= weight;
    weighted.weight = 1.0;
    return weighted;
  }

  Particle operator+(const Particle & other) const
  {
    Particle sum = *this;
    sum.x += other.x;
    sum.y += other.y;
    sum.yaw += other.yaw;
    sum.x_vel += other.x_vel;
    sum.yaw_vel += other.yaw_vel;
    sum.weight += other.weight;
    return sum;
  }
};

}  // namespace localization

#endif  // PARTICLE_HPP_
